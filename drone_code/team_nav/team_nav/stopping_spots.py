"""Finds where the drone stops on a planned route, and moves those stops away from boulders.

This runs on the drone and in the simulator. route_planner.py uses it after straightening a
route. route_follower.py uses its rule for which waypoints the drone stops at.

Why stops matter
----------------
The drone stops at every real corner of its route (route_follower.py explains why). A stop is
the worst place for a boulder to be under the bottom laser. When a boulder comes under the
laser, the laser's reading suddenly drops. For the next few seconds, if the drone moves sideways
slower than about 0.32 m/s, PX4 may stop trusting the laser for height (docs/ARCHITECTURE.md,
section 7). A drone slowing down for a corner, or pausing at one, is exactly that slow.

So every stop is kept at least STOP_KEEP_OFF_BOULDER_METRES from the edge of every place a
boulder might be. That means the boulders in the drone's map, and also those in any other map
of the course the planner is given, such as the organizers' drawing. Boulder positions are
uncertain, so one map's best guess isn't enough.

How a stop is moved
-------------------
An ordinary corner may move anywhere within LARGEST_STOP_MOVE_METRES. The two legs on either
side must keep as much room from walls as before (or at least the planner's comfortable room).
They must keep as far from boulders as the planner keeps while moving, or, where the old legs
already had to pass closer, come no closer than those did. And the stop must not move across a
checkpoint line. Of the places that pass, the one furthest from boulders wins, up to the
keep-off distance. Among equally good places, the one that adds least to the route wins.

The two ends of an opening crossing, such as the window, can't move sideways, because the drone
must fly straight through the opening's middle. Instead, a new stop is added further back along
the crossing line (before the line-up point) or further on (after the exit point). The old point
stays as a waypoint that the drone flies straight through without stopping.

If no place passes, the stop stays where it is, and stops_near_boulders() reports it.

What this file does not do: choose the route. It only adjusts the stops of a planned route.
"""
from dataclasses import dataclass

import numpy as np

from team_nav.boulder_distances import closest_approach_to_each_boulder_metres, distance_to_nearest_boulder_metres
from team_nav.clearance_grid import room_along_line_metres
from team_nav.course_map import line_pieces_cross
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES

# Our own choice. A waypoint where the route turns less than this, seen from above, is passed
# without stopping. test_route_follower.py checks that the drone cuts such a turn by less than
# route_follower.py's CORNER_TOLERANCE_METRES.
STRAIGHT_ON_LARGEST_TURN_DEGREES = 10.0
# Our own choice, the same 5 cm as route_follower.py's CORNER_TOLERANCE_METRES. A leg shorter
# than this has no clear direction, so the waypoints at its ends count as stops.
SHORTEST_LEG_WITH_A_DIRECTION_METRES = 0.05

# Our own choice. The bottom laser's beam is somewhere between 1 and 27 degrees wide (we don't
# know yet). At the widest, at our 0.8 m cruise height, it sees the floor up to 0.19 m from the
# point straight below the drone. In simulator flights, the camera tracking had drifted about
# 0.20 m by the time the drone reached the window. Together that is about 0.40 m.
STOP_KEEP_OFF_BOULDER_METRES = 0.40
# Our own choice. A stop is moved by at most this much, so the route keeps its shape.
LARGEST_STOP_MOVE_METRES = 0.40
# Our own choice: half of route_planner.py's 5 cm grid square. Places to move a stop to are
# tried this far apart.
STOP_MOVE_STEP_METRES = 0.025
# Our own choice, the same half grid square route_planner.py allows when straightening. A moved
# leg may have this much less room than the old one, because the old one was only measured at
# points this far apart.
ROOM_TOLERANCE_METRES = 0.025
# Our own choice. A new place must be at least this much further from boulders than the old one,
# or the move isn't worth making.
SMALLEST_WORTHWHILE_GAIN_METRES = 0.01

# How each waypoint may be moved. route_planner.py chooses one for every waypoint.
MAY_MOVE_ANYWHERE_NEARBY = "may move anywhere nearby"
NEW_STOP_FURTHER_BACK = "a new stop may be added further back along the crossing"
NEW_STOP_FURTHER_ON = "a new stop may be added further on along the crossing"
STAYS_WHERE_IT_IS = "stays where it is"


@dataclass
class HowWaypointMayMove:
    rule: str
    # For the two crossing rules: the direction of the crossing seen from above, as a unit vector.
    crossing_direction: np.ndarray = None


@dataclass
class StopsKeptAwayFromBoulders:
    waypoints_metres: np.ndarray    # The route with its stops moved or added.
    new_number_of_each_old_waypoint: list


def find_stopping_spots(waypoints_metres):
    """For each waypoint, True if the drone stops there: the first and last waypoints, every one
    where the route turns by more than STRAIGHT_ON_LARGEST_TURN_DEGREES seen from above, and the
    ends of any leg that only climbs or descends, with no sideways part."""
    waypoints_metres = np.asarray(waypoints_metres, dtype=float)
    is_stop = [True] * len(waypoints_metres)
    for number in range(1, len(waypoints_metres) - 1):
        arriving = waypoints_metres[number][:2] - waypoints_metres[number - 1][:2]
        leaving = waypoints_metres[number + 1][:2] - waypoints_metres[number][:2]
        arriving_length, leaving_length = np.linalg.norm(arriving), np.linalg.norm(leaving)
        if arriving_length < SHORTEST_LEG_WITH_A_DIRECTION_METRES or leaving_length < SHORTEST_LEG_WITH_A_DIRECTION_METRES:
            continue
        cosine_of_turn = float(arriving @ leaving) / (arriving_length * leaving_length)
        turn_degrees = np.degrees(np.arccos(np.clip(cosine_of_turn, -1.0, 1.0)))
        is_stop[number] = bool(turn_degrees > STRAIGHT_ON_LARGEST_TURN_DEGREES)
    return is_stop


def stops_near_boulders(waypoints_metres, boulders):
    """Every stop closer than STOP_KEEP_OFF_BOULDER_METRES to a boulder's edge, as (waypoint
    number, boulder name, distance in metres; negative means above it)."""
    near = []
    is_stop = find_stopping_spots(waypoints_metres)
    for number, waypoint in enumerate(np.asarray(waypoints_metres, dtype=float)):
        if not is_stop[number]:
            continue
        distance_metres, boulder_name = _distance_to_nearest_boulder(waypoint, boulders)
        if distance_metres < STOP_KEEP_OFF_BOULDER_METRES:
            near.append((number, boulder_name, distance_metres))
    return near


def keep_stops_away_from_boulders(waypoints_metres, how_each_waypoint_may_move, boulders, shapes,
                                  checkpoint_lines, comfortable_room_metres, moving_keep_off_metres):
    """Moves, or adds, stops so they are further from boulders, as described at the top.

    shapes: what the drone could hit at its cruise height, as clearance_grid.py measures room.
    checkpoint_lines: the map's checkpoint lines, which a stop must not move across.
    comfortable_room_metres: the room from walls the planner is happy with. A moved leg may have
    less room than the old one only if it still has this much.
    moving_keep_off_metres: the distance from boulders the planner keeps while moving. A moved leg
    may come closer to a boulder than the old one only if it still keeps this far away."""
    waypoints = [np.asarray(waypoint, dtype=float) for waypoint in waypoints_metres]
    rules = list(how_each_waypoint_may_move)
    already_tried = [rule.rule == STAYS_WHERE_IT_IS for rule in rules]
    new_number_of_each_old_waypoint = list(range(len(waypoints)))
    checks = _MoveChecks(boulders, shapes, checkpoint_lines, comfortable_room_metres, moving_keep_off_metres)

    while True:
        number = _first_stop_to_try(waypoints, already_tried, boulders)
        if number is None:
            break
        already_tried[number] = True
        change = _best_change(waypoints, number, rules[number], checks)
        if change is None:
            continue
        inserted_at = change.apply(waypoints, rules, already_tried)
        if inserted_at is not None:
            for old_number, new_number in enumerate(new_number_of_each_old_waypoint):
                if new_number >= inserted_at:
                    new_number_of_each_old_waypoint[old_number] = new_number + 1

    return StopsKeptAwayFromBoulders(np.array(waypoints), new_number_of_each_old_waypoint)


def _first_stop_to_try(waypoints, already_tried, boulders):
    is_stop = find_stopping_spots(waypoints)
    for number, waypoint in enumerate(waypoints):
        if already_tried[number] or not is_stop[number]:
            continue
        if _distance_to_nearest_boulder(waypoint, boulders)[0] < STOP_KEEP_OFF_BOULDER_METRES:
            return number
    return None


def _distance_to_nearest_boulder(point_metres, boulders):
    distances_metres, names = distance_to_nearest_boulder_metres(np.asarray(point_metres, dtype=float)[np.newaxis, :2], boulders)
    return float(distances_metres[0]), str(names[0])


# ---- Trying places to move a stop to ---------------------------------------------------------

@dataclass
class _Change:
    """One way to move a stop: replace the waypoint at number with new_points."""
    number: int
    new_points: list
    new_stop_index_in_new_points: int

    def apply(self, waypoints, rules, already_tried):
        """Changes the lists in place. Returns the number of the added waypoint, or None if a
        waypoint was moved rather than added."""
        if len(self.new_points) == 1:
            waypoints[self.number] = self.new_points[0]
            return None
        waypoints[self.number:self.number + 1] = self.new_points
        # The old crossing point stays put, and the new stop isn't moved again.
        rules[self.number:self.number + 1] = [HowWaypointMayMove(STAYS_WHERE_IT_IS)] * 2
        already_tried[self.number:self.number + 1] = [True, True]
        return self.number + self.new_stop_index_in_new_points


def _best_change(waypoints, number, how_it_may_move, checks):
    """The best acceptable change for the stop at number, or None if no place is better."""
    current_distance_metres = _distance_to_nearest_boulder(waypoints[number], checks.boulders)[0]
    candidates = _candidate_changes(waypoints, number, how_it_may_move)
    scored = []
    for change in candidates:
        new_stop = change.new_points[change.new_stop_index_in_new_points]
        distance_metres = _distance_to_nearest_boulder(new_stop, checks.boulders)[0]
        if distance_metres < current_distance_metres + SMALLEST_WORTHWHILE_GAIN_METRES:
            continue
        gain_metres = min(distance_metres, STOP_KEEP_OFF_BOULDER_METRES)
        scored.append((-gain_metres, _added_length_metres(waypoints, number, change), len(scored), change))
    scored.sort(key=lambda entry: entry[:3])

    # Checking room is the slow part, so the best-scoring places are checked first.
    for _, _, _, change in scored:
        if checks.change_is_acceptable(waypoints, number, change):
            return change
    return None


def _candidate_changes(waypoints, number, how_it_may_move):
    point = waypoints[number]
    if how_it_may_move.rule == MAY_MOVE_ANYWHERE_NEARBY:
        return [_Change(number, [_moved(point, offset)], 0) for offset in _offsets_within_reach()]

    changes = []
    for distance_metres in _slide_distances():
        if how_it_may_move.rule == NEW_STOP_FURTHER_BACK:
            new_stop = _moved(point, -how_it_may_move.crossing_direction * distance_metres)
            changes.append(_Change(number, [new_stop, point], 0))
        elif how_it_may_move.rule == NEW_STOP_FURTHER_ON:
            new_stop = _moved(point, how_it_may_move.crossing_direction * distance_metres)
            changes.append(_Change(number, [point, new_stop], 1))
    return changes


def _moved(point, sideways_offset_metres):
    """The point moved sideways by the offset, at the same height."""
    moved_point = point.copy()
    moved_point[:2] = point[:2] + sideways_offset_metres
    return moved_point


def _offsets_within_reach():
    steps = int(LARGEST_STOP_MOVE_METRES / STOP_MOVE_STEP_METRES)
    offsets = []
    for east_steps in range(-steps, steps + 1):
        for north_steps in range(-steps, steps + 1):
            offset = np.array([east_steps, north_steps]) * STOP_MOVE_STEP_METRES
            if 0.0 < np.linalg.norm(offset) <= LARGEST_STOP_MOVE_METRES:
                offsets.append(offset)
    return offsets


def _slide_distances():
    """How far along the crossing a new stop may be added. It starts at the shortest leg that
    has a direction, so the old crossing point doesn't become a stop again."""
    return np.arange(SHORTEST_LEG_WITH_A_DIRECTION_METRES + STOP_MOVE_STEP_METRES,
                     LARGEST_STOP_MOVE_METRES + STOP_MOVE_STEP_METRES / 2.0, STOP_MOVE_STEP_METRES)


def _added_length_metres(waypoints, number, change):
    old_points = [waypoints[number - 1], waypoints[number], waypoints[number + 1]]
    new_points = [waypoints[number - 1]] + change.new_points + [waypoints[number + 1]]
    return _length_seen_from_above_metres(new_points) - _length_seen_from_above_metres(old_points)


def _length_seen_from_above_metres(points):
    total_metres = 0.0
    for start, end in zip(points[:-1], points[1:]):
        total_metres += float(np.linalg.norm(end[:2] - start[:2]))
    return total_metres


class _MoveChecks:
    """The rules a moved stop must pass, from the top of this file."""

    def __init__(self, boulders, shapes, checkpoint_lines, comfortable_room_metres, moving_keep_off_metres):
        self.boulders = boulders
        self.shapes = shapes
        self.checkpoint_lines = checkpoint_lines
        self.comfortable_room_metres = comfortable_room_metres
        self.moving_keep_off_metres = moving_keep_off_metres

    def change_is_acceptable(self, waypoints, number, change):
        old_point = waypoints[number]
        new_stop = change.new_points[change.new_stop_index_in_new_points]
        if self._moves_across_a_checkpoint_line(old_point, new_stop):
            return False
        old_legs = [(waypoints[number - 1], old_point), (old_point, waypoints[number + 1])]
        new_points = [waypoints[number - 1]] + change.new_points + [waypoints[number + 1]]
        new_legs = list(zip(new_points[:-1], new_points[1:]))
        # The legs through an opening, which change height, are never moved, so they are left out.
        old_legs = [leg for leg in old_legs if leg[0][2] == leg[1][2]]
        new_legs = [leg for leg in new_legs if leg[0][2] == leg[1][2]]
        return self._keeps_room(old_legs, new_legs) and self._keeps_away_from_boulders(old_legs, new_legs)

    def _moves_across_a_checkpoint_line(self, old_point, new_point):
        for line in self.checkpoint_lines:
            if line_pieces_cross(old_point[:2], new_point[:2], line.start_metres, line.end_metres):
                return True
        return False

    def _keeps_room(self, old_legs, new_legs):
        room_needed_metres = REQUIRED_ROOM_METRES
        if old_legs:
            old_room_metres = min(room_along_line_metres(self.shapes, start[:2], end[:2])[0] for start, end in old_legs)
            room_needed_metres = max(REQUIRED_ROOM_METRES, min(old_room_metres, self.comfortable_room_metres) - ROOM_TOLERANCE_METRES)
        for start, end in new_legs:
            if room_along_line_metres(self.shapes, start[:2], end[:2])[0] < room_needed_metres:
                return False
        return True

    def _keeps_away_from_boulders(self, old_legs, new_legs):
        old_closest_metres = self._closest_to_each_boulder_metres(old_legs)
        new_closest_metres = self._closest_to_each_boulder_metres(new_legs)
        for old_metres, new_metres in zip(old_closest_metres, new_closest_metres):
            # The same rule route_planner.py uses when straightening: legs that kept outside the
            # keep-off distance must still do so, and legs that were already closer may come
            # no closer (give or take the tolerance).
            if old_metres >= self.moving_keep_off_metres:
                distance_needed_metres = self.moving_keep_off_metres
            else:
                distance_needed_metres = old_metres - ROOM_TOLERANCE_METRES
            if new_metres < distance_needed_metres:
                return False
        return True

    def _closest_to_each_boulder_metres(self, legs):
        closest_metres = np.full(len(self.boulders), np.inf)
        for start, end in legs:
            approaches = closest_approach_to_each_boulder_metres(self.boulders, start, end)
            for boulder_number, (_, distance_metres) in enumerate(approaches):
                closest_metres[boulder_number] = min(closest_metres[boulder_number], distance_metres)
        return closest_metres
