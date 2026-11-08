"""Checks a path flown high above the walls (2.3 m) against the bottom laser's rules, and measures
every wall crossing on it.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. fallback_routes.py uses it to choose its turn points and to describe the routes it makes.

Why these rules
---------------
Drone 16's PX4 (the flight controller's software) takes its height mainly from the bottom laser,
which measures the distance straight down. High above the course, a wall top passes under the
laser as the drone crosses it, and the reading suddenly jumps from about 2.3 m to about 0.5 m.
PX4 may then stop trusting the laser, or wrongly believe it, especially if the drone slows down
soon after (docs/ARCHITECTURE.md, section 7). The team's rules for the fallback missions
(2026-11-07):
  - cross every wall square-on (perpendicular to it), at 0.5 m/s or more;
  - never stop or turn while a wall, a boulder or the tunnel is under the laser (a turn that only
    camera tracking's drift could bring over one is a caution, not a rule break);
  - change height only over open floor (fallback_routes.py places the climbs and descents).
At 2.3 m nothing in the course blocks the drone except the safety net, so the only thing that can
be hit is the net.

What "under the laser" means
----------------------------
The laser may see a cone up to 27 degrees wide (estimated; the organizers say about 1 degree). A
thing whose top is d metres below the drone is under the laser when the drone's centre is within
d x tan(13.5 degrees) of its edge: about 0.13 m for a wall top, 0.43 m for a boulder. Places to
turn or stop also keep camera tracking's possible drift on top of that.

What this file does not do: choose a route (fallback_routes.py does), or fly one.
"""
import math
from dataclasses import dataclass, field

import numpy as np

from team_nav.course_map import distance_to_line_piece_metres
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES

# Estimated. PX4's driver for the laser (a VL53L1X) says its cone is 27 degrees wide; the
# organizers say about 1 degree. We take the wider one, as stopping_spots.py does.
LASER_HALF_ANGLE_DEGREES = 13.5
# Our own choice, from simulator flights on 2026-11-07: camera tracking had drifted about 0.20 m
# by the window (stopping_spots.py uses the same). A turn or a stop keeps this much extra room.
POSITION_DRIFT_METRES = 0.20
# The team's rule (2026-11-07) is "perpendicular". Our own choice of how close counts: the same
# 10 degrees below which the route follower does not count a waypoint as a corner (stopping_spots.py).
LARGEST_CROSSING_ANGLE_DEGREES = 10.0
# Our own choice. A crossing this close to a wall's end is "near the end": the laser may see the
# end face and the floor together.
WALL_END_MARGIN_METRES = 0.25
# Our own choice: the path is checked at points this far apart.
SAMPLE_SPACING_METRES = 0.02
# Our own choice. A path whose direction changes by more than this between two checked points is
# turning there.
TURNING_DEGREES = 0.5
# The route follower (route_follower.py, with the fallback missions' settings in fallback_routes.py)
# makes the drone start from rest at a stop. A simple model of PX4 (it flies towards the target at
# about 0.77 m/s per metre of distance, docs/ARCHITECTURE.md section 9) with the target sliding at
# 1.0 m/s says the drone reaches 0.5 m/s after about 0.25 m. Our own choice: 0.35 m, with a margin.
RUN_UP_METRES = 0.35
# Our own choice. The drone trails its target by up to the fallback missions' 1.0 m lead, so it
# starts slowing down about that far before a stop. A wall crossed closer than this to the next
# stop is crossed while slowing down.
SLOW_DOWN_METRES = 1.0


@dataclass
class WallCrossing:
    wall_name: str
    degrees_off_perpendicular: float
    metres_from_last_stop: float          # Along the path, to where the wall first comes under the laser.
    metres_to_next_stop: float            # Along the path, from where the wall leaves the laser.
    crossing_point_metres: np.ndarray     # Where the path crosses the wall's centre line (east, north).

    def describe(self):
        return (f"crosses {self.wall_name} {self.degrees_off_perpendicular:.1f} degrees off square, "
                f"{self.metres_from_last_stop:.2f} m after the last stop and {self.metres_to_next_stop:.2f} m before "
                f"the next stop")


@dataclass
class PathCheck:
    """The result of checking one high path. Rule breaks are full sentences."""
    length_metres: float
    rule_breaks: list = field(default_factory=list)     # The team's rules broken: avoid at almost any cost.
    cautions: list = field(default_factory=list)        # Worth avoiding, and worth knowing about.
    crossings: list = field(default_factory=list)       # WallCrossing, in order along the path.

    def cost(self):
        """Smaller is better: rule breaks first, then cautions, then length."""
        return (len(self.rule_breaks), len(self.cautions), self.length_metres)


def laser_reach_metres(flying_height_metres, top_height_metres):
    """How far from a thing's edge the laser may still see its top, from this flying height."""
    drop_metres = max(flying_height_metres - top_height_metres, 0.0)
    return drop_metres * math.tan(math.radians(LASER_HALF_ANGLE_DEGREES))


def check_high_path(course_map, path_points_metres, flying_height_metres):
    """Checks a path flown at flying_height_metres from a stop at its first point to a stop at its
    last. path_points_metres is a list of (east, north) corners; arcs are many short pieces."""
    samples, along, headings = sample_path(path_points_metres)
    check = PathCheck(length_metres=float(along[-1]))
    _check_net(course_map, samples, check)
    _check_tunnels(course_map, samples, flying_height_metres, check)
    _check_walls(course_map, samples, along, headings, flying_height_metres, check)
    _check_run_up_and_slow_down(check)
    _check_turns(course_map, samples, headings, flying_height_metres, check)
    _check_low_obstacles_passed(course_map, samples, flying_height_metres, check)
    return check


def open_floor_clearances(course_map, point_metres, flying_height_metres):
    """For a place where the drone stops, turns, climbs or descends: (thing, metres it is from the
    room it needs) for every wall, tunnel and low obstacle. Negative means the thing is under the
    laser there (allowing for drift)."""
    point = np.asarray(point_metres, dtype=float)[np.newaxis, :2]
    margins = []
    for name, distance_metres, top_metres in _distances_to_everything(course_map, point):
        needed_metres = laser_reach_metres(flying_height_metres, top_metres) + POSITION_DRIFT_METRES
        margins.append((name, float(distance_metres[0]) - needed_metres))
    return margins


# ---- Sampling ------------------------------------------------------------------------------------

def sample_path(path_points_metres):
    """Points every SAMPLE_SPACING_METRES along the path, the distance along it to each, and the
    path's direction (a unit vector) at each."""
    points = np.asarray(path_points_metres, dtype=float)[:, :2]
    samples, along, headings = [points[0]], [0.0], []
    travelled_metres = 0.0
    for start, end in zip(points[:-1], points[1:]):
        length_metres = float(np.linalg.norm(end - start))
        if length_metres < 1e-9:
            continue
        direction = (end - start) / length_metres
        steps = max(1, int(math.ceil(length_metres / SAMPLE_SPACING_METRES)))
        for step in range(1, steps + 1):
            samples.append(start + direction * length_metres * step / steps)
            along.append(travelled_metres + length_metres * step / steps)
            headings.append(direction)
        travelled_metres += length_metres
    headings.insert(0, headings[0])
    return np.array(samples), np.array(along), np.array(headings)


# ---- The rules -----------------------------------------------------------------------------------

def _check_net(course_map, samples, check):
    room = np.minimum(samples - course_map.lowest_corner_metres, course_map.highest_corner_metres - samples).min(axis=1)
    least_room_metres = float(room.min())
    if least_room_metres < REQUIRED_ROOM_METRES:
        check.rule_breaks.append(f"The path comes {least_room_metres:.2f} m from the safety net; the drone needs "
                                 f"{REQUIRED_ROOM_METRES:.2f} m.")


def _check_tunnels(course_map, samples, flying_height_metres, check):
    for tube in course_map.tubes:
        reach_metres = laser_reach_metres(flying_height_metres, tube.outside_height_metres)
        if np.any(distance_to_tunnel_metres(samples, tube) <= reach_metres):
            check.rule_breaks.append(f"The path passes over the {tube.name}, whose top would come under the laser.")


def _check_walls(course_map, samples, along, headings, flying_height_metres, check):
    for wall in course_map.walls:
        reach_metres = laser_reach_metres(flying_height_metres, wall.top_height_metres)
        distance = distance_to_line_piece_metres(samples, wall.start_metres, wall.end_metres) - wall.thickness_metres / 2.0
        for first, last in _runs_of_true(distance <= reach_metres):
            _check_one_wall_run(wall, samples, along, headings, first, last, check)
    check.crossings.sort(key=lambda crossing: crossing.metres_from_last_stop)


def _check_one_wall_run(wall, samples, along, headings, first, last, check):
    """One stretch of the path where this wall is under the laser."""
    if first == 0 or last == len(samples) - 1:
        check.rule_breaks.append(f"The path starts or ends with {wall.name} under the laser, so the drone would stop "
                                 f"above it.")
        return
    side = _side_of_wall(wall, samples)
    if side[first - 1] == side[last + 1]:
        check.rule_breaks.append(f"The path passes along or over the end of {wall.name} without crossing it.")
        return
    if _turn_degrees_within(headings, first, last) > TURNING_DEGREES:
        check.rule_breaks.append(f"The path turns while {wall.name} is under the laser.")
    crossing_index = first + int(np.argmax(side[first:last + 2] != side[first - 1])) - 1
    crossing_point = samples[crossing_index + 1].copy()
    along_wall_metres = float((crossing_point - wall.start_metres) @ wall.direction())
    if not 0.0 <= along_wall_metres <= wall.length_metres():
        check.rule_breaks.append(f"The path passes over the end of {wall.name} without crossing it.")
        return
    heading = headings[crossing_index + 1]
    degrees_off = math.degrees(math.asin(min(1.0, abs(float(heading @ wall.direction())))))
    crossing = WallCrossing(wall.name, degrees_off, float(along[first]), float(along[-1] - along[last]),
                            crossing_point)
    check.crossings.append(crossing)
    if degrees_off > LARGEST_CROSSING_ANGLE_DEGREES:
        check.rule_breaks.append(f"The path crosses {wall.name} {degrees_off:.0f} degrees off square; the most we "
                                 f"allow is {LARGEST_CROSSING_ANGLE_DEGREES:.0f}.")
    if min(along_wall_metres, wall.length_metres() - along_wall_metres) < WALL_END_MARGIN_METRES:
        check.cautions.append(f"The path crosses {wall.name} within {WALL_END_MARGIN_METRES:.2f} m of its end.")


def _check_run_up_and_slow_down(check):
    if not check.crossings:
        return
    first = check.crossings[0]
    last = min(check.crossings, key=lambda crossing: crossing.metres_to_next_stop)
    if first.metres_from_last_stop < RUN_UP_METRES:
        check.cautions.append(f"{first.wall_name} comes under the laser only {first.metres_from_last_stop:.2f} m after "
                              f"a stop, so the drone may cross it slower than 0.5 m/s (it needs about "
                              f"{RUN_UP_METRES:.2f} m to speed up).")
    if last.metres_to_next_stop < SLOW_DOWN_METRES:
        check.cautions.append(f"The drone stops only {last.metres_to_next_stop:.2f} m after crossing {last.wall_name}, "
                              f"so it is already slowing down there (it needs about {SLOW_DOWN_METRES:.1f} m).")


def _check_turns(course_map, samples, headings, flying_height_metres, check):
    """Turning with something under the laser breaks the rule. Turning where camera tracking's
    drift could put it under the laser is a caution."""
    turning = _turning_points(headings)
    if not np.any(turning):
        return
    for name, distance, top_metres in _distances_to_everything(course_map, samples[turning]):
        reach_metres = laser_reach_metres(flying_height_metres, top_metres)
        closest_metres = float(distance.min())
        if closest_metres <= reach_metres:
            check.rule_breaks.append(f"The path turns with {name} under the laser ({closest_metres:.2f} m from its "
                                     f"edge; the laser may see {reach_metres:.2f} m around).")
        elif closest_metres < reach_metres + POSITION_DRIFT_METRES:
            check.cautions.append(f"The path turns {closest_metres:.2f} m from {name}'s edge; with "
                                  f"{POSITION_DRIFT_METRES:.2f} m of drift the laser could see it.")


def _check_low_obstacles_passed(course_map, samples, flying_height_metres, check):
    """Flying straight over a boulder at speed is allowed, but better avoided."""
    for obstacle in course_map.low_obstacles:
        reach_metres = laser_reach_metres(flying_height_metres, obstacle.top_height_metres)
        distance = np.linalg.norm(samples - obstacle.centre_metres, axis=1) - obstacle.radius_metres
        if np.any(distance <= reach_metres):
            check.cautions.append(f"The path passes over {obstacle.name} (at speed, without turning).")


# ---- Small helpers -------------------------------------------------------------------------------

def _distances_to_everything(course_map, points):
    """(name, distance from each point to the thing's edge, the thing's top height) for every wall,
    tunnel and low obstacle."""
    found = []
    for wall in course_map.walls:
        distance = distance_to_line_piece_metres(points, wall.start_metres, wall.end_metres) - wall.thickness_metres / 2.0
        found.append((wall.name, distance, wall.top_height_metres))
    for tube in course_map.tubes:
        found.append((tube.name, distance_to_tunnel_metres(points, tube), tube.outside_height_metres))
    for obstacle in course_map.low_obstacles:
        distance = np.linalg.norm(points - obstacle.centre_metres, axis=1) - obstacle.radius_metres
        found.append((obstacle.name, distance, obstacle.top_height_metres))
    return found


def distance_to_tunnel_metres(points, tube):
    """Each point's distance to the tunnel seen from above: a rectangle as long as the tunnel and as
    wide as its outside, with square ends. Zero inside it."""
    entrance = np.asarray(tube.entrance_centre_metres, dtype=float)
    along = (np.asarray(tube.exit_centre_metres, dtype=float) - entrance) / tube.length_metres()
    relative = np.asarray(points, dtype=float) - entrance
    along_metres = relative @ along
    sideways_metres = np.abs(relative[:, 0] * along[1] - relative[:, 1] * along[0])
    beyond_the_ends_metres = np.maximum(np.maximum(-along_metres, along_metres - tube.length_metres()), 0.0)
    beyond_the_sides_metres = np.maximum(sideways_metres - tube.outside_width_metres / 2.0, 0.0)
    return np.hypot(beyond_the_ends_metres, beyond_the_sides_metres)


def _runs_of_true(flags):
    """[(first index, last index)] of each stretch where flags is True."""
    runs, start = [], None
    for index, flag in enumerate(flags):
        if flag and start is None:
            start = index
        if not flag and start is not None:
            runs.append((start, index - 1))
            start = None
    if start is not None:
        runs.append((start, len(flags) - 1))
    return runs


def _side_of_wall(wall, samples):
    """+1 or -1 for each sample: which side of the wall's centre line it is on."""
    relative = samples - wall.start_metres
    direction = wall.direction()
    return np.sign(direction[0] * relative[:, 1] - direction[1] * relative[:, 0])


def _turn_degrees_within(headings, first, last):
    window = headings[first:last + 2]
    cosines = np.clip(np.sum(window[:-1] * window[1:], axis=1), -1.0, 1.0)
    return float(np.degrees(np.arccos(cosines)).sum()) if len(cosines) else 0.0


def _turning_points(headings):
    cosines = np.clip(np.sum(headings[:-1] * headings[1:], axis=1), -1.0, 1.0)
    turning = np.degrees(np.arccos(cosines)) > TURNING_DEGREES
    return np.concatenate([[False], turning])
