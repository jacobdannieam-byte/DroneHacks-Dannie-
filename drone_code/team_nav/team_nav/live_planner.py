"""Finds a safe path on the live map from the drone to its next goal. Called again and again
during the flight, so the path changes as soon as the depth camera sees something new.

REAL DRONE CODE, also used unchanged in the simulator.

How it chooses a path
---------------------
It is the classic "shortest path on squared paper" search (Dijkstra's method with a distance
guess, called A*). The drone may move from a square to any of its 8 neighbours. Each move costs
its length, made more expensive in three cases:
  - Close to something blocked. Below ROOM_NEEDED_METRES of room a square is forbidden; between
    that and COMFORTABLE_ROOM_METRES the cost rises, so the path keeps to the middle of gaps.
  - Near a boulder. They cost BOULDER_COST_FACTOR times more, so the path goes round boulders
    when there is room: a boulder under the bottom laser can make PX4 lose its height.
  - Through squares the camera has not seen yet. They cost UNSEEN_COST_FACTOR times more, so the
    drone prefers space it knows is empty, but can still plan into the unknown when it must.
    The drone always faces where it flies, so it sees an unknown square before it reaches it.

The goals
---------
The goals come from the mission file and the map, not from this code: for each checkpoint in
the mission, the far side of its line (the map's far_side_point), then the landing pad's centre.
A goal is reached when the drone crosses the checkpoint line, or comes within
GOAL_REACHED_METRES of the goal point.
"""
import heapq
import math
from dataclasses import dataclass

import numpy as np

from team_nav.clearance_grid import GRID_SQUARE_METRES
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES

# From drone_dimensions.py: the drone's radius (0.30 m, propeller tips) plus a 5 cm margin.
ROOM_NEEDED_METRES = REQUIRED_ROOM_METRES

# Our own choice, the same as route_planner.py: with this much room the path pays nothing extra.
COMFORTABLE_ROOM_METRES = 0.6

# Our own choice. Squeezing past an obstacle at the smallest allowed room costs this many times
# the length.
TIGHT_ROOM_COST_FACTOR = 4.0

# Our own choice. Moving through a square that has not been seen costs this many times more.
UNSEEN_COST_FACTOR = 2.0

# Our own choice. A metre near a boulder costs as much as this many metres elsewhere, so the
# path goes well out of its way to avoid one, but still crosses one if it must. It was 6, and in
# Gazebo run 20261107_212021 the path went straight over a boulder; after that PX4's height was
# wrong by about 0.2 m and the height check stopped the run. Crossing a boulder (about 0.9 m
# near it) now costs about 36 m, more than any way round inside the 6 m course.
BOULDER_COST_FACTOR = 40.0

# Our own choice. If the goal itself has too little room (the map was wrong), the nearest square
# with enough room within this distance becomes the goal.
LARGEST_GOAL_SHIFT_METRES = 0.6

# Our own choice. Close enough to a goal point to count as there.
GOAL_REACHED_METRES = 0.25

NEIGHBOUR_STEPS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


@dataclass
class Goal:
    stage_name: str            # Lower case, for example "maze", or "to the landing pad".
    point_metres: np.ndarray   # (east, north) to fly to.
    line_start_metres: object  # The checkpoint line's ends, or None for the landing pad.
    line_end_metres: object


def goals_from_mission(course_map, mission, landing_stage_name):
    """One goal per checkpoint in the mission, then the landing pad. For each checkpoint, the
    first line in the mission's order of preference that does not run across an opening in a
    wall (such as the window): crossing an opening needs a height change this navigation can't
    do yet."""
    goals = []
    for checkpoint in mission.checkpoints:
        line = _first_line_without_an_opening(course_map, checkpoint)
        goals.append(Goal(checkpoint.name.lower(), np.asarray(line.far_side_point_metres, dtype=float),
                          np.asarray(line.start_metres, dtype=float), np.asarray(line.end_metres, dtype=float)))
    goals.append(Goal(landing_stage_name, np.asarray(course_map.landing_pad_centre_metres[:2], dtype=float), None, None))
    return goals


def _first_line_without_an_opening(course_map, checkpoint):
    for line_name in checkpoint.line_names_in_order_of_preference:
        line = course_map.checkpoint_line_named(line_name)
        if line.opening_name is None:
            return line
    raise ValueError(f"The checkpoint '{checkpoint.name}' has only lines across openings in walls, which live "
                     f"navigation can't fly yet. Add a line at flying height to the mission.")


def has_reached(goal, previous_point_metres, drone_point_metres):
    """True once the drone is near the goal point, or its last move crossed the checkpoint line."""
    if np.linalg.norm(np.asarray(drone_point_metres[:2]) - goal.point_metres) < GOAL_REACHED_METRES:
        return True
    if goal.line_start_metres is None or previous_point_metres is None:
        return False
    return _segments_cross(goal.line_start_metres, goal.line_end_metres,
                           np.asarray(previous_point_metres[:2]), np.asarray(drone_point_metres[:2]))


def find_path(live_map, start_metres, goal_metres, room=None):
    """A list of (east, north) points from the drone to the goal, square by square, or None if
    no safe path exists on the map as it is now."""
    if room is None:
        room = live_map.room_metres()
    allowed = room >= ROOM_NEEDED_METRES
    start = live_map.square_of(start_metres)
    if start is None:
        return None
    # The drone may already be in a tight spot (for example right after a new wall was seen).
    # Let it start there, so it can find its way out.
    allowed[start] = True
    goal = _nearest_allowed_square(live_map, allowed, goal_metres)
    if goal is None:
        return None
    step_costs = _cost_per_metre(room, live_map.seen_free(), live_map.near_a_boulder())
    came_from = _search(allowed, step_costs, start, goal)
    if goal not in came_from:
        return None
    return [live_map.centre_of(square) for square in _path_back(came_from, goal)]


def _cost_per_metre(room, seen_free, near_a_boulder):
    """1 in open, seen space; more near obstacles, near boulders and in unseen squares."""
    tightness = np.clip((COMFORTABLE_ROOM_METRES - room) / (COMFORTABLE_ROOM_METRES - ROOM_NEEDED_METRES), 0.0, 1.0)
    cost = 1.0 + (TIGHT_ROOM_COST_FACTOR - 1.0) * tightness
    cost = np.where(near_a_boulder, cost * BOULDER_COST_FACTOR, cost)
    return np.where(seen_free, cost, cost * UNSEEN_COST_FACTOR)


def _search(allowed, step_costs, start, goal):
    """A* search over the squares. Returns {square: the square it was reached from}."""
    came_from = {start: None}
    best_cost = {start: 0.0}
    waiting = [(0.0, start)]
    while waiting:
        _, square = heapq.heappop(waiting)
        if square == goal:
            break
        for east_step, north_step in NEIGHBOUR_STEPS:
            neighbour = (square[0] + east_step, square[1] + north_step)
            if not (0 <= neighbour[0] < allowed.shape[0] and 0 <= neighbour[1] < allowed.shape[1]):
                continue
            if not allowed[neighbour]:
                continue
            step_length = math.hypot(east_step, north_step) * GRID_SQUARE_METRES
            cost = best_cost[square] + step_length * step_costs[neighbour]
            if cost < best_cost.get(neighbour, math.inf):
                best_cost[neighbour] = cost
                came_from[neighbour] = square
                guess_to_goal = math.hypot(goal[0] - neighbour[0], goal[1] - neighbour[1]) * GRID_SQUARE_METRES
                heapq.heappush(waiting, (cost + guess_to_goal, neighbour))
    return came_from


def _path_back(came_from, goal):
    squares = []
    square = goal
    while square is not None:
        squares.append(square)
        square = came_from[square]
    return squares[::-1]


def _nearest_allowed_square(live_map, allowed, goal_metres):
    goal = live_map.square_of(goal_metres)
    if goal is not None and allowed[goal]:
        return goal
    reach = int(math.ceil(LARGEST_GOAL_SHIFT_METRES / GRID_SQUARE_METRES))
    goal_index = np.floor((np.asarray(goal_metres[:2]) - live_map.lowest_corner_metres) / GRID_SQUARE_METRES).astype(int)
    best, best_distance = None, math.inf
    for east in range(goal_index[0] - reach, goal_index[0] + reach + 1):
        for north in range(goal_index[1] - reach, goal_index[1] + reach + 1):
            if not (0 <= east < allowed.shape[0] and 0 <= north < allowed.shape[1]) or not allowed[east, north]:
                continue
            distance = math.hypot(east - goal_index[0], north - goal_index[1])
            if distance < best_distance:
                best, best_distance = (east, north), distance
    if best is None or best_distance * GRID_SQUARE_METRES > LARGEST_GOAL_SHIFT_METRES:
        return None
    return best


def _segments_cross(a_start, a_end, b_start, b_end):
    def side(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    return (side(a_start, a_end, b_start) * side(a_start, a_end, b_end) < 0
            and side(b_start, b_end, a_start) * side(b_start, b_end, a_end) < 0)

