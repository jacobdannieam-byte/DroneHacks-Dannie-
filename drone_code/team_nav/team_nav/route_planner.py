"""Plans the drone's route from a course map and a mission.

This runs on the drone and in the simulator. The flight program calls plan_route once at
startup, and again whenever the drone changes its map because the cameras saw something the map
did not have. Nothing about the route is written into this code: it comes entirely from the map
file (where things are) and the mission file (which checkpoint lines to cross, in what order).

How it works
------------
  1. clearance_grid.py covers the course with squares 5 cm across and works out, for each
     square, how much room the drone would have there at its usual flying height.
  2. A search finds the cheapest way from the start to the landing pad that crosses the
     mission's checkpoint lines in order, each in the right direction. Picture one copy of the
     grid for each checkpoint still to clear: crossing the next line moves the search onto the
     next copy, and the pad only counts on the last copy. Each step costs its length, plus
     extra for squeezing close to walls, so the route keeps to the middle of corridors. (This is
     Dijkstra's shortest-path search.)
  3. The zigzag route from square to square is straightened into a few long straight legs.
     A straight leg is only accepted if it keeps about as much room as the zigzag it replaces.
  4. An opening such as the window is always crossed straight through its middle, at the
     opening's height (opening_crossings.py). Everywhere else the drone flies at the mission's
     cruise height.

"The whole drone crosses" a checkpoint line, so the planner aims to get the drone's centre at
least one drone radius past each line.

If a checkpoint can be cleared through more than one line (the window or Gate B), the planner
tries the lines in the mission's order of preference and uses the first that has a route.

What this file does not do: fly the route, or change the map. It only plans.
"""
import heapq
import itertools
import math
from dataclasses import dataclass

import numpy as np

from team_nav.clearance_grid import GRID_SQUARE_METRES, make_clearance_grid, room_along_line_metres
from team_nav.course_map import distance_to_shapes_metres
from team_nav.drone_dimensions import DRONE_RADIUS_METRES, REQUIRED_ROOM_METRES, heights_spanned_metres
from team_nav.mission import check_mission_fits_map
from team_nav.opening_crossings import OpeningCrossing, OpeningCrossingNotPossible, plan_opening_crossing

# Our own choice. Room of this much or more is comfortable. With less, the route pays a penalty.
COMFORTABLE_ROOM_METRES = 0.6
# Our own choice. Each metre of route costs 1. A metre flown with less than comfortable room
# costs extra, rising smoothly to CLOSENESS_COST_WEIGHT extra at the least room the drone may
# have. So the planner would rather fly 3 m further than fly 1 m squeezed past a wall, and it
# goes down the middle of any gap narrower than 1.2 m. With comfortable room there is no
# penalty, so in open space it takes the shortest way.
CLOSENESS_COST_WEIGHT = 3.0
# Our own choice: half a grid square. A straight leg may pass this much closer to a corner than
# the square centres it replaces, because the square centres were only measured every 5 cm.
STRAIGHTENING_TOLERANCE_METRES = 0.025

# The eight moves from a square to a neighbouring one: (squares east, squares north, length in squares).
NEIGHBOUR_MOVES = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)), (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
]


class RoutePlanningError(Exception):
    """No route was found. The message says why, in full sentences."""


class _NoRouteThroughTheseLines(Exception):
    """No route crosses this particular choice of lines. The planner may try another choice."""


@dataclass
class PlannedRoute:
    map_name: str
    waypoints_metres: np.ndarray        # N x 3: metres east, metres north, height above the floor.
    checkpoints_cleared: list           # (checkpoint name, line name, waypoint number where it is cleared)
    length_metres: float
    least_room_metres: float            # The closest the drone's centre comes to anything it could hit.
    least_room_near: str                # The name of that thing.

    def describe(self):
        lines = [
            f"Route on the map '{self.map_name}': {self.length_metres:.1f} m long, {len(self.waypoints_metres)} waypoints.",
            f"The least room is {self.least_room_metres:.2f} m, next to the {self.least_room_near} "
            f"(the drone needs at least {REQUIRED_ROOM_METRES:.2f} m).",
        ]
        for checkpoint_name, line_name, waypoint_number in self.checkpoints_cleared:
            lines.append(f"  {checkpoint_name}: cleared through '{line_name}' at waypoint {waypoint_number + 1}.")
        lines.append("Waypoints (metres east, metres north, height above the floor):")
        for number, waypoint in enumerate(self.waypoints_metres):
            lines.append(f"  {number + 1:2d}: ({waypoint[0]:+.2f}, {waypoint[1]:+.2f}, {waypoint[2]:.2f})")
        return "\n".join(lines)


@dataclass
class _FloorCrossing:
    """A checkpoint line on the floor, moved one drone radius towards its far side, so that a
    drone whose centre crosses it has its whole body across the real line."""
    line_name: str
    start_metres: np.ndarray
    end_metres: np.ndarray
    towards_far_side: np.ndarray        # A unit vector at right angles to the line.


def plan_route(course_map, mission, start_metres=None, checkpoints_already_cleared=0):
    """Plans a route from start_metres (the launch mark if not given) that clears the mission's
    remaining checkpoints in order and ends above the centre of the landing pad. Returns a
    PlannedRoute, or raises RoutePlanningError."""
    check_mission_fits_map(mission, course_map)
    if start_metres is None:
        start_metres = course_map.launch_mark_centre_metres
    start_metres = np.asarray(start_metres, dtype=float)[:2]
    cruise_grid = make_clearance_grid(course_map, *heights_spanned_metres(mission.cruise_height_metres))
    remaining_checkpoints = mission.checkpoints[checkpoints_already_cleared:]

    # Every way of choosing one line per checkpoint, most preferred first. For example, with
    # the window preferred to Gate B: first the route through the window, then through Gate B.
    choices_in_order = itertools.product(*[checkpoint.line_names_in_order_of_preference for checkpoint in remaining_checkpoints])
    reasons = []
    for line_names in choices_in_order:
        try:
            return _plan_through_lines(course_map, mission, cruise_grid, start_metres, remaining_checkpoints, line_names)
        except _NoRouteThroughTheseLines as reason:
            reasons.append(f"Through {', then '.join(line_names)}: {reason}")
    raise RoutePlanningError(f"Found no route on the map '{course_map.name}'. " + " ".join(reasons))


def _plan_through_lines(course_map, mission, cruise_grid, start_metres, checkpoints, line_names):
    stages = [_make_stage(course_map, line_name, mission.cruise_height_metres) for line_name in line_names]
    goal_metres = course_map.landing_pad_centre_metres
    square_path = _search_for_square_path(cruise_grid, stages, start_metres, goal_metres)
    waypoints_metres, cleared_at = _straighten_and_join(square_path, stages, cruise_grid, start_metres, goal_metres, mission.cruise_height_metres)
    least_room_metres, least_room_near = _least_room_on_route(waypoints_metres, stages, cleared_at, cruise_grid)

    checkpoints_cleared = []
    for checkpoint, line_name, waypoint_number in zip(checkpoints, line_names, cleared_at):
        checkpoints_cleared.append((checkpoint.name, line_name, waypoint_number))
    return PlannedRoute(
        map_name=course_map.name,
        waypoints_metres=waypoints_metres,
        checkpoints_cleared=checkpoints_cleared,
        length_metres=float(np.sum(np.linalg.norm(np.diff(waypoints_metres, axis=0), axis=1))),
        least_room_metres=least_room_metres,
        least_room_near=least_room_near,
    )


def _make_stage(course_map, line_name, cruise_height_metres):
    line = course_map.checkpoint_line_named(line_name)
    if line.opening_name is not None:
        try:
            return plan_opening_crossing(course_map, line, cruise_height_metres)
        except OpeningCrossingNotPossible as reason:
            raise _NoRouteThroughTheseLines(str(reason)) from reason

    along = (line.end_metres - line.start_metres) / np.linalg.norm(line.end_metres - line.start_metres)
    towards_far_side = np.array([-along[1], along[0]])
    if np.dot(line.far_side_point_metres - line.start_metres, towards_far_side) < 0:
        towards_far_side = -towards_far_side
    shift = towards_far_side * DRONE_RADIUS_METRES
    return _FloorCrossing(line.name, line.start_metres + shift, line.end_metres + shift, towards_far_side)


# ---- Step 2: searching the grid --------------------------------------------------------------

def _search_for_square_path(grid, stages, start_metres, goal_metres):
    """Dijkstra's search over one copy of the grid per stage, plus one for the last leg to the
    pad. Returns the path as a list of (copy number, square number)."""
    square_count = grid.squares_east * grid.squares_north
    is_free = (grid.room_metres >= REQUIRED_ROOM_METRES).ravel().tolist()
    cost_per_metre = _cost_per_metre(grid.room_metres).ravel().tolist()
    crossing_tests = [_crossing_test(grid, stage, is_free) for stage in stages]

    start_node = _free_square_number(grid, start_metres, is_free, "the start")
    goal_node = len(stages) * square_count + _free_square_number(grid, goal_metres, is_free, "the landing pad's centre")
    best_cost = {start_node: 0.0}
    came_from = {start_node: None}
    queue = [(0.0, start_node)]
    furthest_copy_reached = 0
    while queue:
        cost_so_far, node = heapq.heappop(queue)
        if cost_so_far > best_cost[node]:
            continue    # A cheaper way to this node was already found.
        if node == goal_node:
            return _path_back_to_start(came_from, node, square_count)
        copy_number, square = divmod(node, square_count)
        furthest_copy_reached = max(furthest_copy_reached, copy_number)
        for next_node, step_cost in _steps_from(copy_number, square, grid, is_free, cost_per_metre, crossing_tests):
            new_cost = cost_so_far + step_cost
            if new_cost < best_cost.get(next_node, math.inf):
                best_cost[next_node] = new_cost
                came_from[next_node] = node
                heapq.heappush(queue, (new_cost, next_node))

    if furthest_copy_reached < len(stages):
        raise _NoRouteThroughTheseLines(
            f"the route can't get across '{stages[furthest_copy_reached].line_name}' "
            f"(the search cleared {furthest_copy_reached} of {len(stages)} lines before running out of places to go)."
        )
    raise _NoRouteThroughTheseLines("every line can be crossed, but there is no way from the last line to the landing pad.")


def _cost_per_metre(room_metres):
    """1 with comfortable room, rising to 1 + CLOSENESS_COST_WEIGHT at the least room allowed."""
    shortfall = (COMFORTABLE_ROOM_METRES - room_metres) / (COMFORTABLE_ROOM_METRES - REQUIRED_ROOM_METRES)
    shortfall = np.clip(shortfall, 0.0, None)
    return 1.0 + CLOSENESS_COST_WEIGHT * shortfall ** 2


def _steps_from(copy_number, square, grid, is_free, cost_per_metre, crossing_tests):
    """Yields (next node, cost) for every step the drone can take from this square."""
    square_count = grid.squares_east * grid.squares_north
    east, north = divmod(square, grid.squares_north)
    test = crossing_tests[copy_number] if copy_number < len(crossing_tests) else None

    if isinstance(test, _OpeningJump) and square == test.line_up_square:
        yield (copy_number + 1) * square_count + test.exit_square, test.length_metres

    for squares_east, squares_north, length_in_squares in NEIGHBOUR_MOVES:
        next_east, next_north = east + squares_east, north + squares_north
        if not (0 <= next_east < grid.squares_east and 0 <= next_north < grid.squares_north):
            continue
        next_square = next_east * grid.squares_north + next_north
        if not is_free[next_square]:
            continue
        next_copy = copy_number
        if isinstance(test, _FloorLineTest) and test.crosses(square, next_square):
            next_copy = copy_number + 1
        step_cost = length_in_squares * GRID_SQUARE_METRES * (cost_per_metre[square] + cost_per_metre[next_square]) / 2.0
        yield next_copy * square_count + next_square, step_cost


@dataclass
class _FloorLineTest:
    """For each square: is its centre on the far side of the line, and how far along the line
    is it? A step crosses the line if it goes from the near side to the far side, level with
    the line rather than past one of its ends."""
    on_far_side: list
    distance_along_metres: list
    line_length_metres: float

    def crosses(self, square, next_square):
        if self.on_far_side[square] or not self.on_far_side[next_square]:
            return False
        middle_along = (self.distance_along_metres[square] + self.distance_along_metres[next_square]) / 2.0
        return 0.0 <= middle_along <= self.line_length_metres


@dataclass
class _OpeningJump:
    """The only way past an opening's line: from the square of its line-up point straight to
    the square of its exit point."""
    line_up_square: int
    exit_square: int
    length_metres: float


def _crossing_test(grid, stage, is_free):
    if isinstance(stage, OpeningCrossing):
        line_up = stage.line_up_point_metres[:2]
        exit_point = stage.exit_point_metres[:2]
        try:
            line_up_square = _free_square_number(grid, line_up, is_free, f"the point where the drone lines up with the {stage.opening_name}")
            exit_square = _free_square_number(grid, exit_point, is_free, f"the point where the drone has passed the {stage.opening_name}")
        except RoutePlanningError as reason:
            raise _NoRouteThroughTheseLines(str(reason)) from reason
        return _OpeningJump(line_up_square, exit_square, float(np.linalg.norm(exit_point - line_up)))

    centres = _all_square_centres(grid)
    along = (stage.end_metres - stage.start_metres) / np.linalg.norm(stage.end_metres - stage.start_metres)
    from_line_start = centres - stage.start_metres
    return _FloorLineTest(
        on_far_side=(from_line_start @ stage.towards_far_side > 0).tolist(),
        distance_along_metres=(from_line_start @ along).tolist(),
        line_length_metres=float(np.linalg.norm(stage.end_metres - stage.start_metres)),
    )


def _all_square_centres(grid):
    east_indices, north_indices = np.meshgrid(np.arange(grid.squares_east), np.arange(grid.squares_north), indexing="ij")
    square_numbers = np.stack([east_indices.ravel(), north_indices.ravel()], axis=1)
    return grid.lowest_corner_metres + (square_numbers + 0.5) * GRID_SQUARE_METRES


def _free_square_number(grid, point_metres, is_free, what_it_is):
    square = grid.square_containing(point_metres)
    if square is None:
        raise RoutePlanningError(f"{what_it_is.capitalize()} ({point_metres[0]:.2f}, {point_metres[1]:.2f}) is outside the course.")
    east, north = square
    if not is_free[east * grid.squares_north + north]:
        raise RoutePlanningError(
            f"{what_it_is.capitalize()} ({point_metres[0]:.2f}, {point_metres[1]:.2f}) has only "
            f"{grid.room_metres[east, north]:.2f} m of room, next to the {grid.nearest_shape_names[east, north]}, "
            f"but the drone needs {REQUIRED_ROOM_METRES:.2f} m."
        )
    return east * grid.squares_north + north


def _path_back_to_start(came_from, node, square_count):
    path = []
    while node is not None:
        path.append(divmod(node, square_count))
        node = came_from[node]
    path.reverse()
    return path


# ---- Step 3: straightening -------------------------------------------------------------------

def _straighten_and_join(square_path, stages, grid, start_metres, goal_metres, cruise_height_metres):
    """Turns the square path into straight legs. Returns the waypoints (N x 3) and, for each
    stage, the number of the waypoint where its line is cleared."""
    pieces = _split_into_pieces(square_path, stages, grid, start_metres, goal_metres)
    waypoints = []
    cleared_at = []
    for copy_number, piece_points in enumerate(pieces):
        line_not_to_cross = stages[copy_number] if copy_number < len(stages) else None
        if not isinstance(line_not_to_cross, _FloorCrossing):
            line_not_to_cross = None
        straight_points = _straighten(piece_points, grid.shapes, line_not_to_cross)
        if waypoints and np.allclose(straight_points[0], waypoints[-1][:2]):
            straight_points = straight_points[1:]   # The previous piece already ends at this point.
        for point in straight_points:
            waypoints.append(np.append(point, cruise_height_metres))
        if copy_number < len(stages):
            stage = stages[copy_number]
            if isinstance(stage, OpeningCrossing):
                waypoints.append(stage.level_start_metres)
                waypoints.append(stage.level_end_metres)
            cleared_at.append(len(waypoints) - 1)
    return np.array(waypoints), cleared_at


def _split_into_pieces(square_path, stages, grid, start_metres, goal_metres):
    """Splits the square path where it moves from one copy of the grid to the next. Each piece
    is a list of 2D points: it starts where the previous one ended and ends where its line is
    crossed (or, for an opening, at the line-up point)."""
    pieces = [[np.asarray(start_metres, dtype=float)]]
    for number, (copy_number, square) in enumerate(square_path):
        centre = grid.centre_of_square_metres(*divmod(square, grid.squares_north))
        if number > 0 and copy_number != square_path[number - 1][0]:
            stage = stages[copy_number - 1]
            if isinstance(stage, OpeningCrossing):
                pieces[-1].append(stage.line_up_point_metres[:2])
                pieces.append([stage.exit_point_metres[:2]])
                continue
            previous_centre = grid.centre_of_square_metres(*divmod(square_path[number - 1][1], grid.squares_north))
            crossing_point = _where_step_crosses(previous_centre, centre, stage)
            pieces[-1].append(crossing_point)
            pieces.append([crossing_point])
        pieces[-1].append(centre)
    pieces[-1].append(np.asarray(goal_metres, dtype=float))
    return pieces


def _where_step_crosses(from_point, to_point, stage):
    """The point where a short step crosses a floor checkpoint line."""
    distance_before = np.dot(from_point - stage.start_metres, stage.towards_far_side)
    distance_after = np.dot(to_point - stage.start_metres, stage.towards_far_side)
    fraction = distance_before / (distance_before - distance_after)
    return from_point + (to_point - from_point) * fraction


def _straighten(points, shapes, line_not_to_cross):
    """Keeps the first and last points and as few in between as possible. From each kept point,
    it reaches as far along the path as a straight leg can go without losing room."""
    room_at_points, _ = distance_to_shapes_metres(np.array(points), shapes)
    kept = [points[0]]
    current = 0
    last = len(points) - 1
    while current < last:
        reach = current + 1
        while reach < last and _straight_leg_is_acceptable(points, room_at_points, current, reach + 1, shapes, line_not_to_cross):
            reach += 1
        kept.append(points[reach])
        current = reach
    return kept


def _straight_leg_is_acceptable(points, room_at_points, first, last, shapes, line_not_to_cross):
    room_on_zigzag = float(np.min(room_at_points[first:last + 1]))
    room_needed = max(REQUIRED_ROOM_METRES, min(room_on_zigzag, COMFORTABLE_ROOM_METRES) - STRAIGHTENING_TOLERANCE_METRES)
    if line_not_to_cross is not None and _legs_cross(points[first], points[last], line_not_to_cross.start_metres, line_not_to_cross.end_metres):
        return False
    room_on_leg, _ = room_along_line_metres(shapes, points[first], points[last])
    return room_on_leg >= room_needed


def _legs_cross(first_start, first_end, second_start, second_end):
    """True if two straight pieces cross each other. Touching at an end does not count."""
    return (
        _on_opposite_sides(first_start, first_end, second_start, second_end)
        and _on_opposite_sides(second_start, second_end, first_start, first_end)
    )


def _on_opposite_sides(point, other_point, line_start, line_end):
    """True if the two points are on opposite sides of the line. A point on the line, or within
    a millionth of a millimetre of it, is on neither side."""
    def side(this_point):
        value = (line_end[0] - line_start[0]) * (this_point[1] - line_start[1]) - (line_end[1] - line_start[1]) * (this_point[0] - line_start[0])
        if abs(value) < 1e-9:
            return 0
        return 1 if value > 0 else -1
    return side(point) * side(other_point) < 0


# ---- Checking the finished route -------------------------------------------------------------

def _least_room_on_route(waypoints_metres, stages, cleared_at, grid):
    """The least room on any leg. Legs through an opening were already measured with the
    opening counted as open, so their room comes from the crossing itself."""
    legs_through_openings = set()
    least_room_metres = math.inf
    least_room_near = ""
    for stage, waypoint_number in zip(stages, cleared_at):
        if isinstance(stage, OpeningCrossing):
            line_up_number = waypoint_number - 2
            legs_through_openings.update([line_up_number, line_up_number + 1, line_up_number + 2])
            if stage.least_room_metres < least_room_metres:
                least_room_metres, least_room_near = stage.least_room_metres, stage.opening_name

    for leg in range(len(waypoints_metres) - 1):
        if leg in legs_through_openings:
            continue
        room_metres, near = room_along_line_metres(grid.shapes, waypoints_metres[leg][:2], waypoints_metres[leg + 1][:2])
        if room_metres < least_room_metres:
            least_room_metres, least_room_near = room_metres, near
    return least_room_metres, least_room_near
