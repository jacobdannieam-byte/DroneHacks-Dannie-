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
     extra for squeezing close to walls, so the route keeps to the middle of corridors, and a
     lot extra for flying near a boulder (see "Boulders" below). (This is Dijkstra's
     shortest-path search.)
  3. The zigzag route from square to square is straightened into a few long straight legs.
     A straight leg is only accepted if it keeps about as much room as the zigzag it replaces,
     and doesn't take the laser closer to a boulder.
  4. An opening such as the window is always crossed straight through its middle, at the
     opening's height (opening_crossings.py). Everywhere else the drone flies at the mission's
     cruise height.

"The whole drone crosses" a checkpoint line, so the planner aims to get the drone's centre at
least one drone radius past each line.

If a checkpoint can be cleared through more than one line (the window or Gate B), the planner
tries the lines in the mission's order of preference and uses the first that has a route.

Boulders
--------
The drone flies over boulders, but a boulder under the bottom laser makes the drone's height
reading suddenly drop, and PX4 may then stop trusting the laser (docs/ARCHITECTURE.md, section 7).
So the planner tries to keep the drone's centre, where the laser is, away from every boulder in
the map, whatever its height. This is a strong preference, not a ban: if the only way through
passes near a boulder, the route still goes there, and PlannedRoute.legs_near_boulders lists
those legs, so the flight program can keep moving briskly over them instead of stopping.

What this file does not do: fly the route, or change the map. It only plans.
"""
import heapq
import itertools
import math
from dataclasses import dataclass

import numpy as np

from team_nav.boulder_distances import closest_approach_to_each_boulder_metres, distance_to_each_boulder_metres
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
# Our own choice. The planner tries to keep the drone's centre at least this far from the edge
# of every boulder. It allows for the boulder being a little away from where the map says, and
# for the laser beam's spread, which we don't know yet (somewhere between 1 and 27 degrees wide).
LASER_KEEP_OFF_BOULDER_METRES = 0.25
# Our own choice. Each metre flown closer to a boulder than that costs this much extra, so the
# planner would rather fly about 10 m further than fly 1 m near a boulder. It costs up to twice
# as much again the deeper inside the keep-off distance it is, so if the route has to pass a
# boulder, it passes as far from the boulder as it can.
BOULDER_COST_PER_METRE = 10.0

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
    legs_near_boulders: list            # (number of the leg's first waypoint, boulder name, closest distance to its edge in metres; negative means over it)

    def describe(self):
        lines = [
            f"Route on the map '{self.map_name}': {self.length_metres:.1f} m long, {len(self.waypoints_metres)} waypoints.",
            f"The least room is {self.least_room_metres:.2f} m, next to the {self.least_room_near} "
            f"(the drone needs at least {REQUIRED_ROOM_METRES:.2f} m).",
        ]
        for checkpoint_name, line_name, waypoint_number in self.checkpoints_cleared:
            lines.append(f"  {checkpoint_name}: cleared through '{line_name}' at waypoint {waypoint_number + 1}.")
        for leg_start_number, boulder_name, distance_metres in self.legs_near_boulders:
            lines.append(
                f"  Between waypoints {leg_start_number + 1} and {leg_start_number + 2}, the drone's centre passes "
                f"{distance_metres:.2f} m from the edge of {boulder_name}. Keep moving there."
            )
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
    legs_near_boulders = _legs_near_boulders(waypoints_metres, cruise_grid)

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
        legs_near_boulders=legs_near_boulders,
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
    cost_per_metre = _cost_per_metre(grid.room_metres, grid.boulder_distance_metres).ravel().tolist()
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


def _cost_per_metre(room_metres, boulder_distance_metres):
    """1 with comfortable room, rising to 1 + CLOSENESS_COST_WEIGHT at the least room allowed,
    plus the cost of flying near a boulder."""
    shortfall = (COMFORTABLE_ROOM_METRES - room_metres) / (COMFORTABLE_ROOM_METRES - REQUIRED_ROOM_METRES)
    shortfall = np.clip(shortfall, 0.0, None)
    return 1.0 + CLOSENESS_COST_WEIGHT * shortfall ** 2 + _boulder_cost_per_metre(boulder_distance_metres)


def _boulder_cost_per_metre(boulder_distance_metres):
    """0 outside the keep-off distance. Inside it, BOULDER_COST_PER_METRE, plus up to twice that
    again: BOULDER_COST_PER_METRE more at the boulder's edge, and about twice more above its centre.
    (The distance is negative above a boulder.)"""
    near_a_boulder = boulder_distance_metres < LASER_KEEP_OFF_BOULDER_METRES
    depth_inside_keep_off = (LASER_KEEP_OFF_BOULDER_METRES - boulder_distance_metres) / LASER_KEEP_OFF_BOULDER_METRES
    depth_inside_keep_off = np.clip(depth_inside_keep_off, 0.0, 2.0)
    return BOULDER_COST_PER_METRE * (near_a_boulder + depth_inside_keep_off)


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
        straight_points = _straighten(piece_points, grid, line_not_to_cross)
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


def _straighten(points, grid, line_not_to_cross):
    """Keeps the first and last points and as few in between as possible. From each kept point,
    it reaches as far along the path as a straight leg can go without losing room or taking
    the laser closer to a boulder."""
    zigzag = _Zigzag(
        points=points,
        room_metres=distance_to_shapes_metres(np.array(points), grid.shapes)[0],
        distance_to_each_boulder_metres=distance_to_each_boulder_metres(np.array(points), grid.boulders),
    )
    kept = [points[0]]
    current = 0
    last = len(points) - 1
    while current < last:
        reach = current + 1
        while reach < last and _straight_leg_is_acceptable(zigzag, current, reach + 1, grid, line_not_to_cross):
            reach += 1
        kept.append(points[reach])
        current = reach
    return kept


@dataclass
class _Zigzag:
    """The path from square centre to square centre, with the room at each of its points and
    the distance from each point to each boulder."""
    points: list
    room_metres: np.ndarray
    distance_to_each_boulder_metres: np.ndarray   # number of points x number of boulders


def _straight_leg_is_acceptable(zigzag, first, last, grid, line_not_to_cross):
    """Can one straight leg replace the zigzag from point number first to point number last?"""
    start, end = zigzag.points[first], zigzag.points[last]
    if line_not_to_cross is not None and _legs_cross(start, end, line_not_to_cross.start_metres, line_not_to_cross.end_metres):
        return False

    room_on_zigzag = float(np.min(zigzag.room_metres[first:last + 1]))
    room_needed = max(REQUIRED_ROOM_METRES, min(room_on_zigzag, COMFORTABLE_ROOM_METRES) - STRAIGHTENING_TOLERANCE_METRES)
    room_on_leg, _ = room_along_line_metres(grid.shapes, start, end)
    if room_on_leg < room_needed:
        return False

    # Each boulder is checked on its own, so that passing close to one boulder doesn't let the
    # leg cut close to another.
    approaches_on_leg = closest_approach_to_each_boulder_metres(grid.boulders, start, end)
    for boulder_number, (_, distance_on_leg_metres) in enumerate(approaches_on_leg):
        distance_on_zigzag_metres = float(np.min(zigzag.distance_to_each_boulder_metres[first:last + 1, boulder_number]))
        if distance_on_leg_metres < _boulder_distance_needed_on_leg(distance_on_zigzag_metres):
            return False
    return True


def _boulder_distance_needed_on_leg(distance_on_zigzag_metres):
    """If the zigzag kept outside a boulder's keep-off distance, a straight leg replacing it must
    too. If the zigzag had to come closer, the leg may come no closer than it did."""
    if distance_on_zigzag_metres >= LASER_KEEP_OFF_BOULDER_METRES:
        return LASER_KEEP_OFF_BOULDER_METRES
    return distance_on_zigzag_metres - STRAIGHTENING_TOLERANCE_METRES


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

def _legs_near_boulders(waypoints_metres, grid):
    """Every leg on which the drone's centre comes closer to a boulder's edge than
    LASER_KEEP_OFF_BOULDER_METRES, as (number of its first waypoint, boulder name, distance).
    A leg near two boulders is listed twice."""
    legs = []
    for leg in range(len(waypoints_metres) - 1):
        approaches = closest_approach_to_each_boulder_metres(grid.boulders, waypoints_metres[leg], waypoints_metres[leg + 1])
        for boulder_name, distance_metres in approaches:
            if distance_metres < LASER_KEEP_OFF_BOULDER_METRES:
                legs.append((leg, boulder_name, distance_metres))
    return legs


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
