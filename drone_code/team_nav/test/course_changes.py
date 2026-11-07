"""Makes changed copies of a course map, for testing the route planner on a changed course.

The real course may change after our scan, and the drone's cameras may find things the map
does not have. Each function here returns a new map with one such change. The original map is
never changed. Each new map's name says what was changed, because the planner's error
messages quote it.

The changes are worked out from each map's own shapes, not from fixed coordinates, so the same
change can be made to the organizers' drawing and to our scan.
"""
import copy

import numpy as np

from team_nav.course_map import LowObstacle, Wall

# Our own choices, for the pretend shapes the tests add.
ADDED_WALL_THICKNESS_METRES = 0.10
ADDED_PILLAR_RADIUS_METRES = 0.15
# Tall enough to block the drone at any height it flies at, and still below the walls' tops.
TALL_HEIGHT_METRES = 1.7
# The pillar behind the window stands this far beyond the window's middle, which is in the
# path of a drone flying straight through.
PILLAR_DISTANCE_BEYOND_WINDOW_METRES = 0.6
# The window's new height in with_window_too_small: less than the drone needs.
TOO_SMALL_OPENING_HEIGHT_METRES = 0.30
# Walls added across the first corridor start this far along M1, from its south end. That is
# far enough north of the launch mark to leave the drone room to take off.
FRACTION_ALONG_FIRST_DIVIDER = 0.75

WINDOW_LINE = "through the window"
GATE_B_LINE = "Gate B"
FIRST_MAZE_DIVIDER = "M1 maze divider"


def copy_with_new_name(course_map, what_changed):
    changed_map = copy.deepcopy(course_map)
    changed_map.name = f"{course_map.name}, {what_changed}"
    return changed_map


def with_window_too_small(course_map):
    """The window is only TOO_SMALL_OPENING_HEIGHT_METRES tall, as if it were half covered."""
    changed_map = copy_with_new_name(course_map, "with the window too small")
    _, window = changed_map.opening_named(changed_map.checkpoint_line_named(WINDOW_LINE).opening_name)
    window.top_height_metres = window.bottom_height_metres + TOO_SMALL_OPENING_HEIGHT_METRES
    return changed_map


def with_pillar_behind_window(course_map):
    """A tall pillar just beyond the window, in the way of a drone flying straight through."""
    changed_map = copy_with_new_name(course_map, "with a pillar behind the window")
    window_middle_metres, towards_far_side = _window_middle_and_far_side(changed_map)
    pillar_centre_metres = window_middle_metres + towards_far_side * PILLAR_DISTANCE_BEYOND_WINDOW_METRES
    changed_map.low_obstacles.append(LowObstacle(
        "pillar behind the window", pillar_centre_metres, ADDED_PILLAR_RADIUS_METRES, TALL_HEIGHT_METRES, "Added by a test.",
    ))
    return changed_map


def with_gate_b_closed(course_map):
    """A wall along the whole of the Gate B line, so the drone can't fly through Gate B."""
    line = course_map.checkpoint_line_named(GATE_B_LINE)
    return with_added_wall(course_map, "wall closing Gate B", line.start_metres, line.end_metres)


def with_tunnel_blocked(course_map):
    """A wall across the middle of the tunnel, from one side to the other."""
    tunnel = course_map.tubes[0]
    middle_metres = (tunnel.entrance_centre_metres + tunnel.exit_centre_metres) / 2.0
    along = (tunnel.exit_centre_metres - tunnel.entrance_centre_metres) / tunnel.length_metres()
    to_the_side = np.array([-along[1], along[0]]) * tunnel.outside_width_metres / 2.0
    return with_added_wall(course_map, "wall blocking the tunnel", middle_metres - to_the_side, middle_metres + to_the_side)


def with_first_corridor_narrowed(course_map, gap_left_metres):
    """A wall reaching east from the west net across the first corridor, leaving a gap of
    gap_left_metres between its end and M1's west face."""
    point_on_divider_metres = _point_along_first_divider(course_map)
    divider = course_map.wall_named(FIRST_MAZE_DIVIDER)
    divider_west_face_x_metres = point_on_divider_metres[0] - divider.thickness_metres / 2.0
    # A wall's end is rounded, so it reaches half its thickness beyond its end point.
    end_x_metres = divider_west_face_x_metres - gap_left_metres - ADDED_WALL_THICKNESS_METRES / 2.0
    west_net_point_metres = np.array([course_map.lowest_corner_metres[0], point_on_divider_metres[1]])
    return with_added_wall(
        course_map, f"first corridor narrowed to {gap_left_metres:.2f} m",
        west_net_point_metres, np.array([end_x_metres, point_on_divider_metres[1]]),
    )


def with_first_corridor_closed(course_map):
    """A wall right across the first corridor, from the west net to M1."""
    point_on_divider_metres = _point_along_first_divider(course_map)
    west_net_point_metres = np.array([course_map.lowest_corner_metres[0], point_on_divider_metres[1]])
    return with_added_wall(course_map, "wall closing the first corridor", west_net_point_metres, point_on_divider_metres)


def with_added_wall(course_map, wall_name, start_metres, end_metres):
    changed_map = copy_with_new_name(course_map, f"with a {wall_name}")
    changed_map.walls.append(Wall(
        name=wall_name,
        start_metres=np.asarray(start_metres, dtype=float),
        end_metres=np.asarray(end_metres, dtype=float),
        thickness_metres=ADDED_WALL_THICKNESS_METRES,
        bottom_height_metres=0.0,
        top_height_metres=TALL_HEIGHT_METRES,
        openings=[],
        source="Added by a test.",
    ))
    return changed_map


def with_low_obstacle(course_map, obstacle_name, centre_metres, radius_metres, top_height_metres):
    changed_map = copy_with_new_name(course_map, f"with a {obstacle_name}")
    changed_map.low_obstacles.append(LowObstacle(
        obstacle_name, np.asarray(centre_metres, dtype=float), radius_metres, top_height_metres, "Added by a test.",
    ))
    return changed_map


def with_wall_moved(course_map, wall_name, shift_metres):
    changed_map = copy_with_new_name(course_map, f"with the {wall_name} moved by {shift_metres}")
    wall = changed_map.wall_named(wall_name)
    wall.start_metres = wall.start_metres + np.asarray(shift_metres, dtype=float)
    wall.end_metres = wall.end_metres + np.asarray(shift_metres, dtype=float)
    return changed_map


def without_wall(course_map, wall_name):
    changed_map = copy_with_new_name(course_map, f"without the {wall_name}")
    changed_map.walls.remove(changed_map.wall_named(wall_name))
    return changed_map


def with_wall_from_other_map(course_map, other_map, wall_name):
    """Swaps one wall for the wall of the same name in another map."""
    changed_map = copy_with_new_name(course_map, f"with the {wall_name} from '{other_map.name}'")
    position = changed_map.walls.index(changed_map.wall_named(wall_name))
    changed_map.walls[position] = copy.deepcopy(other_map.wall_named(wall_name))
    return changed_map


def with_low_obstacle_moved(course_map, obstacle_name, new_centre_metres):
    changed_map = copy_with_new_name(course_map, f"with {obstacle_name} moved")
    for obstacle in changed_map.low_obstacles:
        if obstacle.name == obstacle_name:
            obstacle.centre_metres = np.asarray(new_centre_metres, dtype=float)
            obstacle.source = "Moved by a test."
    return changed_map


def without_low_obstacle(course_map, obstacle_name):
    changed_map = copy_with_new_name(course_map, f"without {obstacle_name}")
    changed_map.low_obstacles = [obstacle for obstacle in changed_map.low_obstacles if obstacle.name != obstacle_name]
    return changed_map


def point_halfway_from_tunnel_to_window(route):
    """Where to put something in the open area after the tunnel, in the way of a planned route:
    halfway along the route from where it clears the tunnel's exit to where it lines up with
    the window."""
    _, _, tunnel_cleared_at = route.checkpoints_cleared[-2]
    _, _, window_cleared_at = route.checkpoints_cleared[-1]
    line_up_waypoint_number = window_cleared_at - 2   # The two waypoints after it are the level part.
    return _point_halfway_along_route(route, tunnel_cleared_at, line_up_waypoint_number)


def point_halfway_from_window_exit_to_landing_pad(route):
    """For a route through the window: halfway along the route from where the drone finishes
    flying straight out of the window to the end, above the landing pad. The landing area there
    is open, with room to fly round something."""
    _, _, window_cleared_at = route.checkpoints_cleared[-1]
    exit_waypoint_number = window_cleared_at + 1   # The drone flies straight on to here after the window.
    return _point_halfway_along_route(route, exit_waypoint_number, len(route.waypoints_metres) - 1)


def point_halfway_up_the_first_corridor(route):
    """Halfway along the route from the launch mark to where it clears the end of the first
    corridor. The route keeps to the middle of the corridor there."""
    _, _, corridor_cleared_at = route.checkpoints_cleared[0]
    return _point_halfway_along_route(route, 0, corridor_cleared_at)


def _point_halfway_along_route(route, first_waypoint_number, last_waypoint_number):
    """The point halfway along the route between two of its waypoints, seen from above."""
    waypoints = route.waypoints_metres[first_waypoint_number:last_waypoint_number + 1, :2]
    leg_lengths_metres = np.linalg.norm(np.diff(waypoints, axis=0), axis=1)
    distance_left_metres = float(np.sum(leg_lengths_metres)) / 2.0
    for leg_number, leg_length_metres in enumerate(leg_lengths_metres):
        if distance_left_metres <= leg_length_metres:
            fraction_along_leg = distance_left_metres / leg_length_metres
            return waypoints[leg_number] + (waypoints[leg_number + 1] - waypoints[leg_number]) * fraction_along_leg
        distance_left_metres -= leg_length_metres
    return waypoints[-1]


def _point_along_first_divider(course_map):
    """The point FRACTION_ALONG_FIRST_DIVIDER of the way along M1, from its south end."""
    divider = course_map.wall_named(FIRST_MAZE_DIVIDER)
    south_end, north_end = divider.start_metres, divider.end_metres
    if south_end[1] > north_end[1]:
        south_end, north_end = north_end, south_end
    return south_end + (north_end - south_end) * FRACTION_ALONG_FIRST_DIVIDER


def _window_middle_and_far_side(course_map):
    """Returns (the window's middle seen from above, a unit vector from there through the
    wall towards the side the drone must end up on)."""
    line = course_map.checkpoint_line_named(WINDOW_LINE)
    wall, window = course_map.opening_named(line.opening_name)
    middle_metres = wall.point_along_wall_metres((window.start_along_wall_metres + window.end_along_wall_metres) / 2.0)
    towards_far_side = np.array([-wall.direction()[1], wall.direction()[0]])
    if np.dot(line.far_side_point_metres - middle_metres, towards_far_side) < 0:
        towards_far_side = -towards_far_side
    return middle_metres, towards_far_side
