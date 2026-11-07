"""Checks a planned route again, without trusting the planner's own sums. Used by the tests.

The planner reports how much room its route leaves and where it clears each checkpoint. A test
that only read those numbers would still pass if the planner worked them out wrongly. So these
checks measure the route again, from the map, in the simplest way we could think of: step along
every leg in 1 cm steps, and at each step look at everything the drone could hit at that
step's height.

check_route raises AssertionError, with a sentence saying what is wrong, if the route:
  - does not start at the start or end above the centre of the landing pad;
  - flies so low or so high that the drone's body would touch the floor or rise above the walls;
  - comes closer to anything than the drone's radius plus its safety margin;
  - does not get the whole drone past each checkpoint line, in the mission's order;
  - brings the drone's centre near a boulder on a leg it doesn't list in legs_near_boulders.
    The flight program reads that list to know where it must keep moving.
"""
import numpy as np

from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.drone_dimensions import DRONE_RADIUS_METRES, REQUIRED_ROOM_METRES, heights_spanned_metres
from team_nav.route_planner import LASER_KEEP_OFF_BOULDER_METRES

# Our own choice: the step along each leg when checking it, a fifth of the planner's grid square.
CHECK_STEP_METRES = 0.01
# Our own choice. The planner checks each leg at points 2.5 cm apart, so between two of its
# points a leg can pass a few millimetres closer to a corner than it measured.
MEASURING_TOLERANCE_METRES = 0.005
# Our own choice. How close the first and last waypoints must be to the start and the pad.
POSITION_TOLERANCE_METRES = 0.001


def check_route(route, course_map, mission, start_metres, checkpoints_already_cleared=0):
    points = points_along_route(route.waypoints_metres)
    _check_start_and_end(route, course_map, start_metres)
    _check_heights(points, course_map)
    _check_room(points, course_map)
    _check_checkpoints(route, points, course_map, mission, checkpoints_already_cleared)
    _check_legs_near_boulders_are_listed(route, course_map)


def points_along_route(waypoints_metres):
    """Points at most CHECK_STEP_METRES apart along the whole route, including every waypoint.
    Each point is (metres east, metres north, height)."""
    points = [waypoints_metres[0]]
    for leg_start, leg_end in zip(waypoints_metres[:-1], waypoints_metres[1:]):
        step_count = max(1, int(np.ceil(np.linalg.norm(leg_end - leg_start) / CHECK_STEP_METRES)))
        for step in range(1, step_count + 1):
            points.append(leg_start + (leg_end - leg_start) * step / step_count)
    return np.array(points)


def least_room_along_route(course_map, points):
    """Returns (the least room in metres, the name of the nearest thing there, that point)."""
    least_room_metres = np.inf
    nearest_name = ""
    worst_point = None
    shapes_at_height = {}
    for point in points:
        height_metres = float(point[2])
        if height_metres not in shapes_at_height:
            shapes_at_height[height_metres] = shapes_blocking_between_heights(course_map, *heights_spanned_metres(height_metres))
        room_metres, names = distance_to_shapes_metres(point[np.newaxis, :2], shapes_at_height[height_metres])
        if room_metres[0] < least_room_metres:
            least_room_metres, nearest_name, worst_point = float(room_metres[0]), names[0], point
    return least_room_metres, nearest_name, worst_point


def closest_approach_to_boulder_metres(waypoints_metres, boulder):
    """The closest the drone's centre comes to the boulder's edge, seen from above, anywhere
    along these waypoints. Negative if it passes right over the boulder."""
    points = points_along_route(waypoints_metres)
    distances_from_centre_metres = np.linalg.norm(points[:, :2] - boulder.centre_metres, axis=1)
    return float(np.min(distances_from_centre_metres)) - boulder.radius_metres


def first_point_with_whole_drone_past(line, points):
    """The number of the first point at which the whole drone is past the checkpoint line: its
    centre is at least one drone radius beyond the line, and level with the line rather than
    beyond one of its ends. None if that never happens."""
    line_length_metres = float(np.linalg.norm(line.end_metres - line.start_metres))
    along = (line.end_metres - line.start_metres) / line_length_metres
    towards_far_side = np.array([-along[1], along[0]])
    if np.dot(line.far_side_point_metres - line.start_metres, towards_far_side) < 0:
        towards_far_side = -towards_far_side

    for number, point in enumerate(points):
        from_line_start = point[:2] - line.start_metres
        distance_past_metres = float(from_line_start @ towards_far_side)
        distance_along_metres = float(from_line_start @ along)
        level_with_line = 0.0 <= distance_along_metres <= line_length_metres
        if level_with_line and distance_past_metres >= DRONE_RADIUS_METRES - POSITION_TOLERANCE_METRES:
            return number
    return None


def _check_start_and_end(route, course_map, start_metres):
    first, last = route.waypoints_metres[0], route.waypoints_metres[-1]
    assert np.linalg.norm(first[:2] - start_metres) < POSITION_TOLERANCE_METRES, (
        f"The route starts at ({first[0]:.2f}, {first[1]:.2f}), not at the start ({start_metres[0]:.2f}, {start_metres[1]:.2f})."
    )
    pad = course_map.landing_pad_centre_metres
    assert np.linalg.norm(last[:2] - pad) < POSITION_TOLERANCE_METRES, (
        f"The route ends at ({last[0]:.2f}, {last[1]:.2f}), not above the landing pad's centre ({pad[0]:.2f}, {pad[1]:.2f})."
    )


def _check_heights(points, course_map):
    lowest_metres, _ = heights_spanned_metres(float(np.min(points[:, 2])))
    _, highest_metres = heights_spanned_metres(float(np.max(points[:, 2])))
    assert lowest_metres > 0.0, f"The route flies so low that the drone's body reaches down to {lowest_metres:.2f} m."
    assert highest_metres < course_map.wall_top_height_metres, (
        f"The route flies so high that the drone's body reaches {highest_metres:.2f} m, above the walls "
        f"({course_map.wall_top_height_metres:.2f} m)."
    )


def _check_room(points, course_map):
    least_room_metres, nearest_name, worst_point = least_room_along_route(course_map, points)
    assert least_room_metres >= REQUIRED_ROOM_METRES - MEASURING_TOLERANCE_METRES, (
        f"At ({worst_point[0]:.2f}, {worst_point[1]:.2f}, height {worst_point[2]:.2f}) the route passes "
        f"{least_room_metres:.2f} m from the {nearest_name}, but the drone needs {REQUIRED_ROOM_METRES:.2f} m."
    )


def _check_checkpoints(route, points, course_map, mission, checkpoints_already_cleared):
    remaining_checkpoints = mission.checkpoints[checkpoints_already_cleared:]
    cleared_names = [checkpoint_name for checkpoint_name, _, _ in route.checkpoints_cleared]
    expected_names = [checkpoint.name for checkpoint in remaining_checkpoints]
    assert cleared_names == expected_names, f"The route clears {cleared_names}, but the mission still needs {expected_names}."

    previous_point_number = -1
    for checkpoint_name, line_name, _ in route.checkpoints_cleared:
        line = course_map.checkpoint_line_named(line_name)
        point_number = first_point_with_whole_drone_past(line, points[previous_point_number + 1:])
        assert point_number is not None, (
            f"The route never gets the whole drone past '{line_name}' for the checkpoint '{checkpoint_name}' "
            "after clearing the checkpoint before it."
        )
        previous_point_number += 1 + point_number


def _check_legs_near_boulders_are_listed(route, course_map):
    listed = set()
    for leg_number, boulder_name, _ in route.legs_near_boulders:
        listed.add((leg_number, boulder_name))

    for leg_number in range(len(route.waypoints_metres) - 1):
        leg = route.waypoints_metres[leg_number:leg_number + 2]
        for boulder in course_map.low_obstacles:
            closest_metres = closest_approach_to_boulder_metres(leg, boulder)
            if closest_metres >= LASER_KEEP_OFF_BOULDER_METRES - MEASURING_TOLERANCE_METRES:
                continue
            assert (leg_number, boulder.name) in listed, (
                f"Between waypoints {leg_number + 1} and {leg_number + 2}, the drone's centre passes "
                f"{closest_metres:.2f} m from the edge of {boulder.name}, but the route doesn't list that leg "
                "as near a boulder, so the flight program wouldn't know to keep moving there."
            )
