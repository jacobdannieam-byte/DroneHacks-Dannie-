"""Tests route_follower.py and start_frame.py with a pretend drone. No simulator is needed.

The pretend drone does roughly what PX4 does with a target position: it flies straight at the
target, faster the further away the target is. It is far simpler than the real thing, so these
tests only check the follower's own rules, and how the follower and the window crossing's shape
work together. The simulator flights (simulator/test_flights/route_test.py) check it with PX4
flying.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from team_nav.drone_dimensions import DRONE_RADIUS_METRES
from team_nav.route_follower import CORNER_HEIGHT_TOLERANCE_METRES, CORNER_TOLERANCE_METRES, RouteFollower
from team_nav.route_planner import plan_route
from team_nav.start_frame import StartFrame

from our_files import SCAN_MAP, load_our_map, load_rulebook_mission

# Measured in a simulator flight on 2026-11-07: PX4 flew the drone sideways at about 0.77 m/s
# for each metre between it and its target.
SIDEWAYS_SPEED_PER_METRE_OF_DISTANCE = 0.77
# From drone 16's PX4 setting MPC_Z_P (simulator/px4_changes/drone_16_settings.sh): the same for
# up and down.
UPWARD_SPEED_PER_METRE_OF_DISTANCE = 1.0
# The flight program sends a new target 10 times per second.
SECONDS_PER_STEP = 0.1
# Our own choice: long enough for our whole course, at 0.5 m/s with a pause at every corner.
LONGEST_FLIGHT_SECONDS = 240.0
# Our own choice: rounding allowance for the pretend drone's small steps.
TOLERANCE_METRES = 0.01
# Our own choice. In the window on our scan, the drone's body has about 17 cm to spare above and
# below. Dipping by no more than this keeps about two thirds of that.
LARGEST_DIP_IN_THE_WINDOW_METRES = 0.06

# A route with a sharp corner, a gentle one, a slight bend the drone flies straight past, and a
# climb, in metres east, north and up.
TEST_ROUTE_METRES = np.array([
    [0.0, 0.0, 0.8],
    [2.0, 0.0, 0.8],
    [2.0, 1.5, 0.8],
    [3.0, 1.8, 1.2],
    [4.5, 2.5, 1.2],
])


def fly_pretend_drone(waypoints_metres):
    """Flies the pretend drone along the route. Returns its positions, one per step."""
    follower = RouteFollower(waypoints_metres)
    drone_metres = np.array(waypoints_metres[0], dtype=float)
    positions = [drone_metres.copy()]
    for _ in range(int(LONGEST_FLIGHT_SECONDS / SECONDS_PER_STEP)):
        if follower.has_finished(drone_metres):
            return np.array(positions)
        target_metres = follower.next_target(drone_metres, SECONDS_PER_STEP)
        speed_per_metre = np.array([
            SIDEWAYS_SPEED_PER_METRE_OF_DISTANCE, SIDEWAYS_SPEED_PER_METRE_OF_DISTANCE, UPWARD_SPEED_PER_METRE_OF_DISTANCE,
        ])
        drone_metres = drone_metres + speed_per_metre * (target_metres - drone_metres) * SECONDS_PER_STEP
        positions.append(drone_metres.copy())
    raise AssertionError(f"The pretend drone didn't finish the route in {LONGEST_FLIGHT_SECONDS:.0f} seconds.")


def sideways_distance_from_route_metres(point_metres, waypoints_metres):
    """The distance from a point to the nearest point on the route's legs, seen from above."""
    point_metres = point_metres[:2]
    nearest_metres = np.inf
    for leg_start, leg_end in zip(waypoints_metres[:-1, :2], waypoints_metres[1:, :2]):
        leg = leg_end - leg_start
        fraction_along = np.clip((point_metres - leg_start) @ leg / (leg @ leg), 0.0, 1.0)
        nearest_metres = min(nearest_metres, float(np.linalg.norm(point_metres - (leg_start + fraction_along * leg))))
    return nearest_metres


def test_pretend_drone_reaches_the_end_of_the_route():
    positions = fly_pretend_drone(TEST_ROUTE_METRES)

    assert np.linalg.norm(positions[-1][:2] - TEST_ROUTE_METRES[-1][:2]) < CORNER_TOLERANCE_METRES
    assert abs(positions[-1][2] - TEST_ROUTE_METRES[-1][2]) < CORNER_HEIGHT_TOLERANCE_METRES


def test_pretend_drone_never_strays_sideways_from_the_planned_line_by_more_than_the_corner_tolerance():
    # Sideways is what matters for walls. Height has its own test, at the window, below.
    positions = fly_pretend_drone(TEST_ROUTE_METRES)

    largest_stray_metres = max(sideways_distance_from_route_metres(position, TEST_ROUTE_METRES) for position in positions)
    assert largest_stray_metres < CORNER_TOLERANCE_METRES + TOLERANCE_METRES


def test_pretend_drone_flies_faster_than_04_metres_per_second_along_a_long_straight_leg():
    # Section 7 of docs/ARCHITECTURE.md: after the laser crosses an edge, PX4 needs the drone moving
    # sideways faster than about 0.32 m/s for a few seconds, so we aim well above that.
    long_straight_route = np.array([[0.0, 0.0, 0.8], [6.0, 0.0, 0.8]])
    positions = fly_pretend_drone(long_straight_route)

    # Speeds in the middle third of the leg, away from the start and the end.
    east_metres = positions[:, 0]
    in_middle = (east_metres > 2.0) & (east_metres < 4.0)
    speeds = np.linalg.norm(np.diff(positions, axis=0), axis=1)[in_middle[1:]] / SECONDS_PER_STEP
    assert np.min(speeds) > 0.45


def test_pretend_drone_keeps_its_speed_past_a_waypoint_in_a_straight_line():
    # Like the window crossing: waypoints in a straight line, with a climb and a level part.
    window_like_route = np.array([
        [0.0, 0.0, 0.8], [2.0, 0.0, 0.8], [2.6, 0.0, 1.2], [3.4, 0.0, 1.2], [4.0, 0.0, 0.8], [6.0, 0.0, 0.8],
    ])
    positions = fly_pretend_drone(window_like_route)

    east_metres = positions[:, 0]
    around_the_middle_waypoints = (east_metres > 2.4) & (east_metres < 3.6)
    speeds = np.linalg.norm(np.diff(positions[:, :2], axis=0), axis=1)[around_the_middle_waypoints[1:]] / SECONDS_PER_STEP
    assert np.min(speeds) > 0.4


def test_pretend_drone_holds_the_window_height_while_any_part_of_it_is_over_the_wall():
    course_map = load_our_map(SCAN_MAP)
    route = plan_route(course_map, load_rulebook_mission())
    window_wall, window = _wall_with_the_window(course_map)

    positions = fly_pretend_drone(route.waypoints_metres)

    heights_over_the_wall = [
        position[2] for position in positions if _drone_overlaps_the_window_wall(position, window_wall, window)
    ]
    assert len(heights_over_the_wall) > 0, "The pretend drone never crossed the window's wall."
    largest_dip_metres = window.centre_height_metres() - min(heights_over_the_wall)
    assert largest_dip_metres < LARGEST_DIP_IN_THE_WINDOW_METRES


@pytest.mark.parametrize("start_heading_degrees", [0.0, 90.0, -35.0])
def test_start_frame_converts_there_and_back(start_heading_degrees):
    frame = StartFrame(launch_mark_centre_metres=(-2.5, 0.9), start_heading_degrees=start_heading_degrees)
    course_point_metres = np.array([1.3, -2.2, 0.8])

    back_again = frame.course_from_start_frame(frame.start_frame_from_course(course_point_metres))

    assert np.allclose(back_again, course_point_metres)


def _wall_with_the_window(course_map):
    for wall in course_map.walls:
        for opening in wall.openings:
            if opening.name == "window":
                return wall, opening
    raise AssertionError("The scan map has no opening called 'window'.")


def _drone_overlaps_the_window_wall(position_metres, wall, window):
    """True if, seen from above, any part of the drone is over the wall, inside the window's
    stretch of it."""
    from_wall_start = position_metres[:2] - wall.start_metres
    along_wall_metres = float(from_wall_start @ wall.direction())
    across_wall_metres = abs(float(from_wall_start[0] * wall.direction()[1] - from_wall_start[1] * wall.direction()[0]))
    within_window_stretch = window.start_along_wall_metres <= along_wall_metres <= window.end_along_wall_metres
    return within_window_stretch and across_wall_metres < DRONE_RADIUS_METRES + wall.thickness_metres / 2.0


def test_start_frame_facing_north_puts_forward_north_and_left_west():
    frame = StartFrame(launch_mark_centre_metres=(-2.5, 0.9), start_heading_degrees=90.0)

    one_metre_forward = frame.course_from_start_frame((1.0, 0.0, 0.8))
    one_metre_left = frame.course_from_start_frame((0.0, 1.0, 0.8))

    assert np.allclose(one_metre_forward, [-2.5, 1.9, 0.8])
    assert np.allclose(one_metre_left, [-3.5, 0.9, 0.8])
