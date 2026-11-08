"""Quick checks of live navigation's pieces: stereo depth, the live map, the path search, the
navigator's rules and the heading sums. Each test runs in well under a second, with no ROS and
no drone.

Run:  python3 -m pytest test/test_live_navigation.py -q
"""
import math

import numpy as np
import pytest

from our_files import load_our_map, load_side_gap_first_mission, SCAN_MAP
from team_nav import stereo_depth
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES
from team_nav.flight_steps import heading_from_rotation, rotation_matrix
from team_nav.live_map import LiveMap
from team_nav.live_navigation import FLYING, HOLDING, TURNING, LiveNavigator, NoWayForward
from team_nav.live_planner import Goal, find_path, goals_from_mission
from team_nav.start_frame import StartFrame

# The mission's cruise height on our maps.
FLYING_HEIGHT_METRES = 0.8


def scan_live_map():
    return LiveMap(load_our_map(SCAN_MAP), FLYING_HEIGHT_METRES, DRONE_HALF_HEIGHT_METRES)


def open_point(live_map):
    """A point on the course with lots of room around it, from the map itself."""
    room = live_map.room_metres()
    square = np.unravel_index(int(np.argmax(room)), room.shape)
    return live_map.centre_of(square)


# ---- Stereo depth -------------------------------------------------------------------------------

def test_stereo_finds_a_known_shift_with_numpy():
    random = np.random.default_rng(1)
    right = random.uniform(0, 255, (96, 160)).astype(np.float32)
    shift_pixels = 12
    left = np.zeros_like(right)
    left[:, shift_pixels:] = right[:, :-shift_pixels]
    disparity = stereo_depth.disparity_pixels(left, right, use_gpu=False)
    middle = disparity[10:-10, 40:-10]
    assert np.isfinite(middle).mean() > 0.8
    assert abs(np.nanmedian(middle) - shift_pixels) < 0.2


def test_points_ahead_of_camera_put_the_picture_centre_straight_ahead():
    distances = np.full((9, 9), 2.0)
    points = stereo_depth.points_ahead_of_camera(distances, 100.0, 4.0, 4.0, every_nth_pixel=1)
    centre = points[4 * 9 + 4]
    assert np.allclose(centre, [2.0, 0.0, 0.0])
    # The top-left pixel is to the left of and above the centre.
    assert points[0][1] > 0 and points[0][2] > 0


# ---- The live map ---------------------------------------------------------------------------------

def test_a_wall_seen_twice_becomes_blocked_and_the_space_before_it_free():
    live_map = scan_live_map()
    camera = open_point(live_map)
    camera_3d = np.array([camera[0], camera[1], FLYING_HEIGHT_METRES])
    wall = np.array([[camera[0] + 0.6, camera[1] + offset, FLYING_HEIGHT_METRES] for offset in np.linspace(-0.2, 0.2, 9)])
    for _picture in range(2):
        live_map.add_points(camera_3d, wall)
    assert live_map.blocked()[live_map.square_of(wall[4])]
    # Two views through the space before the wall: its score has gone down, toward "free".
    assert live_map.scores[live_map.square_of(camera + np.array([0.3, 0.0]))] < 0


def test_floor_points_are_not_walls():
    live_map = scan_live_map()
    camera = open_point(live_map)
    floor = np.array([[camera[0] + 0.6, camera[1], 0.0]] * 5)
    for _picture in range(5):
        live_map.add_points(np.array([camera[0], camera[1], FLYING_HEIGHT_METRES]), floor)
    assert not live_map.blocked()[live_map.square_of(floor[0])]


def test_a_map_wall_seen_through_becomes_free():
    live_map = scan_live_map()
    blocked_squares = np.argwhere(live_map.blocked() & ~live_map.always_blocked)
    wall_square = tuple(blocked_squares[len(blocked_squares) // 2])
    wall_point = live_map.centre_of(wall_square)
    camera = np.array([wall_point[0] - 0.05, wall_point[1], FLYING_HEIGHT_METRES])
    beyond = np.array([[wall_point[0] + 1.0, wall_point[1], FLYING_HEIGHT_METRES]])
    for _picture in range(5):
        live_map.add_points(camera, beyond)
    assert not live_map.blocked()[wall_square]


# ---- The path search -----------------------------------------------------------------------------

def test_a_path_exists_from_the_launch_mark_to_the_first_goal_on_the_scan_map():
    course_map = load_our_map(SCAN_MAP)
    live_map = LiveMap(course_map, FLYING_HEIGHT_METRES, DRONE_HALF_HEIGHT_METRES)
    goals = goals_from_mission(course_map, load_side_gap_first_mission(), "to the landing pad")
    path = find_path(live_map, course_map.launch_mark_centre_metres, goals[0].point_metres)
    assert path is not None and len(path) > 5
    assert goals[-1].stage_name == "to the landing pad"


def test_the_path_goes_around_a_newly_seen_wall():
    live_map = scan_live_map()
    start = open_point(live_map)
    goal = start + np.array([1.0, 0.0])
    straight = find_path(live_map, start, goal)
    wall = np.array([[start[0] + 0.5, start[1] + offset, FLYING_HEIGHT_METRES] for offset in np.linspace(-0.3, 0.3, 25)])
    for _picture in range(3):
        live_map.add_points(np.array([start[0], start[1], FLYING_HEIGHT_METRES]), wall)
    around = find_path(live_map, start, goal)
    assert straight is not None and around is not None
    farthest_sideways = max(abs(point[1] - start[1]) for point in around)
    assert farthest_sideways > 0.3


# ---- The navigator's rules -----------------------------------------------------------------------

def navigator_with_one_goal(live_map, goal_point):
    goal = Goal("test", np.asarray(goal_point, dtype=float), None, None)
    return LiveNavigator(live_map, [goal], FLYING_HEIGHT_METRES)


def test_the_navigator_turns_first_when_the_path_is_behind_it():
    live_map = scan_live_map()
    start = open_point(live_map)
    navigator = navigator_with_one_goal(live_map, start + np.array([-1.0, 0.0]))
    step = navigator.next_step([start[0], start[1], FLYING_HEIGHT_METRES], 0.0, 0.0)
    assert step.activity == TURNING
    assert abs(abs(step.heading_radians) - math.pi) < 0.3
    assert np.linalg.norm(step.target_metres[:2] - start) < 0.01


def look_ahead_at_far_wall(live_map, start, metres_ahead=2.0):
    """The camera sees a patterned wall straight ahead (east), which clears the space before it."""
    wall = np.array([[start[0] + metres_ahead, start[1] + offset, FLYING_HEIGHT_METRES]
                     for offset in np.linspace(-1.0, 1.0, 80)])
    live_map.add_points(np.array([start[0], start[1], FLYING_HEIGHT_METRES]), wall)


def test_the_navigator_flies_forward_when_facing_space_it_has_seen():
    live_map = scan_live_map()
    start = open_point(live_map)
    look_ahead_at_far_wall(live_map, start)
    navigator = navigator_with_one_goal(live_map, start + np.array([1.0, 0.0]))
    step = navigator.next_step([start[0], start[1], FLYING_HEIGHT_METRES], 0.0, 0.0)
    assert step.activity == FLYING
    assert step.target_metres[0] > start[0] + 0.1


def test_the_navigator_does_not_move_into_space_the_camera_has_not_seen():
    # As in front of a plain wall with no pattern: the camera gives no distances at all.
    live_map = scan_live_map()
    start = open_point(live_map)
    navigator = navigator_with_one_goal(live_map, start + np.array([1.0, 0.0]))
    step = navigator.next_step([start[0], start[1], FLYING_HEIGHT_METRES], 0.0, 0.0)
    assert step.activity == HOLDING
    assert np.linalg.norm(step.target_metres[:2] - start) < 0.01


def test_a_drone_that_does_not_turn_as_asked_stops_the_run():
    live_map = scan_live_map()
    start = open_point(live_map)
    navigator = navigator_with_one_goal(live_map, start + np.array([-1.0, 0.0]))
    try:
        for cycle in range(400):
            navigator.next_step([start[0], start[1], FLYING_HEIGHT_METRES], 0.0, cycle * 0.1)
    except NoWayForward as stop:
        assert "turn" in str(stop)
        return
    raise AssertionError("The navigator kept asking for a turn that never happened.")


def test_the_navigator_does_not_move_into_a_wall_just_ahead():
    live_map = scan_live_map()
    start = open_point(live_map)
    navigator = navigator_with_one_goal(live_map, start + np.array([1.5, 0.0]))
    look_ahead_at_far_wall(live_map, start)
    wall = np.array([[start[0] + 0.45, start[1] + offset, FLYING_HEIGHT_METRES] for offset in np.linspace(-1.0, 1.0, 60)])
    for _picture in range(3):
        live_map.add_points(np.array([start[0], start[1], FLYING_HEIGHT_METRES]), wall)
    step = navigator.next_step([start[0], start[1], FLYING_HEIGHT_METRES], 0.0, 0.0)
    moved_toward_wall = step.target_metres[0] - start[0]
    assert moved_toward_wall < 0.05


def test_after_a_slide_the_drone_waits_to_get_back():
    # On drone 16 a turn made the drone slide 0.5 m from its hold point (2026-11-08).
    live_map = scan_live_map()
    start = open_point(live_map)
    navigator = navigator_with_one_goal(live_map, start + np.array([1.0, 0.0]))
    navigator.hold_point = np.array([start[0], start[1], FLYING_HEIGHT_METRES])
    slid = np.array([start[0] - 0.3, start[1], FLYING_HEIGHT_METRES])
    navigator.now_seconds = 0.0
    assert not navigator._settled(slid)
    navigator.now_seconds = 4.0
    assert not navigator._settled(slid)
    navigator.now_seconds = 5.5
    with pytest.raises(NoWayForward, match="could not return to its hold point"):
        navigator._settled(slid)
    assert navigator._settled(np.array([start[0] + 0.05, start[1], FLYING_HEIGHT_METRES]))


# ---- Heading sums ---------------------------------------------------------------------------------

def test_heading_from_a_quarter_turn_quaternion():
    quarter_left = (0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4))
    assert abs(heading_from_rotation(quarter_left) - math.pi / 2) < 1e-9
    # After a quarter turn left, the drone's forward is the starting left.
    assert np.allclose(rotation_matrix(quarter_left) @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])


def test_start_frame_headings_go_there_and_back():
    frame = StartFrame([0.0, 0.0], 90.0)
    assert abs(frame.course_heading_from_start_frame(0.0) - math.pi / 2) < 1e-9
    assert abs(frame.start_frame_heading_from_course(frame.course_heading_from_start_frame(0.7)) - 0.7) < 1e-9


def test_when_map_walls_block_every_path_the_planner_plans_through_unseen_map_walls():
    live_map = scan_live_map()
    start = open_point(live_map)
    # Pretend the map file has a wall all round the drone, which the camera has never seen.
    ring = np.zeros(live_map.shape, dtype=bool)
    centre = live_map.square_of(start)
    ring[centre[0] - 12:centre[0] + 13, centre[1] - 12:centre[1] + 13] = True
    ring[centre[0] - 10:centre[0] + 11, centre[1] - 10:centre[1] + 11] = False
    live_map.scores[ring] = 1.5
    goal = start + np.array([1.2, 0.0])
    assert find_path(live_map, start, goal) is None
    navigator = navigator_with_one_goal(live_map, goal)
    navigator.next_step([start[0], start[1], FLYING_HEIGHT_METRES], 0.0, 0.0)
    assert navigator.path is not None


def test_the_path_goes_round_a_boulder_the_camera_sees():
    live_map = scan_live_map()
    start = open_point(live_map)
    goal = start + np.array([1.2, 0.0])
    boulder_top = np.array([[start[0] + 0.6 + dx, start[1] + dy, 0.45]
                            for dx in np.linspace(-0.15, 0.15, 7) for dy in np.linspace(-0.15, 0.15, 7)])
    for _picture in range(3):
        live_map.add_points(np.array([start[0], start[1], FLYING_HEIGHT_METRES]), boulder_top)
    path = np.array(find_path(live_map, start, goal))
    boulder_centre = start + np.array([0.6, 0.0])
    assert np.min(np.linalg.norm(path - boulder_centre, axis=1)) > 0.25


# ---- Drift and height corrections -----------------------------------------------------------------

def test_wall_alignment_finds_a_known_drift_across_the_long_barrier():
    from team_nav.wall_alignment import WallAlignment
    course_map = load_our_map(SCAN_MAP)
    alignment = WallAlignment(course_map)
    barrier = [wall for wall in course_map.walls if wall.name == "Long barrier"][0]
    # Points on the barrier's north face, seen 8 cm too far north (camera tracking has drifted).
    face_north = barrier.start_metres[1] + barrier.thickness_metres / 2.0
    points = np.array([[east, face_north + 0.08] for east in np.linspace(-2.5, 0.5, 200)])
    for _picture in range(60):
        shifted = points + np.r_[alignment.correction_metres]
        for east in np.linspace(-2.5, 0.5, 4):
            # The camera 1 m north of the barrier, moving along it.
            alignment.add_points([east, face_north + 1.0], shifted)
    assert abs(alignment.correction_metres[1] + 0.08) < 0.02
    # Points along one wall say nothing about the direction along it.
    assert abs(alignment.correction_metres[0]) < 0.02


def test_wall_alignment_ignores_walls_from_the_sketch():
    from team_nav.wall_alignment import WallAlignment
    course_map = load_our_map("../course_maps/sketch_2026-11-08")
    alignment = WallAlignment(course_map)
    top_wall = [wall for wall in course_map.walls if wall.name.startswith("Sketch top wall")][0]
    points = np.array([[east, top_wall.start_metres[1] + 0.1] for east in np.linspace(-2.5, -1.2, 200)])
    for _picture in range(20):
        alignment.add_points([-1.8, top_wall.start_metres[1] - 0.8], points)
    assert np.allclose(alignment.correction_metres, 0.0)


def test_height_keeper_asks_for_more_height_when_the_drone_has_sunk():
    from team_nav.height_keeper import LARGEST_CORRECTION_METRES, HeightKeeper
    keeper = HeightKeeper()
    for _cycle in range(10):
        keeper.update(0.8, 0.7, 0.1)
    assert 0.05 < keeper.correction_metres < 0.15
    for _cycle in range(200):
        keeper.update(0.8, 0.2, 0.1)
    assert keeper.correction_metres == LARGEST_CORRECTION_METRES
