"""Tests for the tunnel_first mission: the slow turn, the heading guard, the start check, the
frames a map target goes through, and the route on our maps.

They run on a computer without ROS or PX4: plain arithmetic, a pretend link where one is needed,
and the organizers' own frame conversion (frames.py, which imports nothing)."""
import importlib.util
import math
import os
import types

import numpy as np
import pytest

from our_files import DRAWING_MAP, SCAN_MAP, load_our_map
from team_nav.flight_steps import DroneLink, SafetyWatch
from team_nav.heading_turn import (GUARD_HOLD, GUARD_LAND, GUARD_OK, LARGEST_LAG_DEGREES, STEP_DEGREES, TURN_DEGREES,
                                   HeadingGuard, TurnRamp, TurnTracker, odom_heading_from_px4_heading, wrapped_degrees)
from team_nav.start_frame import StartFrame
from team_nav.tunnel_first_route import GO_LINE_EAST_METRES, S_METRES, plan_tunnel_first_route
from team_nav.tunnel_first_run import start_offset_problem

ORGANIZERS_FRAMES_FILE = os.path.join(os.path.dirname(__file__), "..", "..", "..", "simulator", "organizers_code",
                                      "skybeta_flight_interface", "skybeta_flight_interface", "frames.py")


def organizers_frames():
    spec = importlib.util.spec_from_file_location("organizers_frames", ORGANIZERS_FRAMES_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# ---- The turn ramp ------------------------------------------------------------------------------

def fly_a_perfect_turn(start_heading_degrees):
    """The ramp's commands when PX4 follows each command exactly, as sent (wrapped) headings."""
    ramp = TurnRamp(start_heading_degrees)
    sent = []
    turned = 0.0
    for _ in range(1000):
        command = ramp.next_command_degrees(turned, 0.0)
        sent.append(wrapped_degrees(command))
        turned = command - start_heading_degrees
        if ramp.command_has_reached_the_end():
            break
    return ramp, sent


def test_the_turn_ramp_steps_half_a_degree_at_a_time_always_to_the_left_across_180_degrees():
    for start in (0.0, 90.0, 170.0, -179.0):
        ramp, sent = fly_a_perfect_turn(start)
        steps = [wrapped_degrees(after - before) for before, after in zip([wrapped_degrees(start)] + sent, sent)]
        assert all(0.0 < step <= STEP_DEGREES + 1e-9 for step in steps)   # Counter-clockwise, never more than 0.5.
        assert sum(steps) == pytest.approx(TURN_DEGREES)
        assert len(sent) == int(TURN_DEGREES / STEP_DEGREES)
        assert wrapped_degrees(sent[-1] - (start + 180.0)) == pytest.approx(0.0, abs=1e-9)


def test_the_turn_ramp_pauses_while_px4_lags_and_never_catches_up_with_a_step():
    ramp = TurnRamp(0.0)
    for _ in range(20):
        ramp.next_command_degrees(ramp.commanded_turn_degrees, 0.0)       # PX4 keeps up: 20 steps, 10 degrees.
    assert ramp.commanded_turn_degrees == pytest.approx(10.0)
    stuck_at = 10.0 - LARGEST_LAG_DEGREES - 0.1                           # PX4 falls more than 5 degrees behind.
    for _ in range(50):
        ramp.next_command_degrees(stuck_at, 0.0)
    assert ramp.commanded_turn_degrees == pytest.approx(10.0)
    assert "lags" in ramp.paused_because
    ramp.next_command_degrees(9.0, 0.0)                                   # PX4 catches up: one step only.
    assert ramp.commanded_turn_degrees == pytest.approx(10.0 + STEP_DEGREES)


def test_the_turn_ramp_pauses_while_the_heading_split_has_drifted():
    ramp = TurnRamp(45.0)
    ramp.next_command_degrees(0.0, 5.1)
    ramp.next_command_degrees(0.0, -5.1)
    assert ramp.commanded_turn_degrees == 0.0 and "split" in ramp.paused_because
    ramp.next_command_degrees(0.0, 4.9)
    assert ramp.commanded_turn_degrees == pytest.approx(STEP_DEGREES)


def test_the_turn_tracker_counts_past_180_degrees():
    tracker = TurnTracker(170.0)
    for heading in (175.0, -178.0, -170.0, -10.0):
        turned = tracker.update(heading)
    assert turned == pytest.approx(180.0)


def test_px4_heading_is_written_the_way_flight_interface_reads_it():
    frames = organizers_frames()
    for odom_heading in (0.0, 0.7, math.pi / 2, -2.5):
        px4_heading = frames.yaw_enu_to_ned(odom_heading)
        assert odom_heading_from_px4_heading(px4_heading) == pytest.approx(odom_heading)


# ---- The heading guard and the start check -----------------------------------------------------

def test_the_heading_guard_holds_after_half_a_second_and_lands_after_three_more():
    guard = HeadingGuard(launch_split_degrees=20.0)                       # A constant offset is normal.
    assert guard.check(0.0, 25.0) == GUARD_OK
    assert guard.check(0.0, 31.0) == GUARD_OK
    assert guard.check(0.4, 31.0) == GUARD_OK
    assert guard.check(0.6, 31.0) == GUARD_HOLD
    assert guard.check(3.4, 31.0) == GUARD_HOLD
    assert guard.check(3.6, 31.0) == GUARD_LAND
    assert guard.check(3.7, 21.0) == GUARD_OK                             # Back within 10: the timer restarts.
    assert guard.check(3.8, None) == GUARD_OK


def test_the_start_is_refused_more_than_0_3_m_sideways_or_0_1_m_in_height_from_the_launch_mark():
    assert start_offset_problem(np.array([0.2, 0.2, 0.05])) is None
    assert "restart the drone stack" in start_offset_problem(np.array([0.25, 0.2, 0.0]))
    assert start_offset_problem(np.array([0.0, 0.0, 0.11])) is not None
    assert start_offset_problem(np.array([1.0, 0.0, 0.0])) is not None


# ---- A fixed map target in the organizers' frame -----------------------------------------------

class PretendLinkForFrames:
    """Just enough of DroneLink for SafetyWatch.send_target_on_course and send_position_from_launch."""

    def __init__(self, heading_degrees):
        half = math.radians(heading_degrees) / 2.0
        self.tracked_rotation = np.array([0.0, 0.0, math.sin(half), math.cos(half)])
        self.launch = types.SimpleNamespace(tracked_position_metres=np.array([0.04, -0.03, 0.01]), px4_z_metres=-0.15)
        self.height_correction_metres = 0.0
        self.sent = []
        self.targets = types.SimpleNamespace(send_position=lambda target, heading: self.sent.append((np.array(target), heading)))

    def position_from_launch(self):
        return np.array([0.3, 0.1, 2.0])

    def send_position_from_launch(self, forward_left_up_metres, heading_radians=None):
        DroneLink.send_position_from_launch(self, forward_left_up_metres, heading_radians)


def test_a_fixed_map_target_reaches_px4_unchanged_whatever_the_drones_heading_now():
    frames = organizers_frames()
    course_map = load_our_map(SCAN_MAP)
    start_frame = StartFrame(course_map.launch_mark_centre_metres, 90.0)
    run_safety = types.SimpleNamespace(check_target=lambda target, drone: None)
    map_target = np.array([S_METRES[0], S_METRES[1], 2.4])
    in_px4_frame = []
    for heading_degrees in (0.0, 90.0, 180.0):
        link = PretendLinkForFrames(heading_degrees)
        SafetyWatch(link, run_safety, start_frame).send_target_on_course(map_target, check_distance_from_drone=False)
        odom_target, _ = link.sent[-1]
        in_px4_frame.append(np.array(frames.setpoint_position_enu_to_ned(tuple(odom_target))))
    assert np.allclose(in_px4_frame[0], in_px4_frame[1]) and np.allclose(in_px4_frame[0], in_px4_frame[2])


# ---- The route on our maps ----------------------------------------------------------------------

@pytest.mark.parametrize("map_name", [DRAWING_MAP, SCAN_MAP])
def test_the_route_reaches_s_and_flies_the_go_line_with_room_on_our_maps(map_name):
    route = plan_tunnel_first_route(load_our_map(map_name))
    assert np.allclose(route.high_leg_metres[-1], S_METRES)
    assert route.go_end_metres[0] == GO_LINE_EAST_METRES
    assert all(text.startswith("The path turns with ") for text in route.high_leg_breaks)
    go_note = [note for note in route.notes if note.startswith("GO line")][0]
    clearance_metres = float(go_note.split("clearance ")[1].split(" m")[0])
    assert clearance_metres >= 0.20


def test_the_height_holds_correction_counts_as_deliberate_in_the_camera_height_comparison(monkeypatch):
    from team_nav.run_safety import DroneCondition
    from team_nav.tunnel_first_run import TunnelFirstSafetyWatch
    monkeypatch.setattr(SafetyWatch, "_gather_condition",
                        lambda self: DroneCondition(now_seconds=0.0, control_is_ours=True, is_armed=True,
                                                    camera_tracking_age_seconds=0.0, camera_tracking_says_lost=False,
                                                    position_on_course_metres=None, tilt_degrees=0.0,
                                                    rotation_degrees_per_second=0.0, px4_height_risen_metres=1.2,
                                                    laser_height_risen_metres=None, raw_laser_available=False,
                                                    px4_uses_laser_for_height=True, px4_says_laser_is_faulty=False))
    watch = TunnelFirstSafetyWatch.__new__(TunnelFirstSafetyWatch)
    watch.height_hold = None
    assert watch._gather_condition().height_correction_metres == 0.0
    watch.height_hold = types.SimpleNamespace(correction_metres=0.14)
    assert watch._gather_condition().height_correction_metres == pytest.approx(0.14)


# ---- Phase 2: the LOOK step's signs, through the real code ----------------------------------------

def picture_with_opening(left_column, right_column, height=320, width=512):
    """A rim (bright, with some texture) around a dark opening between two columns."""
    random = np.random.default_rng(7)
    picture = 180.0 + 20.0 * random.random((height, width))
    picture[:, left_column:right_column] = 30.0 + 10.0 * random.random((height, right_column - left_column))
    return picture


def test_an_opening_in_the_left_part_of_the_picture_gives_a_left_offset_that_goes_east_with_the_nose_south():
    from team_nav.tunnel_look import TUNNEL_INSIDE_WIDTH_METRES, body_left_on_course, find_opening_edges
    fx, cx = 317.0, 256.0
    look = find_opening_edges(picture_with_opening(60, 360), fx, cx)
    assert look.found_both()
    assert abs(look.left_column - 60) <= 2 and abs(look.right_column - 360) <= 2
    assert look.left_offset_metres > 0.0
    expected = -TUNNEL_INSIDE_WIDTH_METRES * ((60 + 360) / 2.0 - cx) / 300.0
    assert look.left_offset_metres == pytest.approx(expected, abs=0.02)
    assert look.distance_metres == pytest.approx(fx * TUNNEL_INSIDE_WIDTH_METRES / 300.0, rel=0.02)
    # The nose faces south after the turn: the course heading the turn locks is -90 degrees.
    start_frame = StartFrame(load_our_map(SCAN_MAP).launch_mark_centre_metres, 90.0)
    nose_south = start_frame.course_heading_from_start_frame(math.radians(180.0))
    correction = look.left_offset_metres * body_left_on_course(nose_south)
    assert correction[0] > 0.0 and abs(correction[1]) < 1e-9


def test_an_edge_cut_off_by_the_border_is_reported_on_its_side():
    from team_nav.tunnel_look import find_opening_edges
    look = find_opening_edges(picture_with_opening(0, 300), 317.0, 256.0)
    assert not look.found_both() and look.clipped_left and not look.clipped_right


def test_the_depth_method_agrees_with_a_tunnel_0_12_m_to_the_left():
    from team_nav.tunnel_look import TUNNEL_INSIDE_WIDTH_METRES, depth_left_offset_metres
    half = TUNNEL_INSIDE_WIDTH_METRES / 2.0
    forward = np.linspace(1.1, 2.4, 60)
    heights = np.linspace(-0.25, 0.25, 5)
    left_wall = [(f, 0.12 + half, h) for f in forward for h in heights]
    right_wall = [(f, 0.12 - half, h) for f in forward for h in heights]
    far_off = [(f, 0.0, 1.0) for f in forward]                                  # Above the band: ignored.
    offset, left, right = depth_left_offset_metres(np.array(left_wall + right_wall + far_off), 1.0)
    assert offset == pytest.approx(0.12, abs=1e-6)


def test_five_steady_frames_are_accepted_and_jitter_is_not():
    from team_nav.tunnel_look import EdgeLook, judge_frames, _measure
    def look(left, right):
        found = EdgeLook(left_column=left, right_column=right)
        _measure(found, 317.0, 256.0)
        return found
    steady = [look(100, 400 + number % 2) for number in range(5)]
    assert judge_frames(steady, [0.1 * number for number in range(5)]).accepted
    jittery = [look(100 + 4 * number, 400 + 4 * number) for number in range(5)]
    assert not judge_frames(jittery, [0.1 * number for number in range(5)]).accepted


def points_at_columns(columns_and_distances, fx=317.0, cx=256.0):
    """Depth points (forward, left, up) from the camera, a few at each picture column's direction."""
    points = []
    for column, distance in columns_and_distances:
        for up in (-0.1, 0.0, 0.1):
            points.append((distance, -(column - cx) * distance / fx, up))
    return np.array(points)


def test_an_edge_pair_that_the_depth_points_put_at_another_distance_is_rejected():
    # Gazebo, 2026-11-08: the far exit's left edge (2.35 m) and the tunnel's outside (4 m) taken as a
    # 316 px pair, which the width puts 1.48 m away.
    from team_nav.tunnel_look import EdgeLook, edges_match_depth, _measure
    look = EdgeLook(left_column=117, right_column=433)
    _measure(look, 317.0, 256.0)
    wrong = points_at_columns([(117, 2.35), (433, 4.0)])
    assert edges_match_depth(look, wrong, 317.0, 256.0)[0] is False
    right = points_at_columns([(117, look.distance_metres), (433, look.distance_metres * 1.05)])
    assert edges_match_depth(look, right, 317.0, 256.0)[0] is True
    assert edges_match_depth(look, None, 317.0, 256.0)[0] is False


def test_with_depth_points_the_widest_pair_at_the_right_distance_is_the_tunnel_mouth():
    # As in Gazebo: the mouth's rim (outer, about 1.06 m away) around the far exit (inner, 2.35 m away).
    from team_nav.tunnel_look import find_opening_edges
    fx, cx = 317.0, 256.0
    picture = picture_with_opening(60, 500)
    picture[:, 190:370] = 120.0 + 10.0 * np.random.default_rng(3).random((320, 180))
    rim_distance = fx * 1.47 / 440.0
    points = points_at_columns([(column, rim_distance) for column in range(55, 66)]
                               + [(column, rim_distance) for column in range(495, 506)]
                               + [(column, 2.35) for column in range(185, 375, 2)])
    look = find_opening_edges(picture, fx, cx, points_from_camera=points, check_depth=True)
    true_offset = -1.47 * ((60 + 500) / 2.0 - cx) / 440.0
    assert look.found_both() and look.left_offset_metres == pytest.approx(true_offset, abs=0.03)
    assert look.distance_metres == pytest.approx(rim_distance, rel=0.05)


@pytest.mark.parametrize("map_name", [DRAWING_MAP, SCAN_MAP])
def test_the_look_backs_0_26_m_north_of_s_and_keeps_0_62_m_from_the_net(map_name, monkeypatch):
    from team_nav.tunnel_first_route import TunnelFirstRouteError
    monkeypatch.delenv("TEAM_NAV_LOOK_BACKUP_METRES", raising=False)
    monkeypatch.delenv("TEAM_NAV_S_NORTH_METRES", raising=False)
    course_map = load_our_map(map_name)
    route = plan_tunnel_first_route(course_map)
    assert np.allclose(route.s_metres, S_METRES)
    assert np.allclose(route.look_point_metres, [S_METRES[0], S_METRES[1] + 0.26])
    assert course_map.highest_corner_metres[1] - route.look_point_metres[1] >= 0.62
    monkeypatch.setenv("TEAM_NAV_LOOK_BACKUP_METRES", "0.40")
    with pytest.raises(TunnelFirstRouteError, match="north net"):
        plan_tunnel_first_route(course_map)
