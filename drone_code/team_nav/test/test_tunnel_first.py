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
