"""Tests the fallback missions' three safety rules from the team's review (2026-11-08), without
flying: the limited height hold (height_hold.py), the position health and the two-way abort
(position_health.py, fallback_abort.py, fallback_run.py), and the launch pose check.

How to run these tests is in the package's README.
"""
import types

import numpy as np
import pytest

from our_files import SCAN_MAP, load_our_map, load_side_gap_first_mission
from team_nav.fallback_abort import choose_landing_spot, line_is_clear
from team_nav.fallback_routes import FALLBACK_PAD, HIGH_HEIGHT_METRES, plan_fallback_route
from team_nav.fallback_run import (LARGEST_LAUNCH_POSE_FROM_ORIGIN_METRES, FallbackPlan, FallbackRun,
                                   pose_too_far_from_origin)
from team_nav.height_hold import (CORRECTION_RATE_METRES_PER_SECOND, LARGEST_DOWNWARD_CORRECTION_METRES,
                                  HeightHeldByCameraTracking)
from team_nav.position_health import PositionHealth
from team_nav.route_follower import RouteFollower
from team_nav.course_map import shapes_blocking_between_heights
from team_nav.drone_dimensions import HALF_HEIGHT_NEEDED_METRES

SECONDS_PER_CYCLE = 0.1
# The team's limit: no target more than this above the route's height.
LARGEST_RISE_ABOVE_CRUISE_METRES = 0.15


# ---- 1. The height hold ---------------------------------------------------------------------------

class PretendHeights:
    """Only what the height hold reads from the drone link. Change the attributes between cycles."""

    def __init__(self, px4_height_metres, camera_height_metres):
        self.px4_height_metres = px4_height_metres
        self.camera_height_metres = camera_height_metres
        self.camera_age_seconds = 0.03
        self.lost = False
        self.events = []
        self.log = types.SimpleNamespace(event=lambda kind, text, **details: self.events.append(text))

    def px4_height_risen_metres(self):
        return self.px4_height_metres

    def position_from_launch(self):
        return np.array([0.0, 0.0, self.camera_height_metres])

    def camera_tracking_age_seconds(self):
        return self.camera_age_seconds

    def camera_tracking_says_lost(self):
        return self.lost


def fly_the_hold(link, change_each_cycle, cycles=200):
    """Runs the height hold on a level route at cruise height; returns every target height and
    every correction. change_each_cycle(cycle, link) changes the pretend readings."""
    waypoints = np.array([[0.0, 0.0, HIGH_HEIGHT_METRES], [50.0, 0.0, HIGH_HEIGHT_METRES]])
    held = HeightHeldByCameraTracking(RouteFollower(waypoints), link)
    heights, corrections = [], []
    for cycle in range(cycles):
        change_each_cycle(cycle, link)
        heights.append(held.next_target(waypoints[0], SECONDS_PER_CYCLE)[2])
        corrections.append(held.correction_metres)
    return np.array(heights), np.array(corrections)


def assert_within_the_limits(heights, corrections):
    assert heights.max() <= HIGH_HEIGHT_METRES + LARGEST_RISE_ABOVE_CRUISE_METRES + 1e-9
    assert np.abs(corrections).max() <= LARGEST_DOWNWARD_CORRECTION_METRES + 1e-9
    steps = np.abs(np.diff(np.concatenate([[0.0], corrections])))
    assert steps.max() <= CORRECTION_RATE_METRES_PER_SECOND * SECONDS_PER_CYCLE + 1e-9


@pytest.mark.parametrize("jump_metres", [+0.5, +2.0, -0.5, -2.0])
def test_a_camera_height_jump_is_ignored_and_never_raises_the_target(jump_metres):
    link = PretendHeights(HIGH_HEIGHT_METRES, HIGH_HEIGHT_METRES)

    def jump_after_two_seconds(cycle, pretend):
        if cycle == 20:
            pretend.camera_height_metres += jump_metres

    heights, corrections = fly_the_hold(link, jump_after_two_seconds)
    assert_within_the_limits(heights, corrections)
    assert np.allclose(corrections, 0.0)                 # the jump was ignored entirely
    assert any("stepped" in text for text in link.events)


def test_a_camera_height_that_slides_down_in_small_steps_raises_the_target_at_most_015_m():
    """Steps under the 0.15 m limit get through; the upward limit and the rate still hold."""
    link = PretendHeights(HIGH_HEIGHT_METRES, HIGH_HEIGHT_METRES)

    def slide_down(cycle, pretend):
        pretend.camera_height_metres -= 0.1

    heights, corrections = fly_the_hold(link, slide_down)
    assert_within_the_limits(heights, corrections)
    assert heights.max() > HIGH_HEIGHT_METRES + 0.1      # it did reach its upward limit


def test_a_frozen_camera_height_while_px4_drifts_never_raises_the_target_more_than_015_m():
    link = PretendHeights(HIGH_HEIGHT_METRES, HIGH_HEIGHT_METRES)

    def px4_drifts_up(cycle, pretend):
        pretend.px4_height_metres += 0.05                # camera tracking's height stays frozen

    heights, corrections = fly_the_hold(link, px4_drifts_up)
    assert_within_the_limits(heights, corrections)


def test_a_stale_or_lost_camera_freezes_the_correction():
    link = PretendHeights(HIGH_HEIGHT_METRES + 0.2, HIGH_HEIGHT_METRES)

    def go_silent_then_lost(cycle, pretend):
        pretend.camera_age_seconds = 2.0 if cycle >= 10 else 0.03
        pretend.lost = cycle >= 50
        pretend.camera_height_metres -= 0.05 if cycle >= 10 else 0.0

    heights, corrections = fly_the_hold(link, go_silent_then_lost)
    assert_within_the_limits(heights, corrections)
    assert np.allclose(corrections[10:], corrections[9])


def test_a_real_px4_drift_is_followed_slowly_down_to_the_limit():
    """The case the hold exists for: PX4 believes 2.0 m where the drone is truly at 2.3 m."""
    link = PretendHeights(2.0, HIGH_HEIGHT_METRES)
    heights, corrections = fly_the_hold(link, lambda cycle, pretend: None, cycles=60)
    assert_within_the_limits(heights, corrections)
    assert corrections[9] == pytest.approx(-0.1)          # 0.1 m/s
    assert heights[-1] == pytest.approx(HIGH_HEIGHT_METRES - LARGEST_DOWNWARD_CORRECTION_METRES)


# ---- 2. Position health ---------------------------------------------------------------------------

def test_steady_reports_moving_at_flying_speed_are_healthy():
    health = PositionHealth()
    for report in range(50):
        health.observe(report * 0.033, [report * 0.033, 0.0, 2.3], 0.03, False, (0, 0))
    assert health.problem() is None


def test_a_position_jump_is_lost_for_the_rest_of_the_run():
    health = PositionHealth()
    health.observe(0.0, [0.0, 0.0, 2.3], 0.03, False, (0, 0))
    health.observe(0.1, [0.5, 0.0, 2.3], 0.03, False, (0, 0))
    assert "jumped" in health.problem()
    health.observe(0.2, [0.5, 0.0, 2.3], 0.03, False, (0, 0))
    assert "jumped" in health.problem()


def test_a_silence_or_a_lost_report_counts_only_while_it_lasts():
    health = PositionHealth()
    health.observe(0.0, [0.0, 0.0, 2.3], 0.8, False, (0, 0))
    assert "silent" in health.problem()
    health.observe(0.9, [0.0, 0.0, 2.3], 0.03, True, (0, 0))
    assert "lost track" in health.problem()
    health.observe(1.0, [0.0, 0.0, 2.3], 0.03, False, (0, 0))
    assert health.problem() is None


def test_a_px4_position_reset_is_lost_for_the_rest_of_the_run():
    health = PositionHealth()
    health.observe(0.0, [0.0, 0.0, 2.3], 0.03, False, (3, 1))
    health.observe(0.1, [0.0, 0.0, 2.3], 0.03, False, (4, 1))
    assert "reset" in health.problem()


# ---- 2. The abort -----------------------------------------------------------------------------------

def test_up_high_the_abort_flies_to_the_pad():
    course_map = load_our_map(SCAN_MAP)
    spot, why = choose_landing_spot(course_map, [0.0, 2.6, HIGH_HEIGHT_METRES])      # over the maze
    assert np.allclose(spot, course_map.landing_pad_centre_metres) and "pad" in why


def test_in_the_tunnel_the_abort_goes_to_open_floor_that_a_straight_line_reaches():
    course_map = load_our_map(SCAN_MAP)
    tube = course_map.tubes[0]
    inside = (tube.entrance_centre_metres + tube.exit_centre_metres) / 2.0
    height = tube.outside_height_metres / 2.0
    spot, why = choose_landing_spot(course_map, [inside[0], inside[1], height])
    assert spot is not None and not np.allclose(spot, course_map.landing_pad_centre_metres)
    blocking = shapes_blocking_between_heights(course_map, height - HALF_HEIGHT_NEEDED_METRES, height + HALF_HEIGHT_NEEDED_METRES)
    assert line_is_clear(blocking, inside, spot)


class PretendWatch:
    def __init__(self, problem, drone):
        self.position_health = types.SimpleNamespace(problem=lambda: problem)
        self.drone = drone
        self.targets_sent = []

    def drone_on_course(self):
        return self.drone

    def observe_position(self):
        pass

    def send_target_on_course(self, target):
        self.targets_sent.append(target)


def abort_run(problem, drone):
    course_map = load_our_map(SCAN_MAP)
    run = FallbackRun.__new__(FallbackRun)
    run.plan = FallbackPlan(course_map, load_side_gap_first_mission(), [], plan_fallback_route(course_map, FALLBACK_PAD), 90.0)
    run.watch = PretendWatch(problem, np.array(drone, dtype=float))
    events = []
    run.log = types.SimpleNamespace(event=lambda kind, text, **details: events.append(text))
    run.link = types.SimpleNamespace(stage="", control_is_ours=lambda: True, is_armed=lambda: True,
                                     now_seconds=lambda: 0.0, record_drone_condition=lambda: None,
                                     wait_one_cycle=lambda: None, log=run.log,
                                     px4_height_risen_metres=lambda: None, position_from_launch=lambda: None,
                                     camera_tracking_age_seconds=lambda: 0.03, camera_tracking_says_lost=lambda: False)
    return run, events


def test_with_the_position_lost_the_abort_never_moves_sideways():
    run, events = abort_run("Camera tracking's position jumped 0.50 m in 0.10 s.", [0.0, 2.6, HIGH_HEIGHT_METRES])
    run.fly_to_a_landing_spot_if_position_is_healthy()
    assert run.watch.targets_sent == []
    assert any("POSITION LOST" in text for text in events)


def test_with_a_healthy_position_the_abort_flies_level_towards_the_pad():
    run, events = abort_run(None, [0.0, 2.6, HIGH_HEIGHT_METRES])
    run.watch.drone_on_course = lambda: run.watch.drone        # the pretend drone stays put
    run.link.now_seconds = iter(np.arange(0.0, 1000.0, 0.1)).__next__
    run.fly_to_a_landing_spot_if_position_is_healthy()
    targets = np.array(run.watch.targets_sent)
    assert len(targets) > 0 and any("POSITION HEALTHY" in text for text in events)
    assert np.allclose(targets[:, 2], HIGH_HEIGHT_METRES)
    towards_pad = load_our_map(SCAN_MAP).landing_pad_centre_metres - run.watch.drone[:2]
    assert (targets[-1][:2] - run.watch.drone[:2]) @ towards_pad > 0


# ---- 3. The launch pose -----------------------------------------------------------------------------

def test_a_launch_pose_within_03_m_of_the_origin_is_accepted_and_one_further_is_refused():
    assert pose_too_far_from_origin(np.array([0.2, -0.2, 0.05])) is None
    refusal = pose_too_far_from_origin(np.array([0.3, 0.1, 0.0]))
    assert refusal is not None and "restart the drone software" in refusal
    assert LARGEST_LAUNCH_POSE_FROM_ORIGIN_METRES == 0.3


def test_the_hold_stops_sending_and_refuses_when_the_launch_pose_is_bad(monkeypatch):
    """flight_steps.hold_above_launch_until_control_is_ours with the fallback's check: no target is
    sent once the check fails, so the hand-over cannot happen."""
    from team_nav import flight_steps

    class PretendLink:
        stage = ""
        log = types.SimpleNamespace(event=lambda *arguments, **details: None)
        targets_sent = 0

        def is_armed(self):
            return False

        def control_is_ours(self):
            return False

        def is_landed(self):
            return False

        def send_position_from_launch(self, target):
            self.targets_sent += 1

        def now_seconds(self):
            return 0.0

    link = PretendLink()
    monkeypatch.setattr(flight_steps, "remember_launch_mark", lambda pretend: None)
    monkeypatch.setattr(flight_steps, "log_launch_readings", lambda pretend: None)
    with pytest.raises(flight_steps.LaunchPoseRefused):
        flight_steps.hold_above_launch_until_control_is_ours(link, 0.8, launch_pose_problem=lambda pretend: "too far")
    assert link.targets_sent == 0


def test_a_drone_that_drifted_above_the_route_aborts_at_no_more_than_the_route_height():
    """Simulator, 2026-11-08: the run stopped with the drone at 2.49 m, and a target at that height
    was refused (run_safety's 2.45 m limit), so it landed over the maze instead of flying to the pad."""
    run, events = abort_run(None, [0.0, 2.6, 2.49])
    run.watch.drone_on_course = lambda: run.watch.drone
    run.link.now_seconds = iter(np.arange(0.0, 1000.0, 0.1)).__next__
    run.fly_to_a_landing_spot_if_position_is_healthy()
    targets = np.array(run.watch.targets_sent)
    assert len(targets) > 0 and np.allclose(targets[:, 2], HIGH_HEIGHT_METRES)
