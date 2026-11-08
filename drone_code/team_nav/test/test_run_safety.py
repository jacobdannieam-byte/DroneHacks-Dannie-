"""Tests run_safety.py: the checks that decide, every cycle, whether the run carries on, holds
still, lands, or has been taken away from our program. They use made-up readings on our scan map.

The main idea tested: one bad reading may make the drone hold still, but never land it. Only a
problem that lasts lands it, except a touch or lost control.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from our_files import SCAN_MAP, load_our_map
from team_nav.height_check import CAMERA_HEIGHT_GAP_LONGEST_SECONDS
from team_nav.run_safety import (
    CAMERA_POSITION_UNUSED_LONGEST_SECONDS, CAMERA_PROBLEM_LONGEST_SECONDS, CARRY_ON, CONTROL_LOST,
    FAR_TARGET_LONGEST_SECONDS, FARTHEST_TARGET_FROM_DRONE_METRES, FRESH_REPORTS_BEFORE_RESUMING_SECONDS,
    HEADING_GAP_LONGEST_SECONDS, HOLD, LAND_HERE, LASER_UNUSED_LONGEST_SECONDS, DroneCondition, RunSafety,
    TargetRefused,
)

SCAN = load_our_map(SCAN_MAP)
# The launch mark at cruise height: the laser sees clear floor there (checked below).
ABOVE_LAUNCH_MARK = np.array([SCAN.launch_mark_centre_metres[0], SCAN.launch_mark_centre_metres[1], 0.8])


def normal_condition(now_seconds=0.0, **changes):
    """A drone hovering normally above the launch mark, with any values changed."""
    values = dict(
        now_seconds=now_seconds, control_is_ours=True, is_armed=True,
        camera_report_age_seconds=0.06, camera_tracking_says_lost=False,
        position_on_course_metres=ABOVE_LAUNCH_MARK, tilt_degrees=2.0, rotation_degrees_per_second=10.0,
        px4_height_risen_metres=0.80, laser_height_risen_metres=0.78,
    )
    values.update(changes)
    return DroneCondition(**values)


def test_the_launch_mark_is_over_clear_floor():
    assert RunSafety(SCAN).height_check.laser_sees_clear_floor(ABOVE_LAUNCH_MARK)


def test_normal_flight_carries_on():
    assert RunSafety(SCAN).check(normal_condition()).action == CARRY_ON


def test_leaving_offboard_or_disarming_means_control_is_lost_whatever_else_happens():
    safety = RunSafety(SCAN)
    not_offboard = safety.check(normal_condition(control_is_ours=False, tilt_degrees=60.0))
    disarmed = safety.check(normal_condition(is_armed=False))

    assert not_offboard.action == CONTROL_LOST and "Offboard" in not_offboard.reason
    assert disarmed.action == CONTROL_LOST and "disarmed" in disarmed.reason


def test_a_sudden_tilt_lands_the_drone_using_px4s_height():
    safety = RunSafety(SCAN)
    assert safety.check(normal_condition(0.00, tilt_degrees=35.0)).action == CARRY_ON   # One report could be a glitch.
    decision = safety.check(normal_condition(0.10, tilt_degrees=35.0))

    assert decision.action == LAND_HERE
    assert not decision.land_using_laser
    assert "hit something" in decision.reason


def test_one_late_camera_report_holds_at_once_but_never_lands():
    safety = RunSafety(SCAN)
    first = safety.check(normal_condition(0.0, camera_report_age_seconds=0.6))
    assert first.action == HOLD and "0.60 s old" in first.reason
    # The reports are fresh again: the hold lasts until they have been fresh for a while.
    assert safety.check(normal_condition(0.1)).action == HOLD
    assert safety.check(normal_condition(0.1 + FRESH_REPORTS_BEFORE_RESUMING_SECONDS)).action == CARRY_ON


def test_late_but_usable_reports_from_a_bad_period_end_the_hold_without_landing():
    # Measured in bad periods on 2026-11-08: most reports 0.15 to 0.43 s old. Under the 0.45 s
    # limit they are usable, so one too-old report gives only a short hold.
    safety = RunSafety(SCAN)
    assert safety.check(normal_condition(0.0, camera_report_age_seconds=0.5)).action == HOLD
    actions = []
    for tenth in range(1, 30):
        age_seconds = 0.30 if tenth % 3 == 0 else 0.20
        actions.append(safety.check(normal_condition(tenth / 10.0, camera_report_age_seconds=age_seconds)).action)
    assert actions[0] == HOLD
    assert actions[-1] == CARRY_ON
    assert LAND_HERE not in actions


def test_camera_tracking_stale_for_three_seconds_lands_using_the_laser():
    safety = RunSafety(SCAN)
    first = safety.check(normal_condition(0.0, camera_report_age_seconds=0.6))
    still_holding = safety.check(normal_condition(CAMERA_PROBLEM_LONGEST_SECONDS - 0.1, camera_report_age_seconds=3.5))
    gave_up = safety.check(normal_condition(CAMERA_PROBLEM_LONGEST_SECONDS, camera_report_age_seconds=3.6))

    assert first.action == HOLD and still_holding.action == HOLD
    assert gave_up.action == LAND_HERE and gave_up.land_using_laser


def test_a_fresh_report_between_late_ones_restarts_the_landing_clock():
    safety = RunSafety(SCAN)
    safety.check(normal_condition(0.0, camera_tracking_says_lost=True))
    safety.check(normal_condition(2.0, camera_report_age_seconds=0.06))
    assert safety.check(normal_condition(2.1, camera_tracking_says_lost=True)).action == HOLD
    assert safety.check(normal_condition(2.1 + CAMERA_PROBLEM_LONGEST_SECONDS - 0.1,
                                         camera_tracking_says_lost=True)).action == HOLD


def test_px4s_height_disagreeing_with_the_laser_over_clear_floor_lands_using_the_laser():
    safety = RunSafety(SCAN)
    assert safety.check(normal_condition(0.0, px4_height_risen_metres=1.05)).action == CARRY_ON
    decision = safety.check(normal_condition(1.1, px4_height_risen_metres=1.05))

    assert decision.action == LAND_HERE
    assert decision.land_using_laser
    assert "stopped using the laser" in decision.reason


def test_the_height_check_keeps_running_while_the_drone_holds_for_camera_tracking():
    safety = RunSafety(SCAN)
    first = safety.check(normal_condition(0.0, camera_report_age_seconds=0.6, px4_height_risen_metres=1.05))
    decision = safety.check(normal_condition(1.1, camera_report_age_seconds=1.6, px4_height_risen_metres=1.05))

    assert first.action == HOLD
    assert decision.action == LAND_HERE and decision.land_using_laser
    assert "stopped using the laser" in decision.reason


def test_without_a_raw_reading_the_height_check_also_runs_during_a_hold():
    safety = RunSafety(SCAN)
    safety.check(no_raw_laser(0.0, uses_laser=True))
    safety.check(no_raw_laser(1.0, uses_laser=False, camera_tracking_says_lost=True))
    decision = safety.check(no_raw_laser(1.0 + LASER_UNUSED_LONGEST_SECONDS, uses_laser=False, faulty=True,
                                         camera_tracking_says_lost=True))
    assert decision.action == LAND_HERE and "faulty" in decision.reason


def test_without_a_laser_reading_the_height_check_waits():
    safety = RunSafety(SCAN)
    safety.check(normal_condition(0.0, px4_height_risen_metres=1.05, laser_height_risen_metres=None))
    assert safety.check(normal_condition(2.0, px4_height_risen_metres=1.05, laser_height_risen_metres=None)).action == CARRY_ON


# ---- Without the laser's raw reading, as on the real drone ----------------------------------------

def no_raw_laser(now_seconds, uses_laser, height_metres=0.8, faulty=False, **changes):
    position = np.array([ABOVE_LAUNCH_MARK[0], ABOVE_LAUNCH_MARK[1], height_metres])
    values = dict(raw_laser_available=False, laser_height_risen_metres=None, position_on_course_metres=position,
                  px4_height_risen_metres=height_metres, px4_uses_laser_for_height=uses_laser,
                  px4_says_laser_is_faulty=faulty)
    values.update(changes)
    return normal_condition(now_seconds, **values)


def test_without_a_raw_reading_px4_stopping_using_the_laser_warns_once_and_flies_on():
    # As on drone 16 on 2026-11-08: PX4 stopped using the laser, but its height stayed close to
    # camera tracking's. The team decided that the flight goes on.
    safety = RunSafety(SCAN)
    assert safety.check(no_raw_laser(0.0, uses_laser=True)).action == CARRY_ON
    assert safety.check(no_raw_laser(1.0, uses_laser=False)).action == CARRY_ON
    decision = safety.check(no_raw_laser(1.0 + LASER_UNUSED_LONGEST_SECONDS, uses_laser=False))
    later = safety.check(no_raw_laser(5.0, uses_laser=False))

    assert decision.action == CARRY_ON and "stopped using the bottom laser" in decision.warning
    assert later.action == CARRY_ON and later.warning == ""
    assert safety.laser_use_check.laser_has_stopped


def test_after_the_laser_stops_a_smaller_height_difference_lands_the_drone():
    # 20 cm is inside the normal 30 cm limit, but outside the 15 cm limit for a stopped laser.
    with_laser = RunSafety(SCAN)
    with_laser.check(no_raw_laser(0.0, uses_laser=True, px4_height_risen_metres=1.0))
    assert with_laser.check(no_raw_laser(1.5, uses_laser=True, px4_height_risen_metres=1.0)).action == CARRY_ON

    without_laser = RunSafety(SCAN)
    without_laser.check(no_raw_laser(0.0, uses_laser=True))
    without_laser.check(no_raw_laser(1.0, uses_laser=False))
    without_laser.check(no_raw_laser(2.1, uses_laser=False))
    without_laser.check(no_raw_laser(3.0, uses_laser=False, px4_height_risen_metres=1.0))
    decision = without_laser.check(no_raw_laser(4.1, uses_laser=False, px4_height_risen_metres=1.0))

    assert decision.action == LAND_HERE and decision.land_using_laser
    assert "more than 0.15 m" in decision.reason


def test_without_a_raw_reading_the_floor_and_a_px4_that_never_used_the_laser_dont_stop_the_run():
    safety = RunSafety(SCAN)
    # Close to the floor PX4 may use the air pressure sensor. Then it never uses the laser at all.
    for tenth in range(50):
        assert safety.check(no_raw_laser(tenth / 10.0, uses_laser=False, height_metres=0.1 + tenth / 50.0)).action == CARRY_ON
    # Missing flags prove nothing either.
    assert safety.check(no_raw_laser(10.0, uses_laser=None)).action == CARRY_ON


def test_without_a_raw_reading_a_faulty_laser_report_lands_the_drone():
    safety = RunSafety(SCAN)
    safety.check(no_raw_laser(0.0, uses_laser=False, faulty=True))
    decision = safety.check(no_raw_laser(LASER_UNUSED_LONGEST_SECONDS + 0.1, uses_laser=False, faulty=True))
    assert decision.action == LAND_HERE and "faulty" in decision.reason


def test_px4s_height_far_from_camera_trackings_lands_the_drone_even_while_px4_says_it_uses_the_laser():
    # As in the simulator flight over a surprise boulder on 2026-11-07: PX4's height 50 cm low,
    # with its laser flag still on.
    safety = RunSafety(SCAN)
    assert safety.check(no_raw_laser(0.0, uses_laser=True, px4_height_risen_metres=0.30)).action == CARRY_ON
    decision = safety.check(no_raw_laser(1.0, uses_laser=True, px4_height_risen_metres=0.30))

    assert decision.action == LAND_HERE and decision.land_using_laser
    assert "camera tracking says 0.80 m" in decision.reason


def test_camera_trackings_height_is_not_compared_while_camera_tracking_is_late():
    safety = RunSafety(SCAN)
    for tenth in range(20):
        decision = safety.check(no_raw_laser(tenth / 10.0, uses_laser=True, px4_height_risen_metres=0.30,
                                             camera_report_age_seconds=0.6))
        assert decision.action == HOLD


def test_a_target_inside_the_course_and_near_the_drone_is_sent_unchanged():
    target = ABOVE_LAUNCH_MARK + [0.0, 0.5, 0.0]
    np.testing.assert_allclose(RunSafety(SCAN).check_target(target, ABOVE_LAUNCH_MARK, 0.0), target)


@pytest.mark.parametrize("target, words", [
    ([-2.9, 0.9, 0.8], "safety net"),            # The drone's side would be in the net.
    ([0.0, 3.5, 0.8], "safety net"),
    ([-2.5, 0.9, 1.70], "wall tops"),
    ([-2.5, 0.9, -0.2], "wall tops"),
    ([-2.5, 0.9, float("nan")], "not a real number"),
])
def test_a_target_outside_the_course_is_refused(target, words):
    with pytest.raises(TargetRefused, match=words):
        RunSafety(SCAN).check_target(target, None, 0.0)


def test_one_target_far_from_the_drone_is_pulled_in_not_refused():
    safety = RunSafety(SCAN)
    sent = safety.check_target(ABOVE_LAUNCH_MARK + [0.0, -1.5, 0.0], ABOVE_LAUNCH_MARK, 0.0)
    np.testing.assert_allclose(sent, ABOVE_LAUNCH_MARK + [0.0, -FARTHEST_TARGET_FROM_DRONE_METRES, 0.0])
    # The next target is near again: nothing stops the run.
    safety.check_target(ABOVE_LAUNCH_MARK + [0.0, -0.3, 0.0], ABOVE_LAUNCH_MARK, 0.1)
    safety.check_target(ABOVE_LAUNCH_MARK + [0.0, -1.5, 0.0], ABOVE_LAUNCH_MARK, 0.2)


def test_targets_that_stay_far_from_the_drone_stop_the_run():
    safety = RunSafety(SCAN)
    far = ABOVE_LAUNCH_MARK + [0.0, -1.5, 0.0]
    safety.check_target(far, ABOVE_LAUNCH_MARK, 0.0)
    safety.check_target(far, ABOVE_LAUNCH_MARK, 0.1)
    safety.check_target(far, ABOVE_LAUNCH_MARK, 0.2)
    with pytest.raises(TargetRefused, match="too fast"):
        safety.check_target(far, ABOVE_LAUNCH_MARK, FAR_TARGET_LONGEST_SECONDS)


def test_a_hold_between_two_far_targets_does_not_count_as_far_in_a_row():
    safety = RunSafety(SCAN)
    far = ABOVE_LAUNCH_MARK + [0.0, -1.5, 0.0]
    safety.check_target(far, ABOVE_LAUNCH_MARK, 0.0)
    assert safety.check(normal_condition(0.1, camera_report_age_seconds=0.6)).action == HOLD
    safety.check_target(far, ABOVE_LAUNCH_MARK, 2.0)


# ---- Rule 5: PX4 stops trusting camera tracking (drone 16, 2026-11-08) --------------------------

def test_a_heading_gap_holds_at_once_with_px4s_heading():
    safety = RunSafety(SCAN)
    decision = safety.check(normal_condition(0.0, px4_heading_gap_degrees=-36.0))
    assert decision.action == HOLD and decision.hold_with_px4_heading


def test_a_heading_gap_like_the_1020_turn_test_does_not_land_the_drone():
    # 10:20 on 2026-11-08: a -36 degree gap for about 1 s, then PX4 went back to the camera.
    safety = RunSafety(SCAN)
    for tenth in range(11):
        assert safety.check(normal_condition(tenth / 10.0, px4_heading_gap_degrees=-36.0)).action == HOLD
    assert safety.check(normal_condition(1.1, px4_heading_gap_degrees=2.0)).action == CARRY_ON


def test_a_heading_gap_that_lasts_lands_the_drone():
    # In the 06:15 flight PX4's heading swung about 190 degrees away from the camera's.
    safety = RunSafety(SCAN)
    assert safety.check(normal_condition(0.0, px4_heading_gap_degrees=-170.0)).action == HOLD
    decision = safety.check(normal_condition(HEADING_GAP_LONGEST_SECONDS, px4_heading_gap_degrees=-170.0))
    assert decision.action == LAND_HERE
    assert "compass" in decision.reason
    assert not decision.land_using_laser


def test_a_small_heading_gap_while_turning_carries_on():
    # Measured: while PX4 used the camera's heading, the gap stayed within 11 degrees in 95 of 100 reports.
    safety = RunSafety(SCAN)
    for step in range(30):
        assert safety.check(normal_condition(step * 0.1, px4_heading_gap_degrees=12.0)).action == CARRY_ON


def test_a_flickering_heading_gap_restarts_the_landing_clock():
    safety = RunSafety(SCAN)
    safety.check(normal_condition(0.0, px4_heading_gap_degrees=40.0))
    assert safety.check(normal_condition(2.0, px4_heading_gap_degrees=3.0)).action == CARRY_ON
    assert safety.check(normal_condition(2.1 + HEADING_GAP_LONGEST_SECONDS - 0.1,
                                         px4_heading_gap_degrees=40.0)).action == HOLD


def test_px4_not_using_the_camera_position_holds_at_once_and_lands_only_if_it_lasts():
    safety = RunSafety(SCAN)
    assert safety.check(normal_condition(0.0, px4_uses_camera_position=False)).action == HOLD
    assert safety.check(normal_condition(1.0, px4_uses_camera_position=False)).action == HOLD
    decision = safety.check(normal_condition(CAMERA_POSITION_UNUSED_LONGEST_SECONDS, px4_uses_camera_position=False))
    assert decision.action == LAND_HERE
    assert "motion sensors" in decision.reason


def test_camera_trust_is_checked_from_takeoff_even_on_the_floor():
    # In the 06:15 flight the swing to the compass came 2.4 s after arming, before the drone rose.
    on_floor = np.array([ABOVE_LAUNCH_MARK[0], ABOVE_LAUNCH_MARK[1], 0.0])
    safety = RunSafety(SCAN)
    first = normal_condition(0.0, position_on_course_metres=on_floor, px4_heading_gap_degrees=166.0,
                             px4_height_risen_metres=0.0, laser_height_risen_metres=0.0)
    later = normal_condition(HEADING_GAP_LONGEST_SECONDS, position_on_course_metres=on_floor,
                             px4_heading_gap_degrees=166.0, px4_height_risen_metres=0.0, laser_height_risen_metres=0.0)
    assert safety.check(first).action == HOLD
    assert safety.check(later).action == LAND_HERE


# ---- Camera tracking's height lags in a climb (drone 16, 12:15 and 12:16 on 2026-11-08) ---------

def climbing(now_seconds, px4_height_metres, camera_height_metres, upward_speed_metres_per_second):
    position = np.array([ABOVE_LAUNCH_MARK[0], ABOVE_LAUNCH_MARK[1], camera_height_metres])
    return no_raw_laser(now_seconds, uses_laser=True, height_metres=camera_height_metres,
                        px4_height_risen_metres=px4_height_metres, position_on_course_metres=position,
                        px4_upward_speed_metres_per_second=upward_speed_metres_per_second)


def test_camera_height_lagging_in_a_climb_does_not_stop_the_run():
    # 12:15: PX4 0.52 m, camera 0.16 m, climbing at about 0.45 m/s.
    safety = RunSafety(SCAN)
    for tenth in range(20):
        assert safety.check(climbing(tenth / 10.0, 0.52, 0.16, 0.45)).action == CARRY_ON


def test_the_same_height_gap_while_level_stops_the_run():
    safety = RunSafety(SCAN)
    safety.check(climbing(0.0, 0.52, 0.16, 0.0))
    decision = safety.check(climbing(CAMERA_HEIGHT_GAP_LONGEST_SECONDS, 0.52, 0.16, 0.0))
    assert decision.action == LAND_HERE and decision.land_using_laser
