"""Tests run_safety.py: the checks that decide, every cycle, whether the run carries on, holds
still, lands, or has been taken away from our program. They use made-up readings on our scan map.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from our_files import SCAN_MAP, load_our_map
from team_nav.run_safety import (
    CARRY_ON, CONTROL_LOST, HOLD, LAND_HERE, LASER_UNUSED_LONGEST_SECONDS, LONGEST_HOLD_SECONDS, DroneCondition,
    RunSafety, TargetRefused,
)

SCAN = load_our_map(SCAN_MAP)
# The launch mark at cruise height: the laser sees clear floor there (checked below).
ABOVE_LAUNCH_MARK = np.array([SCAN.launch_mark_centre_metres[0], SCAN.launch_mark_centre_metres[1], 0.8])


def normal_condition(now_seconds=0.0, **changes):
    """A drone hovering normally above the launch mark, with any values changed."""
    values = dict(
        now_seconds=now_seconds, control_is_ours=True, is_armed=True,
        camera_tracking_age_seconds=0.03, camera_tracking_says_lost=False,
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


def test_silent_camera_tracking_holds_then_lands_using_the_laser():
    safety = RunSafety(SCAN)
    first = safety.check(normal_condition(0.0, camera_tracking_age_seconds=0.5))
    still_holding = safety.check(normal_condition(LONGEST_HOLD_SECONDS - 0.1, camera_tracking_age_seconds=5.0))
    gave_up = safety.check(normal_condition(LONGEST_HOLD_SECONDS + 0.1, camera_tracking_age_seconds=5.1))

    assert first.action == HOLD and "0.50 s old" in first.reason
    assert still_holding.action == HOLD
    assert gave_up.action == LAND_HERE and gave_up.land_using_laser


def test_a_hold_ends_when_tracking_returns_and_a_new_hold_starts_its_own_clock():
    safety = RunSafety(SCAN)
    safety.check(normal_condition(0.0, camera_tracking_says_lost=True))
    assert safety.check(normal_condition(3.0)).action == CARRY_ON
    assert safety.check(normal_condition(4.0, camera_tracking_says_lost=True)).action == HOLD
    assert safety.check(normal_condition(4.0 + LONGEST_HOLD_SECONDS - 0.5, camera_tracking_says_lost=True)).action == HOLD


def test_px4s_height_disagreeing_with_the_laser_over_clear_floor_lands_using_the_laser():
    safety = RunSafety(SCAN)
    assert safety.check(normal_condition(0.0, px4_height_risen_metres=1.05)).action == CARRY_ON
    decision = safety.check(normal_condition(1.1, px4_height_risen_metres=1.05))

    assert decision.action == LAND_HERE
    assert decision.land_using_laser
    assert "stopped using the laser" in decision.reason


def test_the_height_check_keeps_running_while_the_drone_holds_for_camera_tracking():
    safety = RunSafety(SCAN)
    first = safety.check(normal_condition(0.0, camera_tracking_age_seconds=0.5, px4_height_risen_metres=1.05))
    decision = safety.check(normal_condition(1.1, camera_tracking_age_seconds=1.6, px4_height_risen_metres=1.05))

    assert first.action == HOLD
    assert decision.action == LAND_HERE and decision.land_using_laser
    assert "stopped using the laser" in decision.reason


def test_without_a_raw_reading_the_height_check_also_runs_during_a_hold():
    safety = RunSafety(SCAN)
    safety.check(no_raw_laser(0.0, uses_laser=True))
    safety.check(no_raw_laser(1.0, uses_laser=False, camera_tracking_says_lost=True))
    decision = safety.check(no_raw_laser(1.0 + LASER_UNUSED_LONGEST_SECONDS, uses_laser=False,
                                         camera_tracking_says_lost=True))
    assert decision.action == LAND_HERE and "stopped using the bottom laser" in decision.reason


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


def test_without_a_raw_reading_px4_stopping_using_the_laser_lands_the_drone():
    safety = RunSafety(SCAN)
    assert safety.check(no_raw_laser(0.0, uses_laser=True)).action == CARRY_ON
    assert safety.check(no_raw_laser(1.0, uses_laser=False)).action == CARRY_ON
    decision = safety.check(no_raw_laser(1.0 + LASER_UNUSED_LONGEST_SECONDS, uses_laser=False))

    assert decision.action == LAND_HERE and decision.land_using_laser
    assert "stopped using the bottom laser" in decision.reason


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
                                             camera_tracking_age_seconds=0.5))
        assert decision.action == HOLD


def test_a_target_inside_the_course_and_near_the_drone_is_accepted():
    RunSafety(SCAN).check_target(ABOVE_LAUNCH_MARK + [0.0, 0.5, 0.0], ABOVE_LAUNCH_MARK)


@pytest.mark.parametrize("target, words", [
    ([-2.9, 0.9, 0.8], "safety net"),            # The drone's side would be in the net.
    ([0.0, 3.5, 0.8], "safety net"),
    ([-2.5, 0.9, 1.70], "wall tops"),
    ([-2.5, 0.9, -0.2], "wall tops"),
    ([-2.5, 0.9, float("nan")], "not a real number"),
])
def test_a_target_outside_the_course_is_refused(target, words):
    with pytest.raises(TargetRefused, match=words):
        RunSafety(SCAN).check_target(target)


def test_a_target_far_from_the_drone_is_refused():
    with pytest.raises(TargetRefused, match="too fast"):
        RunSafety(SCAN).check_target(ABOVE_LAUNCH_MARK + [0.0, -1.5, 0.0], ABOVE_LAUNCH_MARK)
