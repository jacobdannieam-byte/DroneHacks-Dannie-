"""position_watchdog.py on its own: each rule trips exactly at its threshold, not before, and only
once the watchdog is watching (from takeoff)."""
import pytest

from team_nav.mission_settings import MissionSettings
from team_nav.position_watchdog import PositionWatchdog

STILL = (0.0, 0.0, 0.0, 1.0)                      # an orientation quaternion (x, y, z, w)


def watching(settings=None, start_seconds=0.0):
    watchdog = PositionWatchdog(settings or MissionSettings())
    watchdog.observe_report(start_seconds, start_seconds, (0.0, 0.0, 0.8), STILL)
    watchdog.start_watching(start_seconds)
    return watchdog


def wobble(index):
    """A tiny change, as real camera tracking always has between reports."""
    return 1e-6 * (index % 2)


# ---------------------------------------------------------------- rule (a): reports stop
def test_rule_a_trips_after_no_report_for_more_than_0_2_s():
    watchdog = watching()
    assert watchdog.check(0.2, motion_requested=False) is None           # exactly 0.2 s: not yet
    rule, reason = watchdog.check(0.21, motion_requested=False)
    assert rule == "a" and "no new camera tracking report for 0.21 s (limit 0.2 s)" in reason


def test_rule_a_does_not_trip_while_reports_keep_arriving():
    watchdog = watching()
    for index in range(1, 100):
        seconds = index * 0.1
        watchdog.observe_report(seconds, seconds, (0.0, 0.0, 0.8 + wobble(index)), STILL)
        assert watchdog.check(seconds + 0.05, motion_requested=True) is None


# ---------------------------------------------------------------- rule (b): a jump
def test_rule_b_allows_1_5_m_per_s_times_the_time_between_reports_plus_5_cm():
    watchdog = watching()
    watchdog.observe_report(0.1, 0.1, (0.199, 0.0, 0.8), STILL)          # allowed 1.5 x 0.1 + 0.05 = 0.2 m
    assert watchdog.tripped is None
    watchdog.observe_report(0.2, 0.2, (0.400, 0.0, 0.8), STILL)          # 0.201 m in 0.1 s: too far
    rule, reason = watchdog.tripped
    assert rule == "b" and "moved 0.20 m in 0.100 s" in reason


def test_rule_b_uses_the_reports_own_timestamps_not_when_they_arrive():
    watchdog = watching()
    # Measured 0.4 s apart, delivered 0.05 s apart: 0.6 m is believable in 0.4 s (0.65 m allowed).
    watchdog.observe_report(0.45, 0.4, (0.6, 0.0, 0.8), STILL)
    assert watchdog.tripped is None
    # Measured 0.05 s apart, delivered 0.4 s apart: 0.2 m is not believable in 0.05 s (0.125 m allowed).
    watchdog.observe_report(0.85, 0.45, (0.8, 0.0, 0.8), STILL)
    assert watchdog.tripped[0] == "b"


def test_rule_b_follows_its_settings():
    settings = MissionSettings(watchdog_jump_speed_metres_per_second=3.0, watchdog_jump_margin_metres=0.1)
    watchdog = watching(settings)
    watchdog.observe_report(0.1, 0.1, (0.39, 0.0, 0.8), STILL)           # 3 x 0.1 + 0.1 = 0.4 m allowed
    assert watchdog.tripped is None


# ---------------------------------------------------------------- rule (c): frozen while moving
def frozen_reports(watchdog, from_seconds, until_seconds):
    """Identical reports 30 times a second, after from_seconds up to until_seconds. Returns the last time."""
    seconds = from_seconds
    while seconds < until_seconds - 1e-9:
        seconds = round(min(seconds + 1 / 30, until_seconds), 6)
        watchdog.observe_report(seconds, seconds, (0.0, 0.0, 0.8), STILL)   # the same, exactly
    return seconds


def test_rule_c_trips_after_0_5_s_of_identical_reports_while_a_target_asks_for_motion():
    watchdog = watching()                                                 # frozen since 0 s
    frozen_reports(watchdog, 0.0, 0.45)
    assert watchdog.check(0.45, motion_requested=True) is None            # motion asked from 0.45 s
    frozen_reports(watchdog, 0.45, 0.9)
    assert watchdog.check(0.9, motion_requested=True) is None
    frozen_reports(watchdog, 0.9, 0.95)
    rule, reason = watchdog.check(0.95, motion_requested=True)
    assert rule == "c" and "bit-identical for 0.50 s" in reason


def test_rule_c_does_not_trip_while_no_motion_is_asked():
    watchdog = watching()
    last = frozen_reports(watchdog, 0.0, 2.0)
    assert watchdog.check(last, motion_requested=False) is None


def test_rule_c_needs_orientation_frozen_too():
    watchdog = watching()
    seconds = 0.0
    for index in range(30):
        seconds = round(seconds + 1 / 30, 6)
        turning = (0.0, 0.0, 0.001 * index, 1.0)                          # same position, turning
        watchdog.observe_report(seconds, seconds, (0.0, 0.0, 0.8), turning)
        assert watchdog.check(seconds, motion_requested=True) is None


# ---------------------------------------------------------------- rule (e): PX4 reset its estimate
def test_rule_e_trips_when_a_reset_counter_changes_after_takeoff():
    watchdog = PositionWatchdog(MissionSettings())
    watchdog.observe_reset_counters(2, 5)                                 # changes before takeoff are fine
    watchdog.observe_reset_counters(3, 5)
    watchdog.observe_report(0.0, 0.0, (0.0, 0.0, 0.8), STILL)
    watchdog.start_watching(0.0)
    watchdog.observe_reset_counters(3, 5)
    assert watchdog.tripped is None
    watchdog.observe_reset_counters(4, 5)
    rule, reason = watchdog.tripped
    assert rule == "e" and "xy_reset_counter 3 -> 4" in reason


def test_rule_e_trips_on_the_heading_counter_too():
    watchdog = watching()
    watchdog.observe_reset_counters(0, 0)                                 # first report after takeoff: baseline
    watchdog.observe_reset_counters(0, 1)
    assert watchdog.tripped[0] == "e" and "heading_reset_counter 0 -> 1" in watchdog.tripped[1]


# ---------------------------------------------------------------- watching, and the first trip
def test_nothing_trips_before_takeoff():
    watchdog = PositionWatchdog(MissionSettings())
    watchdog.observe_report(0.0, 0.0, (0.0, 0.0, 0.0), STILL)
    watchdog.observe_report(0.1, 0.1, (5.0, 0.0, 0.0), STILL)            # a jump while not watching
    watchdog.observe_reset_counters(1, 1)
    assert watchdog.check(10.0, motion_requested=True) is None


def test_the_first_rule_to_trip_is_kept_until_the_next_takeoff():
    watchdog = watching()
    watchdog.observe_report(0.1, 0.1, (1.0, 0.0, 0.8), STILL)            # rule (b)
    assert watchdog.check(5.0, motion_requested=True)[0] == "b"           # rule (a) would also trip now
    watchdog.observe_report(6.0, 6.0, (1.0, 0.0, 0.8), STILL)
    watchdog.start_watching(6.0)
    assert watchdog.tripped is None


@pytest.mark.parametrize("missing", [0.0, -1.0, None])
def test_a_report_without_a_timestamp_uses_its_arrival_time(missing):
    watchdog = watching()
    watchdog.observe_report(0.1, missing, (0.1, 0.0, 0.8), STILL)        # 0.1 m in 0.1 s: fine
    assert watchdog.tripped is None
