"""Tests the pass-or-fail rules of the hand tests and table checks (team_nav/hand_tests/) with
made-up readings, so no drone or ROS is needed.

Each test builds the readings a real drone would give in one situation, good or bad, and checks
the verdict. Readings are plain objects with the same fields as drone_listener.py's
TrackingReport and LaserReading, because drone_listener.py itself needs ROS.

How to run these tests is in the package's README.
"""
import math
from types import SimpleNamespace

from team_nav.hand_tests import (
    directions, estimator_sources, interface_settings, laser_heights, laser_over_step, report_rates, touch_tilt,
    tracking_still_drift, tunnel_carry,
)
from team_nav.hand_tests.hand_test_tools import combine, failed, heading_change_degrees, passed, px4_silence_problem


def tracking_report(arrival_seconds, forward_metres, left_metres, up_metres=0.8, heading_degrees=0.0):
    return SimpleNamespace(arrival_seconds=arrival_seconds, forward_metres=forward_metres, left_metres=left_metres,
                           up_metres=up_metres, heading_degrees=heading_degrees)


def laser_reading(arrival_seconds, distance_metres):
    return SimpleNamespace(arrival_seconds=arrival_seconds, distance_metres=distance_metres, signal_quality_percent=-1)


# ---- Shared pieces ------------------------------------------------------------------------------

def test_heading_change_takes_the_short_way_round():
    assert heading_change_degrees(170.0, -170.0) == 20.0
    assert heading_change_degrees(-170.0, 170.0) == -20.0
    assert heading_change_degrees(0.0, 90.0) == 90.0


def test_combine_passes_only_if_every_part_passes_and_names_the_failures():
    assert combine([passed("A."), passed("B.")]).passed
    verdict = combine([passed("A."), failed("B broke.")])
    assert not verdict.passed and verdict.reason == "B broke."


def test_a_silent_flight_controller_gets_one_clear_sentence():
    assert px4_silence_problem(0.2) is None
    verdict = px4_silence_problem(10.0)
    assert not verdict.passed
    assert verdict.reason.startswith("No PX4 report for 10 s: is the flight controller powered (battery connected)?")
    assert "since the test started" in px4_silence_problem(math.inf).reason


# ---- Laser heights ------------------------------------------------------------------------------

def readings_at(tape_height, laser_height, px4_uses_laser=True, raw_heights=(), tilt_degrees=0.0, count=300):
    """count PX4 reports whose dist_bottom puts the laser's lens at laser_height."""
    distance_to_ground = laser_height + laser_heights.LASER_BELOW_CENTRE_METRES
    return laser_heights.ReadingsAtOneHeight(tape_height, [distance_to_ground] * count, [px4_uses_laser] * count,
                                             False, list(raw_heights), tilt_degrees)


def held_at_every_height(error_metres=0.0, lowest_height_px4_uses=0.0, raw_error_metres=None):
    held = []
    for height in laser_heights.HELD_HEIGHTS_METRES:
        raw = [] if raw_error_metres is None else [height + raw_error_metres] * 15
        held.append(readings_at(height, height + error_metres, height >= lowest_height_px4_uses, raw))
    return held


def test_a_laser_that_px4_uses_and_that_matches_the_tape_passes():
    verdict = laser_heights.judge_laser_heights(readings_at(None, 0.06), held_at_every_height(error_metres=0.02))
    assert verdict.passed, verdict.reason
    assert "PX4 used it from 0.10 m up" in verdict.reason


def test_the_result_says_from_what_height_px4_starts_using_the_laser():
    verdict = laser_heights.judge_laser_heights(readings_at(None, 0.06, px4_uses_laser=False),
                                                held_at_every_height(lowest_height_px4_uses=0.30))
    assert verdict.passed, verdict.reason
    assert verdict.measurements["heights_where_px4_used_the_laser_metres"] == [0.30, 0.50, 0.80, 1.20]
    lines = verdict.measurements["what_each_height_showed"]
    assert lines[0].startswith("On the floor: PX4 is NOT using the laser")
    assert "At 0.10 m: PX4 is NOT using the laser" in lines[1] and "from the barometer" in lines[1]
    assert lines[3].startswith("At 0.30 m: PX4 IS using the laser") and "tape 0.30 m" in lines[3]


def test_px4_never_using_the_laser_and_no_raw_reading_fails_with_what_to_check():
    verdict = laser_heights.judge_laser_heights(readings_at(None, 0.06, px4_uses_laser=False),
                                                held_at_every_height(lowest_height_px4_uses=9.0))
    assert not verdict.passed
    assert "PX4 never used the laser at any held height" in verdict.reason and "QGroundControl" in verdict.reason


def test_unknown_flags_count_as_not_using_the_laser():
    readings = readings_at(0.50, 0.50, px4_uses_laser=None)
    assert not readings.px4_uses_the_laser()


def test_a_laser_8_cm_off_at_one_height_fails_and_names_the_height():
    held = held_at_every_height()
    held[3] = readings_at(0.50, 0.58)
    verdict = laser_heights.judge_laser_heights(readings_at(None, 0.06), held)
    assert not verdict.passed and "At 0.50 m" in verdict.reason and "8 cm off" in verdict.reason


def test_the_raw_laser_alone_can_show_the_laser_works_and_is_corrected_for_tilt():
    verdict = laser_heights.judge_laser_heights(readings_at(None, 0.06, px4_uses_laser=False),
                                                held_at_every_height(lowest_height_px4_uses=9.0, raw_error_metres=0.01))
    assert verdict.passed, verdict.reason
    tilted = readings_at(0.50, 0.50, raw_heights=[0.50 / math.cos(math.radians(8.0))] * 15, tilt_degrees=8.0)
    assert laser_heights.judge_one_height(tilted).problem == ""


def test_a_raw_laser_10_cm_off_fails():
    verdict = laser_heights.judge_laser_heights(readings_at(None, 0.06),
                                                held_at_every_height(raw_error_metres=0.10))
    assert not verdict.passed and "the raw laser read" in verdict.reason


def test_missing_px4_reports_at_a_height_are_a_problem():
    outcome = laser_heights.judge_one_height(readings_at(0.30, 0.30, count=2))
    assert outcome.problem and not outcome.laser_was_seen


# ---- Directions ---------------------------------------------------------------------------------

def snapshots_for(tracking_start_heading=30.0, px4_forward_bearing=70.0, tracking_left_sign=1.0,
                  px4_left_turn_sign=-1.0, px4_turn=-90.0):
    """The four snapshots a correct drone gives for the moves A -> B (1 m forward), B -> C (1 m
    left) and a left turn at C. The tracking frame and PX4's north are turned by arbitrary angles,
    as on the real drone. The sign arguments let a test break one direction."""
    def tracking_position(forward, left):
        heading = math.radians(tracking_start_heading)
        left = tracking_left_sign * left
        return (forward * math.cos(heading) - left * math.sin(heading), forward * math.sin(heading) + left * math.cos(heading))

    def px4_position(forward, left):
        forward_bearing = math.radians(px4_forward_bearing)
        left_bearing = math.radians(px4_forward_bearing + px4_left_turn_sign * 90.0)
        north = forward * math.cos(forward_bearing) + left * math.cos(left_bearing)
        east = forward * math.sin(forward_bearing) + left * math.sin(left_bearing)
        return north, east

    def snapshot(forward, left, turned):
        tracking_forward, tracking_left = tracking_position(forward, left)
        north, east = px4_position(forward, left)
        return directions.Snapshot(tracking_forward, tracking_left, tracking_start_heading + (90.0 if turned else 0.0),
                                   north, east, 10.0 + (px4_turn if turned else 0.0))

    return snapshot(0, 0, False), snapshot(1, 0, False), snapshot(1, 1, False), snapshot(1, 1, True)


def test_correct_directions_pass_whatever_way_tracking_and_px4_started():
    verdict = directions.judge_directions(*snapshots_for())
    assert verdict.passed, verdict.reason
    verdict = directions.judge_directions(*snapshots_for(tracking_start_heading=-150.0, px4_forward_bearing=200.0))
    assert verdict.passed, verdict.reason


def test_tracking_with_left_and_right_swapped_fails():
    verdict = directions.judge_directions(*snapshots_for(tracking_left_sign=-1.0))
    assert not verdict.passed and "left move" in verdict.reason


def test_px4_with_left_and_right_swapped_fails():
    verdict = directions.judge_directions(*snapshots_for(px4_left_turn_sign=1.0))
    assert not verdict.passed and "swapped" in verdict.reason


def test_px4_turning_the_wrong_way_fails():
    verdict = directions.judge_directions(*snapshots_for(px4_turn=90.0))
    assert not verdict.passed and "PX4 saw the 90 degree left turn as +90" in verdict.reason


# ---- Touch tilt ---------------------------------------------------------------------------------

def attitude_samples(tilts_degrees, reports_per_second=50.0, rotation=5.0):
    return [(index / reports_per_second, tilt, rotation) for index, tilt in enumerate(tilts_degrees)]


def test_still_then_tilted_passes():
    held_still = touch_tilt.run_detector_over(attitude_samples([1.0] * 250), 5.0)
    tilted = touch_tilt.run_detector_over(attitude_samples([0.0] * 100 + [30.0] * 100 + [0.0] * 300), 10.0)
    verdict = touch_tilt.judge_touch_tilt(held_still, tilted)
    assert verdict.passed, verdict.reason


def test_a_false_alarm_while_still_fails():
    held_still = touch_tilt.run_detector_over(attitude_samples([22.0] * 250), 5.0)
    tilted = touch_tilt.run_detector_over(attitude_samples([30.0] * 500), 10.0)
    verdict = touch_tilt.judge_touch_tilt(held_still, tilted)
    assert not verdict.passed and "false alarm" in verdict.reason


def test_not_tilting_far_enough_fails_with_advice():
    held_still = touch_tilt.run_detector_over(attitude_samples([1.0] * 250), 5.0)
    tilted = touch_tilt.run_detector_over(attitude_samples([15.0] * 500), 10.0)
    verdict = touch_tilt.judge_touch_tilt(held_still, tilted)
    assert not verdict.passed and "only tilted 15 degrees" in verdict.reason


def test_too_few_attitude_reports_fail():
    held_still = touch_tilt.run_detector_over(attitude_samples([1.0] * 10, reports_per_second=2.0), 5.0)
    tilted = touch_tilt.run_detector_over(attitude_samples([30.0] * 500), 10.0)
    assert not touch_tilt.judge_touch_tilt(held_still, tilted).passed


# ---- Tunnel carry -------------------------------------------------------------------------------

def round_trip(length_metres=6.0, speed=0.5, reports_per_second=30.0, end_offset_metres=0.1):
    """Out along forward and back again, ending end_offset_metres to the left of the start."""
    reports = []
    seconds_each_way = length_metres / speed
    count_each_way = int(seconds_each_way * reports_per_second)
    for index in range(count_each_way):
        reports.append(tracking_report(index / reports_per_second, index / count_each_way * length_metres, 0.0))
    for index in range(count_each_way + 1):
        fraction = index / count_each_way
        reports.append(tracking_report(seconds_each_way + index / reports_per_second,
                                       length_metres * (1 - fraction), end_offset_metres * fraction))
    return reports


def test_a_clean_round_trip_passes_and_reports_the_drift():
    summary = tunnel_carry.summarise_carry(round_trip(), [1] * 100)
    verdict = tunnel_carry.judge_carry(summary)
    assert verdict.passed, verdict.reason
    assert abs(verdict.measurements["return_error_metres"] - 0.1) < 0.001
    assert abs(verdict.measurements["distance_carried_metres"] - 12.0) < 0.05


def test_a_gap_a_jump_a_lost_report_or_a_big_drift_each_fail():
    reports = round_trip()
    with_gap = reports[:100] + [tracking_report(r.arrival_seconds + 0.5, r.forward_metres, r.left_metres) for r in reports[100:]]
    with_jump = reports[:100] + [tracking_report(r.arrival_seconds, r.forward_metres, r.left_metres + 0.3) for r in reports[100:]]
    assert "stopped for 0.53 s" in tunnel_carry.judge_carry(tunnel_carry.summarise_carry(with_gap, [1])).reason
    assert "jumped" in tunnel_carry.judge_carry(tunnel_carry.summarise_carry(with_jump, [1])).reason
    assert "lost" in tunnel_carry.judge_carry(tunnel_carry.summarise_carry(reports, [1, 2, 1])).reason
    drifted = tunnel_carry.summarise_carry(round_trip(end_offset_metres=0.5), [1])
    assert "0.50 m from where it started" in tunnel_carry.judge_carry(drifted).reason


# ---- Laser over a step --------------------------------------------------------------------------

def walk_over_box(spot_width_metres, box_height=0.30, carry_height=0.80, speed=0.2, readings_per_second=5.0):
    """Walks from 1.5 m before a box (0.5 m long) to 1 m past it. The laser reading blends the
    floor and the box top in proportion to how much of its spot is over the box."""
    box_start, box_end = 1.5, 2.0
    laser = []
    tracking = []
    seconds = (box_end + 1.0) / speed
    for index in range(int(seconds * 30)):
        time_seconds = index / 30
        tracking.append(tracking_report(time_seconds, time_seconds * speed, 0.0))
    for index in range(int(seconds * readings_per_second)):
        time_seconds = index / readings_per_second
        position = time_seconds * speed
        half_spot = max(spot_width_metres / 2, 1e-6)
        overlap = max(0.0, min(position + half_spot, box_end) - max(position - half_spot, box_start))
        fraction_on_box = overlap / (2 * half_spot)
        laser.append(laser_reading(time_seconds, carry_height - fraction_on_box * box_height))
    return laser, tracking


def test_px4s_estimate_that_never_shows_the_box_fails_without_blaming_the_laser():
    laser, tracking = walk_over_box(spot_width_metres=0.01, box_height=0.0)
    verdict = laser_over_step.judge_step(laser_over_step.analyse_step(laser, tracking, 0.30), 0.30, from_px4_estimate=True)
    assert not verdict.passed and "PX4's dist_bottom never showed the box" in verdict.reason


def test_a_step_seen_in_px4s_estimate_gives_an_upper_bound():
    laser, tracking = walk_over_box(spot_width_metres=0.10)
    verdict = laser_over_step.judge_step(laser_over_step.analyse_step(laser, tracking, 0.30), 0.30, from_px4_estimate=True)
    assert verdict.passed and "at most this wide" in verdict.reason


def test_a_narrow_beam_shows_a_sudden_step():
    laser, tracking = walk_over_box(spot_width_metres=0.01)
    verdict = laser_over_step.judge_step(laser_over_step.analyse_step(laser, tracking, 0.30), 0.30)
    assert verdict.passed, verdict.reason
    for crossing in verdict.measurements["crossings"]:
        assert crossing["readings_in_between"] <= 1


def test_a_wide_beam_shows_a_gradual_step_of_about_its_width():
    laser, tracking = walk_over_box(spot_width_metres=0.38)
    analysis = laser_over_step.analyse_step(laser, tracking, 0.30)
    verdict = laser_over_step.judge_step(analysis, 0.30)
    assert verdict.passed, verdict.reason
    for crossing in analysis.crossings:
        narrowest, widest = crossing.spot_width_range_metres()
        # The middle 60% of a 0.38 m blend lies "in between", about 0.23 m, give or take one reading.
        assert narrowest - 0.05 <= 0.23 <= widest + 0.05


def test_a_step_that_does_not_match_the_box_fails():
    laser, tracking = walk_over_box(spot_width_metres=0.01, box_height=0.20)
    verdict = laser_over_step.judge_step(laser_over_step.analyse_step(laser, tracking, 0.30), 0.30)
    assert not verdict.passed


def test_walking_too_fast_to_see_the_box_fails_with_advice():
    laser, tracking = walk_over_box(spot_width_metres=0.01, speed=1.5)
    verdict = laser_over_step.judge_step(laser_over_step.analyse_step(laser, tracking, 0.30), 0.30)
    assert not verdict.passed and "Walk more slowly" in verdict.reason


def test_px4_laser_flag_sentence():
    assert "used the laser throughout" in laser_over_step.describe_px4_laser_flag([True, True, None])
    assert "not using the laser in 1 of 2" in laser_over_step.describe_px4_laser_flag([True, False])
    assert "never used the laser" in laser_over_step.describe_px4_laser_flag([False, False])
    assert "never arrived" in laser_over_step.describe_px4_laser_flag([None])


# ---- Report rates -------------------------------------------------------------------------------

def counts_with(rate_for):
    counts = []
    for report in report_rates.EXPECTED_REPORTS:
        rate = rate_for(report)
        counts.append(report_rates.ReportCount(report, report.topic, rate is not None, rate or 0.0,
                                               0.1 if rate else math.inf))
    return counts


def test_every_report_at_twice_its_lowest_rate_passes():
    assert report_rates.judge_report_rates(counts_with(lambda report: 2 * report.lowest_per_second)).passed


def test_a_silent_flight_controller_is_named_as_the_likely_cause():
    def rate(report):
        return 0.0 if report.topic.startswith("px4:") else 2 * report.lowest_per_second
    verdict = report_rates.judge_report_rates(counts_with(rate))
    assert not verdict.passed
    assert verdict.reason.startswith("No PX4 report for 10 s: is the flight controller powered (battery connected)?")


def test_missing_attitude_fails_but_a_missing_raw_laser_or_camera_pictures_do_not():
    def rate(report):
        if report.topic in ("px4:px4_attitude", "px4:laser") or report.topic.startswith("/oak/"):
            return None
        return 2 * report.lowest_per_second
    verdict = report_rates.judge_report_rates(counts_with(rate))
    assert not verdict.passed
    assert "PX4 attitude" in verdict.reason and "no program offers" in verdict.reason
    assert "Bottom laser" not in verdict.reason and "camera pictures" not in verdict.reason


# ---- Estimator flags ----------------------------------------------------------------------------

def expected_flags():
    return {name: value for name, value, _ in estimator_sources.EXPECTED_FLAGS}


def test_the_flags_drone_16s_settings_should_give_pass():
    assert estimator_sources.judge_estimator_flags(expected_flags()).passed


def test_px4_not_using_the_laser_for_height_fails_and_says_so():
    flags = expected_flags()
    flags["cs_rng_hgt"] = False
    verdict = estimator_sources.judge_estimator_flags(flags)
    assert not verdict.passed and "cs_rng_hgt is False" in verdict.reason


# ---- Interface settings -------------------------------------------------------------------------

def test_both_settings_false_and_no_demo_passes():
    settings = {"auto_offboard_on_arm": False, "allow_mode_request": False}
    assert interface_settings.judge_interface_settings(settings, ["flight_interface", "px4_odom_bridge"]).passed


def test_auto_offboard_on_arm_true_fails():
    settings = {"auto_offboard_on_arm": True, "allow_mode_request": False}
    verdict = interface_settings.judge_interface_settings(settings, ["flight_interface"])
    assert not verdict.passed and "auto_offboard_on_arm = True" in verdict.reason


def test_the_demo_running_or_no_answer_fails():
    settings = {"auto_offboard_on_arm": False, "allow_mode_request": False}
    assert not interface_settings.judge_interface_settings(settings, ["flight_interface", "square_mission"]).passed
    assert not interface_settings.judge_interface_settings(None, ["something_else"]).passed


# ---- Camera tracking held still -----------------------------------------------------------------

def still_reports(wander_metres=0.0, reports_per_second=30.0, seconds=30.0):
    count = int(reports_per_second * seconds)
    return [tracking_report(index / reports_per_second, wander_metres * index / count, 0.0) for index in range(count)]


def test_tracking_that_stays_put_passes():
    assert tracking_still_drift.judge_still_drift(still_reports(0.005), [1] * 10, 30.0).passed


def test_tracking_that_wanders_or_is_lost_or_slow_fails():
    assert "wandered up to 2.9 cm" in tracking_still_drift.judge_still_drift(still_reports(0.03), [1], 30.0).reason
    assert "lost" in tracking_still_drift.judge_still_drift(still_reports(), [1, 2], 30.0).reason
    assert "only 5.0 times" in tracking_still_drift.judge_still_drift(still_reports(reports_per_second=5.0), [1], 30.0).reason
