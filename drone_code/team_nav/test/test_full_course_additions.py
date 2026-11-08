"""Tests the four additions to full_course (2026-11-08) with a pretend drone. No ROS messages are sent.

  - the settings: each addition is on unless switched off, and --baseline gives Aditya's version;
  - the first checkpoint's turn-back point, recomputed from the sketch map;
  - the new arrival rule, and the first checkpoint's "whole drone past the line, no pause" rule;
  - the steadiness rule both of those use.

The stop options are in test_full_course_stops.py and the start check in test_tracking_start_check.py.
Gazebo must still check the real aircraft's response.
"""
import math
from types import SimpleNamespace

import numpy as np
import pytest

from our_files import load_our_map
from team_nav.practice_flights.full_course import (
    ARRIVAL_RULE, FullCourseFlight, additions_from_arguments, parse_arguments, plan_the_flight,
)
from team_nav.practice_flights.full_course_route import (
    BASELINE_FIRST_CHECKPOINT_NORTH_METRES, FOOTPRINT_NORTH_METRES, FOOTPRINT_SOUTH_METRES,
    MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES, footprint_distance_past_line, footprint_points, route_waypoints,
)
from team_nav.practice_flights.steadiness import SteadinessRule, SteadinessWatch
from real_course_route import real_course_mission
from test_flight_steps import PretendLink

SKETCH_MAP = 'sketch_2026-11-08'


# ---- Settings -----------------------------------------------------------------------------------

def test_every_addition_is_on_unless_switched_off():
    additions = additions_from_arguments(parse_arguments([]))
    assert additions.start_check and additions.takeoff_settle and additions.stop_options and additions.new_arrival_rule


def test_each_addition_has_its_own_switch():
    additions = additions_from_arguments(parse_arguments(['--no-takeoff-settle']))
    assert not additions.takeoff_settle
    assert additions.start_check and additions.stop_options and additions.new_arrival_rule


def test_baseline_switches_off_all_four():
    additions = additions_from_arguments(parse_arguments(['--baseline']))
    assert not (additions.start_check or additions.takeoff_settle or additions.stop_options
                or additions.new_arrival_rule)


def test_baseline_flies_the_original_route_and_refuses_stop_options():
    course_map = load_our_map(SKETCH_MAP)
    baseline = additions_from_arguments(parse_arguments(['--baseline']))
    plan = plan_the_flight(course_map, baseline, '', fixed_sketch_route=True)
    assert plan.route_to_fly_metres[1, 1] == BASELINE_FIRST_CHECKPOINT_NORTH_METRES
    with pytest.raises(ValueError, match='stop options are switched off'):
        plan_the_flight(course_map, baseline, 'tunnel', fixed_sketch_route=True)


def test_new_rule_turns_back_in_the_middle_of_the_first_checkpoint_room():
    course_map = load_our_map('real_course_2026-11-08')
    plan = plan_the_flight(course_map, additions_from_arguments(parse_arguments([])), '',
                           mission=real_course_mission())
    assert plan.route_to_fly_metres[1, 1] == MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES
    assert plan.stop is None


# ---- The first checkpoint's turn-back point -------------------------------------------------------

def test_turn_back_point_is_the_middle_of_the_room_on_the_sketch_map():
    """Recomputes 2.515 m from the map: halfway between the whole drone being past the line and
    the propellers touching the top wall."""
    course_map = load_our_map(SKETCH_MAP)
    line = course_map.checkpoint_line_named('end of first corridor')
    top_wall = course_map.wall_named('Sketch top wall (north-west)')
    whole_drone_past_north = line.start_metres[1] + FOOTPRINT_SOUTH_METRES
    touching_north = top_wall.start_metres[1] - top_wall.thickness_metres / 2 - FOOTPRINT_NORTH_METRES
    assert MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES == pytest.approx(
        (whole_drone_past_north + touching_north) / 2, abs=0.001)


def test_route_through_the_middle_of_the_room_passes_every_route_check():
    points = route_waypoints(load_our_map(SKETCH_MAP), MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES)
    assert points[1, :2] == pytest.approx([-2.51, 2.515])


# ---- The steadiness rule ------------------------------------------------------------------------

RULE = SteadinessRule(largest_horizontal_error_metres=0.08, largest_height_error_metres=0.08,
                      largest_distance_metres=math.inf, largest_speed_metres_per_second=0.05, seconds_held=0.5)
TARGET = np.array([1.0, 2.0, 0.75])


def test_steady_only_after_the_whole_hold_time():
    watch = SteadinessWatch(RULE)
    close = TARGET + [0.05, 0.05, 0.05]
    answers = [watch.update(seconds, close, TARGET, 0.02) for seconds in (0.0, 0.2, 0.4, 0.5)]
    assert answers == [False, False, False, True]


def test_one_fast_reading_starts_the_hold_again():
    watch = SteadinessWatch(RULE)
    assert not watch.update(0.0, TARGET, TARGET, 0.02)
    assert not watch.update(0.4, TARGET, TARGET, 0.06)
    assert not watch.update(0.5, TARGET, TARGET, 0.02)
    assert not watch.update(0.9, TARGET, TARGET, 0.02)
    assert watch.update(1.0, TARGET, TARGET, 0.02)


@pytest.mark.parametrize('position,speed', [
    (TARGET + [0.09, 0.0, 0.0], 0.0),       # too far sideways
    (TARGET + [0.0, 0.0, -0.09], 0.0),      # too low
    (TARGET, 0.05),                         # not below the speed limit
    (None, 0.0),                            # no position
    (TARGET, None),                         # no speed
])
def test_readings_outside_the_rule_are_never_steady(position, speed):
    watch = SteadinessWatch(RULE)
    for seconds in np.arange(0.0, 2.0, 0.1):
        assert not watch.update(seconds, position, TARGET, speed)


def test_straight_line_limit_counts_height_and_sideways_together():
    rule = SteadinessRule(math.inf, math.inf, 0.10, 0.05, 0.0)
    assert rule.is_met(TARGET + [0.07, 0.0, 0.07], TARGET, 0.0)
    assert not rule.is_met(TARGET + [0.08, 0.0, 0.08], TARGET, 0.0)


# ---- Arrival, with a pretend drone -----------------------------------------------------------------

def pretend_flight_on_a_slow_drone(start_position, start_seconds_before_moving=3.0):
    """A FullCourseFlight whose drone starts moving after a delay, then follows the target slowly.
    Returns (flight, position, speeds), where speeds holds each cycle's speed."""
    link = PretendLink(height_metres=0.75)
    position = np.array(start_position, dtype=float)
    last_target = position.copy()
    speeds = []

    def send(target, heading_on_course_radians):
        last_target[:] = target

    def cycle():
        before = position.copy()
        if link.seconds >= start_seconds_before_moving:
            position[:] += 0.08 * (last_target - position)
        speeds.append(float(np.linalg.norm(position - before)) / 0.1)
        link.seconds += 0.1

    link.wait_one_cycle = cycle
    link.px4_speed_metres_per_second = lambda: speeds[-1] if speeds else 0.0
    flight = object.__new__(FullCourseFlight)
    flight.link = link
    flight.course_map = load_our_map(SKETCH_MAP)
    flight.body = footprint_points()
    flight.watch = SimpleNamespace(drone_on_course=lambda: position.copy(), check=lambda: False,
                                   send_target_on_course=send)
    return flight, position, speeds


def test_new_arrival_rule_waits_until_close_and_slow_for_half_a_second():
    flight, position, speeds = pretend_flight_on_a_slow_drone([0.0, 0.0, 0.75])
    end = np.array([1.0, 0.0, 0.75])
    flight.fly_leg(position.copy(), end, flight.steady_arrival_at(end))
    assert np.linalg.norm(position[:2] - end[:2]) <= ARRIVAL_RULE.largest_horizontal_error_metres
    # The last 0.5 s of readings (6 cycles) were all slow.
    assert max(speeds[-6:]) < ARRIVAL_RULE.largest_speed_metres_per_second


def test_first_checkpoint_counts_as_soon_as_the_whole_drone_is_past_the_line():
    course_map = load_our_map(SKETCH_MAP)
    route = route_waypoints(course_map, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES)
    flight, position, speeds = pretend_flight_on_a_slow_drone(route[0], start_seconds_before_moving=0.0)
    flight.fly_leg(route[0], route[1], flight.whole_drone_past_the_first_line())
    line = course_map.checkpoint_line_named('end of first corridor')
    distance_past_metres, _ = footprint_distance_past_line(position, footprint_points(), line)
    assert 0.02 < distance_past_metres < 0.05
    # It did not wait to reach the turn-back point or to slow down.
    assert position[1] < route[1, 1] - 0.03
    assert speeds[-1] > ARRIVAL_RULE.largest_speed_metres_per_second


def test_leg_1_uses_the_steady_rule_when_the_flight_lands_there():
    flight, _, _ = pretend_flight_on_a_slow_drone([0.0, 0.0, 0.75])
    flight.additions = additions_from_arguments(parse_arguments([]))
    end = np.array([-2.51, 2.515, 0.75])
    landing_test = flight.arrival_test_for_leg(1, end, is_last_leg=True)
    passing_test = flight.arrival_test_for_leg(1, end, is_last_leg=False)
    assert landing_test.__qualname__.startswith('FullCourseFlight.steady_arrival_at')
    assert passing_test.__qualname__.startswith('FullCourseFlight.whole_drone_past_the_first_line')
