"""Tests practice_flights/response_measurements.py with made-up drones whose answers we know: a
perfect first-order system (section 9's model), one with a dead time, one that overshoots, and
one that never gets there.

How to run these tests is in the package's README.
"""
import math

import pytest

from team_nav.practice_flights.response_measurements import (
    SETTLED_TOLERANCE_METRES, PositionSample, measure_step_response, measure_trailing_distance,
)

# Made up for the tests: about the time constant section 9 estimates.
TIME_CONSTANT_SECONDS = 1.3
# The flight's rhythm: one sample every tenth of a second.
SAMPLE_SECONDS = 0.1


def first_order_step(step_metres, seconds_long, dead_time_seconds=0.0, start_metres=0.0):
    """Samples of a first-order system following a step that starts at time 0."""
    samples = []
    for number in range(int(round(seconds_long / SAMPLE_SECONDS)) + 1):
        seconds = number * SAMPLE_SECONDS
        moving_seconds = max(0.0, seconds - dead_time_seconds)
        covered = 1.0 - math.exp(-moving_seconds / TIME_CONSTANT_SECONDS)
        samples.append(PositionSample(seconds, start_metres + step_metres, start_metres + step_metres * covered))
    return samples


def test_a_first_order_drone_takes_one_time_constant_to_cover_63_percent():
    step = measure_step_response(first_order_step(1.0, 8.0), 0.0, 0.0, 1.0)
    # Straight lines between samples 0.1 s apart put the crossing within a few milliseconds.
    assert step.delay_to_63_percent_seconds == pytest.approx(TIME_CONSTANT_SECONDS, abs=0.01)
    # Within 5 cm of a 1 m step means 95% covered: ln(20) time constants, rounded up to a sample.
    assert step.seconds_until_settled == pytest.approx(TIME_CONSTANT_SECONDS * math.log(20.0), abs=SAMPLE_SECONDS)
    assert step.overshoot_metres == 0.0


def test_a_dead_time_adds_to_the_delay():
    step = measure_step_response(first_order_step(1.0, 8.0, dead_time_seconds=0.3), 0.0, 0.0, 1.0)
    assert step.delay_to_63_percent_seconds == pytest.approx(TIME_CONSTANT_SECONDS + 0.3, abs=0.01)


def test_a_step_backwards_is_measured_the_same_way():
    step = measure_step_response(first_order_step(-1.0, 8.0, start_metres=1.0), 0.0, 1.0, 0.0)
    assert step.step_metres == -1.0
    assert step.delay_to_63_percent_seconds == pytest.approx(TIME_CONSTANT_SECONDS, abs=0.01)


def test_an_overshoot_is_reported_and_settling_waits_for_the_last_excursion():
    samples = [PositionSample(0.0, 1.0, 0.0), PositionSample(1.0, 1.0, 0.8), PositionSample(2.0, 1.0, 1.12),
               PositionSample(3.0, 1.0, 1.03), PositionSample(4.0, 1.0, 0.94), PositionSample(5.0, 1.0, 1.01)]
    step = measure_step_response(samples, 0.0, 0.0, 1.0)
    assert step.overshoot_metres == pytest.approx(0.12)
    assert step.seconds_until_settled == 5.0
    assert abs(1.01 - 1.0) <= SETTLED_TOLERANCE_METRES


def test_a_drone_that_never_gets_there_has_no_delay_and_never_settles():
    samples = [PositionSample(seconds, 1.0, 0.1 * seconds) for seconds in range(6)]
    step = measure_step_response(samples, 0.0, 0.0, 1.0)
    assert step.delay_to_63_percent_seconds is None
    assert step.seconds_until_settled is None
    assert "never" in step.describe()


def test_a_step_of_no_length_is_refused():
    with pytest.raises(ValueError, match="no length"):
        measure_step_response([PositionSample(0.0, 0.0, 0.0)], 0.0, 0.0, 0.0)


def first_order_slide_backwards(speed_metres_per_second, seconds_long):
    """A first-order drone, starting settled at 1 m, following a target that slides back."""
    samples = []
    drone_metres = 1.0
    for number in range(int(round(seconds_long / SAMPLE_SECONDS)) + 1):
        seconds = number * SAMPLE_SECONDS
        target_metres = 1.0 - speed_metres_per_second * seconds
        samples.append(PositionSample(seconds, target_metres, drone_metres))
        # One small step of the filter: the drone closes the gap at rate gap / time constant.
        drone_metres += (target_metres - drone_metres) * SAMPLE_SECONDS / TIME_CONSTANT_SECONDS
    return samples


def test_a_first_order_drone_trails_a_sliding_target_by_speed_times_time_constant():
    speed = 0.25
    slide_seconds = 12.0     # Long enough to become fully steady, unlike the 4 s flight.
    trailing = measure_trailing_distance(first_order_slide_backwards(speed, slide_seconds), 0.0, slide_seconds, -speed)
    # The made-up drone steps forward once per sample, which adds about half a sample of delay.
    expected_seconds = TIME_CONSTANT_SECONDS + SAMPLE_SECONDS / 2
    assert trailing.trailing_seconds == pytest.approx(expected_seconds, rel=0.1)
    assert trailing.average_trailing_metres == pytest.approx(speed * expected_seconds, rel=0.1)
    assert trailing.largest_trailing_metres >= trailing.average_trailing_metres


def test_a_slide_with_no_samples_says_so():
    trailing = measure_trailing_distance([], 0.0, 4.0, 0.25)
    assert trailing.average_trailing_metres is None
    assert "not enough samples" in trailing.describe()
