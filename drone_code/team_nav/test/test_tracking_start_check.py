"""Tests the start check (tracking_start_check.py) and that full_course refuses to start on it
before sending anything. No ROS messages are sent."""
import numpy as np
import pytest

from team_nav.course_run import EXIT_REFUSED_TO_START
from team_nav.practice_flights import full_course
from team_nav.practice_flights.full_course import FullCourseFlight, additions_from_arguments, parse_arguments
from team_nav.practice_flights.tracking_start_check import (
    LARGEST_HEIGHT_OFFSET_METRES, LARGEST_HORIZONTAL_OFFSET_METRES, problem_with_tracking_on_the_launch_mark,
)
from test_flight_steps import PretendLink


@pytest.mark.parametrize('tracked', [
    (0.0, 0.0, 0.0),
    (-0.005, 0.0014, -0.0025),          # What the Gazebo pretend tracking read on the mark (run 20261108_122536).
    (0.18, -0.24, 0.09),                # 0.30 m sideways exactly, 9 cm up.
    (0.0, 0.0, -0.10),
])
def test_tracking_close_to_where_it_started_passes(tracked):
    assert problem_with_tracking_on_the_launch_mark(tracked) is None


@pytest.mark.parametrize('tracked', [
    (0.31, 0.0, 0.0),                   # Just over 0.30 m forward.
    (0.0, -0.25, 0.0) + np.array([0.18, 0.0, 0.0]),     # 0.31 m diagonally.
    (0.0, 0.0, 0.11),                   # Just over 0.10 m up.
    (0.0, 0.0, -0.11),                  # Just over 0.10 m down.
    (1.5, 0.4, 0.0),                    # Carried across the room after tracking started.
])
def test_tracking_far_from_where_it_started_refuses_with_the_fix(tracked):
    problem = problem_with_tracking_on_the_launch_mark(tracked)
    assert problem is not None
    assert f'{LARGEST_HORIZONTAL_OFFSET_METRES:.2f} m sideways' in problem
    assert f'{LARGEST_HEIGHT_OFFSET_METRES:.2f} m in height' in problem
    assert 'restart the drone software' in problem
    assert 'launch mark' in problem


def test_no_tracking_report_refuses():
    assert 'has not reported' in problem_with_tracking_on_the_launch_mark(None)


def make_flight_on_the_ground(tracked_position, words=()):
    link = PretendLink(height_metres=0.0)
    link.tracked_position = lambda: np.array(tracked_position, dtype=float)
    flight = object.__new__(FullCourseFlight)
    flight.link = link
    flight.log = link.log
    flight.additions = additions_from_arguments(parse_arguments(list(words)))
    return flight, link


def test_flight_refuses_to_start_before_sending_anything(monkeypatch):
    monkeypatch.setattr(full_course.flight_steps, 'wait_until_ready_on_the_ground', lambda link: None)
    flight, link = make_flight_on_the_ground([0.5, 0.0, 0.0])
    flight.fly = lambda: pytest.fail('The flight must not start after a failed start check.')
    assert flight.run() == EXIT_REFUSED_TO_START
    assert link.targets.sent == []
    kinds = [kind for kind, _ in link.log.events]
    assert kinds[-1] == 'refused'
    assert 'restart the drone software' in link.log.events[-1][1]


def test_switched_off_start_check_lets_the_flight_start(monkeypatch):
    monkeypatch.setattr(full_course.flight_steps, 'wait_until_ready_on_the_ground', lambda link: None)
    flight, link = make_flight_on_the_ground([0.5, 0.0, 0.0], words=['--no-start-check'])
    flown = []
    flight.fly = lambda: flown.append(True)
    assert flight.run() == 0
    assert flown == [True]
