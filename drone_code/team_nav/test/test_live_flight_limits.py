"""Tests the limits that end live navigation early, without ROS or a flight: the run's time budget
(165 s after the hand-over) and the no-progress rule, both of which make the drone land where it
is. Also checks that the nose turns more slowly while the drone is moving, and that the depth
pictures are timed with the link's clock (so a test bench can run faster than real time).

How to run these tests is in the package's README.
"""
import math

import numpy as np
import pytest

from team_nav.course_run import LIVE_LANDING_RESERVE_SECONDS, SCORED_RUN_LIMIT_SECONDS
from team_nav.depth_to_map import DepthToMap
from team_nav.flight_steps import RunMustStop
from team_nav.live_flight import (
    FLYING_TURN_RATE_DEGREES_PER_SECOND, NO_PROGRESS_DISTANCE_METRES, NO_PROGRESS_SECONDS, LiveFlight,
)
from team_nav.live_planner import Goal

HAND_OVER_SECONDS = 100.0


class PretendLink:
    def __init__(self):
        self.seconds = 0.0

    def now_seconds(self):
        return self.seconds


class PretendNavigator:
    """Only what the limits read: the goals and which one the drone is flying to."""

    def __init__(self):
        self.goals = [Goal("first corridor", np.array([0.0, 2.0]), None, None),
                      Goal("maze", np.array([2.0, 2.0]), None, None)]
        self.goal_number = 0

    @property
    def current_goal(self):
        return self.goals[self.goal_number]


def live_flight_with_limits():
    """A LiveFlight with only what the limits use, and the deadline course_run.py gives it."""
    flight = LiveFlight.__new__(LiveFlight)
    flight.link = PretendLink()
    flight.navigator = PretendNavigator()
    flight.latest_navigation_end_seconds = HAND_OVER_SECONDS + SCORED_RUN_LIMIT_SECONDS - LIVE_LANDING_RESERVE_SECONDS
    flight.stages_finished = []
    flight.progress_goal_number = None
    flight.nearest_to_goal_metres = None
    flight.nearest_to_goal_seconds = None
    flight.commanded_heading_radians = None
    flight.last_cycle_seconds = None
    return flight


def test_navigation_stops_165_seconds_after_the_hand_over_and_names_the_cleared_checkpoints():
    flight = live_flight_with_limits()
    flight.stages_finished = ["first corridor", "maze"]
    flight._stop_if_out_of_time(HAND_OVER_SECONDS + 164.9)
    with pytest.raises(RunMustStop, match="Checkpoints cleared: first corridor, maze."):
        flight._stop_if_out_of_time(HAND_OVER_SECONDS + 165.0)


def test_a_drone_that_makes_no_progress_lands():
    flight = live_flight_with_limits()
    flight._stop_if_stuck(np.array([0.0, 0.0, 0.8]), 0.0)
    # Less than NO_PROGRESS_DISTANCE_METRES nearer: that is not progress.
    almost_there = np.array([0.0, NO_PROGRESS_DISTANCE_METRES - 0.05, 0.8])
    flight._stop_if_stuck(almost_there, NO_PROGRESS_SECONDS - 0.1)
    with pytest.raises(RunMustStop, match="No checkpoint was cleared."):
        flight._stop_if_stuck(almost_there, NO_PROGRESS_SECONDS + 0.1)


def test_progress_or_a_new_goal_starts_the_count_again():
    flight = live_flight_with_limits()
    flight._stop_if_stuck(np.array([0.0, 0.0, 0.8]), 0.0)
    flight._stop_if_stuck(np.array([0.0, NO_PROGRESS_DISTANCE_METRES + 0.05, 0.8]), 20.0)
    flight._stop_if_stuck(np.array([0.0, NO_PROGRESS_DISTANCE_METRES + 0.05, 0.8]), 40.0)
    flight.navigator.goal_number = 1
    flight._stop_if_stuck(np.array([0.0, NO_PROGRESS_DISTANCE_METRES + 0.05, 0.8]), 50.0)
    flight._stop_if_stuck(np.array([0.0, NO_PROGRESS_DISTANCE_METRES + 0.05, 0.8]), 50.0 + NO_PROGRESS_SECONDS - 0.1)


def test_the_nose_turns_slowly_while_the_drone_moves():
    flight = live_flight_with_limits()
    flight._turned_gently_toward(math.pi / 2, 0.0, 0.0, FLYING_TURN_RATE_DEGREES_PER_SECOND)
    heading = flight._turned_gently_toward(math.pi / 2, 0.0, 0.1, FLYING_TURN_RATE_DEGREES_PER_SECOND)
    assert math.degrees(heading) == pytest.approx(FLYING_TURN_RATE_DEGREES_PER_SECOND * 0.1)


def test_depth_pictures_are_timed_with_the_links_clock():
    depth = DepthToMap.__new__(DepthToMap)
    depth.link = PretendLink()
    depth.last_picture_seconds = None
    assert depth.seconds_since_last_picture() == math.inf
    depth.last_picture_seconds = 10.0
    depth.link.seconds = 11.5
    assert depth.seconds_since_last_picture() == pytest.approx(1.5)
