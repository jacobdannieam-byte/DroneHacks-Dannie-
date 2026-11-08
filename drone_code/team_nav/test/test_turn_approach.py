"""Check the live navigator's turn and approach decisions without ROS or a flight."""

import math

import numpy as np
import pytest

from our_files import load_our_map
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES
from team_nav.live_map import HIGHEST_SCORE, LOWEST_SCORE, LiveMap
from team_nav.live_navigation import (
    APPROACHING_TURN,
    HOLDING,
    LONGEST_SETTLING_SECONDS,
    LONGEST_SECONDS_WITHOUT_A_PATH,
    TURNING,
    WAITING_FOR_TURN_VIEW,
    LiveNavigator,
    NoWayForward,
)
from team_nav.live_planner import Goal


@pytest.fixture
def turn_setup():
    live_map = LiveMap(load_our_map("sketch_2026-11-08"), 0.8, DRONE_HALF_HEIGHT_METRES)
    live_map.scores.fill(LOWEST_SCORE)
    start_metres = live_map.centre_of(live_map.square_of([0.0, 0.0]))
    goal = Goal("maze", start_metres + [0.0, 1.2], None, None)
    navigator = LiveNavigator(live_map, [goal], 0.8)
    navigator.last_plan_seconds = 0.0
    return navigator, live_map, np.append(start_metres, 0.8)


def test_turn_keeps_its_heading_and_waits_for_new_depth(turn_setup):
    navigator, _, drone_metres = turn_setup
    navigator.path = [drone_metres[:2] + [0.0, 0.6]]
    first = navigator.next_step(drone_metres, 0.0, 0.0, depth_map_updates=10)
    assert first.activity == TURNING
    assert first.heading_radians == pytest.approx(math.pi / 2)
    np.testing.assert_allclose(first.target_metres, drone_metres)

    navigator.path = None
    still_turning = navigator.next_step(drone_metres, 0.4, 0.2, depth_map_updates=11)
    assert still_turning.heading_radians == first.heading_radians
    for now_seconds in (0.3, 0.4):
        step = navigator.next_step(drone_metres, math.pi / 2, now_seconds, depth_map_updates=12)
        assert step.activity == WAITING_FOR_TURN_VIEW
        assert navigator.turn_heading_radians is not None

    navigator.next_step(drone_metres, math.pi / 2, 0.5, depth_map_updates=13)
    assert navigator.turn_heading_radians is None


def test_moves_forward_to_a_visible_turn_pocket_before_rotating(turn_setup):
    navigator, live_map, drone_metres = turn_setup
    start_metres = drone_metres[:2]
    # At the start there is room for the body, but less than the observed turn slide requires.
    live_map.scores[live_map.square_of(start_metres + [0.0, 0.8])] = HIGHEST_SCORE
    pocket_metres = start_metres + [0.34, 0.0]
    navigator.path = [start_metres, pocket_metres, pocket_metres + [0.0, 0.6]]

    approach = navigator.next_step(drone_metres, 0.0, 0.0)
    assert approach.activity == APPROACHING_TURN
    assert approach.heading_radians == 0.0
    np.testing.assert_allclose(approach.target_metres[:2], pocket_metres)

    turn = navigator.next_step(approach.target_metres, 0.0, 0.2)
    assert turn.activity == TURNING
    assert turn.heading_radians == pytest.approx(math.pi / 2)


def test_lands_if_the_drone_cannot_regain_the_turn_hold_point(turn_setup):
    navigator, _, drone_metres = turn_setup
    navigator.path = [drone_metres[:2] + [0.0, 0.6]]
    assert navigator.next_step(drone_metres, 0.0, 0.0).activity == TURNING

    slid_metres = drone_metres + [0.3, 0.0, 0.0]
    navigator.next_step(slid_metres, math.pi / 2, 1.0)
    with pytest.raises(NoWayForward, match="could not return to its hold point"):
        navigator.next_step(slid_metres, math.pi / 2, 1.0 + LONGEST_SETTLING_SECONDS)


def test_holds_when_no_route_is_available(turn_setup):
    navigator, live_map, drone_metres = turn_setup
    step = navigator.next_step(drone_metres, 0.2, 0.0)
    assert step.activity == HOLDING
    assert step.heading_radians == 0.2
    np.testing.assert_allclose(step.target_metres, drone_metres)
    live_map.scores.fill(HIGHEST_SCORE)
    live_map.hit_by_camera.fill(True)
    with pytest.raises(NoWayForward, match="no route"):
        navigator.next_step(drone_metres, 0.2, LONGEST_SECONDS_WITHOUT_A_PATH + 0.1)
