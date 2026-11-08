"""Check the live navigator's turn and approach decisions without ROS or a flight.

A turn on the spot starts only when the drone is nearly still and the same new direction has been
asked for over TURN_AGREEMENT_SECONDS; until then the drone holds where it is (STOPPING_TO_TURN).
"""

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
    STOPPING_TO_TURN,
    TURN_AGREEMENT_SECONDS,
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
    # These tests set the path by hand, so the navigator must not plan its own until a turn ends.
    navigator.last_plan_seconds = math.inf
    return navigator, live_map, np.append(start_metres, 0.8)


def step_until_turning(navigator, drone_metres, heading, start_seconds):
    """Steps a still drone, once per 0.1 s cycle, until the navigator starts the turn. Until then it
    must hold where the drone is, with the nose unchanged."""
    for cycle in range(30):
        step = navigator.next_step(drone_metres, heading, start_seconds + cycle * 0.1, depth_map_updates=cycle)
        if step.activity == TURNING:
            return step, start_seconds + cycle * 0.1
        assert step.activity == STOPPING_TO_TURN
        assert step.heading_radians == heading
        np.testing.assert_allclose(step.target_metres[:2], drone_metres[:2])
    raise AssertionError("The navigator never started the turn although the drone was still and the path steady.")


def test_turn_keeps_its_heading_and_waits_for_new_depth(turn_setup):
    navigator, _, drone_metres = turn_setup
    navigator.path = [drone_metres[:2] + [0.0, 0.6]]
    first, now_seconds = step_until_turning(navigator, drone_metres, 0.0, 0.0)
    assert now_seconds >= TURN_AGREEMENT_SECONDS
    assert first.heading_radians == pytest.approx(math.pi / 2)
    np.testing.assert_allclose(first.target_metres, drone_metres)

    navigator.path = None
    still_turning = navigator.next_step(drone_metres, 0.4, now_seconds + 0.2, depth_map_updates=100)
    assert still_turning.heading_radians == first.heading_radians
    for extra_seconds in (0.3, 0.4):
        step = navigator.next_step(drone_metres, math.pi / 2, now_seconds + extra_seconds, depth_map_updates=101)
        assert step.activity == WAITING_FOR_TURN_VIEW
        assert navigator.turn_heading_radians is not None

    navigator.next_step(drone_metres, math.pi / 2, now_seconds + 0.5, depth_map_updates=102)
    assert navigator.turn_heading_radians is None


def test_moves_forward_to_a_visible_turn_pocket_before_rotating(turn_setup):
    navigator, live_map, drone_metres = turn_setup
    start_metres = drone_metres[:2]
    # At the start there is room for the body, but less than the observed turn slide requires.
    live_map.scores[live_map.square_of(start_metres + [0.0, 0.45])] = HIGHEST_SCORE
    pocket_metres = start_metres + [0.34, 0.0]
    navigator.path = [start_metres, pocket_metres, pocket_metres + [0.0, 0.6]]

    approach = navigator.next_step(drone_metres, 0.0, 0.0)
    assert approach.activity == APPROACHING_TURN
    assert approach.heading_radians == 0.0
    np.testing.assert_allclose(approach.target_metres[:2], pocket_metres)

    # The drone has just moved, so it first stops at the pocket, then turns there.
    turn, _now_seconds = step_until_turning(navigator, approach.target_metres, 0.0, 0.2)
    assert turn.heading_radians == pytest.approx(math.pi / 2)
    np.testing.assert_allclose(turn.target_metres[:2], pocket_metres)


def test_lands_if_the_drone_cannot_regain_the_turn_hold_point(turn_setup):
    navigator, _, drone_metres = turn_setup
    navigator.path = [drone_metres[:2] + [0.0, 0.6]]
    _turn, now_seconds = step_until_turning(navigator, drone_metres, 0.0, 0.0)

    slid_metres = drone_metres + [0.3, 0.0, 0.0]
    navigator.next_step(slid_metres, math.pi / 2, now_seconds + 1.0)
    with pytest.raises(NoWayForward, match="could not return to its hold point"):
        navigator.next_step(slid_metres, math.pi / 2, now_seconds + 1.0 + LONGEST_SETTLING_SECONDS)


def test_a_direction_that_flips_between_plans_does_not_start_a_turn(turn_setup):
    # As in the 10:44 run of 2026-11-08: one plan asked for a heading 60 degrees the wrong way.
    navigator, _, drone_metres = turn_setup
    north_path = [drone_metres[:2] + [0.0, 0.6]]
    north_west_path = [drone_metres[:2] + [-0.6, 0.6]]
    for cycle in range(20):
        navigator.path = north_path if (cycle // 3) % 2 == 0 else north_west_path
        step = navigator.next_step(drone_metres, 0.0, cycle * 0.1)
        assert step.activity == STOPPING_TO_TURN
        assert step.heading_radians == 0.0


def test_a_moving_drone_stops_before_it_turns(turn_setup):
    navigator, _, drone_metres = turn_setup
    navigator.path = [drone_metres[:2] + [0.0, 0.6]]
    # Drifting east at 0.2 m/s, faster than a turn may start at.
    for cycle in range(10):
        moving_metres = drone_metres + [0.02 * cycle, 0.0, 0.0]
        step = navigator.next_step(moving_metres, 0.0, cycle * 0.1)
        assert step.activity == STOPPING_TO_TURN
    # Now it is still at the hold point: the turn starts.
    held_metres = step.target_metres
    turn, _now_seconds = step_until_turning(navigator, held_metres, 0.0, 1.0)
    assert turn.heading_radians == pytest.approx(math.pi / 2)


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
