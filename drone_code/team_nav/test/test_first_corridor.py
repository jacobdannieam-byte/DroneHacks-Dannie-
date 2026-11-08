"""Check first-corridor targets on the team's course map without ROS or a flight."""

import math
from dataclasses import replace

import numpy as np
import pytest

from our_files import load_our_map, load_side_gap_first_mission
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES
from team_nav.live_map import HIGHEST_SCORE, LOWEST_SCORE, LiveMap
from team_nav.live_navigation import (
    CORRIDOR_BLOCKED,
    CORRIDOR_CONFIRMING_EXIT,
    CORRIDOR_EXIT_CONFIRM_SECONDS,
    CORRIDOR_LEAD_METRES,
    FLYING,
    HOLDING,
    LONGEST_SECONDS_WITHOUT_A_PATH,
    LiveNavigator,
    NoWayForward,
)
from team_nav.live_planner import goals_from_mission


@pytest.fixture
def corridor():
    course_map = load_our_map("sketch_2026-11-08")
    mission = load_side_gap_first_mission()
    goals = goals_from_mission(course_map, mission, "to the landing pad")
    height_metres = mission.cruise_height_metres
    live_map = LiveMap(course_map, height_metres, DRONE_HALF_HEIGHT_METRES)
    navigator = LiveNavigator(live_map, goals, height_metres, stop_after_stage="first corridor")
    drone_metres = np.append(course_map.launch_mark_centre_metres, height_metres)
    return navigator, live_map, drone_metres


def test_corridor_holds_without_depth_evidence(corridor):
    navigator, _, drone_metres = corridor
    heading = navigator.current_goal.straight_heading_radians
    step = navigator.next_step(drone_metres, heading, 0.0)
    assert step.activity == CORRIDOR_BLOCKED
    np.testing.assert_allclose(step.target_metres, drone_metres)


def test_corridor_holds_if_it_needs_a_sharp_turn_too_close_to_the_net(corridor):
    navigator, _, drone_metres = corridor
    step = navigator.next_step(drone_metres, 0.0, 0.0)
    assert step.activity == HOLDING
    assert step.heading_radians == 0.0
    np.testing.assert_allclose(step.target_metres, drone_metres)


def test_a_blocked_cell_causes_a_hold_then_forward_motion_if_it_clears(corridor):
    navigator, live_map, drone_metres = corridor
    heading = navigator.current_goal.straight_heading_radians
    live_map.scores[~live_map.blocked()] = LOWEST_SCORE
    square = live_map.square_of(drone_metres[:2] + [0.0, 0.45])
    live_map.scores[square] = HIGHEST_SCORE

    blocked = navigator.next_step(drone_metres, heading, 0.0)
    assert blocked.activity == CORRIDOR_BLOCKED
    assert blocked.heading_radians == heading
    assert navigator.turn_heading_radians is None
    np.testing.assert_allclose(blocked.target_metres, drone_metres)

    live_map.scores[square] = LOWEST_SCORE
    clear = navigator.next_step(drone_metres, heading, 1.0)
    assert clear.activity == FLYING
    assert clear.heading_radians == heading
    assert clear.target_metres[1] > drone_metres[1]
    assert np.linalg.norm(clear.target_metres[:2] - drone_metres[:2]) <= CORRIDOR_LEAD_METRES + 1e-9


def test_a_persistent_block_ends_the_run(corridor):
    navigator, _, drone_metres = corridor
    heading = navigator.current_goal.straight_heading_radians
    navigator.next_step(drone_metres, heading, 0.0)
    with pytest.raises(NoWayForward, match="first corridor"):
        navigator.next_step(drone_metres, heading, LONGEST_SECONDS_WITHOUT_A_PATH + 0.1)


def test_depth_seeing_through_mapped_wall_does_not_open_q1(corridor):
    navigator, live_map, drone_metres = corridor
    navigator.stop_after_stage = ""
    goal = navigator.current_goal
    live_map.scores.fill(LOWEST_SCORE)
    before_wall_metres = np.array([goal.point_metres[0], 2.40, drone_metres[2]])
    step = navigator.next_step(before_wall_metres, goal.straight_heading_radians, 0.0)
    assert step.activity == CORRIDOR_BLOCKED
    np.testing.assert_allclose(step.target_metres, before_wall_metres)


def test_corridor_practice_reaches_the_mapped_line_and_ends(corridor):
    navigator, live_map, drone_metres = corridor
    goal = navigator.current_goal
    assert goal.straight_heading_radians == pytest.approx(math.pi / 2)
    live_map.scores[~live_map.blocked()] = LOWEST_SCORE
    for cycle in range(30):
        step = navigator.next_step(drone_metres, math.pi / 2, cycle * 0.1)
        assert step.heading_radians == pytest.approx(math.pi / 2)
        drone_metres = step.target_metres
        if step.finished:
            break
    assert step.finished
    assert step.stages_finished == ["first corridor"]
    assert goal.line_start_metres[1] <= drone_metres[1] < goal.line_start_metres[1] + 0.3
    assert step.heading_radians == goal.straight_heading_radians


def test_course_requires_the_whole_drone_past_the_line(corridor):
    navigator, _, drone_metres = corridor
    goal = navigator.current_goal
    heading = goal.straight_heading_radians
    assert math.degrees(heading) == pytest.approx(90.0, abs=3.0)
    assert all(other.straight_heading_radians is None for other in navigator.goals[1:])

    # Move the test line away from the end wall so the drone can fit beyond it.
    earlier_metres = np.array([0.0, -0.6])
    goal = replace(goal, point_metres=goal.point_metres + earlier_metres,
                   line_start_metres=goal.line_start_metres + earlier_metres,
                   line_end_metres=goal.line_end_metres + earlier_metres)
    navigator.goals = [goal]
    navigator.stop_after_stage = ""
    just_past_line_metres = np.array([goal.point_metres[0], goal.line_start_metres[1] + 0.05, drone_metres[2]])
    assert not navigator.next_step(just_past_line_metres, heading, 0.0).finished

    at_goal_metres = np.append(goal.point_metres, drone_metres[2])
    confirming = navigator.next_step(at_goal_metres, heading, 1.0)
    assert confirming.activity == CORRIDOR_CONFIRMING_EXIT
    assert not confirming.finished
    step = navigator.next_step(at_goal_metres, heading, 1.0 + CORRIDOR_EXIT_CONFIRM_SECONDS + 0.01)
    assert step.finished
    assert step.stages_finished == ["first corridor"]
    assert step.heading_radians == heading


def test_full_course_practice_looks_for_q2_at_the_drawn_line(corridor):
    navigator, live_map, drone_metres = corridor
    navigator.stop_after_stage = ""
    navigator.full_course_practice = True
    goal = navigator.current_goal
    heading = goal.straight_heading_radians
    live_map.scores.fill(LOWEST_SCORE)
    exit_metres = np.array([goal.point_metres[0], goal.line_start_metres[1] + 0.05, drone_metres[2]])

    confirming = navigator.next_step(exit_metres, heading, 0.0)
    assert confirming.activity == CORRIDOR_CONFIRMING_EXIT
    step = navigator.next_step(exit_metres, heading, CORRIDOR_EXIT_CONFIRM_SECONDS + 0.01)
    assert not step.finished
    assert step.stages_finished == ["first corridor"]
    assert step.stages_begun == ["maze"]
    assert navigator.turn_heading_radians is None
