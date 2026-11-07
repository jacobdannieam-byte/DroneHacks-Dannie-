"""Tests course_run.py's planning and stage bookkeeping, without flying: the stage names come from
the mission, a practice run's route is cut where its stage ends, an unknown stage is refused, and
stages begin and finish in order as the route's target moves on.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from our_files import MISSIONS_FOLDER, SCAN_MAP, DRAWING_MAP, load_our_map
from team_nav.course_run import (
    CLIMB_STAGE, TO_THE_PAD_STAGE, CoursePlan, CourseRun, check_stop_after_stage, split_file_names,
)
from team_nav.mission import load_mission
from team_nav.route_planner import plan_route
from team_nav.run_mode import read_run_mode

EXPECTED_STAGES = [CLIMB_STAGE, "first corridor", "maze", "tunnel", "chicane"]


@pytest.fixture(scope="module")
def scan_plan():
    course_map = load_our_map(SCAN_MAP)
    mission = load_mission(f"{MISSIONS_FOLDER}/dronehacks_2026_side_gap_first.json")
    other_boulders = load_our_map(DRAWING_MAP).low_obstacles
    route = plan_route(course_map, mission, other_possible_boulders=other_boulders)
    return CoursePlan(course_map, mission, other_boulders, route, start_heading_degrees=90.0)


def test_the_stages_are_the_climb_then_the_missions_checkpoints(scan_plan):
    assert scan_plan.stage_names() == EXPECTED_STAGES


def test_a_practice_run_flies_only_up_to_where_its_stage_ends(scan_plan):
    maze_end_waypoint = dict(scan_plan.route_stages)["maze"]
    waypoints = scan_plan.waypoints_to_fly("maze")

    assert len(waypoints) == maze_end_waypoint + 1
    assert np.allclose(waypoints[-1], scan_plan.route.waypoints_metres[maze_end_waypoint])
    assert len(scan_plan.waypoints_to_fly("")) == len(scan_plan.route.waypoints_metres)


def test_an_unknown_stage_is_refused_with_the_list_of_stages():
    practice = read_run_mode({"TEAM_NAV_RUN_MODE": "practice", "TEAM_NAV_STOP_AFTER_STAGE": "window"})
    with pytest.raises(ValueError, match="climb, first corridor, maze, tunnel, chicane"):
        check_stop_after_stage(practice, EXPECTED_STAGES)
    check_stop_after_stage(read_run_mode({"TEAM_NAV_RUN_MODE": "practice", "TEAM_NAV_STOP_AFTER_STAGE": "Maze"}),
                           EXPECTED_STAGES)


def test_file_names_can_be_given_in_one_word():
    assert split_file_names(["a.json, b.json", "c.json", ""]) == ["a.json", "b.json", "c.json"]


class PretendLog:
    def __init__(self):
        self.stage_events = []

    def event(self, kind, text, **details):
        if kind == "stage":
            self.stage_events.append((details["stage"], details["event"]))


class PretendLink:
    def __init__(self):
        self.log = PretendLog()
        self.stage = None

    def now_seconds(self):
        return 0.0


def course_run_for_bookkeeping(plan, stop_after_stage):
    """A CourseRun with only what the stage bookkeeping uses."""
    run = CourseRun.__new__(CourseRun)
    run.plan = plan
    run.link = PretendLink()
    run.log = run.link.log
    run.stop_after_stage = stop_after_stage
    return run


def test_stages_begin_and_finish_in_order_as_the_target_moves_on(scan_plan):
    run = course_run_for_bookkeeping(scan_plan, "")
    stages_still_to_finish = list(scan_plan.route_stages)
    run.begin_stage(stages_still_to_finish[0][0])
    for next_waypoint_number in range(len(scan_plan.route.waypoints_metres) + 1):
        run.finish_stages_passed(stages_still_to_finish, next_waypoint_number)
    run.finish_stage()

    expected = []
    for name in EXPECTED_STAGES[1:] + [TO_THE_PAD_STAGE]:
        expected += [(name, "begins"), (name, "finished")]
    assert run.log.stage_events == expected


def test_a_practice_run_stays_in_its_last_stage_until_the_drone_gets_there(scan_plan):
    run = course_run_for_bookkeeping(scan_plan, "maze")
    stages_still_to_finish = list(scan_plan.route_stages)
    run.begin_stage(stages_still_to_finish[0][0])
    for next_waypoint_number in range(len(scan_plan.waypoints_to_fly("maze")) + 1):
        run.finish_stages_passed(stages_still_to_finish, next_waypoint_number)

    assert run.log.stage_events[-1] == ("maze", "begins")
    assert run.stage_name == "maze"
