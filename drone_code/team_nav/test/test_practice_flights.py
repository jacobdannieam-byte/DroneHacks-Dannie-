"""Tests the practice flights without flying: the runner's list and refusals, the out-and-back's
checks before it starts, and the mock practice course's map and mission with the real planner.

How to run these tests is in the package's README.
"""
import os

import numpy as np
import pytest

from our_files import DRAWING_MAP, MAPS_FOLDER, MISSIONS_FOLDER, SCAN_MAP, load_our_map
from team_nav import course_run
from team_nav.course_map import load_course_map
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES
from team_nav.mission import load_mission
from team_nav.practice_flights import mock_gap_and_window, out_and_back, run_practice_flight
from team_nav.route_planner import plan_route
from team_nav.run_mode import read_run_mode
from team_nav.start_frame import StartFrame

MOCK_MAP_FILE = os.path.join(MAPS_FOLDER, mock_gap_and_window.MAP_FILE)
MOCK_MISSION_FILE = os.path.join(MISSIONS_FOLDER, mock_gap_and_window.MISSION_FILE)
PRACTICE = read_run_mode({"TEAM_NAV_RUN_MODE": "practice"})


# ---- The runner ---------------------------------------------------------------------------------

def test_with_no_name_the_runner_lists_every_flight(capsys):
    assert run_practice_flight.main([]) == 0
    printed = capsys.readouterr().out
    assert "out_and_back" in printed and "mock_gap_and_window" in printed


def test_an_unknown_flight_is_refused(capsys):
    assert run_practice_flight.main(["hover_and_land"]) == course_run.EXIT_REFUSED_TO_START
    assert "no practice flight called 'hover_and_land'" in capsys.readouterr().out


def test_competition_mode_is_refused_before_any_record_is_made(monkeypatch, tmp_path, capsys):
    monkeypatch.delenv("TEAM_NAV_RUN_MODE", raising=False)
    monkeypatch.setenv("TEAM_NAV_RECORDS_FOLDER", str(tmp_path))
    assert run_practice_flight.main(["out_and_back"]) == course_run.EXIT_REFUSED_TO_START
    assert "practice mode" in capsys.readouterr().out
    assert os.listdir(tmp_path) == []


def test_ros_arguments_are_not_taken_for_a_flight_name(capsys):
    assert run_practice_flight.main(["--ros-args", "-r", "__node:=x"]) == 0


# ---- The out-and-back's checks before it starts -------------------------------------------------

@pytest.mark.parametrize("map_name", [SCAN_MAP, DRAWING_MAP])
def test_the_course_launch_mark_has_room_for_the_out_and_back(map_name):
    course_map = load_our_map(map_name)
    start_frame = StartFrame(course_map.launch_mark_centre_metres, course_run.DEFAULT_START_HEADING_DEGREES)
    for hover_height_metres in (out_and_back.LOWEST_HOVER_HEIGHT_METRES, out_and_back.HIGHEST_HOVER_HEIGHT_METRES):
        assert out_and_back.problems_with_the_line_flown(course_map, start_frame, hover_height_metres) == []


def test_the_out_and_back_is_refused_if_the_drone_faces_a_wall():
    # Facing east from the drawing's launch mark, 1 m forward goes through the first maze divider.
    course_map = load_our_map(DRAWING_MAP)
    start_frame = StartFrame(course_map.launch_mark_centre_metres, 0.0)
    problems = out_and_back.problems_with_the_line_flown(course_map, start_frame, 0.5)
    assert len(problems) == 1 and "M1 maze divider" in problems[0]


def test_the_out_and_back_is_refused_over_a_boulder():
    course_map = load_our_map(DRAWING_MAP)
    start_frame = StartFrame(course_map.launch_mark_centre_metres, 90.0)
    boulder = course_map.low_obstacles[0]
    boulder.centre_metres = start_frame.course_from_start_frame((0.5, 0.0, 0.0))[:2]
    problems = out_and_back.problems_with_the_line_flown(course_map, start_frame, 0.5)
    assert any(boulder.name in problem and "bottom laser" in problem for problem in problems)


def test_the_out_and_back_refuses_a_stage_to_stop_after_and_a_bad_height():
    arguments = out_and_back.parse_arguments(["--hover-height-metres", "1.5"])
    stopping = read_run_mode({"TEAM_NAV_RUN_MODE": "practice", "TEAM_NAV_STOP_AFTER_STAGE": "climb"})
    problems = out_and_back.problems_with_the_settings(arguments, stopping)
    assert len(problems) == 2
    assert out_and_back.problems_with_the_settings(out_and_back.parse_arguments([]), PRACTICE) == []


# ---- The mock practice course -------------------------------------------------------------------

@pytest.fixture(scope="module")
def mock_route():
    return plan_route(load_course_map(MOCK_MAP_FILE), load_mission(MOCK_MISSION_FILE))


def test_every_shape_in_the_mock_map_says_it_is_a_proposal_to_measure():
    course_map = load_course_map(MOCK_MAP_FILE)
    sources = [course_map.edges_source, course_map.launch_mark_source, course_map.landing_pad_source]
    for wall in course_map.walls:
        sources.append(wall.source)
        sources += [opening.source for opening in wall.openings]
    sources += [line.source for line in course_map.checkpoint_lines]
    for source in sources:
        assert "proposal, to be measured and replaced on the day" in source


def test_the_mock_gap_is_a_bit_wider_than_the_drone_needs():
    course_map = load_course_map(MOCK_MAP_FILE)
    gap_line = course_map.checkpoint_line_named("mock gap")
    west_wall, east_wall = course_map.wall_named("Gap wall, west"), course_map.wall_named("Gap wall, east")
    # The planner rounds each wall end by half the wall's thickness.
    usable_width_metres = (np.linalg.norm(gap_line.end_metres - gap_line.start_metres)
                           - west_wall.thickness_metres / 2 - east_wall.thickness_metres / 2)
    assert 2 * REQUIRED_ROOM_METRES < usable_width_metres < 2 * REQUIRED_ROOM_METRES + 0.25


def test_the_mock_window_has_a_low_bottom_edge():
    _, window = load_course_map(MOCK_MAP_FILE).opening_named("mock window")
    assert 0.3 <= window.bottom_height_metres <= 0.5


def test_the_planner_flies_the_mock_course_through_the_gap_then_the_window(mock_route):
    assert [name for name, _, _ in mock_route.checkpoints_cleared] == ["Mock gap", "Mock window"]
    assert mock_route.least_room_metres >= REQUIRED_ROOM_METRES
    course_map = load_course_map(MOCK_MAP_FILE)
    assert np.allclose(mock_route.waypoints_metres[-1][:2], course_map.landing_pad_centre_metres)
    # The window is crossed at its middle height, which is the cruise height here.
    assert np.allclose(mock_route.waypoints_metres[:, 2], 0.8)


def test_the_mock_flight_gives_course_run_the_practice_files_and_no_borrowed_boulders():
    arguments = course_run.parse_arguments(mock_gap_and_window.course_run_words(["--ignore-raw-laser"]))
    assert arguments.map == mock_gap_and_window.MAP_FILE
    assert arguments.mission == mock_gap_and_window.MISSION_FILE
    assert arguments.other_boulder_maps == []
    assert arguments.ignore_raw_laser


def test_a_later_option_replaces_the_mock_flights_map():
    arguments = course_run.parse_arguments(mock_gap_and_window.course_run_words(["--map", "measured_copy.json"]))
    assert arguments.map == "measured_copy.json"


def test_the_mock_course_stages_can_be_stopped_after(mock_route):
    course_map = load_course_map(MOCK_MAP_FILE)
    plan = course_run.CoursePlan(course_map, load_mission(MOCK_MISSION_FILE), [], mock_route, 90.0)
    assert plan.stage_names() == ["climb", "mock gap", "mock window"]
    stopping = read_run_mode({"TEAM_NAV_RUN_MODE": "practice", "TEAM_NAV_STOP_AFTER_STAGE": "mock gap"})
    course_run.check_stop_after_stage(stopping, plan.stage_names())
