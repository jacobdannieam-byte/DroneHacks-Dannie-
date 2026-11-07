"""drift_correction.py: the course map's distance to the wall straight ahead (checked against
Kenny's distance_to_shapes_metres by stepping along the line), and the team's rule for when, and how
far, to move the position estimate."""
import math
from pathlib import Path

import numpy as np
import pytest

from team_nav.course_map import distance_to_shapes_metres, load_course_map
from team_nav.drift_correction import MapRanging, decide_correction
from team_nav.mission_settings import MissionSettings
from team_nav.start_frame import StartFrame

MAPS = Path(__file__).resolve().parents[1] / "course_maps"
SETTINGS = MissionSettings()
CRUISE_HEIGHT_METRES = 0.8
STEP_METRES = 0.002


@pytest.fixture(scope="module")
def ranging():
    course_map = load_course_map(MAPS / "organizers_drawing.json")
    return MapRanging(course_map, StartFrame(course_map.launch_mark_centre_metres, 90.0))


def distance_by_stepping(ranging, forward, left, heading_radians, height_metres):
    """Walk along the line in small steps until Kenny's distance function says a shape is touched."""
    shapes = ranging.shapes_at(height_metres)
    start = ranging.start_frame.course_from_start_frame((forward, left, height_metres))[:2]
    direction = (math.cos(heading_radians) * ranging.start_frame.forward_on_course
                 + math.sin(heading_radians) * ranging.start_frame.left_on_course)
    along = np.arange(STEP_METRES, 8.0, STEP_METRES)
    points = start + along[:, np.newaxis] * direction
    distances, names = distance_to_shapes_metres(points, shapes)
    touching = distances <= STEP_METRES / 2.0                              # zero-thickness shapes too
    assert touching.any()
    first = int(np.argmax(touching))
    return along[first], names[first]


@pytest.mark.parametrize("forward, left, heading_degrees", [
    (0.0, 0.0, 0.0), (1.5, -0.1, -90.0), (1.5, -1.0, 180.0), (0.2, -1.3, -90.0), (0.4, -2.2, 0.0),
    (1.6, -3.1, -120.0), (-1.7, -4.5, 135.0), (-1.1, -3.1, 70.0), (-1.9, -1.9, 90.0), (0.0, 0.0, 33.0)])
def test_the_map_distance_matches_stepping_along_the_line(ranging, forward, left, heading_degrees):
    heading = math.radians(heading_degrees)
    distance, name = ranging.distance_ahead(forward, left, heading, CRUISE_HEIGHT_METRES)
    stepped, stepped_name = distance_by_stepping(ranging, forward, left, heading, CRUISE_HEIGHT_METRES)
    assert distance == pytest.approx(stepped, abs=2 * STEP_METRES)
    assert name == stepped_name


def test_the_window_is_open_at_its_height_and_solid_below_it(ranging):
    """The organizers' window is 0.32 to 1.20 m high; a slice at 0.2 m has the whole wall."""
    window_wall = ranging.course_map.wall_named("C2a chicane wall (window)")
    middle = (window_wall.point_along_wall_metres(0.81))
    across = np.array([-window_wall.direction()[1], window_wall.direction()[0]])
    standing = ranging.start_frame.start_frame_from_course((*(middle - 0.8 * across), 0.0))
    heading = math.atan2(across @ ranging.start_frame.left_on_course, across @ ranging.start_frame.forward_on_course)
    low, low_name = ranging.distance_ahead(standing[0], standing[1], heading, 0.2)
    high, high_name = ranging.distance_ahead(standing[0], standing[1], heading, CRUISE_HEIGHT_METRES)
    assert low_name == "C2a chicane wall (window)"
    assert low == pytest.approx(0.8 - window_wall.thickness_metres / 2.0, abs=1e-6)
    assert high > low + 0.1


def test_a_shorter_depth_than_the_map_moves_the_estimate_forward():
    decision = decide_correction(1.10, 1.25, "M1 maze wall", SETTINGS)
    assert decision.shift_metres == pytest.approx(0.15)
    assert "moved the estimate +0.15 m along the heading" in decision.reason


def test_a_longer_depth_than_the_map_moves_it_back():
    assert decide_correction(1.35, 1.25, "M1", SETTINGS).shift_metres == pytest.approx(-0.10)


def test_each_correction_is_capped_at_0_2_metres():
    assert decide_correction(1.0, 1.35, "M1", SETTINGS).shift_metres == pytest.approx(0.2)
    assert decide_correction(1.35, 1.0, "M1", SETTINGS).shift_metres == pytest.approx(-0.2)


@pytest.mark.parametrize("measured, map_metres, name, why", [
    (None, 1.0, "M1", "no good depth reading"),
    (1.0, math.inf, None, "no wall straight ahead in the map"),
    (1.0, 1.0, "the safety net", "only the safety net"),
    (0.35, 0.39, "M1", "outside 0.4 to 2.5 m"),
    (2.45, 2.55, "M1", "outside 0.4 to 2.5 m"),
    (1.0, 1.41, "M1", "differ by 0.41 m, too much to trust"),
    (1.6, 1.0, "M1", "differ by 0.60 m, too much to trust"),
])
def test_no_correction_without_a_good_reading_or_a_believable_wall(measured, map_metres, name, why):
    decision = decide_correction(measured, map_metres, name, SETTINGS)
    assert decision.shift_metres is None
    assert why in decision.reason


def test_the_wall_distance_limits_include_their_ends():
    assert decide_correction(0.45, 0.4, "M1", SETTINGS).shift_metres == pytest.approx(-0.05)
    assert decide_correction(2.45, 2.5, "M1", SETTINGS).shift_metres == pytest.approx(0.05)
