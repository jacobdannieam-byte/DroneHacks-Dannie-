"""Tests full_course's fixed route on real_course_2026-11-08.json (full_course_fixed_route.py). No ROS."""
import math

import numpy as np
import pytest

from real_course_route import real_course_map_and_route
from team_nav.practice_flights.astar_geometry import RouteGeometry
from team_nav.practice_flights.full_course_fixed_route import LAUNCH_EAST_NORTH_METRES
from team_nav.practice_flights.full_course_route import (
    CHECKPOINT_LINE_NAMES, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES, first_full_crossings, samples_along_leg,
)

# The Gazebo check's limit (simulator/gazebo_full/check_footprint_clearance.py).
LEAST_ALLOWED_CLEARANCE_METRES = 0.08


def test_route_is_the_teams_route():
    _, route = real_course_map_and_route()
    assert route[0, :2] == pytest.approx(LAUNCH_EAST_NORTH_METRES)
    assert route[3, :2] == pytest.approx([-2.51, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES])
    assert route[4, :2] == pytest.approx([2.30, 2.45])
    assert route[5, :2] == pytest.approx([2.30, -0.80])
    assert route[-1, :2] == pytest.approx([-2.0422, -0.7925])
    assert np.all(route[:, 2] == 0.75)


def test_tunnel_leg_runs_down_the_middle_of_the_tunnel():
    course_map, route = real_course_map_and_route()
    assert route[4, 0] == pytest.approx(course_map.tubes[0].exit_centre_metres[0], abs=0.01)


def test_propellers_keep_the_gazebo_limit_on_every_leg_facing_north():
    course_map, route = real_course_map_and_route()
    geometry = RouteGeometry(course_map, 0.75)
    for start, end in zip(route, route[1:]):
        clearance = geometry.footprint_clearance(samples_along_leg(start, end), math.pi / 2)
        assert clearance.min() >= LEAST_ALLOWED_CLEARANCE_METRES


def test_route_crosses_all_four_lines_in_order():
    course_map, route = real_course_map_and_route()
    crossed = [name for name, _, _ in first_full_crossings(course_map, route)]
    assert crossed == list(CHECKPOINT_LINE_NAMES)
