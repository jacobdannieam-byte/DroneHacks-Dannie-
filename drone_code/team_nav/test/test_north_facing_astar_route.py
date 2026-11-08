"""Tests full_course's A* route on real_course_2026-11-08.json (north_facing_astar_route.py).
No ROS. Uses the route planned once in real_course_route.py."""
import math

import numpy as np
import pytest

from real_course_route import real_course_map_and_route, real_course_mission
from team_nav.practice_flights.astar_geometry import RouteGeometry
from team_nav.practice_flights.full_course_route import (
    CHECKPOINT_LINE_NAMES, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES, first_full_crossings, samples_along_leg,
)
from team_nav.practice_flights.north_facing_astar_route import (
    BACK_OFF_NORTH_METRES, PROPELLER_CLEARANCE_METRES, map_with_every_opening_closed,
)


def test_route_starts_north_to_the_first_checkpoint_turn_back_point():
    _, route = real_course_map_and_route()
    assert route[1, :2] == pytest.approx([-2.5146, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES])


def test_route_visits_every_goal_in_order():
    _, route = real_course_map_and_route()
    goal_numbers = []
    for goal in real_course_mission()['goals'][1:]:
        distances = np.linalg.norm(route[:, :2] - goal['point_metres'][:2], axis=1)
        assert distances.min() < 1e-6
        goal_numbers.append(int(np.argmin(distances)))
    assert goal_numbers == sorted(goal_numbers)


def test_tunnel_goal_is_in_the_drawings_tunnel():
    course_map, route = real_course_map_and_route()
    tunnel_goal = real_course_mission()['goals'][2]['point_metres']
    assert tunnel_goal[0] == pytest.approx(course_map.tubes[0].exit_centre_metres[0], abs=0.01)


def test_propellers_keep_the_required_clearance_facing_north():
    """Every point after the back-off point, with the window closed, as the planner saw it."""
    course_map, route = real_course_map_and_route()
    geometry = RouteGeometry(map_with_every_opening_closed(course_map))
    for start, end in zip(route[1:], route[2:]):
        samples = samples_along_leg(start, end)
        samples = samples[samples[:, 1] <= BACK_OFF_NORTH_METRES + 1e-9] if start[1] > BACK_OFF_NORTH_METRES else samples
        clearance = geometry.footprint_clearance(samples, math.pi / 2)
        assert clearance.min() >= PROPELLER_CLEARANCE_METRES - 1e-9


def test_route_crosses_all_four_lines_and_enters_the_chicane_through_gate_b():
    course_map, route = real_course_map_and_route()
    crossed = [name for name, _, _ in first_full_crossings(course_map, route)]
    assert crossed == list(CHECKPOINT_LINE_NAMES)
    window_wall, _ = course_map.opening_named('window')
    for start, end in zip(route, route[1:]):
        for sample in samples_along_leg(start, end):
            # Never closer to the window wall's line than the airframe's half width.
            along = window_wall.end_metres - window_wall.start_metres
            fraction = np.clip((sample - window_wall.start_metres) @ along / (along @ along), 0, 1)
            assert np.linalg.norm(sample - window_wall.start_metres - fraction * along) > 0.25
