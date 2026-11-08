"""Tests where each full_course stop option lands on the sketch map (full_course_stops.py).

Plain arithmetic on the map, no ROS. The expected spots were worked out by hand from the route
and the checkpoint lines, then confirmed by the code; Gazebo must still check the landings.
"""
from dataclasses import replace

import numpy as np
import pytest

from our_files import load_our_map
from team_nav.course_map import LowObstacle
from team_nav.practice_flights.full_course_route import (
    MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES, MINIMUM_BODY_CLEARANCE_METRES, footprint_distance_past_line,
    footprint_points, route_waypoints,
)
from team_nav.practice_flights.full_course_stops import (
    AT_NEXT_WAYPOINT, STRAIGHT_DOWN, describe_every_option, plan_stop,
)

# Our own choice, the same as full_course.ARRIVAL_RULE's sideways tolerance.
ARRIVAL_TOLERANCE_METRES = 0.08

# (option, landing spot east/north, the route point the shortened route lands after).
EXPECTED_LANDINGS = [
    ('corridor', (-2.510, 2.515), 0),
    ('gate_a', (1.550, 2.120), 4),
    ('tunnel', (2.060, -0.500), 6),
    ('chicane', (-1.030, -1.153), 12),
]


def sketch_map_and_route():
    course_map = load_our_map('sketch_2026-11-08')
    return course_map, route_waypoints(course_map, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES)


@pytest.mark.parametrize('option,spot,last_route_point_before', EXPECTED_LANDINGS)
def test_each_option_lands_straight_down_just_past_its_line(option, spot, last_route_point_before):
    course_map, route = sketch_map_and_route()
    plan = plan_stop(course_map, route, option)
    assert plan.how == STRAIGHT_DOWN
    assert plan.landing_spot_metres[:2] == pytest.approx(spot, abs=0.005)
    # The shortened route is the full route up to the leg with the crossing, then the spot.
    np.testing.assert_allclose(plan.route_to_fly_metres[:last_route_point_before + 1],
                               route[:last_route_point_before + 1])
    np.testing.assert_allclose(plan.route_to_fly_metres[-1], plan.landing_spot_metres)


@pytest.mark.parametrize('option,spot,last_route_point_before', EXPECTED_LANDINGS)
def test_each_landing_spot_is_open_floor_wholly_past_the_line(option, spot, last_route_point_before):
    course_map, route = sketch_map_and_route()
    plan = plan_stop(course_map, route, option)
    line = course_map.checkpoint_line_named(plan.line_name)
    distance_past_metres, is_within_ends = footprint_distance_past_line(plan.landing_spot_metres, footprint_points(), line)
    assert is_within_ends
    assert distance_past_metres == pytest.approx(plan.distance_past_line_metres)
    assert plan.clearance_metres >= MINIMUM_BODY_CLEARANCE_METRES
    # Even stopped at the edge of the arrival tolerance, the whole drone is past the line and
    # clear of everything. The first corridor leaves the least: about 5 mm each way.
    assert distance_past_metres - ARRIVAL_TOLERANCE_METRES > 0
    assert plan.clearance_metres - ARRIVAL_TOLERANCE_METRES > 0


def test_tunnel_option_lands_beyond_the_tunnel_exit():
    course_map, route = sketch_map_and_route()
    plan = plan_stop(course_map, route, 'tunnel')
    tunnel = course_map.tubes[0]
    footprint_north_edge = plan.landing_spot_metres[1] + footprint_points()[:, 1].max()
    assert footprint_north_edge < tunnel.exit_centre_metres[1] - MINIMUM_BODY_CLEARANCE_METRES
    assert plan.nearest_thing == 'Tunnel'
    assert plan.clearance_metres == pytest.approx(0.105, abs=0.005)


def test_spot_that_is_not_open_floor_carries_on_to_the_next_waypoint():
    course_map, route = sketch_map_and_route()
    # A short object just north-east of the tunnel option's spot: it blocks the spot's footprint
    # (by 7 mm) but leaves the next waypoint, 0.15 m further south, clear.
    course_map.low_obstacles.append(LowObstacle('test object', np.array([2.33, -0.30]), 0.03, 0.10, 'test'))
    plan = plan_stop(course_map, route, 'tunnel')
    assert plan.how == AT_NEXT_WAYPOINT
    assert plan.landing_spot_metres[:2] == pytest.approx(route[7, :2])
    np.testing.assert_allclose(plan.route_to_fly_metres, route[:8])


def test_refuses_when_neither_spot_is_open_floor():
    course_map, route = sketch_map_and_route()
    boulder = course_map.low_obstacles[0]
    course_map.low_obstacles.append(replace(boulder, name='test boulder', centre_metres=np.array([2.06, -0.58]),
                                            radius_metres=0.05, top_height_metres=0.10))
    with pytest.raises(ValueError, match='nowhere safe to land'):
        plan_stop(course_map, route, 'tunnel')


def test_unknown_option_is_refused_with_the_known_names():
    course_map, route = sketch_map_and_route()
    with pytest.raises(ValueError, match='corridor, gate_a, tunnel, chicane'):
        plan_stop(course_map, route, 'window')


def test_every_option_is_described_for_the_screen():
    course_map, route = sketch_map_and_route()
    sentences = describe_every_option(course_map, route)
    assert len(sentences) == 4
    for sentence, (option, spot, _) in zip(sentences, EXPECTED_LANDINGS):
        assert sentence.startswith(f'{option}: ')
        assert f'east {spot[0]:.3f} m, north {spot[1]:.3f} m' in sentence
