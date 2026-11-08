"""Tests where each full_course stop option lands (full_course_stops.py), on the A* route over
real_course_2026-11-08.json, the default map. Plain arithmetic on the map, no ROS.

The expected spots were worked out from the route and the checkpoint lines, then confirmed by the
code; Gazebo must still check the landings.
"""
import numpy as np
import pytest

from our_files import load_our_map
from real_course_route import real_course_map_and_route
from team_nav.course_map import LowObstacle
from team_nav.practice_flights.full_course_route import (
    MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES, MINIMUM_BODY_CLEARANCE_METRES, footprint_distance_past_line,
    footprint_points, route_waypoints,
)
from team_nav.practice_flights.full_course_stops import (
    AT_NEXT_WAYPOINT, LANDING_DISTANCE_PAST_LINE_METRES, STRAIGHT_DOWN, describe_every_option, plan_stop,
)

# Our own choice, the same as full_course.ARRIVAL_RULE's sideways tolerance.
ARRIVAL_TOLERANCE_METRES = 0.08

# (option, landing spot east/north, least distance the whole drone must be past the line).
EXPECTED_LANDINGS = [
    ('corridor', (-2.515, 2.466), LANDING_DISTANCE_PAST_LINE_METRES),
    ('gate_a', (0.729, 2.154), LANDING_DISTANCE_PAST_LINE_METRES),
    ('tunnel', (2.297, -0.750), 0.25),
    ('chicane', (-0.544, -0.889), LANDING_DISTANCE_PAST_LINE_METRES),
]


@pytest.mark.parametrize('option,spot,least_distance_past', EXPECTED_LANDINGS)
def test_each_option_lands_straight_down_past_its_line(option, spot, least_distance_past):
    course_map, route = real_course_map_and_route()
    plan = plan_stop(course_map, route, option)
    assert plan.how == STRAIGHT_DOWN
    assert plan.landing_spot_metres[:2] == pytest.approx(spot, abs=0.005)
    assert plan.distance_past_line_metres >= least_distance_past
    # The shortened route is the full route up to the spot's leg, then the spot.
    legs_before_the_spot = len(plan.route_to_fly_metres) - 1
    np.testing.assert_allclose(plan.route_to_fly_metres[:legs_before_the_spot], route[:legs_before_the_spot])
    np.testing.assert_allclose(plan.route_to_fly_metres[-1], plan.landing_spot_metres)


@pytest.mark.parametrize('option,spot,least_distance_past', EXPECTED_LANDINGS)
def test_each_landing_spot_is_open_floor_wholly_past_the_line(option, spot, least_distance_past):
    course_map, route = real_course_map_and_route()
    plan = plan_stop(course_map, route, option)
    line = course_map.checkpoint_line_named(plan.line_name)
    distance_past_metres, is_within_ends = footprint_distance_past_line(plan.landing_spot_metres, footprint_points(), line)
    assert is_within_ends
    assert distance_past_metres == pytest.approx(plan.distance_past_line_metres)
    assert plan.clearance_metres >= MINIMUM_BODY_CLEARANCE_METRES
    # Even stopped at the edge of the arrival tolerance, the whole drone is past the line and clear.
    assert distance_past_metres - ARRIVAL_TOLERANCE_METRES > 0
    assert plan.clearance_metres - ARRIVAL_TOLERANCE_METRES > 0


def test_tunnel_option_lands_a_quarter_metre_beyond_the_tunnel_exit():
    course_map, route = real_course_map_and_route()
    plan = plan_stop(course_map, route, 'tunnel')
    tunnel = course_map.tubes[0]
    footprint_north_edge = plan.landing_spot_metres[1] + footprint_points()[:, 1].max()
    assert footprint_north_edge < tunnel.exit_centre_metres[1] - 0.25
    assert plan.nearest_thing == 'Tunnel'


def test_spot_that_is_not_open_floor_carries_on_to_the_next_waypoint():
    course_map, route = real_course_map_and_route()
    # A short object touching the north edge of the chicane option's spot. The next waypoint,
    # 0.11 m further south, keeps 0.11 m from it.
    course_map.low_obstacles.append(LowObstacle('test object', np.array([-0.54, -0.70]), 0.02, 0.10, 'test'))
    plan = plan_stop(course_map, route, 'chicane')
    assert plan.how == AT_NEXT_WAYPOINT
    assert plan.landing_spot_metres[:2] == pytest.approx([-0.6, -1.0])


def test_refuses_when_neither_spot_is_open_floor():
    course_map, route = real_course_map_and_route()
    course_map.low_obstacles.append(LowObstacle('test boulder', np.array([-0.57, -0.94]), 0.05, 0.10, 'test'))
    with pytest.raises(ValueError, match='nowhere safe to land'):
        plan_stop(course_map, route, 'chicane')


def test_spot_search_carries_on_into_the_next_leg():
    """Aditya's fixed sketch route ends its tunnel leg 0.24 m past the line, short of 0.25 m, so the
    tunnel spot is on the next leg."""
    course_map = load_our_map('sketch_2026-11-08')
    route = route_waypoints(course_map, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES)
    plan = plan_stop(course_map, route, 'tunnel')
    assert plan.distance_past_line_metres >= 0.25
    assert plan.landing_spot_metres[1] < route[7, 1]


def test_unknown_option_is_refused_with_the_known_names():
    course_map, route = real_course_map_and_route()
    with pytest.raises(ValueError, match='corridor, gate_a, tunnel, chicane'):
        plan_stop(course_map, route, 'window')


def test_every_option_is_described_for_the_screen():
    course_map, route = real_course_map_and_route()
    sentences = describe_every_option(course_map, route)
    assert len(sentences) == 4
    for sentence, (option, spot, _) in zip(sentences, EXPECTED_LANDINGS):
        assert sentence.startswith(f'{option}: ')
        assert f'east {spot[0]:.3f} m, north {spot[1]:.3f} m' in sentence
