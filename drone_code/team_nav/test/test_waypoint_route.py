"""Check full-course waypoint planning and progression without ROS or a flight."""

from dataclasses import replace
import math

import numpy as np
import pytest

from our_files import DRAWING_MAP, load_our_map, load_side_gap_first_mission
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES
from team_nav.live_map import LOWEST_SCORE, LiveMap
from team_nav.route_planner import RoutePlanningError
from team_nav.waypoint_route import WaypointNavigator, plan_waypoint_route


def test_refuses_when_the_current_sketch_cannot_clear_q1():
    with pytest.raises(RoutePlanningError, match="can't get across 'end of first corridor'"):
        plan_waypoint_route(load_our_map("sketch_2026-11-08"), load_side_gap_first_mission())


def test_drawing_route_refuses_the_tight_turn():
    with pytest.raises(RoutePlanningError, match="turn with only .* mapped room"):
        plan_waypoint_route(load_our_map(DRAWING_MAP), load_side_gap_first_mission())


def test_waypoints_advance_through_the_checkpoints_in_order_then_to_the_pad():
    # An open map keeps the organizer's checkpoint geometry while providing room to test turns.
    course_map = replace(load_our_map(DRAWING_MAP), walls=[], tubes=[], low_obstacles=[],
                         lowest_corner_metres=np.array([-4.5, -4.5]), highest_corner_metres=np.array([4.5, 4.5]))
    mission = load_side_gap_first_mission()
    route = plan_waypoint_route(course_map, mission)
    live_map = LiveMap(course_map, mission.cruise_height_metres, DRONE_HALF_HEIGHT_METRES)
    live_map.scores[~live_map.blocked()] = LOWEST_SCORE
    navigator = WaypointNavigator(live_map, course_map, route, "pad", "", math.pi / 2)
    completed = []
    position_metres = route.waypoints_metres[0]
    heading = math.pi / 2
    for cycle in range(300):
        step = navigator.next_step(position_metres, heading, cycle * 0.1, depth_map_updates=cycle)
        completed.extend(step.stages_finished)
        position_metres, heading = step.target_metres, step.heading_radians
        if step.finished:
            break
    assert step.finished
    assert completed == ["first corridor", "maze", "tunnel", "chicane", "pad"]
    assert np.linalg.norm(position_metres[:2] - course_map.landing_pad_centre_metres) < 0.1
