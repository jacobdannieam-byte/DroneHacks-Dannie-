"""Check forward-only A* routing, turn geometry, and the controlled moved-obstacle case."""
import json
import math
from pathlib import Path
import numpy as np
import pytest
from our_files import MISSIONS_FOLDER, load_our_map
from team_nav.practice_flights.astar_geometry import RouteGeometry, ChicaneErrorMargin
from team_nav.practice_flights.astar_route import plan_mission, search_route


@pytest.fixture(scope='module')
def comparison():
    mission = json.loads((Path(MISSIONS_FOLDER)/'astar_five_goals.json').read_text())
    maps = [load_our_map(name) for name in ['sketch_2026-11-08','astar_moved_obstacle']]
    return mission,maps,[plan_mission(course_map,mission) for course_map in maps]


def test_first_checkpoint_turns_forward_without_retreat():
    course_map = load_our_map('sketch_2026-11-08')
    geometry = RouteGeometry(course_map)
    stop = np.array([-2.5146,2.451])
    heading = math.atan2(-1,4)
    assert geometry.crossing_clear(stop,math.pi/2,course_map.checkpoint_line_named('end of first corridor'))
    assert geometry.turn_clear(stop,math.pi/2,heading)
    assert geometry.segment_clear(stop,stop+[.4,-.1],heading)
    assert not geometry.turn_clear(stop,math.pi/2,-math.pi/2)


def test_plans_keep_five_goals_heights_and_clear_every_turn(comparison):
    mission,maps,plans = comparison
    for course_map,plan in zip(maps,plans):
        geometry = RouteGeometry(course_map)
        start = np.array(plan['start_metres'][:2])
        heading = math.pi/2
        for stage,goal in zip(plan['stages'],mission['goals']):
            assert not stage.get('reverse_first_segment',False)
            np.testing.assert_allclose(stage['points'][-1],goal['point_metres'])
            checked = ChicaneErrorMargin(geometry) if stage['name']=='Chicane exit' else geometry
            for point in stage['points']:
                assert point[2]==.75
                end = np.array(point[:2])
                wanted = math.atan2(end[1]-start[1],end[0]-start[0])
                assert checked.turn_clear(start,heading,wanted)
                assert checked.segment_clear(start,end,wanted)
                start,heading = end,wanted
        assert plan['stages'][1]['points'][0][0]>plan['stages'][0]['points'][-1][0]


def test_moved_obstacle_requires_and_receives_a_different_route(comparison):
    _,maps,plans = comparison
    assert plans[0]['stages'][1]['points']!=plans[1]['stages'][1]['points']
    moved = RouteGeometry(maps[1])
    start = np.array(plans[0]['stages'][0]['points'][-1][:2])
    old_route_blocked = False
    for point in plans[0]['stages'][1]['points']:
        end = np.array(point[:2]);heading = math.atan2(end[1]-start[1],end[0]-start[0])
        old_route_blocked |= not moved.segment_clear(start,end,heading)
        start = end
    assert old_route_blocked


def test_unreachable_goal_is_refused():
    geometry = RouteGeometry(load_our_map('sketch_2026-11-08'))
    geometry.segment_clear = lambda *args: False
    with pytest.raises(ValueError,match='no forward route'):
        search_route(geometry,[-2.5,1.],[1.,2.],math.pi/2)


def test_full_airframe_clearance_agrees_with_dense_propeller_rings():
    # Independent ring sampling catches a sign/frame error in the analytic polygon check.
    geometry = RouteGeometry(load_our_map('sketch_2026-11-08'))
    from team_nav.course_map import distance_to_shapes_metres
    angles = np.linspace(0,2*math.pi,720)
    rings = np.concatenate([np.column_stack((f+.0889*np.cos(angles),l+.0889*np.sin(angles)))
                           for f,l in [(.08,.16),(.08,-.16),(-.15,.14),(-.15,-.14)]])
    centre = np.array([-2.5146,2.451])
    for heading in np.linspace(-.24,math.pi/2,25):
        rotation = np.array([[math.cos(heading),-math.sin(heading)],[math.sin(heading),math.cos(heading)]])
        distance,_ = distance_to_shapes_metres(rings@rotation.T+centre,geometry.shapes)
        exact = geometry.footprint_clearance([centre],heading)[0]
        assert exact<=distance.min()+1e-5
        assert exact>=.05
