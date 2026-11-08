"""Check the updated map's direct leg and refuse unsupported window crossings."""
import math
import numpy as np
from our_files import load_our_map
from team_nav.practice_flights.corrected_course import first_two_plan, window_proposal
from team_nav.practice_flights.astar_geometry import RouteGeometry
import pytest


def test_corrected_first_two_goals_are_direct_and_clear():
    course_map = load_our_map('real_course_2026-11-08')
    plan = first_two_plan(course_map)
    assert len(plan['stages']) == 2
    assert all(len(stage['points']) == 1 for stage in plan['stages'])
    start, end = [np.array(stage['points'][0][:2]) for stage in plan['stages']]
    heading = math.atan2(end[1] - start[1], end[0] - start[0])
    room = RouteGeometry(course_map).footprint_clearance(np.linspace(start, end, 300), heading)
    assert room.min() > .24


def test_through_window_requires_live_alignment():
    proposal = window_proposal(load_our_map('real_course_2026-11-08'), 'through')
    assert not proposal['flight_ready']
    assert proposal['requires_sensor_alignment']
    assert not proposal['refusal_reasons']
    assert np.isclose(proposal['crossing_height_metres'], 1.3626)
    assert proposal['points_metres'][0][2] == proposal['points_metres'][-1][2] == .75


def test_over_window_refuses_course_ceiling_violation():
    proposal = window_proposal(load_our_map('real_course_2026-11-08'), 'over')
    assert not proposal['flight_ready']
    assert proposal['crossing_height_metres'] > proposal['target_height_limit_metres']
    assert 'target-height limit' in ' '.join(proposal['refusal_reasons'])


def test_missing_window_cannot_silently_be_used():
    course_map = load_our_map('real_course_2026-11-08')
    for wall in course_map.walls:
        wall.openings = []
    with pytest.raises(StopIteration):
        window_proposal(course_map, 'through')


@pytest.mark.parametrize('alternative', ['through', 'over'])
def test_refused_routes_never_start_ros_or_publish_targets(monkeypatch, alternative):
    from types import SimpleNamespace
    from team_nav.practice_flights import corrected_course
    from pathlib import Path
    import our_files
    def unexpected_flight(*args, **kwargs):
        pytest.fail('An infeasible proposal must not start flight control.')
    monkeypatch.setattr(corrected_course.first_maze_leg, 'fly_with_ros', unexpected_flight)
    monkeypatch.setattr(corrected_course, 'find_package_file', lambda *args: Path(our_files.MISSIONS_FOLDER).parent/'course_maps'/corrected_course.MAP_NAME)
    if alternative == 'through':
        course_map = load_our_map('real_course_2026-11-08')
        opening = next(wall for wall in course_map.walls if wall.openings).openings[0]
        opening.end_along_wall_metres = opening.start_along_wall_metres + .58
        monkeypatch.setattr(corrected_course,'load_course_map',lambda *args:course_map)
    log = SimpleNamespace(save_text_file=lambda *args: None, event=lambda *args: None)
    assert corrected_course.run(['--route', alternative], SimpleNamespace(stop_after_stage=None), log) == 2
