import json
from pathlib import Path

import numpy as np
import pytest

from examples.build_s0_course_ablation import ROOT, SCOPE, build, course_document, participant_geometry
from examples.hybrid_launch import configured
from onboard.adapters.geometry import R_ENU_NED
from onboard.adapters.geometry import rot_z

MAP = ROOT/'drone_code/team_nav/course_maps/phone_scan_2026-11-06.json'
MISSION = ROOT/'drone_code/team_nav/missions/dronehacks_2026_side_gap_first.json'
PROBE = ROOT/'artifacts/taskforce_batch_008/c1-check/disarmed-1.json'


def test_route_retains_four_checkpoint_order_and_landing():
    course = course_document(MAP, MISSION, dict(frame_id='participant_course', clock_id='gazebo_sim', epoch=0))
    marks = course['participant_route']['checkpoint_waypoints']
    assert [line for _, line, _ in marks] == ['end of first corridor', 'Gate A', 'tunnel exit', 'Gate B']
    assert [index for _, _, index in marks] == sorted(index for _, _, index in marks)
    assert len(course['stages']) == 23
    assert course['stages'][-1]['kind'] == 'landing'
    assert np.allclose(course['stages'][-1]['position_m'], [-1.7069, -.4724, .76])
    assert all(s['semantic']['intent'] == 'investigate_candidate_passage' and s['metric_fallback'] for s in course['stages'])
    assert not any(s.get('scan_on_arrival') for s in course['stages'])


def test_generated_profile_preserves_sensor_covariance_and_course_transform(tmp_path):
    launcher = build(PROBE, tmp_path/'profiles', map_path=MAP, mission_path=MISSION)
    settings = json.loads(launcher.read_text())
    profile = json.loads((launcher.parent/'h1.json').read_text())
    assert settings['experiment_scope'] == SCOPE
    assert settings['strategy'] == 'hybrid_shadow'
    assert 'stopping_envelope' not in settings and 'map_assisted' not in settings
    assert profile['strategy']['identifier'] == 'hybrid_existing'
    assert profile['vehicle']['state_position_error_m'] > .4  # Native recorded covariance retained.
    origin = np.array(profile['alignment']['ned_origin_in_map_m'])
    assert profile['spatial']['reference']['frame_id'] == 'odom'
    assert profile['alignment']['map_from_enu_yaw_rad'] == -np.pi/2
    assert np.allclose(origin, [0., 0., 0.])
    local = rot_z(-np.pi/2) @ R_ENU_NED @ [1., 0., -.76]
    mapped = rot_z(settings['course_from_odom_yaw_rad']) @ local + settings['course_from_odom_translation_m']
    assert np.allclose(mapped, [-2.5146, 1.9144, .8])
    course = json.loads((launcher.parent/'course.json').read_text())
    assert np.allclose(course['stages'][0]['position_m'], [0., 0., .76])
    assert np.allclose(settings['course_from_odom_translation_m'], [-2.5146, .9144, .04])
    east_in_local = rot_z(-np.pi/2) @ R_ENU_NED @ [0., 1., 0.]
    assert np.allclose(east_in_local, [0., -1., 0.])
    mode, runtime = configured(launcher)
    try:
        assert mode == 'hybrid_shadow' and runtime.loop.replay.hybrid.reasoner.variant == 'G0'
        assert not runtime.executor.gateway.enabled
        assert runtime.executor.stopping_envelope is None
    finally:
        runtime.close()
    with pytest.raises(FileExistsError):
        build(PROBE, launcher.parent, map_path=MAP, mission_path=MISSION)


def test_input_probe_failure_does_not_create_profile(tmp_path):
    probe = json.loads(PROBE.read_text())
    probe['healthy'] = False
    source = tmp_path/'probe.json'
    source.write_text(json.dumps(probe))
    with pytest.raises(ValueError, match='healthy simulator probe'):
        build(source, tmp_path/'out', map_path=MAP, mission_path=MISSION)
    assert not (tmp_path/'out').exists()


def test_participant_geometry_remains_real_obstacle_shapes():
    course = participant_geometry(MAP)
    from team_nav.course_map import shapes_blocking_between_heights, distance_to_shapes_metres
    shapes = shapes_blocking_between_heights(course, .6, 1.)
    assert len(shapes) > 0
    distances, _ = distance_to_shapes_metres(np.array([[-2.5146, .9144]]), shapes)
    assert .35 < distances[0] < .6  # Real narrow start; never fabricated empty world.


def test_actual_d0_closed_loop_crosses_all_route_stages_without_classic_follower(tmp_path):
    from examples.s0_course_rollout import run
    launcher = build(PROBE, tmp_path/'rollout', map_path=MAP, mission_path=MISSION)
    report = run(launcher, MAP, max_steps=1000)
    assert report['result'] == 'hybrid mission complete'
    assert report['final_stage'] == report['stage_count'] - 1 == 22
    assert [row['reached_stage'] for row in report['reached_stages']] == list(range(22))
    assert report['observed_free_voxels'] == 0
    assert report['counts']['g0:insufficient_information'] == report['steps']
    assert any(key.startswith('candidate:') for key in report['counts'])
