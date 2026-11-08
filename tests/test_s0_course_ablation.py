import json
from copy import deepcopy
from pathlib import Path

import numpy as np
import pytest

from examples.build_s0_course_ablation import ROOT, SCOPE, build, course_document, participant_geometry, ground_alignment
from examples.hybrid_launch import configured
from onboard.adapters.geometry import R_ENU_NED
from onboard.adapters.geometry import rot_z

MAP = ROOT/'drone_code/team_nav/course_maps/phone_scan_2026-11-06.json'
MISSION = ROOT/'drone_code/team_nav/missions/dronehacks_2026_side_gap_first.json'
PROBE = ROOT/'artifacts/taskforce_batch_008/c1-check/disarmed-1.json'


def stable_probe(tmp_path):
    """Real rig fixture with explicitly synthetic stationary frame-offset samples."""
    probe = json.loads(PROBE.read_text())
    last = probe['last_observations']
    now = last['/clock']['sim_ns']
    source = probe['px4_clock_samples'][-1]
    rows = []
    for stamp in range(now-500_000_000, now, 10_000_000):
        row = deepcopy(source)
        row['position'] = [.002, -.003, .045]
        row['receipt_ns'] = stamp+1_000_000
        row['evidence'][0]['stamp_ns'] = stamp
        rows.append(row)
    probe['px4_clock_samples'] = rows
    last['/gazebo_hybrid/simulated_estimator/odometry'].update(
        position=[.005,.004,-.001], capture_ns=now-80_000_000, receipt_ns=now-20_000_000)
    last['/fmu/out/vehicle_local_position']['evidence'][0].update(speed_mps=.01, stamp_ns=now-20_000_000)
    path = tmp_path/'synthetic-stationary-probe.json'
    path.write_text(json.dumps(probe))
    return path


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
    source = stable_probe(tmp_path)
    launcher = build(source, tmp_path/'profiles', map_path=MAP, mission_path=MISSION)
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
    assert np.allclose(origin, [.003, .001, .044])
    rotation = rot_z(-np.pi/2) @ R_ENU_NED
    local = rotation @ (rotation.T @ (np.array([1.,0.,.76])-origin)) + origin
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
        build(source, launcher.parent, map_path=MAP, mission_path=MISSION)


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
    launcher = build(stable_probe(tmp_path), tmp_path/'rollout', map_path=MAP, mission_path=MISSION)
    report = run(launcher, MAP, max_steps=1000)
    assert report['result'] == 'hybrid mission complete'
    assert report['final_stage'] == report['stage_count'] - 1 == 22
    assert [row['reached_stage'] for row in report['reached_stages']] == list(range(22))
    assert report['observed_free_voxels'] == 0
    assert report['counts']['g0:insufficient_information'] == report['steps']
    assert any(key.startswith('candidate:') for key in report['counts'])


def test_ground_alignment_rejects_unsettled_or_unpaired_samples(tmp_path):
    probe = json.loads(stable_probe(tmp_path).read_text())
    aligned = ground_alignment(probe, 'a'*64)
    assert aligned['native_bracket_ns'][0] == aligned['visual_capture_ns'] == aligned['native_bracket_ns'][1]
    moving = deepcopy(probe)
    moving['px4_clock_samples'][-1]['position'][2] += .03
    with pytest.raises(ValueError, match='not settled'):
        ground_alignment(moving, 'a'*64)
    moving = deepcopy(probe)
    moving['last_observations']['/fmu/out/vehicle_local_position']['evidence'][0]['speed_mps'] = .4
    with pytest.raises(ValueError, match='stationary'):
        ground_alignment(moving, 'a'*64)
    missing = deepcopy(probe)
    stamp = aligned['visual_capture_ns']
    missing['px4_clock_samples'] = [r for r in missing['px4_clock_samples'] if abs(r['evidence'][0]['stamp_ns']-stamp)>20_000_000]
    with pytest.raises(ValueError, match='bracket'):
        ground_alignment(missing, 'a'*64)


def test_real_attempt002_source_ahead_of_callback_preserves_original_times():
    probe = json.loads((ROOT/'tests/fixtures/attempt002_ground_alignment.json').read_text())
    before = deepcopy(probe)
    assert max(r['evidence'][0]['stamp_ns']-r['receipt_ns'] for r in probe['px4_clock_samples']) == 4_000_000
    fit = ground_alignment(probe, probe['source_sha256'])
    assert fit['native_bracket_ns'] == [47_000_000_000, 47_000_000_000]
    assert np.allclose(fit['ned_origin_in_map_m'], [.0010321889440860084,-.000330248293988796,-.0030906565387330207])
    assert probe == before
    over = deepcopy(probe)
    over['px4_clock_samples'][0]['receipt_ns'] = over['px4_clock_samples'][0]['evidence'][0]['stamp_ns']-50_000_001
    with pytest.raises(ValueError, match='invalid native'):
        ground_alignment(over, probe['source_sha256'])
    not_caught_up = deepcopy(probe)
    not_caught_up['px4_clock_samples'][-1]['evidence'][0]['stamp_ns'] = probe['last_observations']['/clock']['sim_ns']+1
    with pytest.raises(ValueError, match='invalid native'):
        ground_alignment(not_caught_up, probe['source_sha256'])
