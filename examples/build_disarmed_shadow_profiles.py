"""Build simulator DISARMED diagnostic profiles from a captured native probe.

Uncertainty settings are labeled sensitivity assumptions, not measured flight
bounds. The produced launcher is shadow-only with no reviewed landing site.
"""
import argparse
from dataclasses import asdict
from hashlib import sha256
import json
import math
from pathlib import Path

import numpy as np

from examples.gazebo_sensor_contract import OPTICAL_TO_FLU, read_camera_rig
from examples.h1_observation import HybridBagProfile, PX4_TYPES
from examples.hybrid_launch import configured
from onboard.adapters.geometry import quat_to_mat
from refnav.reasoning.schema import MissionGoal
from simulator.gazebo_trial.make_hybrid_profile import assemble

ROOT = Path(__file__).resolve().parents[1]
NAMES = ('hybrid_existing', 'hybrid_information_gain', 'hybrid_learned_ranker', 'hybrid_physics_informed')


def diagnostic_profile(probe, evidence_id):
    if (probe.get('tracking_source') != 'simulated_estimator'
            or not all(probe.get(k) is True for k in ('disarmed', 'landed', 'healthy', 'advancing_clock'))
            or probe.get('missing') != [] or probe.get('graph', {}).get('issues') != []):
        raise ValueError('fresh complete disarmed/landed healthy simulator probe required')
    last = probe['last_observations']
    info = last['/simulator_only/oak/depth/camera_info']
    tf, = [r for r in last['/tf_static']['evidence'] if r['frame'] == 'base_link' and r['child_frame'] == info['frame']]
    rig = read_camera_rig(ROOT / 'simulator/gazebo_trial/models')
    q = tf['rotation_xyzw']
    if (tf['valid'] is not True or (info['width'], info['height']) != (rig.width, rig.height)
            or not np.allclose(tf['translation_m'], rig.translation, atol=1e-6, rtol=0)
            or not np.allclose(quat_to_mat(q[3], *q[:3]), OPTICAL_TO_FLU, atol=1e-6, rtol=0)):
        raise ValueError('native camera rig differs from pinned simulator model')
    if any(info['d']) or info['p'][3] or info['p'][7]:
        raise ValueError('rectified zero-offset camera projection required')
    variances = [v for row in probe['px4_clock_samples'] for v in row['position_variance']]
    if not variances or not all(math.isfinite(v) and v >= 0 for v in variances):
        raise ValueError('finite observed PX4 covariance required')
    variance = max(variances)
    worst_row, worst_axis = max(
        ((row, axis) for row in probe['px4_clock_samples'] for axis in range(3)),
        key=lambda item: item[0]['position_variance'][item[1]])
    now = last['/clock']['sim_ns']
    reference = {'frame_id': 'odom', 'clock_id': 'gazebo_sim', 'epoch': 0, 'body_frame': 'base_link'}
    document = {
        'scope': 'DISARMED_SIMULATOR_DIAGNOSTIC_ONLY_NOT_MOTION_REVIEWED',
        'spatial': {'record_clock': 'ros_sim', 'rectified': True, 'reference': reference,
            'position_error_m': .03, 'angle_error_rad': .02,
            'calibration': {'camera_frame': info['frame'], 'body_frame': 'base_link',
                'width': info['width'], 'height': info['height'],
                'K': [info['k'][0], info['k'][4], info['k'][2], info['k'][5]],
                'R_body_optical': OPTICAL_TO_FLU, 't_body_optical_m': tf['translation_m'],
                'depth_error_m': .015, 'calibration_id': 'native-probe-' + evidence_id[:16],
                'validated_pixel_footprint': True, 'extrinsic_position_error_m': 0., 'extrinsic_angle_error_rad': 0.}},
        'clock': {'source_clock_id': 'px4_sim', 'ros_clock_id': 'gazebo_sim', 'offset_ns': 0,
            'max_error_ns': 0, 'valid_from_ns': max(0, now-10_000_000_000),
            'valid_until_ns': now+180_000_000_000, 'evidence_id': 'sim-identity-clock-probe-' + evidence_id[:16]},
        'ned_frame': 'local_ned',
        'alignment': {'time_origin_ns': 0, 'map_from_enu_yaw_rad': -math.pi/2,
            'ned_origin_in_map_m': [0., 0., 0.], 'position_error_m': .10, 'angle_error_rad': .03,
            'provenance': 'diagnostic-authored-simulator-axes',
            'estimator_reset_counter': last['/fmu/out/vehicle_odometry']['evidence'][0]['reset'][0]},
        'vehicle': {'radius_m': .35, 'state_position_error_m': 3*math.sqrt(variance), 'tracking_error_m': .1},
        'objective_m': [.4, 0., .8], 'px4_topics': {role: '/fmu/out/'+role for role in PX4_TYPES},
        'ev_reset_counter': last['/fmu/in/vehicle_visual_odometry']['evidence'][0]['reset'][0],
        'local_reset_counters': last['/fmu/out/vehicle_local_position']['evidence'][0]['reset'],
        'tracking_source': 'simulated_estimator', 'mission_goal': asdict(MissionGoal()),
        'diagnostic_covariance_provenance': {
            'formula': '3 * sqrt(max position variance over all probe samples and axes)',
            'variance_m2': variance, 'axis_ned': ('north', 'east', 'down')[worst_axis],
            'source_publish_us': worst_row.get('px4_publish_us'),
            'receipt_ns': worst_row.get('receipt_ns'),
            'isotropic_application': True,
            'qualification': 'diagnostic sensitivity only; covariance is not a certified error bound'},
        'diagnostic_assumptions': ['Pose/depth/TF uncertainty and identity clock are sensitivity assumptions.',
            'Observed covariance is not a worst-case physical error bound.',
            'No free-space bubble; reference-map control and background registration disabled.',
            'Use only for current captured reset epoch; changed reset markers must refuse.']}
    HybridBagProfile.from_dict(document)
    return document


def build(probe_path, output_dir):
    probe_path, output_dir = Path(probe_path), Path(output_dir)
    raw = probe_path.read_bytes()
    digest = sha256(raw).hexdigest()
    base = diagnostic_profile(json.loads(raw), digest)
    reference = {key: base['spatial']['reference'][key] for key in ('frame_id', 'epoch', 'clock_id')}
    course = {'reference': reference, 'route_review': 'Disarmed hypothetical objective only; no flight approval',
        'stages': [{'stage_id': 'diagnostic', 'kind': 'waypoint', 'position_m': [.4, 0., .8],
                    'semantic': asdict(MissionGoal()), 'metric_fallback': True, 'timeout_s': 30},
                   {'stage_id': 'unused_landing', 'kind': 'landing', 'position_m': [0., 0., 0.]}]}
    output_dir.mkdir(parents=True, exist_ok=False)
    course_path = output_dir/'course.json'
    course_path.write_text(json.dumps(course, indent=2)+'\n', encoding='utf-8')
    outputs = []
    for name in NAMES:
        document = dict(base, strategy=json.loads((ROOT/'scenarios/strategies'/f'{name}.json').read_text()))
        h1 = output_dir/f'{name}-h1.json'
        h1.write_text(json.dumps(document, indent=2)+'\n', encoding='utf-8')
        launcher = output_dir/f'{name}.json'
        settings = assemble(h1, course_path, launcher, mode='hybrid_shadow', profile_id='disarmed-'+name+'-'+digest[:12])
        settings['review_scope'] = base['scope']
        launcher.write_text(json.dumps(settings, indent=2)+'\n', encoding='utf-8')
        mode, runtime = configured(launcher)
        runtime.close()
        outputs.append(str(launcher))
    (output_dir/'scope.json').write_text(json.dumps({'probe_sha256': digest, 'scope': base['scope'],
        'positive_motion_expected': False, 'map_control': False, 'background_registration': False,
        'profiles': outputs}, indent=2)+'\n', encoding='utf-8')
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(build(args.probe, args.output_dir)))


if __name__ == '__main__':
    main()
