"""Conservative offline preparation, with explicit exclusions and no weight promotion.

All joins use PX4 source time; receipt/source clock agreement is required.
Sessions sharing a code fingerprint stay in the same split. Raw data is read-only.
"""
import argparse
import bisect
import collections
import gzip
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.residual import residual_features, RESIDUAL_LIMITS


def load(path, *, sample=False):
    result, rejected = [], 0
    if not path.exists():
        return result, 0
    with gzip.open(path, 'rt', newline='') as f:
        for row in csv.DictReader(f):
            try:
                if None in row or any(v is None or '\0' in v for v in row.values()):
                    raise ValueError()
                published = float(row['timestamp']) * 1e-6
                t = float(row.get('timestamp_sample', row['timestamp'])) * 1e-6 if sample else published
                receipt = float(row['received_unix_time_seconds'])
                if not np.isfinite([t, receipt, published]).all() or not -.05 <= receipt-published <= .1 or t>published or t<=0:
                    raise ValueError()
                if result and t <= result[-1][0]:
                    # A source-time discontinuity cannot be repaired by sorting.
                    return [], rejected + len(result) + 1
                result.append((t, row))
            except (ValueError, KeyError, TypeError):
                rejected += 1
    return result, rejected


def preceding(stream, times, t, age):
    index = bisect.bisect_right(times, t) - 1
    if index < 0 or t-stream[index][0] > age:
        raise ValueError('missing_or_stale_join')
    return stream[index][1]


def yaw_rate(row):
    q = np.array([float(row[f'q[{i}]']) for i in range(4)])
    rates = np.array([float(row[f'angular_velocity[{i}]']) for i in range(3)])
    if row.get('pose_frame') != '1' or not np.isfinite(np.r_[q, rates]).all() or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('invalid_attitude_rate')
    w, x, y, z = q
    roll = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    pitch = math.asin(np.clip(2*(w*y-z*x), -1, 1))
    if max(abs(roll), abs(pitch)) > math.radians(20):
        raise ValueError('excessive_tilt')
    return (rates[1]*math.sin(roll)+rates[2]*math.cos(roll))/math.cos(pitch)


def prepare(source, output):
    output.mkdir(parents=True, exist_ok=False)
    reports, datasets = [], collections.defaultdict(list)
    nominal = DynamicsModel()
    groups = collections.defaultdict(set)
    for meta_path in sorted(source.rglob('session.json.gz')):
        with gzip.open(meta_path, 'rt') as f:
            meta = json.load(f)
        folder = meta_path.parent
        fingerprint = meta.get('team_nav_overall_fingerprint')
        if not fingerprint:
            reports.append({'session': str(folder.relative_to(source)), 'excluded': 'missing_code_fingerprint'})
            continue
        # Conservative grouping: repeated/overlapping sessions on the same code
        # cannot leak between train and validation/test.
        group = hashlib.sha256((str(meta.get('computer_name')) + fingerprint).encode()).hexdigest()
        bucket = int(group[:8], 16) % 10
        split = 'test' if bucket == 0 else 'validation' if bucket == 1 else 'train'
        groups[split].add(group)
        names = {'state': 'vehicle_local_position', 'status': 'vehicle_status_v1',
                 'land': 'vehicle_land_detected', 'flags': 'estimator_status_flags', 'odometry': 'vehicle_odometry'}
        streams, corrupt = {}, {}
        for key, name in names.items():
            streams[key], corrupt[key] = load(folder / 'topics' / ('fmu__out__' + name + '.csv.gz'),sample=key in ('state','odometry'))
        streams['command'], corrupt['command'] = load(folder / 'topics/fmu__in__trajectory_setpoint.csv.gz')
        times = {k: [r[0] for r in v] for k, v in streams.items()}
        exclusions, accepted = collections.Counter(), 0
        reference = LocalNED('real_px4_ned', 0, 'recorded_px4_source')
        state_rows = streams['state']
        for index, (t,a) in enumerate(state_rows):
            next_index = bisect.bisect_left(times['state'],t+.02)
            if next_index>=len(state_rows):
                break
            end,b = state_rows[next_index]
            try:
                dt = end-t
                if not .01 <= dt <= .05:
                    raise ValueError('state_gap')
                window = state_rows[index:next_index+1]
                if any(a[k] != row[k] for _,row in window for k in a if k.endswith('_reset_counter')):
                    raise ValueError('estimator_reset')
                for _,row in window:
                    if not all(row.get(k) == 'True' for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control')):
                        raise ValueError('state_invalid')
                    if row.get('dead_reckoning') != 'False':
                        raise ValueError('dead_reckoning')
                for stamp in (t, end):
                    status = preceding(streams['status'], times['status'], stamp, .75)
                    land = preceding(streams['land'], times['land'], stamp, .5)
                    flags = preceding(streams['flags'], times['flags'], stamp, 1.25)
                    if status.get('arming_state') != '2' or status.get('nav_state') != '14' or status.get('failsafe') != 'False':
                        raise ValueError('not_healthy_armed_offboard')
                    if land.get('landed') != 'False' or land.get('ground_contact') != 'False':
                        raise ValueError('ground_contact')
                    if flags.get('cs_ev_pos') != 'True' or flags.get('cs_yaw_align') != 'True':
                        raise ValueError('external_vision_or_heading_unconfirmed')
                command = preceding(streams['command'], times['command'], t, .1)
                end_command = preceding(streams['command'], times['command'], end, .1)
                keys = [f'position[{i}]' for i in range(3)] + ['yaw']
                if any(command[k] != end_command[k] for k in keys):
                    raise ValueError('command_changed_inside_window')
                target = [float(command[k]) for k in keys]
                if not np.isfinite(target).all():
                    raise ValueError('not_finite_position_yaw_control')
                position = [float(a[k]) for k in ('x', 'y', 'z')]
                velocity = [float(a[k]) for k in ('vx', 'vy', 'vz')]
                next_velocity = [float(b[k]) for k in ('vx', 'vy', 'vz')]
                heading, next_heading = float(a['heading']), float(b['heading'])
                odom = preceding(streams['odometry'], times['odometry'], t, .03)
                next_odom = preceding(streams['odometry'], times['odometry'], end, .03)
                if odom['reset_counter'] != next_odom['reset_counter']:
                    raise ValueError('odometry_reset')
                if odom['timestamp_sample'] == next_odom['timestamp_sample']:
                    raise ValueError('yaw_rate_not_advancing')
                rate, next_rate = yaw_rate(odom), yaw_rate(next_odom)
                state = DroneState(reference, t, position, velocity, heading, rate)
                command_model = PositionSetpoint(reference, t, end, target[:3], target[3])
                tracking = nominal._tracking(state, command_model, dt)
                features, gate = residual_features(state, command_model, dt, tracking)
                acceleration = np.r_[(np.array(next_velocity)-velocity)/dt, (next_rate-rate)/dt]
                limits = np.r_[nominal.parameters.acceleration_limits_mps2, nominal.parameters.yaw_acceleration_limit_rad_s2]
                if not np.isfinite(acceleration).all() or np.any(np.abs(acceleration) > limits):
                    raise ValueError('acceleration_outside_model_domain')
                residual = acceleration - np.r_[tracking[1], tracking[3]]
                label = np.clip(residual / RESIDUAL_LIMITS, -np.array(gate), gate)
                datasets[split].append((features, gate, label, t, group))
                accepted += 1
            except (ValueError, KeyError, TypeError) as error:
                exclusions[str(error) or 'invalid_numeric'] += 1
        reports.append({'session': folder.relative_to(source).as_posix(), 'group': group, 'split': split,
                        'source_rows': {k: len(v) for k, v in streams.items()},
                        'malformed_or_clock_rejected': corrupt, 'accepted_samples': accepted,
                        'exclusions': dict(exclusions)})
    report = {'training_ready': False, 'weights_changed': False,
              'reason': 'Candidate arrays require reviewed calibration, noise/label-quality and held-out flight coverage before weight promotion.',
              'groups': {k: sorted(v) for k, v in groups.items()}, 'sessions': reports,
              'accepted_samples': sum(r.get('accepted_samples', 0) for r in reports)}
    report['arrays'] = {}
    for split, values in datasets.items():
        path = output / (split + '.npz')
        np.savez_compressed(path, features=np.array([v[0] for v in values], dtype=np.float32),
                            gates=np.array([v[1] for v in values], dtype=np.float32),
                            targets=np.array([v[2] for v in values], dtype=np.float32),
                            source_seconds=np.array([v[3] for v in values]), groups=np.array([v[4] for v in values]))
        report['arrays'][split] = {'samples': len(values), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}
    (output / 'manifest.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = prepare(args.source, args.output)
    print(json.dumps({'accepted_samples': result['accepted_samples'], 'training_ready': result['training_ready']}))
