"""Read-only inventory of exported Classical telemetry; never exports training labels.

Counts describe recorded messages, not independent flights or qualified samples.
Usage: python -m examples.audit_classical_telemetry SOURCE_TELEMETRY OUTPUT_JSON
"""
import argparse
import collections
import csv
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np


def rows(path):
    with gzip.open(path, 'rt', newline='') as stream:
        yield from csv.DictReader(stream)


def numbers(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return ({'count': len(a), 'min': float(a.min()), 'median': float(np.median(a)),
             'p95': float(np.quantile(a, .95)), 'max': float(a.max())} if len(a) else {'count': 0})


def audit(root):
    files = sorted(p for p in root.rglob('*') if p.is_file())
    manifest = []
    for path in files:
        with path.open('rb') as stream:
            digest = hashlib.file_digest(stream, 'sha256').hexdigest()
        manifest.append({'path': path.relative_to(root).as_posix(), 'bytes': path.stat().st_size,
                         'sha256': digest})
    sessions = []
    for metadata in sorted(root.rglob('session.json.gz')):
        with gzip.open(metadata, 'rt') as f:
            meta = json.load(f)
        folder = metadata.parent
        topics = folder / 'topics'
        row = {'path': folder.relative_to(root).as_posix(), 'run_mode': meta.get('run_mode'),
               'code_fingerprint': meta.get('team_nav_overall_fingerprint'), 'streams': {}}
        for name in ('fmu__in__trajectory_setpoint', 'fmu__out__vehicle_local_position',
                     'fmu__out__vehicle_status_v1', 'fmu__out__vehicle_land_detected'):
            path = topics / (name + '.csv.gz')
            if not path.exists():
                continue
            count, armed, valid, position_commands, velocity_commands = 0, 0, 0, 0, 0
            malformed = 0
            resets, previous = collections.Counter(), {}
            times, source_times = [], []
            for item in rows(path):
                try:
                    if None in item or any(v is None or '\x00' in v for v in item.values()):
                        raise ValueError('truncated or NUL-containing row')
                    receipt = float(item['seconds_since_start'])
                    source = float(item['timestamp']) * 1e-6
                    if not np.isfinite(receipt) or not np.isfinite(source):
                        raise ValueError('invalid timestamp')
                    if name.endswith('trajectory_setpoint'):
                        command_position = [float(item[f'position[{i}]']) for i in range(3)]
                        command_velocity = [float(item[f'velocity[{i}]']) for i in range(3)]
                except (ValueError, TypeError, KeyError):
                    malformed += 1
                    continue
                count += 1
                times.append(receipt)
                source_times.append(source)
                armed += item.get('arming_state') == '2'
                valid += all(item.get(k) == 'True' for k in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid', 'heading_good_for_control'))
                if name.endswith('trajectory_setpoint'):
                    position_commands += all(np.isfinite(command_position))
                    velocity_commands += all(np.isfinite(command_velocity))
                for key, value in item.items():
                    if key.endswith('_reset_counter'):
                        resets[key] += key in previous and previous[key] != value
                        previous[key] = value
            row['streams'][name] = {'rows': count, 'malformed_rows_rejected': malformed, 'receipt_span_s': times[-1] - times[0] if times else 0,
                                   'receipt_gap_s': numbers(np.diff(times)),
                                   'source_nonincreasing_steps': int(np.sum(np.diff(source_times) <= 0)),
                                   'armed_rows': armed, 'all_position_velocity_heading_valid_rows': valid,
                                   'finite_position_command_rows': position_commands,
                                   'finite_velocity_command_rows': velocity_commands,
                                   'reset_counter_transitions': dict(resets)}
        sessions.append(row)
    depth = []
    pooled_depth = []
    for path in sorted(root.rglob('depth_camera.csv.gz')):
        items, malformed = [], 0
        for item in rows(path):
            try:
                parsed = {key: float(item[key]) for key in ('picture_time_seconds', 'calculation_seconds', 'trusted_fraction', 'points_published')}
                if not all(np.isfinite(list(parsed.values()))):
                    raise ValueError('nonfinite depth statistic')
            except (ValueError, TypeError, KeyError):
                malformed += 1
                continue
            items.append(parsed)
        pooled_depth.extend(items)
        depth.append({'path': path.relative_to(root).as_posix(), 'rows': len(items),
                      'malformed_rows_rejected': malformed,
                      **{key: numbers([float(r[key]) for r in items]) for key in
                         ('calculation_seconds', 'trusted_fraction', 'points_published')},
                      'capture_gap_s': numbers(np.diff([float(r['picture_time_seconds']) for r in items]))})
    return {'schema': 1, 'training_ready': False,
            'limitations': ['Presence and per-stream statistics do not establish synchronized healthy airborne segments.',
                            'Reset transitions count observed changes, not the number of resets during recording gaps.',
                            'No ground-truth labels or calibration acceptance inferred from file names.'],
            'file_count': len(files), 'bytes': sum(r['bytes'] for r in manifest),
            'depth_image_count': sum('/depth_pictures/' in r['path'] for r in manifest),
            'session_count': len(sessions),
            'sessions_with_recorded_armed_status': sum(s['streams'].get('fmu__out__vehicle_status_v1', {}).get('armed_rows', 0) > 0 for s in sessions),
            'pooled_depth': {key: numbers([r[key] for r in pooled_depth]) for key in ('calculation_seconds', 'trusted_fraction', 'points_published')},
            'sessions': sessions, 'depth_sessions': depth, 'manifest': manifest}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    report = audit(args.source)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({k: report[k] for k in ('file_count', 'bytes', 'depth_image_count', 'session_count', 'sessions_with_recorded_armed_status')}))
