"""Offline byte/SQLite audit of a finalized observation bundle, not flight approval.

Requires the original raw files. Hash agreement does not authenticate their
producer, decode sensor content, or establish vehicle clearance.
"""
import argparse
import hashlib
import json
from pathlib import Path

from simulator.gazebo_trial.recording_gate import bag_snapshot


def digest(path):
    result = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            result.update(block)
    return result.hexdigest()


def audit(manifest, recording_report, bag):
    files = manifest.get('raw_files', {})
    if not files:
        raise ValueError('raw evidence manifest is empty')
    verified = {}
    for name, expected in files.items():
        path = Path(name).resolve(strict=True)
        if (path.stat().st_size != expected['bytes'] or
                digest(path) != expected['sha256']):
            raise ValueError('raw evidence mismatch: ' + name)
        verified[path] = expected['sha256']
    bag = Path(bag).resolve(strict=True)
    # This recorder supports a single unsplit SQLite recording. Refuse silently
    # ignoring additional segments or an unmanifested live WAL.
    databases = list(bag.glob('*.db3'))
    recording_files = set(bag.glob('*.db3')) | set(bag.glob('*.db3-wal'))
    expected_db = bag / (bag.name + '_0.db3')
    if databases != [expected_db]:
        raise ValueError('expected exactly one unsplit recording database')
    for path in [expected_db, bag / 'metadata.yaml', *bag.glob('*.db3-wal')]:
        if path.resolve() not in verified:
            raise ValueError('recording file absent from manifest: ' + str(path))
    if recording_report.get('passed') is not True:
        raise ValueError('recording report did not pass')
    topics = recording_report.get('topics', {})
    required = {'/clock', '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_local_position'}
    if not required <= topics.keys():
        raise ValueError('required observation topics absent')
    minimum = recording_report.get('minimum_duration_ns')
    if type(minimum) is not int or minimum < 30_000_000_000:
        raise ValueError('at least 30 seconds of observation required')
    actual = bag_snapshot(bag, topics, finalized=True, minimum_duration_ns=minimum)
    if actual != recording_report or actual.get('passed') is not True:
        raise ValueError('raw SQLite contents disagree with recording report')
    # Recheck after SQLite inspection to reject raw files changing mid-audit.
    if any(digest(path) != sha for path, sha in verified.items()):
        raise ValueError('raw evidence changed during audit')
    if recording_files != set(bag.glob('*.db3')) | set(bag.glob('*.db3-wal')):
        raise ValueError('recording file set changed during audit')
    return {'byte_and_sqlite_audit': 'PASS', 'verified_files': len(verified),
            'clock_duration_ns': actual['clock_duration_ns'],
            'qualification': 'integrity only; no decoded telemetry, clearance, or motion approval'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('manifest', type=Path)
    parser.add_argument('recording_report', type=Path)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = audit(json.loads(args.manifest.read_text()),
                   json.loads(args.recording_report.read_text()), args.bag)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)


if __name__ == '__main__':
    main()
