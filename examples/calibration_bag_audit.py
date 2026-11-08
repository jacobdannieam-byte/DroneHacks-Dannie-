"""Offline native event/command correlation. No ROS nodes or command publishers.

Run inside the pinned ROS image with a finalized raw bag and recorder JSON.
Correlation is necessary evidence, not an independent flight approval.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sqlite3


def correlate(recording, owner_events, commands, targets, modes=()):
    if recording.get('provenance_scope') != 'reviewed_calibration' or recording.get('recording_complete') is not True:
        raise ValueError('complete reviewed calibration recording required')
    if recording.get('recording_faults') or recording.get('pending_clock_samples'):
        raise ValueError('faulted or pending recording')
    identity = {k: recording[k] for k in ('trial_id', 'owner_id', 'profile_sha256')}
    rows = [e for e in recording['events'] if e['event'] != 'disarm']
    expected_names = ['land_command'] if recording['trial_kind'] == 'hover_land' else ['brake_command', 'land_command']
    if [e['event'] for e in rows] != expected_names or len(owner_events) != len(rows):
        raise ValueError('missing or unexpected raw owner events')
    for event in rows:
        expected = {**identity, **{k:event[k] for k in ('event','stamp_ns','px4_issue_us')}}
        if sum(raw == expected for raw in owner_events) != 1:
            raise ValueError('recorder event does not match raw owner message')
        stamp = event['px4_issue_us']
        if event['event'] == 'land_command':
            matching = [c for c in commands if c['timestamp'] == stamp and c['command'] == 21]
            if len(matching) != 1 or any(matching[0].get(k) != 1 for k in ('target_system','target_component')):
                raise ValueError('land event lacks unique native LAND command')
        else:
            matching = [t for t in targets if t['timestamp'] == stamp]
            mode = [m for m in modes if m['timestamp'] == stamp]
            if (len(matching) != 1 or len(matching[0]['velocity']) != 3 or
                    any(not math.isfinite(v) or v != 0 for v in matching[0]['velocity']) or
                    len(matching[0].get('position', [])) != 3 or
                    not all(math.isnan(v) for v in matching[0]['position']) or
                    len(mode) != 1 or mode[0].get('velocity') is not True or
                    mode[0].get('position') is not False):
                raise ValueError('brake event lacks native zero-velocity setpoint')
    if sum(c['command'] == 21 for c in commands) != 1:
        raise ValueError('unexpected LAND command count')
    return {'raw_command_correlation': 'PASS', 'trial_id':identity['trial_id'],
            'review_status':'unreviewed', 'events_matched':len(rows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    raw = args.recording.read_bytes()
    recording = json.loads(raw)
    databases = list(args.bag.glob('*.db3'))
    if len(databases) != 1 or not (args.bag/'metadata.yaml').is_file():
        raise ValueError('one finalized unsplit raw SQLite recording required')
    if any(p.stat().st_size for p in args.bag.glob('*.db3-wal')):
        raise ValueError('raw bag must be finalized with empty WAL')
    database = databases[0]
    before = hashlib.sha256(database.read_bytes()).hexdigest()
    events, commands, targets, modes = [], [], [], []
    with sqlite3.connect(database.resolve().as_uri()+'?mode=ro&immutable=1',uri=True) as db:
        if db.execute('pragma quick_check').fetchone()[0] != 'ok':
            raise ValueError('raw bag integrity failed')
        for topic_id, name, kind in db.execute('select id,name,type from topics'):
            if name not in ('/qualification/events','/fmu/in/vehicle_command','/fmu/in/trajectory_setpoint','/fmu/in/offboard_control_mode'):
                continue
            for (data,) in db.execute('select data from messages where topic_id=? order by id',(topic_id,)):
                m = deserialize_message(data,get_message(kind))
                if name == '/qualification/events': events.append(json.loads(m.data))
                elif name.endswith('vehicle_command'):
                    commands.append({k:getattr(m,k) for k in ('timestamp','command','target_system','target_component')})
                elif name.endswith('trajectory_setpoint'):
                    targets.append(dict(timestamp=m.timestamp,velocity=list(m.velocity),position=list(m.position)))
                else: modes.append(dict(timestamp=m.timestamp,velocity=m.velocity,position=m.position))
    result = correlate(recording,events,commands,targets,modes)
    if (before != hashlib.sha256(database.read_bytes()).hexdigest() or
            list(args.bag.glob('*.db3')) != databases or
            any(p.stat().st_size for p in args.bag.glob('*.db3-wal'))):
        raise ValueError('raw recording changed during audit')
    result.update(raw_bag_sha256=before,recording_sha256=hashlib.sha256(raw).hexdigest())
    with args.output.open('x') as stream: json.dump(result,stream,indent=2)


if __name__ == '__main__': main()
