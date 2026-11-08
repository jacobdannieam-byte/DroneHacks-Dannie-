"""Offline C0 evidence review; deserialize only, never initialize ROS or publish."""
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def main(root):
    bag = root / 'observations'
    paths = sorted(bag.glob('*.db3'))
    assert len(paths) == 1 and (bag / 'metadata.yaml').is_file(), 'finalized single bag required'
    assert not any(p.stat().st_size for p in bag.glob('*-wal')), 'nonempty WAL'
    initial = sha(paths[0])
    db = sqlite3.connect(paths[0].resolve().as_uri() + '?mode=ro&immutable=1', uri=True)
    assert db.execute('pragma quick_check').fetchone()[0] == 'ok'
    wanted = {'/fmu/out/vehicle_local_position', '/fmu/out/vehicle_status_v1',
              '/fmu/out/vehicle_land_detected', '/fmu/out/vehicle_command_ack',
              '/fmu/in/vehicle_command', '/fmu/in/trajectory_setpoint',
              '/fmu/in/offboard_control_mode', '/qualification/events'}
    data = {}
    for ident, name, kind in db.execute('select id,name,type from topics'):
        if name in wanted:
            cls = get_message(kind)
            data[name] = [(t, deserialize_message(blob, cls)) for t, blob in db.execute(
                'select timestamp,data from messages where topic_id=? order by id', (ident,))]
    db.close()
    assert sorted(bag.glob('*.db3')) == paths and sha(paths[0]) == initial
    assert not any(p.stat().st_size for p in bag.glob('*-wal')), 'WAL appeared'
    missing = wanted - data.keys()
    assert not missing, str(missing)
    messages = lambda suffix: [m for _, m in data[suffix]]
    commands = messages('/fmu/in/vehicle_command')
    arms = [m for m in commands if m.command == 400 and m.param1 == 1]
    lands = [m for m in commands if m.command == 21]
    faults = []
    if len(arms) != 1 or len(lands) != 1:
        faults.append('expected exactly one ARM and LAND')
    assert arms and lands, 'no flight command window'
    begin, land = arms[0].timestamp, lands[0].timestamp
    statuses = messages('/fmu/out/vehicle_status_v1')
    armed = [m for m in statuses if m.timestamp >= begin and m.arming_state == 2]
    disarmed = [m for m in statuses if m.timestamp > land and m.arming_state == 1]
    end = disarmed[0].timestamp if disarmed else statuses[-1].timestamp
    if not armed or not disarmed or end - land > 30_000_000:
        faults.append('missing timely native armed/disarmed transition')
    auto_land = [m for m in statuses if land < m.timestamp <= end and m.nav_state == 18 and m.arming_state == 2]
    flight_status = [m for m in statuses if begin - 750_000 <= m.timestamp <= end]
    status_gaps = [b.timestamp-a.timestamp for a,b in zip(flight_status,flight_status[1:])]
    status_duplicates = [dict(stamp_us=b.timestamp, identical_all_fields=(a == b),
        changed_fields=[k for k in a.get_fields_and_field_types() if getattr(a,k) != getattr(b,k)])
        for a,b in zip(flight_status,flight_status[1:]) if a.timestamp == b.timestamp]
    if not status_gaps or any(g < 0 or g > 750_000 for g in status_gaps) or any(not d['identical_all_fields'] for d in status_duplicates):
        faults.append('native status nonadvancing or stale gap')
    if not auto_land:
        faults.append('missing armed AUTO_LAND acknowledgment')
    landed = [m for m in messages('/fmu/out/vehicle_land_detected') if m.landed and end - 750_000 <= m.timestamp <= end + 750_000]
    if not landed:
        faults.append('no fresh landed report at disarm')
    all_positions = messages('/fmu/out/vehicle_local_position')
    positions = [m for m in all_positions if begin <= m.timestamp_sample <= end]
    assert positions, 'empty full-rate flight window'
    resets = ['xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter']
    first_reset = [getattr(positions[0], k) for k in resets]
    gaps = [b.timestamp_sample - a.timestamp_sample for a, b in zip(positions, positions[1:])]
    invalid = [m.timestamp_sample for m in positions if not all(getattr(m, k) for k in ['xy_valid','z_valid','v_xy_valid','v_z_valid']) or not all(math.isfinite(getattr(m, k)) for k in ['x','y','z','vx','vy','vz'])]
    changed = [m.timestamp_sample for m in positions if [getattr(m, k) for k in resets] != first_reset]
    speeds = [(math.sqrt(m.vx*m.vx + m.vy*m.vy + m.vz*m.vz), m.timestamp_sample) for m in positions]
    bounds = {k: [min(getattr(m,k) for m in positions), max(getattr(m,k) for m in positions)] for k in ['x','y','z']}
    if invalid or changed or any(g <= 0 or g > 100_000 for g in gaps):
        faults.append('invalid/reset/nonadvancing/gapped full-rate position')
    if max(speeds)[0] > .5:
        faults.append('actual speed exceeds .5m/s')
    if any(abs(getattr(m,k)) > .5 for m in positions for k in ['x','y']) or any(not -1.25 <= m.z <= .02 for m in positions):
        faults.append('position outside approved box')
    targets = messages('/fmu/in/trajectory_setpoint')
    bad_targets = [m.timestamp for m in targets if not all(math.isnan(v) for v in m.position) or any(abs(v) > 1e-7 for v in m.velocity[:2]) or not -.200001 <= m.velocity[2] <= 0]
    if bad_targets or any(m.timestamp >= land for m in targets):
        faults.append('unexpected position/horizontal/vertical or post-LAND setpoint')
    if any(m.command not in (176,400,21) or (m.command == 400 and m.param1 != 1) for m in commands):
        faults.append('unexpected native command')
    settle = [m for m in positions if land - 504_000 <= m.timestamp_sample < land]
    settle_ok = bool(settle) and settle[-1].timestamp_sample-settle[0].timestamp_sample >= 480_000 and all(.8 <= -m.z <= 1.2 and math.sqrt(m.vx*m.vx+m.vy*m.vy+m.vz*m.vz) <= .05 for m in settle)
    if not settle_ok:
        faults.append('full-rate pre-LAND settling window not established')
    recording = json.loads((root/'calibration_recording.json').read_text())
    raw_by_stamp = {m.timestamp_sample:m for m in all_positions}
    mismatches = []
    for row in recording['samples']:
        m = raw_by_stamp.get(row['source_stamp_us'])
        if m is None or row['position_m'] != [m.x,m.y,m.z] or row['velocity_mps'] != [m.vx,m.vy,m.vz]:
            mismatches.append(row['source_stamp_us'])
    if mismatches or recording['recording_faults'] or recording['pending_clock_samples'] or not recording['recording_complete']:
        faults.append('recorder raw binding/completeness failure')
    report = dict(scope='single_C0_offline_review_not_stopping_or_hardware_qualification',
        faults=faults, bag_sha256=initial, topic_counts={k:len(v) for k,v in data.items()},
        flight_window_us=[begin,end], land_us=land, land_to_disarm_s=(end-land)/1e6,
        auto_land_us=[m.timestamp for m in auto_land], landed_near_disarm_us=[m.timestamp for m in landed],
        full_rate_samples=len(positions), max_speed_mps_and_stamp_us=max(speeds), position_bounds_ned_m=bounds,
        sample_gap_us=[min(gaps),max(gaps)], invalid_samples=invalid, reset_changes=changed,
        status_gap_us=[min(status_gaps),max(status_gaps)] if status_gaps else [],
        status_duplicates=status_duplicates,
        pre_land_settling_check=settle_ok, recorder_samples=len(recording['samples']),recorder_raw_mismatches=mismatches,
        commands=[dict(stamp_us=m.timestamp,command=m.command,param1=m.param1,param2=m.param2) for m in commands],
        command_acks=[dict(stamp_us=m.timestamp,command=m.command,result=m.result) for m in messages('/fmu/out/vehicle_command_ack')],
        status_transitions=[dict(stamp_us=m.timestamp,armed=m.arming_state,nav=m.nav_state) for i,m in enumerate(statuses) if i==0 or (m.arming_state,m.nav_state)!=(statuses[i-1].arming_state,statuses[i-1].nav_state)],
        geometry_contact_proof=False)
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main(Path(sys.argv[1]))
