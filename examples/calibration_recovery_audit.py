"""Offline R1 audit of a finalized raw ROS bag and independently retained receipts."""

import argparse
from hashlib import sha256
import json
import math
from pathlib import Path
import sqlite3

from examples.h1_observation import ClockMapping, RESET_FIELDS
from refnav.calibration_trial import RECOVERY_LEASE_NS
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD


TOPICS = {
    'status': '/fmu/out/vehicle_status_v1', 'landed': '/fmu/out/vehicle_land_detected',
    'positions': '/fmu/out/vehicle_local_position', 'commands': '/fmu/in/vehicle_command',
    'targets': '/fmu/in/trajectory_setpoint', 'modes': '/fmu/in/offboard_control_mode',
}
OTHER_OFFBOARD_FLAGS = ('acceleration', 'attitude', 'body_rate', 'thrust_and_torque', 'direct_actuator', 'actuator')


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def validate_operator_receipt(owner, operator, *, handoff=None):
    """Validate receipt fields and optional file digests, separate from native evidence."""
    lease = owner['recovery_lease']
    land_ros, land_wall = operator['land_issue_ros_ns'], operator['land_issue_wall_ns']
    _require(operator['trial_id'] == owner['trial_id'] and operator['operator'] == owner['recovery_operator']
             and operator['command'] == 'px4-commander land' and operator['exit_code'] == 0
             and all(operator.get(k) is True for k in ('owner_exited', 'idle_motion_graph', 'fresh_healthy_in_lane'))
             and type(operator.get('status_px4_us')) is int and operator['status_px4_us'] > 0
             and type(land_ros) is int and type(land_wall) is int and land_ros > lease['expiry_detected_ros_ns']
             and 0 < land_wall - lease['expiry_detected_wall_ns'] <= 60_000_000_000,
             'Missing separately observed operator handoff and LAND receipt')
    if handoff is not None:
        _require(all(operator.get(k) == v for k, v in handoff.items()), 'Operator handoff artifacts do not match receipts')
    return land_ros, land_wall


def audit(owner, recording, operator, raw, *, position_mode):
    identity = ('trial_id', 'owner_id', 'profile_sha256', 'trial_kind')
    _require(owner['trial_kind'] == 'authority_expiry' and owner['result'] == 'planned_authority_expiry'
             and not owner['events'] and not owner['input_fault']
             and all(owner[k] == recording[k] for k in identity)
             and recording.get('provenance_scope') == 'reviewed_calibration'
             and recording['recording_complete'] is True and not recording['recording_faults']
             and recording['pending_clock_samples'] == 0,
             'Complete matched R1 owner and recorder evidence required')
    clock = ClockMapping(**recording['clock_mapping'])
    lease = owner['recovery_lease']
    last_ros, last_wall, last = (lease[k] for k in
        ('last_output_ros_ns', 'last_output_wall_ns', 'last_output_px4_us'))
    expiry_ros, expiry_wall = (lease[k] for k in ('expiry_detected_ros_ns', 'expiry_detected_wall_ns'))
    _require(all(type(n) is int for n in (last_ros, last_wall, last, expiry_ros, expiry_wall))
             and clock.source_stamp_us(last_ros) == last and expiry_ros > last_ros
             and lease['deadline_wall_ns'] == last_wall + RECOVERY_LEASE_NS
             and lease['deadline_wall_ns'] <= expiry_wall < lease['deadline_wall_ns'] + 300_000_000,
             'Invalid stationary lease expiry receipt')

    commands, targets, modes = (raw[k] for k in ('commands', 'targets', 'modes'))
    _require([m.command for m in commands] == [176, 400]
             and commands[0].param1 == 1. and commands[0].param2 == 6.
             and commands[1].param1 == 1. and commands[1].param2 == 0.
             and commands[0].timestamp < commands[1].timestamp < last,
             'Unexpected native command or forced disarm')
    stamps = [m.timestamp for m in targets]
    _require(bool(stamps) and stamps == [m.timestamp for m in modes] and stamps[-1] == last
             and all(0 < b - a <= 300_000 for a, b in zip(stamps, stamps[1:]))
             and all(len(m.position) == 3 and len(m.velocity) == 3 and len(m.acceleration) == 3
                     and all(math.isnan(v) for v in (*m.position, *m.acceleration, m.yaw, m.yawspeed))
                     and tuple(getattr(m, 'jerk', (0., 0., 0.))) == (0., 0., 0.) for m in targets)
             and all(tuple(m.velocity[:2]) == (0., 0.) and -.200001 <= m.velocity[2] <= 0. for m in targets)
             and all(m.velocity and not m.position and not any(getattr(m, flag, False) for flag in OTHER_OFFBOARD_FLAGS)
                     for m in modes) and tuple(targets[-1].velocity) == (0., 0., 0.),
             'Missing stationary final target/heartbeat or unexpected post-expiry output')

    _require(operator.get('synthetic') is not True, 'Synthetic receipt cannot qualify native recovery')
    land_ros, land_wall = validate_operator_receipt(owner, operator)
    land = clock.source_stamp_us(land_ros)
    raw_status = []
    for row in raw['status']:
        if raw_status and row.timestamp == raw_status[-1].timestamp and row == raw_status[-1]:
            continue  # Same native message is not a second observation.
        raw_status.append(row)
    begin = commands[1].timestamp
    latest = next((m for m in reversed(raw_status) if m.timestamp <= last), None)
    after_last = [m for m in raw_status if m.timestamp > last]
    position = next((m for m in after_last if m.nav_state != NAV_OFFBOARD), None)
    before_land = [m for m in after_last if m.timestamp < land]
    _require(latest is not None and latest.arming_state == 2 and latest.nav_state == NAV_OFFBOARD
             and last - latest.timestamp <= 750_000
             and position is not None and position.nav_state == position_mode and position.arming_state == 2
             and position.timestamp * 1000 + clock.offset_ns - clock.max_error_ns > expiry_ros
             and 0 < position.timestamp - last <= 2_000_000 and before_land
             and all(m.nav_state == position_mode and m.arming_state == 2
                     for m in before_land if m.timestamp >= position.timestamp)
             and 0 < land - before_land[-1].timestamp <= 750_000
             and before_land[-1].timestamp >= position.timestamp
             and operator['status_px4_us'] == before_land[-1].timestamp,
             'No timely native Position fallback and fresh pre-LAND state')
    land_states = [m for m in after_last if m.timestamp > land]
    disarm = next((m for m in after_last if m.arming_state == 1), None)
    auto_land = next((m for m in land_states if m.arming_state == 2 and m.nav_state == NAV_AUTO_LAND), None)
    _require(disarm is not None and 0 < disarm.timestamp - land <= 30_000_000
             and auto_land is not None and auto_land.timestamp < disarm.timestamp
             and all(m.arming_state == 2 and m.nav_state == (position_mode if m.timestamp < auto_land.timestamp else NAV_AUTO_LAND)
                     for m in land_states if m.timestamp < disarm.timestamp)
             and any(m.landed and abs(m.timestamp - disarm.timestamp) <= 1_500_000 for m in raw['landed']),
             'No timely native AUTO_LAND and landed/disarmed transition')
    status_window = [m for m in raw_status if begin - 750_000 <= m.timestamp <= disarm.timestamp]
    _require(len(status_window) > 1 and all(0 < b.timestamp - a.timestamp <= 750_000
             for a, b in zip(status_window, status_window[1:])), 'Invalid native status cadence')
    by_stamp = {row['source_stamp_us']: row for row in recording['status_reports']}
    for m in (position, before_land[-1], disarm):
        row = by_stamp.get(m.timestamp, {})
        _require(row.get('nav_state') == m.nav_state and row.get('arming_state') == m.arming_state
                 and type(row.get('receipt_wall_ns')) is int, 'Native status does not match recorder receipt')
    _require(expiry_wall <= by_stamp[position.timestamp]['receipt_wall_ns'] <= last_wall + 4_000_000_000
             and 0 <= land_wall - by_stamp[before_land[-1].timestamp]['receipt_wall_ns'] <= 3_000_000_000
             and 0 < by_stamp[disarm.timestamp]['receipt_wall_ns'] - land_wall <= 30_000_000_000,
             'Recovery exceeded wall deadline or used a stale handoff')

    positions = [m for m in raw['positions'] if begin <= m.timestamp_sample <= disarm.timestamp]
    _require(len(positions) > 1 and positions[0].timestamp_sample - begin <= 100_000
             and disarm.timestamp - positions[-1].timestamp_sample <= 100_000
             and all(0 < b.timestamp_sample - a.timestamp_sample <= 100_000 for a, b in zip(positions, positions[1:])),
             'Missing full-rate position coverage')
    reset = tuple(getattr(positions[0], field) for field in RESET_FIELDS)
    for m in positions:
        _require(all((m.xy_valid, m.z_valid, m.v_xy_valid, m.v_z_valid))
                 and reset == tuple(getattr(m, field) for field in RESET_FIELDS)
                 and all(math.isfinite(v) for v in (m.x, m.y, m.z, m.vx, m.vy, m.vz))
                 and -.5 <= m.x <= .5 and -.5 <= m.y <= .5 and -1.25 <= m.z <= .02
                 and math.hypot(m.vx, m.vy, m.vz) <= .5, 'R1 position, speed or estimator limit exceeded')
    before = [m for m in positions if last - 700_000 <= m.timestamp_sample <= last]
    settled = []
    for m in reversed(before):
        if not .8 <= -m.z <= 1.2 or math.hypot(m.vx, m.vy, m.vz) > .05:
            break
        settled.append(m)
    _require(bool(settled) and last - settled[0].timestamp_sample <= 100_000
             and settled[0].timestamp_sample - settled[-1].timestamp_sample >= 500_000,
             'No independent stationary interval before the final target')
    return dict(result='PASS', review_status='unreviewed', trial_id=owner['trial_id'],
                last_output_px4_us=last, position_fallback_px4_us=position.timestamp,
                land_issue_ros_ns=land_ros, disarm_px4_us=disarm.timestamp,
                peak_native_3d_speed_mps=max(math.hypot(m.vx, m.vy, m.vz) for m in positions),
                interpretation='One stationary SITL expiry and operator recovery; no braking envelope')


def _sha(path):
    with path.open('rb') as stream:
        digest = sha256()
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('owner', 'recording', 'operator', 'bag', 'output'):
        parser.add_argument(name, type=Path)
    parser.add_argument('--operator-preflight', type=Path, required=True)
    parser.add_argument('--console-log', type=Path, required=True)
    args = parser.parse_args()
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from px4_msgs.msg import VehicleStatus

    files = list(args.bag.glob('*.db3'))
    metadata = args.bag / 'metadata.yaml'
    _require(len(files) == 1 and metadata.is_file() and not any(p.stat().st_size for p in args.bag.glob('*.db3-wal')),
             'One finalized unsplit raw bag required')
    database = files[0]
    hashes = [_sha(p) for p in (database, metadata)]
    raw = {}
    from contextlib import closing
    with closing(sqlite3.connect(database.resolve().as_uri() + '?mode=ro&immutable=1', uri=True)) as db:
        _require(db.execute('pragma quick_check').fetchone()[0] == 'ok', 'Raw bag integrity failure')
        for role, topic in TOPICS.items():
            rows = db.execute('select id,type from topics where name=?', (topic,)).fetchall()
            _require(len(rows) == 1, f'One raw topic required for {role}')
            topic_id, kind = rows[0]
            message_type = get_message(kind)
            raw[role] = [deserialize_message(data, message_type) for (data,) in
                         db.execute('select data from messages where topic_id=? order by id', (topic_id,))]
    paths = (args.owner, args.recording, args.operator)
    inputs = [path.read_bytes() for path in paths]
    owner, recording, operator = (json.loads(data) for data in inputs)
    handoff = dict(operator_preflight_sha256=_sha(args.operator_preflight), console_log_sha256=_sha(args.console_log))
    validate_operator_receipt(owner, operator, handoff=handoff)
    result = audit(owner, recording, operator, raw, position_mode=VehicleStatus.NAVIGATION_STATE_POSCTL)
    _require(list(args.bag.glob('*.db3')) == files and [_sha(p) for p in (database, metadata)] == hashes
             and not any(p.stat().st_size for p in args.bag.glob('*.db3-wal')), 'Raw bag changed during audit')
    result.update(raw_bag_sha256=hashes[0], raw_metadata_sha256=hashes[1],
                  **handoff,
                  input_sha256=dict(zip(('owner', 'recording', 'operator'), (sha256(data).hexdigest() for data in inputs))))
    with args.output.open('x', encoding='utf-8') as file:
        json.dump(result, file, indent=2, allow_nan=False)
        file.write('\n')


if __name__ == '__main__':
    main()
