"""One-shot operator LAND receipt capture for separately reviewed isolated R1 recovery."""

import argparse
from datetime import datetime, timezone
from hashlib import sha256
import json
import math
from pathlib import Path
import subprocess
import sys
from time import monotonic_ns

from examples.calibration_recovery_audit import validate_operator_receipt
from examples.calibration_trial_owner import OWNER_ID, NativeInputs, load_approval, load_profile, verify_landing_speed
from examples.gazebo_container import isolated, sha
from refnav.calibration_trial import NS


IDENTITY = ('revision', 'image_sha256', 'world_sha256', 'vehicle_sha256',
            'profile_sha256', 'geometry_evidence_sha256', 'ros_domain_id')
CONSOLE = ['../bin/px4-commander', 'land']
PX4_ROOTFS = '/opt/PX4-Autopilot/build/px4_sitl_default/rootfs'
ISSUE_MAX_DELAY_NS = 10_000_000
HANDOFF_WALL_NS = 30 * NS  # Leave the other half of the supervisor window for disarm.


def _read(path):
    return json.loads(path.read_bytes())


def _json_bytes(value):
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + '\n').encode()


def issue_pair(checked, read_ros, read_wall, *, deadline_wall_ns):
    before = read_wall()
    now = read_ros()
    wall = read_wall()
    if (not 0 <= wall - before <= 1_000_000 or not 0 <= wall - checked['wall_ns'] <= ISSUE_MAX_DELAY_NS
            or wall >= deadline_wall_ns or now < checked['ros_ns']):
        return None
    return dict(checked, ros_ns=now, wall_ns=wall, wall_sample_before_ns=before)


def capture(root, owner, snapshot, send, *, synthetic, issue_check=None):
    """Reserve every evidence filename, then sample immediately before one attempt."""
    preflight_path = root / 'operator-preflight.json'
    console_path = root / 'operator-console.log'
    receipt_path = root / 'operator-land.json'
    with (receipt_path.open('xb') as receipt_file, preflight_path.open('xb') as preflight_file,
          console_path.open('xb') as console_file):
        try:
            checked = snapshot()
            if checked is not None and issue_check is not None:
                issued = issue_check(checked)
                if issued is None:
                    preflight_file.write(_json_bytes(dict(checked, synthetic=synthetic, refused_final_issue=True)))
                checked = issued
        except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as error:
            receipt_file.write(_json_bytes({'result': 'refused_preflight', 'fault': type(error).__name__,
                                            'synthetic': synthetic}))
            return False
        if checked is None:
            receipt_file.write(_json_bytes({'result': 'refused_preflight', 'synthetic': synthetic}))
            return False
        try:
            code = send(console_file)
        except (OSError, subprocess.SubprocessError, KeyboardInterrupt) as error:
            console_file.write(f'\nConsole attempt interrupted: {type(error).__name__}\n'.encode())
            code = 2
        console_file.flush()
        preflight = _json_bytes(dict(checked, synthetic=synthetic))
        preflight_file.write(preflight)
        receipt = dict(trial_id=owner['trial_id'], operator=owner['recovery_operator'], synthetic=synthetic,
            command='px4-commander land', exit_code=code, owner_exited=checked['owner_exited'],
            idle_motion_graph=checked['idle_motion_graph'], fresh_healthy_in_lane=checked['fresh_healthy_in_lane'],
            status_px4_us=checked['status_px4_us'], land_issue_ros_ns=checked['ros_ns'],
            land_issue_wall_ns=checked['wall_ns'], operator_preflight_sha256=sha256(preflight).hexdigest(),
            console_log_sha256=sha(console_path))
        if 'operator_capture_sha256' in checked:
            receipt['operator_capture_sha256'] = checked['operator_capture_sha256']
        receipt_file.write(_json_bytes(receipt))
        if code == 0:
            validate_operator_receipt(owner, receipt)
        return code == 0


def dry_run(root):
    now = monotonic_ns()
    owner = dict(trial_id='synthetic_operator_rehearsal', recovery_operator='fixture',
                 recovery_lease=dict(expiry_detected_ros_ns=NS, expiry_detected_wall_ns=now))
    fixture = dict(owner_exited=True, idle_motion_graph=True, fresh_healthy_in_lane=True,
                   status_px4_us=1_100_000, ros_ns=1_200_000_000, wall_ns=now + 200_000_000,
                   source='synthetic clock and Position state')

    def fixture_send(log):
        log.write(b'SYNTHETIC REHEARSAL ONLY - no console process was started\n')
        return 0

    return capture(root, owner, lambda: fixture, fixture_send, synthetic=True)


def authorize(args):
    import os

    if not isolated():
        raise ValueError('Native recovery requires the reviewed loopback-only SITL container')
    owner = _read(args.owner)
    plan_doc, plan, mapping = load_profile(args.profile)
    if (owner.get('mode') != 'calibration_trial' or owner.get('owner_id') != OWNER_ID
            or owner.get('trial_kind') != plan.trial_kind or plan.trial_kind != 'authority_expiry'
            or owner.get('result') != 'planned_authority_expiry' or owner.get('input_fault')
            or owner.get('events') != [] or owner.get('profile_sha256') != sha(args.profile)
            or owner.get('geometry_evidence_sha256') != plan_doc['geometry_evidence_sha256']
            or owner.get('ros_domain_id') != int(os.environ['ROS_DOMAIN_ID'])):
        raise ValueError('Exact planned R1 owner exit report required')
    identity = {field: owner[field] for field in IDENTITY}
    approval, expires, contract = load_approval(args.approval, identity, args.recording_contract,
                                                trial_kind=plan.trial_kind)
    authorization = _read(args.authorization)
    if (approval.get('operator_console_land_authorized') is not True
            or approval.get('operator_capture_sha256') != sha(Path(__file__))
            or not isinstance(args.operator, str) or not args.operator.strip()
            or not args.operator == approval['recovery_operator'] == owner.get('recovery_operator')
            or approval['trial_id'] != owner.get('trial_id') or authorization.get('trial_id') != owner['trial_id']
            or authorization.get('authorized_artifact') is not True
            or authorization.get('approval_sha256') != sha(args.approval)
            or any(authorization.get(field) != value for field, value in identity.items())
            or contract['clock_mapping'] != vars(mapping)):
        raise ValueError('Separate current console LAND approval for the named recovery operator required')
    return owner, plan, mapping, expires


def _owner_process_present():
    module = b'examples.calibration_trial_owner'
    for process in Path('/proc').glob('[0-9]*/cmdline'):
        try:
            args = process.read_bytes().split(b'\0')
        except FileNotFoundError:  # Process exited during the scan.
            continue
        if module in args or any(arg.endswith(b'examples/calibration_trial_owner.py') for arg in args):
            return True
    return False


def execute_reviewed(args):
    evidence = Path('/evidence')
    if (args.output_dir.resolve() != evidence or args.owner.resolve() != evidence / 'calibration_owner.json'
            or args.authorization.resolve() != evidence / 'calibration_authorization.json'):
        raise ValueError('Use this active isolated trial evidence directory for the one-shot receipt')
    owner, plan, mapping, expires = authorize(args)
    owner_sha, approval_sha, helper_sha = sha(args.owner), sha(args.approval), sha(Path(__file__))
    expiry = owner['recovery_lease']
    until = min(expiry['expiry_detected_wall_ns'] + HANDOFF_WALL_NS,
                monotonic_ns() + int((expires - datetime.now(timezone.utc)).total_seconds() * NS))
    if not expiry['expiry_detected_wall_ns'] < monotonic_ns() < until:
        raise ValueError('Recovery lease or operator approval expired')
    verify_landing_speed()  # Read-only check before the operator can issue any command.

    import rclpy
    from px4_msgs.msg import EstimatorStatusFlags, VehicleLandDetected, VehicleLocalPosition, VehicleStatus
    from std_msgs.msg import Bool
    from examples.gazebo_sensor_contract import SIM_HEALTH
    from onboard.diagnostics.phase1_probe import payloads, px4_topic
    from onboard.diagnostics.sitl_graph import inspect
    from refnav.node import PX4_QOS

    inputs = NativeInputs(mapping, plan.receipt_wall_limit_s)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = rclpy.create_node('calibration_operator_receipt')

    def clocks():
        return node.get_clock().now().nanoseconds, monotonic_ns()

    def native(role, message, value):
        inputs.offer(role, message.timestamp, message.timestamp, value, *clocks())

    try:
        ports = (
            ('vehicle_local_position', VehicleLocalPosition, lambda m: inputs.position(m, *clocks())),
            ('vehicle_status', VehicleStatus, lambda m: native('authority', m, (
                {m.ARMING_STATE_ARMED: True, m.ARMING_STATE_DISARMED: False}.get(m.arming_state), int(m.nav_state)))),
            ('vehicle_land_detected', VehicleLandDetected, lambda m: native('landed', m, bool(m.landed))),
            ('estimator_status_flags', EstimatorStatusFlags,
             lambda m: native('flags', m, next(payloads('px4_flags', m))['valid'])),
        )
        topics = [px4_topic(role, kind) for role, kind, _ in ports]
        for topic, (_, kind, callback) in zip(topics, ports):
            node.create_subscription(kind, topic, callback, PX4_QOS)
        node.create_subscription(Bool, SIM_HEALTH, lambda m: inputs.health(m.data, *clocks()), 10)

        def in_lane(obs):
            return (obs is not None and obs.armed is True and obs.landed is False
                    and obs.nav_state == VehicleStatus.NAVIGATION_STATE_POSCTL
                    and all(math.isfinite(v) for v in (*obs.position, *obs.velocity))
                    and math.hypot(*obs.velocity) <= plan.actual_speed_limit_mps
                    and all(lo <= p - origin <= hi for p, origin, lo, hi in zip(
                        obs.position, plan.origin_ned_m, plan.minimum_offset_m, plan.maximum_offset_m)))

        progress_ros = None
        progress_wall = None
        armed_position_ready = False
        while rclpy.ok() and monotonic_ns() < until:
            rclpy.spin_once(node, timeout_sec=.02)
            now, wall = clocks()
            if progress_ros is not None and now < progress_ros:
                return False
            if now > expiry['expiry_detected_ros_ns'] and (progress_ros is None or now > progress_ros):
                armed_position_ready = progress_ros is not None and in_lane(inputs.observe(now, wall))
                progress_ros, progress_wall = now, wall
                if armed_position_ready:
                    break
        if not armed_position_ready:
            return False

        def snapshot():
            graph = inspect(node, strict_motion=True)
            owner_exited = (not _owner_process_present() and all(
                f"{namespace.rstrip('/')}/{name}" != OWNER_ID for name, namespace in node.get_node_names_and_namespaces()))
            endpoints = {topic: node.get_publishers_info_by_topic(topic) for topic in (*topics, SIM_HEALTH)}
            if (not owner_exited or graph['issues'] or any(graph['endpoints'].values())
                    or any(len(rows) != 1 for rows in endpoints.values()) or not isolated()
                    or datetime.now(timezone.utc) >= expires
                    or sha(args.owner) != owner_sha or sha(args.approval) != approval_sha
                    or sha(Path(__file__)) != helper_sha):
                return None
            before = monotonic_ns()
            now = node.get_clock().now().nanoseconds
            wall = monotonic_ns()
            obs = inputs.observe(now, wall)
            if (not 0 <= wall - before <= 1_000_000 or not 0 <= wall - progress_wall <= 100_000_000
                    or not wall < until or now < progress_ros or not in_lane(obs)
                    or now - obs.authority_stamp_ns + mapping.max_error_ns > 750_000_000):
                return None
            return dict(ros_ns=now, wall_ns=wall, wall_sample_before_ns=before,
                        owner_exited=True, idle_motion_graph=True, fresh_healthy_in_lane=True,
                        status_px4_us=(obs.authority_stamp_ns - mapping.offset_ns) // 1000,
                        observed_position_ned_m=obs.position, observed_velocity_ned_mps=obs.velocity,
                        status_armed=obs.armed, nav_state=obs.nav_state, graph=graph,
                        native_publisher_gid={topic: bytes(rows[0].endpoint_gid).hex() for topic, rows in endpoints.items()},
                        owner_report_sha256=owner_sha, approval_sha256=approval_sha,
                        operator_capture_sha256=helper_sha)

        def send(log):
            return subprocess.run(CONSOLE, cwd=PX4_ROOTFS, stdout=log, stderr=subprocess.STDOUT, timeout=2.).returncode

        def issue_check(checked):
            if datetime.now(timezone.utc) >= expires:
                return None
            issue = issue_pair(checked, lambda: node.get_clock().now().nanoseconds, monotonic_ns,
                               deadline_wall_ns=until)
            if issue is None:
                return None
            obs = inputs.observe(issue['ros_ns'], issue['wall_ns'])
            if (not in_lane(obs) or (obs.authority_stamp_ns - mapping.offset_ns) // 1000 != checked['status_px4_us']
                    or obs.position != checked['observed_position_ned_m']
                    or obs.velocity != checked['observed_velocity_ned_mps']):
                return None
            return issue

        return capture(args.output_dir, owner, snapshot, send, synthetic=False, issue_check=issue_check)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--dry-run', action='store_true')
    mode.add_argument('--execute-reviewed', action='store_true')
    for name in ('owner', 'profile', 'approval', 'authorization', 'recording-contract'):
        parser.add_argument('--' + name, type=Path)
    parser.add_argument('--operator')
    args = parser.parse_args()
    try:
        if args.dry_run:
            return int(not dry_run(args.output_dir))
        if any(getattr(args, field) is None for field in
               ('owner', 'profile', 'approval', 'authorization', 'recording_contract', 'operator')):
            raise ValueError('Exact owner, approval, authorization, profile, contract and operator required')
        return int(not execute_reviewed(args))
    except (ValueError, KeyError, TypeError, OSError, subprocess.SubprocessError) as error:
        print(f'calibration_operator_capture: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
