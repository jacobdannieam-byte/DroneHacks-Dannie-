"""Separately reviewed isolated SITL calibration owner and subscriber-only preflight."""

import argparse
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
from time import monotonic_ns

from examples.gazebo_container import isolated, sha
from examples.gazebo_sensor_contract import ESTIMATOR_FLAGS_MAX_AGE_NS, SIM_HEALTH
from examples.h1_observation import ClockMapping, RESET_FIELDS
from onboard.diagnostics.phase1_probe import payloads, px4_topic
from onboard.diagnostics.sitl_graph import COMMAND_TOPICS, inspect
from refnav.calibration_trial import CalibrationPlan, CalibrationTrial, Observation, NS, STOP_MPS
from refnav.core import NAV_OFFBOARD


OWNER_ID = '/calibration_trial_owner'
EVENTS = '/qualification/events'
CONTROL = '/calibration_trial/control'
MAX_AGE_NS = dict(authority=750_000_000, landed=1_500_000_000, position=300_000_000,
                  flags=ESTIMATOR_FLAGS_MAX_AGE_NS, health=300_000_000)
MAX_DEFER_NS = 50_000_000
PARAMETER_OVERRIDES = {'MPC_LAND_SPEED': .3}
GEOMETRY_SOURCES = (
    'scenarios/calibration/straight_lane_v1.json',
    'simulator/gazebo_trial/models/oak_d_s2_camera/model.sdf',
    'simulator/gazebo_trial/models/drone_16_with_oak_d_s2/model.sdf',
    'simulator/gazebo_trial/make_drone_16_model.py',
    'simulator/gazebo_trial/course_world_sdf.py',
    'simulator/gazebo_trial/calibration_lane.py',
)


def _read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def load_profile(path):
    raw = _read(path)
    if (not re.fullmatch('[0-9a-f]{64}', raw.get('geometry_evidence_sha256', ''))
            or not isinstance(raw.get('recovery_review'), str) or not raw['recovery_review'].strip()
            or raw.get('parameter_overrides') != PARAMETER_OVERRIDES):
        raise ValueError('Surveyed lane evidence and designated recovery review required')
    return raw, CalibrationPlan(**raw['plan']), ClockMapping(**raw['clock_mapping'])


def verify_geometry(path, identity, plan):
    if sha(path) != identity['geometry_evidence_sha256']:
        raise ValueError('Geometry evidence bytes differ from reviewed profile')
    geometry = _read(path)
    sources = geometry['source_sha256']
    root = Path(__file__).resolve().parents[1]
    expected = {relative: sha(root / relative) for relative in GEOMETRY_SOURCES}
    expected['externally_supplied_generated_drone_model.sdf'] = identity['vehicle_sha256']
    if sources != expected or geometry.get('world_sha256') != identity['world_sha256']:
        raise ValueError('Geometry evidence does not bind the generated world, vehicle and model dependencies')
    low, high = (geometry['geometry'][key] for key in ('navigation_low_enu_m', 'navigation_high_enu_m'))
    bounds_low, bounds_high = plan.minimum_offset_m, plan.maximum_offset_m
    if (not all(low[axis] <= bounds_low[index] < bounds_high[index] <= high[axis]
                for axis, index in ((0, 1), (1, 0)))
            or not low[2] <= plan.height_m - plan.hover_tolerance_m < plan.height_m + plan.hover_tolerance_m <= high[2]
            or bounds_low[2] < -high[2] or bounds_high[2] > .02):
        raise ValueError('Calibration bounds exceed reviewed lane')


def load_approval(path, identity, recorder_contract, *, trial_kind, now=None):
    approval = _read(path)
    expires = datetime.fromisoformat(approval['expires_utc'].replace('Z', '+00:00'))
    now = datetime.now(timezone.utc) if now is None else now
    if (expires.tzinfo is None or not now < expires or approval.get('mode') != 'calibration_trial'
            or any(approval.get(key) != value for key, value in identity.items())
            or approval.get('parameter_overrides') != PARAMETER_OVERRIDES
            or approval.get('recording_contract_sha256') != sha(recorder_contract)
            or not re.fullmatch('[A-Za-z0-9_-]{1,64}', approval.get('trial_id', ''))
            or not all(isinstance(approval.get(key), str) and approval[key].strip()
                       for key in ('reviewed_by', 'recovery_operator'))
            or not all(approval.get(key) is True for key in
                       ('isolation_reviewed', 'recovery_reviewed', 'recording_ready',
                        'allow_failed_simulator_termination'))):
        raise ValueError('Current reviewed approval for this exact calibration deployment required')
    contract = _read(recorder_contract)
    if (contract.get('review_status') != 'reviewed' or contract.get('trial_id') != approval['trial_id']
            or contract.get('trial_kind') != trial_kind
            or contract.get('owner_id') != OWNER_ID or contract.get('profile_sha256') != identity['profile_sha256']
            or contract.get('frame_id') != 'px4_local_ned' or contract.get('topics', {}).get('events') != EVENTS):
        raise ValueError('Recorder contract differs from the calibration owner and trial')
    return approval, expires, contract


def verify_recorder_ready(path, contract, approval, recorder_pid):
    ready = _read(path)
    if (ready.get('state') != 'ready' or ready.get('recorder_pid') != recorder_pid
            or ready.get('contract_sha256') != approval['recording_contract_sha256']
            or any(ready.get(key) != contract[key] for key in ('trial_id', 'owner_id', 'profile_sha256', 'trial_kind'))
            or type(ready.get('ros_stamp_ns')) is not int or ready['ros_stamp_ns'] <= 0):
        raise ValueError('Recorder readiness does not bind this live process and trial')
    return ready


def verify_landing_speed():
    result = subprocess.run(['../bin/px4-param', 'show', 'MPC_LAND_SPEED'],
        cwd='/opt/PX4-Autopilot/build/px4_sitl_default/rootfs', check=True, timeout=2.,
        capture_output=True, text=True)
    values = re.findall(r'(?m)^.*\bMPC_LAND_SPEED\b[^:\n]*:\s*([0-9.]+)[ \t]*$', result.stdout)
    if len(values) != 1 or float(values[0]) != PARAMETER_OVERRIDES['MPC_LAND_SPEED']:
        raise ValueError('Effective native LAND speed differs from reviewed calibration override')


class NativeInputs:
    """Conservative PX4/ROS clock joins with bounded deferral and no freshness renewal."""

    def __init__(self, mapping, wall_limit_s):
        self.mapping = mapping
        self.wall_limit_ns = wall_limit_s * NS
        self.samples = {}
        self.pending = {}
        self.source_stamps = {}
        self.reset_identity = None
        self.fault = None
        self.clock_failures = []

    def offer(self, role, source_us, publish_us, value, ros_ns, wall_ns):
        if ros_ns < self.mapping.valid_from_ns or ros_ns <= 0:
            return
        if (type(source_us) is not int or type(publish_us) is not int
                or not 0 < source_us <= publish_us):
            self.fault = 'invalid_px4_stamp'
            return
        prior = self.source_stamps.get(role, 0)
        if source_us == prior:  # DDS duplicates never extend a lease.
            return
        if source_us < prior:
            self.fault = 'px4_source_rewind'
            return
        self.source_stamps[role] = source_us
        stamp = source_us * 1000 + self.mapping.offset_ns
        upper = publish_us * 1000 + self.mapping.offset_ns + self.mapping.max_error_ns
        if (stamp - self.mapping.max_error_ns < self.mapping.valid_from_ns
                or max(upper, ros_ns) > self.mapping.valid_until_ns or upper - ros_ns > MAX_DEFER_NS):
            self.fault = 'px4_clock_mapping_outside_review'
            return
        self.advance(ros_ns, wall_ns)
        if role in self.pending:
            self.fault = 'overlapping_px4_clock_deferral'
            return
        row = (stamp, wall_ns, value)
        if upper > ros_ns:
            self.pending[role] = (upper, row)
        else:
            self.samples[role] = row

    def advance(self, ros_ns, wall_ns):
        for role, (upper, row) in tuple(self.pending.items()):
            if wall_ns - row[1] > MAX_DEFER_NS:
                self.clock_failures.append(dict(role=role,target_ns=upper,now_ns=ros_ns,
                    receipt_wall_ns=row[1],now_wall_ns=wall_ns,source_ns=row[0]))
                self.fault = 'px4_clock_deferral_expired'
                del self.pending[role]
            elif upper <= ros_ns:
                self.samples[role] = row
                del self.pending[role]

    def health(self, ok, ros_ns, wall_ns):
        self.samples['health'] = (ros_ns, wall_ns, ok is True)

    def position(self, msg, ros_ns, wall_ns):
        reset = tuple(getattr(msg, name, None) for name in RESET_FIELDS)
        if not all(type(counter) is int and 0 <= counter <= 255 for counter in reset):
            self.fault = 'invalid_estimator_reset_counters'
            return
        if self.reset_identity is not None and reset != self.reset_identity:
            self.fault = 'estimator_reset'
        self.reset_identity = reset
        valid = all(getattr(msg, field, False) for field in ('xy_valid', 'z_valid', 'v_xy_valid', 'v_z_valid'))
        self.offer('position', msg.timestamp_sample, msg.timestamp,
                   ((msg.x, msg.y, msg.z), (msg.vx, msg.vy, msg.vz), getattr(msg, 'heading', None)) if valid else None, ros_ns, wall_ns)

    def observe(self, ros_ns, wall_ns):
        self.advance(ros_ns, wall_ns)
        if self.fault is not None or not self.mapping.valid_from_ns <= ros_ns <= self.mapping.valid_until_ns:
            return None
        for role, limit in MAX_AGE_NS.items():
            row = self.samples.get(role)
            error = 0 if role == 'health' else self.mapping.max_error_ns
            if (row is None or not 0 <= ros_ns - row[0] <= limit - error
                    or not 0 <= wall_ns - row[1] <= self.wall_limit_ns):
                return None
        pos = self.samples['position'][2]
        armed, nav = self.samples['authority'][2]
        if pos is None or type(armed) is not bool or not self.samples['flags'][2] or not self.samples['health'][2]:
            return None
        return Observation(pos[0], pos[1], self.samples['position'][0], self.samples['authority'][0], armed,
                           self.samples['landed'][2], nav, pos[2])


def position_messages(intent, stamp):
    """Shared native serialization; also exercised on a private ROS topic."""
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
    if (intent.velocity_ned_mps is not None or intent.position_ned_m is None or intent.yaw_rad is None
            or not all(math.isfinite(x) for x in (*intent.position_ned_m, intent.yaw_rad))):
        raise ValueError('Ambiguous or invalid position control mode')
    return (OffboardControlMode(timestamp=stamp, position=True),
            TrajectorySetpoint(timestamp=stamp, position=list(intent.position_ned_m),
                velocity=[math.nan] * 3, acceleration=[math.nan] * 3, yaw=float(intent.yaw_rad), yawspeed=math.nan))


def publication_reason(observation, latest, elapsed, now_ns, issued_ns, *, diagnostic_native=False):
    if issued_ns < now_ns or elapsed < 0:
        return 'clock_rewind'
    if latest is None:
        return 'native_observation_lost'
    changed = latest != observation
    if diagnostic_native and observation is not None:
        # A fresh position does not revoke an already computed native lifecycle
        # command. Authority changes still do; model intents replan separately.
        changed = any(getattr(latest,k)!=getattr(observation,k) for k in ('armed','landed','nav_state'))
    if changed:
        return 'observation_superseded'
    if elapsed >= (300_000_000 if diagnostic_native else 50_000_000):
        return 'computation_deadline'
    return None


def _run(plan, mapping, *, approval=None, expires=None, contract=None, identity=None,
         seconds=8., recorder_pid=None, bag_pid=None, recorder_ready=None, driver=None, preview=None):
    import rclpy
    from px4_msgs.msg import (EstimatorStatusFlags, OffboardControlMode, TrajectorySetpoint, VehicleCommand,
                              VehicleLandDetected, VehicleLocalPosition, VehicleStatus)
    from std_msgs.msg import Bool, String
    from refnav.node import CMD_ARM, CMD_LAND, CMD_SET_MODE, PX4_MAIN_MODE_OFFBOARD, PX4_QOS

    owner = approval is not None
    driver = CalibrationTrial(plan) if driver is None else driver
    inputs = NativeInputs(mapping, plan.receipt_wall_limit_s)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])
    node = rclpy.create_node(OWNER_ID[1:])
    report = {'mode': 'calibration_trial' if owner else 'calibration_check', **identity}
    clock_node = clock_executor = clock_thread = None
    if getattr(driver, '_diagnostic_motion_unbounded', False):
        # Humble's /clock subscription shares the normal single-thread executor.
        # Model rollout must not delay clock callbacks and make current PX4 data
        # appear to come from the future. Only the clock runs on this executor;
        # observations, decisions, and publications remain serialized below.
        from rclpy.executors import SingleThreadedExecutor
        from threading import Thread
        clock_node = rclpy.create_node('diagnostic_model_clock')
        clock_executor = SingleThreadedExecutor()
        clock_executor.add_node(clock_node)
        clock_thread = Thread(target=clock_executor.spin, daemon=True)
        clock_thread.start()

    def clocks():
        return (clock_node or node).get_clock().now().nanoseconds, monotonic_ns()

    def native(role, msg, value):
        inputs.offer(role, msg.timestamp, msg.timestamp, value, *clocks())

    def armed(msg):
        return {msg.ARMING_STATE_DISARMED: False, msg.ARMING_STATE_ARMED: True}.get(msg.arming_state)

    def ready(obs):
        return (obs is not None and obs.armed is False and obs.landed is True and obs.nav_state != NAV_OFFBOARD
                and all(math.isfinite(n) for n in (*obs.position, *obs.velocity))
                and math.dist(obs.position, plan.origin_ned_m) <= plan.origin_tolerance_m
                and all(lo <= p - origin <= hi for p, origin, lo, hi in zip(
                    obs.position, plan.origin_ned_m, plan.minimum_offset_m, plan.maximum_offset_m))
                and math.hypot(*obs.velocity) <= STOP_MPS)

    def process_start(pid):
        if pid is None or pid <= 1:
            return None
        try:
            stat = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
            return None if stat[0] == 'Z' else stat[19]
        except (OSError, IndexError):
            return None

    def recorders_running():
        return (recorder_start is not None and bag_start is not None and recorder_pid != bag_pid
                and (process_start(recorder_pid), process_start(bag_pid)) == (recorder_start, bag_start))

    def approved_now():
        return (datetime.now(timezone.utc) < expires and monotonic_ns() < approval_wall_deadline)

    def admitted():
        return (isolated() and approved_now()
                and recorders_running()
                and not inspect(node, OWNER_ID, strict_motion=True)['issues']
                and len(node.get_publishers_info_by_topic(EVENTS)) == 1
                and event_pub.get_subscription_count() >= 1)

    section_input = None

    def position_input(message):
        inputs.position(message,*clocks())
        if section_input is not None:
            section_input.position(message)

    try:
        bindings = {
            'vehicle_local_position': (VehicleLocalPosition, position_input),
            'vehicle_status': (VehicleStatus, lambda m: native('authority', m, (armed(m), int(m.nav_state)))),
            'vehicle_land_detected': (VehicleLandDetected, lambda m: native('landed', m, bool(m.landed))),
            'estimator_status_flags': (EstimatorStatusFlags,
                lambda m: native('flags', m, next(payloads('px4_flags', m))['valid'])),
        }
        topics = {role: px4_topic(role, kind) for role, (kind, _) in bindings.items()}
        if owner and (contract['clock_mapping'] != vars(mapping) or contract['clock_id'] != mapping.ros_clock_id
                      or any(contract['topics'][role] != topics[role]
                             for role in ('vehicle_local_position', 'vehicle_status'))):
            raise ValueError('Recorder and calibration clock or native topics differ')
        for role, (kind, callback) in bindings.items():
            node.create_subscription(kind, topics[role], callback, PX4_QOS)
        node.create_subscription(Bool, SIM_HEALTH, lambda m: inputs.health(m.data, *clocks()), 10)
        if getattr(driver, 'config', {}).get('full_course') and not getattr(driver,'_diagnostic_motion_unbounded',False):
            node.create_subscription(String, '/simulator_only/touches', lambda _: driver.request_land(), 10)
        if hasattr(driver, 'section_policy') and not getattr(driver, '_diagnostic_motion_unbounded', False):
            from examples.section_depth_input import attach
            from examples.gazebo_sensor_contract import SIM_DEPTH, SIM_INFO
            from px4_msgs.msg import VehicleAttitude
            section_input = attach(node,driver.section_policy,mapping,topics['vehicle_local_position'],
                                   px4_topic('vehicle_attitude',VehicleAttitude),SIM_DEPTH,SIM_INFO)

        publishers = None

        def observe(now_ns, wall_ns):
            nonlocal publishers
            observation = inputs.observe(now_ns, wall_ns)
            if section_input is not None:
                section_input.update(now_ns/NS)
            if observation is None:
                return None
            endpoints = [node.get_publishers_info_by_topic(topic) for topic in (*topics.values(), SIM_HEALTH)]
            if any(len(entries) != 1 for entries in endpoints):
                return None
            identity = tuple(bytes(entries[0].endpoint_gid) for entries in endpoints)
            if publishers is not None and identity != publishers:
                inputs.fault = 'native_publisher_changed'
                return None
            publishers = identity
            return observation

        first_ready = None
        stable_checked = False
        diagnostic = getattr(driver, '_diagnostic_motion_unbounded', False)
        deadline = monotonic_ns() + seconds * NS
        obs = None
        while rclpy.ok() and monotonic_ns() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            now_ns, wall_ns = clocks()
            obs = observe(now_ns, wall_ns)
            if not ready(obs):
                first_ready = None
            elif first_ready is None:
                first_ready = now_ns
            elif now_ns - first_ready >= NS:
                if diagnostic:
                    graph = inspect(node, strict_motion=True)
                    now_ns, wall_ns = clocks()
                    obs = observe(now_ns, wall_ns)
                    if not ready(obs) or graph['issues']:
                        continue
                    stable_checked = True
                break
        if not stable_checked:
            graph = inspect(node, strict_motion=True)
        now_ns, wall_ns = clocks()
        checked = stable_checked if diagnostic else (rclpy.ok() and first_ready is not None and now_ns - first_ready >= NS
                   and ready(observe(now_ns, wall_ns)) and not graph['issues'])
        if section_input is not None and not diagnostic:
            capture = driver.section_policy.observation
            checked = checked and capture is not None and driver.section_policy.fault is None and 0<=now_ns/NS-capture.stamp<=.25
        report.update(disarmed_check=bool(checked), graph=graph, ros_ns=now_ns, input_fault=inputs.fault,
                      input_clock_failures=inputs.clock_failures,
                      observed_ned_m=None if obs is None else obs.position,
                      expected_origin_ned_m=plan.origin_ned_m, topics=topics)
        if section_input is not None:
            report['section_depth'] = dict(capture_available=driver.section_policy.observation is not None,
                                          fault=driver.section_policy.fault,rejected=section_input.rejected)
        if not owner and checked and preview is not None:
            report['model_preview'] = preview(obs, now_ns)
        if not owner or not checked:
            return report, int(not checked)
        recorder_start, bag_start = process_start(recorder_pid), process_start(bag_pid)
        approval_wall_deadline = monotonic_ns() + int((expires - datetime.now(timezone.utc)).total_seconds() * NS)
        if not approved_now() or not recorders_running() or recorder_ready['ros_stamp_ns'] > now_ns:
            raise ValueError('Approval or recording expired before owner attachment')
        verify_landing_speed()

        ocm_pub, sp_pub, cmd_pub = (node.create_publisher(kind, topic, PX4_QOS) for topic, kind in
            zip(COMMAND_TOPICS, (OffboardControlMode, TrajectorySetpoint, VehicleCommand)))
        event_pub = node.create_publisher(String, EVENTS, 10)

        def control(message):
            try:
                request = json.loads(message.data)
            except ValueError:
                return
            if request == {'trial_id': approval['trial_id'], 'request': 'land'}:
                driver.request_land()

        node.create_subscription(String, CONTROL, control, 10)
        discovery_deadline = monotonic_ns() + 3 * NS
        while rclpy.ok() and not admitted() and monotonic_ns() < discovery_deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        next_tick = monotonic_ns()
        while rclpy.ok() and driver.phase != 'done':
            rclpy.spin_once(node, timeout_sec=.02)
            now_ns, wall_ns = clocks()
            if diagnostic:
                inputs.advance(now_ns, wall_ns)
            if wall_ns < next_tick:
                continue
            if getattr(driver, '_diagnostic_motion_unbounded', False):
                # Drain the depth-one native subscriptions after inference. A
                # single spin can select a camera/clock callback and leave a
                # newer position queued while the next rollout starts.
                for _ in range(16):
                    rclpy.spin_once(node, timeout_sec=0.)
                now_ns, wall_ns = clocks()
                inputs.advance(now_ns, wall_ns)
                if inputs.pending and inputs.fault is None:
                    # Resolve clock-fenced receipts before starting synchronous
                    # inference. Do not let its work consume a deferral's lease.
                    continue
            next_tick = wall_ns + 50_000_000
            observation = observe(now_ns, wall_ns)
            if observation is not None and hasattr(driver,'prepare_section') and not diagnostic:
                driver.prepare_section(observation,now_ns)
                # Perception has no command authority. Re-read time and validate
                # native inputs before starting the unchanged publication fence.
                now_ns,wall_ns=clocks()
                observation=observe(now_ns,wall_ns)
            intent = driver.step(now_ns, wall_ns, observation, admitted=admitted())
            if intent is None:
                continue
            if not admitted():
                report['publication_failure'] = dict(reason='authority_or_recording_admission_lost')
                driver.stop('publication_fence_closed')
                break
            issued_ros_ns, issued_wall_ns = clocks()
            latest = inputs.observe(issued_ros_ns, issued_wall_ns)
            elapsed = issued_wall_ns - wall_ns
            reason = publication_reason(observation, latest, elapsed, now_ns, issued_ros_ns,
                diagnostic_native=(getattr(driver, '_diagnostic_motion_unbounded', False)
                                   and intent.position_ned_m is None))
            if reason is not None:
                if (reason in ('observation_superseded', 'computation_deadline')
                        and hasattr(driver, 'discard_model_intent')
                        and driver.discard_model_intent(intent, reason=reason,
                            ros_ns=issued_ros_ns, elapsed_wall_ns=elapsed)):
                    continue
                report['publication_failure'] = dict(reason=reason, elapsed_wall_ns=elapsed,
                    decision_ros_ns=now_ns, issued_ros_ns=issued_ros_ns)
                driver.stop('publication_fence_closed')
                break
            stamp = mapping.source_stamp_us(issued_ros_ns)
            if intent.position_ned_m is not None:
                mode, target = position_messages(intent, stamp)
                ocm_pub.publish(mode)
                sp_pub.publish(target)
            if intent.velocity_ned_mps is not None:
                mode = OffboardControlMode(timestamp=stamp, velocity=True)
                target = TrajectorySetpoint(timestamp=stamp, position=[math.nan] * 3,
                                            velocity=list(intent.velocity_ned_mps), acceleration=[math.nan] * 3,
                                            yaw=math.nan if intent.yaw_rad is None else intent.yaw_rad,
                                            yawspeed=math.nan if intent.yaw_rate_rad_s is None else intent.yaw_rate_rad_s)
                ocm_pub.publish(mode)
                sp_pub.publish(target)
            if intent.command is not None:
                cmd, p1, p2 = {'offboard': (CMD_SET_MODE, 1., float(PX4_MAIN_MODE_OFFBOARD)),
                               'arm': (CMD_ARM, 1., 0.), 'land': (CMD_LAND, 0., 0.)}[intent.command]
                cmd_pub.publish(VehicleCommand(timestamp=stamp, command=cmd, param1=p1, param2=p2,
                    target_system=1, target_component=1, source_system=1, source_component=1, from_external=True))
            event = driver.published(intent, issued_ros_ns, stamp, wall_ns=issued_wall_ns)
            if event is not None:
                event_pub.publish(String(data=json.dumps(dict(event, trial_id=approval['trial_id'],
                    owner_id=OWNER_ID, profile_sha256=identity['profile_sha256']), allow_nan=False)))
        report.update(trial_id=approval['trial_id'], trial_kind=plan.trial_kind, owner_id=OWNER_ID,
                      input_clock_failures=inputs.clock_failures,
                      result=driver.result or 'interrupted',
                      recovery_reason=driver.recovery_reason, events=driver.events,
                      native_land_ack_ns=driver.native_land_ack_ns, native_disarm_ns=driver.native_disarm_ns,
                      max_observed_speed_mps=driver.max_observed_speed_mps,
                      recovery_operator=approval['recovery_operator'], input_fault=inputs.fault)
        if diagnostic:
            now_ns, wall_ns = clocks()
            report['native_input_snapshot'] = dict(now_ros_ns=now_ns, now_wall_ns=wall_ns,
                samples={role:dict(ros_age_ns=now_ns-row[0],wall_age_ns=wall_ns-row[1],value=row[2])
                         for role,row in inputs.samples.items()},pending_roles=list(inputs.pending))
        if plan.trial_kind == 'authority_expiry':
            report['recovery_lease'] = driver.recovery_lease
        return report, int(driver.result != 'complete')
    finally:
        if clock_executor is not None:
            clock_executor.shutdown()
            clock_thread.join(timeout=2.)
            clock_node.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('profile', 'world', 'vehicle', 'geometry-evidence', 'output'):
        parser.add_argument(f'--{name}', type=Path, required=True)
    parser.add_argument('--revision', required=True)
    parser.add_argument('--image-sha256', required=True)
    parser.add_argument('--seconds', type=int, choices=range(4, 31), default=8)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--disarmed-check', action='store_true')
    mode.add_argument('--approval', type=Path)
    parser.add_argument('--authorize-only', action='store_true')
    parser.add_argument('--recording-contract', type=Path)
    parser.add_argument('--recorder-pid', type=int)
    parser.add_argument('--recorder-ready', type=Path)
    parser.add_argument('--bag-pid', type=int)
    args = parser.parse_args()
    try:
        if (not isolated() or not re.fullmatch('[0-9a-f]{40}', args.revision)
                or not re.fullmatch('sha256:[0-9a-f]{64}', args.image_sha256)):
            raise ValueError('Pinned source/image and isolated loopback-only SITL container required')
        raw, plan, mapping = load_profile(args.profile)
        identity = dict(revision=args.revision, image_sha256=args.image_sha256, world_sha256=sha(args.world),
                        vehicle_sha256=sha(args.vehicle), profile_sha256=sha(args.profile),
                        geometry_evidence_sha256=raw['geometry_evidence_sha256'],
                        ros_domain_id=int(os.environ['ROS_DOMAIN_ID']))
        verify_geometry(args.geometry_evidence, identity, plan)
        approval = expires = contract = recorder_ready = None
        if args.authorize_only and args.approval is None:
            raise ValueError('Authorize-only requires the exact reviewed approval artifact')
        if args.approval:
            if (args.recording_contract is None
                    or not args.authorize_only and None in (args.recorder_pid, args.bag_pid, args.recorder_ready)):
                raise ValueError('Reviewed recorder contract and both live recorder processes required')
            approval, expires, contract = load_approval(args.approval, identity, args.recording_contract,
                                                        trial_kind=plan.trial_kind)
            if not args.authorize_only:
                recorder_ready = verify_recorder_ready(args.recorder_ready, contract, approval, args.recorder_pid)
        with args.output.open('x', encoding='utf-8') as output:
            if args.authorize_only:
                json.dump(dict(identity, trial_id=approval['trial_id'], approval_sha256=sha(args.approval),
                               parameter_overrides=PARAMETER_OVERRIDES, authorized_artifact=True), output)
                output.write('\n')
                return 0
            try:
                report, code = _run(plan, mapping, approval=approval, expires=expires, contract=contract,
                                    identity=identity, seconds=args.seconds, recorder_pid=args.recorder_pid,
                                    bag_pid=args.bag_pid, recorder_ready=recorder_ready)
            except (ValueError, RuntimeError, OSError, subprocess.SubprocessError, KeyboardInterrupt) as error:
                report, code = dict(identity, result='interrupted_or_refused', fault=str(error)), 2
            json.dump(report, output, indent=2, allow_nan=False)
            output.write('\n')
        return code
    except (ValueError, OSError, KeyError, TypeError) as error:
        print(f'calibration_trial_owner: {error}', file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
