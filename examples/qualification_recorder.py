"""Subscriber-only qualification recorder. Never creates command publishers.

Events must be recorded by the single trial owner at publication time, in the
same ROS clock as the telemetry. Raw rosbag recording remains mandatory.
"""
import argparse
import json
import math
import os
import re
import signal
from dataclasses import asdict
from hashlib import sha256
from pathlib import Path
from time import monotonic, monotonic_ns

from examples.h1_observation import ClockMapping
from onboard.diagnostics.phase1_probe import RESET_FIELDS

POSITION_SILENCE_NS = 100_000_000  # Same capture-gap limit as stopping analysis.
STATUS_SILENCE_NS = 750_000_000  # Reviewed simulated-estimator authority cadence.
RECEIPT_WALL_SILENCE_NS = 3_000_000_000  # Maximum calibration owner receipt allowance.
CLOCK_STALL_NS = 300_000_000  # Same steady-clock stall limit as the native owner.


def initial_clock_ready(read_clock, spin, wall=monotonic, *, timeout_s=5.):
    """Initialize ROS time before subscribing to PX4; never relax sample age checks.

    This is a pre-readiness phase only. No sample/event subscription or owner
    readiness exists yet; the independent raw bag is already recording.
    """
    deadline = wall() + timeout_s
    previous = None
    while wall() < deadline:
        spin()
        now = read_clock()
        if type(now) is not int or now < 0:
            return False
        if now > 0:
            if previous is not None:
                if now < previous:
                    return False
                if now > previous:
                    return True
            previous = now
    return False


def drain_clock_pending(record, read_clock, spin, wall=monotonic_ns):
    """After telemetry subscriptions close, finish already-received clock-fenced samples."""
    deadline = wall() + 50_000_000
    while record.pending and not record.faults and wall() <= deadline:
        spin()
        record.clock(read_clock(), wall_ns=wall())
    if record.pending and not record.faults:
        record.faults.append('recorder_shutdown_clock_pending')


def event_publisher_identity(node, topic, owner_id):
    """Inspect Humble's singleton graph endpoint; this is not message authentication.

    Pinned Humble rclpy has no callback publisher GID. Independent raw command
    correlation is mandatory before promoting any resulting measurement.
    """
    endpoints = node.get_publishers_info_by_topic(topic)
    if len(endpoints) != 1:
        raise ValueError('exactly one event publisher required')
    endpoint = endpoints[0]
    name = endpoint.node_namespace.rstrip('/') + '/' + endpoint.node_name
    expected_gid = bytes(endpoint.endpoint_gid)
    if name != owner_id or not expected_gid or not any(expected_gid):
        raise ValueError('event publisher differs from reviewed owner')
    return dict(publisher_node=name, publisher_gid=expected_gid.hex(),
                publisher_binding='singleton_graph_only')


class TrialRecording:
    def __init__(self, clock_id, frame_id, *, clock_mapping=None, trial_id=None,
                 owner_id=None, profile_sha256=None, trial_kind='stopping'):
        if trial_kind not in ('hover_land', 'stopping', 'authority_expiry'):
            raise ValueError('explicit calibration trial kind required')
        self.document = dict(clock_id=clock_id, frame_id=frame_id, trial_kind=trial_kind, samples=[], events=[])
        self.mapping = clock_mapping
        if clock_mapping is not None:
            if (not isinstance(clock_mapping, ClockMapping) or clock_mapping.ros_clock_id != clock_id
                    or not isinstance(trial_id, str) or not trial_id.strip()
                    or not isinstance(owner_id, str) or not re.fullmatch(r'(/[A-Za-z_][A-Za-z_0-9]*)+', owner_id)
                    or not isinstance(profile_sha256, str) or not re.fullmatch('[0-9a-f]{64}', profile_sha256)):
                raise ValueError('reviewed recorder clock and trial identity required')
            self.document.update(provenance_scope='reviewed_calibration',
                clock_mapping=asdict(clock_mapping), trial_id=trial_id, owner_id=owner_id,
                profile_sha256=profile_sha256, status_reports=[])
        else:
            if any(value is not None for value in (trial_id, owner_id, profile_sha256)):
                raise ValueError('trial identity requires an explicit reviewed clock mapping')
            self.document['provenance_scope'] = 'legacy_synthetic'
        self.faults = []
        self.last_time = None
        self.seen_armed = False
        self.last_status_stamp = None
        self.pending = []
        self.last_clock_ns = None
        self.reset_counters = None
        self.event_publisher_gid = None
        self.monitor_clock = None

    def _stamp(self, message, kind):
        value = getattr(message, 'timestamp_sample' if kind == 'sample' else 'timestamp', None)
        if type(value) is not int or value <= 0:
            raise ValueError('invalid source stamp')
        return value

    def _submit(self, kind, message, receipt_ns, wall_ns):
        wall_ns = monotonic_ns() if wall_ns is None else wall_ns
        try:
            if type(receipt_ns) is not int or receipt_ns < 0 or type(wall_ns) is not int:
                raise ValueError('invalid receipt')
            source = self._stamp(message, kind)
            if self.mapping is not None:
                mapping = self.mapping
                publish = getattr(message, 'timestamp', None)
                if type(publish) is not int or publish < source:
                    raise ValueError('invalid publication stamp')
                mapped = source * 1000 + mapping.offset_ns
                target = publish * 1000 + mapping.offset_ns + mapping.max_error_ns
                if not (mapping.valid_from_ns <= mapped - mapping.max_error_ns
                        and mapping.valid_from_ns <= receipt_ns <= mapping.valid_until_ns
                        and target <= mapping.valid_until_ns):
                    raise ValueError('source outside reviewed epoch')
                self._expire_pending(receipt_ns, wall_ns)
                if target > receipt_ns or any(item[0] == kind for item in self.pending):
                    if target - receipt_ns > 50_000_000 or len(self.pending) >= 32:
                        raise ValueError('clock wait exceeds bound')
                    self.pending.append((kind, message, receipt_ns, wall_ns, target))
                    return
            self._ingest(kind, message, receipt_ns, receipt_ns, wall_ns, wall_ns)
        except (ValueError, TypeError, AttributeError, OverflowError):
            self.faults.append(kind + '_clock_or_age')

    def _expire_pending(self, now_ns, wall_ns):
        if any(wall_ns-item[3] > 50_000_000 or now_ns-item[2] > 50_000_000
               or wall_ns < item[3] for item in self.pending):
            self.pending.clear()
            self.faults.append('clock_wait_expired')

    def clock(self, now_ns, *, wall_ns=None):
        """Release ahead-of-clock native data; never rewrite callback timestamps."""
        wall_ns = monotonic_ns() if wall_ns is None else wall_ns
        if type(now_ns) is not int or now_ns < 0 or type(wall_ns) is not int:
            self.faults.append('invalid_clock'); return
        if self.last_clock_ns is not None and now_ns < self.last_clock_ns:
            self.pending.clear()
            self.faults.append('clock_rewind'); return
        self.last_clock_ns = now_ns
        self._expire_pending(now_ns, wall_ns)
        waiting = []
        for kind, message, receipt, arrival_wall, target in self.pending:
            if target > now_ns:
                waiting.append((kind, message, receipt, arrival_wall, target))
                continue
            try:
                self._ingest(kind, message, receipt, now_ns, arrival_wall, wall_ns)
            except (ValueError, TypeError, AttributeError, OverflowError):
                self.faults.append(kind + '_clock_or_age')
        self.pending = waiting

    def _ingest(self, kind, message, receipt_ns, ingestion_ns, arrival_wall, ingestion_wall):
        source = self._stamp(message, kind)
        stamp = source * 1000 if self.mapping is None else self.mapping.capture(source, ingestion_ns)
        error = 0 if self.mapping is None else self.mapping.max_error_ns
        if not 0 <= ingestion_ns-stamp+error <= 100_000_000:
            raise ValueError('stale sample')
        provenance = dict(source_stamp_us=source, mapped_stamp_ns=stamp, receipt_ns=receipt_ns,
                          ingestion_ns=ingestion_ns, receipt_wall_ns=arrival_wall,
                          ingestion_wall_ns=ingestion_wall)
        if self.mapping is not None:
            provenance['source_publish_us'] = message.timestamp
        if kind == 'status':
            self._status(message, stamp, provenance)
        else:
            self._sample(message, stamp, provenance)

    def status(self, message, receipt_ns, *, wall_ns=None):
        self._submit('status', message, receipt_ns, wall_ns)

    def _status(self, message, stamp, provenance):
        if self.last_status_stamp is not None and stamp <= self.last_status_stamp:
            self.faults.append('nonadvancing_status'); return
        self.last_status_stamp = stamp
        if self.mapping is not None:
            status = dict(**provenance, arming_state=message.arming_state)
            if self.document['trial_kind'] == 'authority_expiry':
                nav_state = getattr(message, 'nav_state', None)
                if type(nav_state) is not int:
                    self.faults.append('missing_native_nav_state')
                    return
                status['nav_state'] = nav_state
            self.document['status_reports'].append(status)
        if message.arming_state == 2:
            self.seen_armed = True
        elif message.arming_state == 1:
            if self.seen_armed and 'disarm_s' not in self.document:
                self._record_event('disarm', stamp, provenance['receipt_ns'], 'native_vehicle_status',
                                   native_provenance=provenance)
        else:
            self.faults.append('unknown_arming_state')

    def _record_event(self, name, stamp, receipt_ns, source, *, native_provenance=None,
                      owner_event=None, publisher=None):
        ingestion = receipt_ns if native_provenance is None else native_provenance['ingestion_ns']
        if not 0 <= ingestion-stamp <= 100_000_000 or name+'_s' in self.document:
            raise ValueError('invalid or duplicate event')
        self.document[name+'_s'] = stamp/1e9
        row = dict(event=name, stamp_ns=stamp, receipt_ns=receipt_ns, source=source)
        if native_provenance is not None:
            row.update(native_provenance)
        if publisher is not None:
            row.update(publisher)
        if self.mapping is not None:
            row.update({key: self.document[key] for key in ('trial_id', 'owner_id', 'profile_sha256')})
            if owner_event is not None:
                row['px4_issue_us'] = owner_event['px4_issue_us']
        self.document['events'].append(row)

    def sample(self, message, receipt_ns, *, wall_ns=None):
        self._submit('sample', message, receipt_ns, wall_ns)

    def _sample(self, message, stamp, provenance):
        values = [message.x, message.y, message.z, message.vx, message.vy, message.vz]
        if (not all(math.isfinite(v) for v in values) or
                not all(getattr(message, field, False) for field in ('xy_valid','z_valid','v_xy_valid','v_z_valid'))):
            self.faults.append('invalid_position_or_velocity'); return
        if self.mapping is not None:
            counters = tuple(getattr(message, field, None) for field in RESET_FIELDS)
            if (any(type(value) is not int or not 0 <= value <= 255 for value in counters)
                    or (self.reset_counters is not None and counters != self.reset_counters)):
                self.faults.append('estimator_reset_or_missing_counters'); return
            self.reset_counters = counters
            provenance['reset_counters'] = dict(zip(RESET_FIELDS, counters))
        if self.last_time is not None and stamp <= self.last_time:
            self.faults.append('nonadvancing_sample'); return
        self.last_time = stamp
        self.document['samples'].append(dict(time_s=stamp/1e9, **provenance,
            position_m=values[:3], velocity_mps=values[3:]))

    def event(self, raw, receipt_ns, *, publisher=None):
        try:
            event=json.loads(raw)
            name=event['event']; stamp=event['stamp_ns']
            if (self.document['trial_kind'] == 'authority_expiry'
                    or name not in ('brake_command','land_command') or type(stamp) is not int):
                raise ValueError()
            if type(receipt_ns) is not int or stamp <= 0:
                raise ValueError()
            if self.mapping is not None:
                if (any(event.get(key) != self.document[key] for key in ('trial_id','owner_id','profile_sha256'))
                        or type(event.get('px4_issue_us')) is not int
                        or event['px4_issue_us'] != self.mapping.source_stamp_us(stamp)
                        or receipt_ns > self.mapping.valid_until_ns):
                    raise ValueError('unbound owner event')
                if publisher is not None:
                    gid = publisher.get('publisher_gid')
                    if (publisher.get('publisher_node') != self.document['owner_id']
                            or publisher.get('publisher_binding') != 'singleton_graph_only'
                            or not isinstance(gid, str) or not re.fullmatch('[0-9a-f]+', gid)
                            or len(gid) % 2 or int(gid, 16) == 0
                            or (self.event_publisher_gid is not None and gid != self.event_publisher_gid)):
                        raise ValueError('event graph owner changed')
                    self.event_publisher_gid = gid
            if not 0 <= receipt_ns-stamp <= 100_000_000 or name+'_s' in self.document:
                raise ValueError()
            self._record_event(name, stamp, receipt_ns, 'owner_event_topic', owner_event=event,
                               publisher=publisher)
        except (ValueError,KeyError,TypeError): self.faults.append('invalid_or_duplicate_event')

    def result(self):
        required = {'hover_land': ('land_command', 'disarm'),
                    'stopping': ('brake_command', 'land_command', 'disarm'),
                    'authority_expiry': ('disarm',)}[self.document['trial_kind']]
        return dict(**self.document, recording_faults=list(self.faults),
                    pending_clock_samples=len(self.pending),
                    recording_complete=bool(self.document['samples']) and not self.faults and not self.pending
                    and all(k+'_s' in self.document for k in required))

    def readiness(self, now_ns, *, pid):
        """A one-time disarmed handshake; ongoing PID/fault supervision remains required."""
        if (self.mapping is None or self.faults or self.pending or not self.document['samples']
                or not self.document['status_reports']):
            return None
        sample, status = self.document['samples'][-1], self.document['status_reports'][-1]
        if (status['arming_state'] != 1 or
                not 0 <= now_ns - sample['mapped_stamp_ns'] + self.mapping.max_error_ns <= 100_000_000 or
                not 0 <= now_ns - status['mapped_stamp_ns'] + self.mapping.max_error_ns <= 100_000_000):
            return None
        return dict(state='ready', recorder_pid=pid, ros_stamp_ns=now_ns,
                    contract_sha256=self.document.get('contract_sha256'),
                    **{key:self.document[key] for key in ('trial_id','owner_id','profile_sha256','trial_kind')})

    def monitor(self, now_ns, wall_ns):
        """After readiness, silence is a fault even when no invalid callback arrives."""
        if self.faults:
            return
        if type(now_ns) is not int or type(wall_ns) is not int or self.mapping is None:
            self.faults.append('invalid_monitor_clock'); return
        previous = self.monitor_clock
        if previous is not None:
            prior_ros, progress_wall = previous
            if now_ns < prior_ros or wall_ns < progress_wall:
                self.faults.append('recorder_clock_rewind'); return
            if wall_ns - progress_wall >= CLOCK_STALL_NS:
                self.faults.append('recorder_clock_stalled'); return
        if previous is None or now_ns != previous[0]:
            self.monitor_clock = (now_ns, wall_ns)
        for name, rows, maximum in (
                ('position', self.document['samples'], POSITION_SILENCE_NS),
                ('status', self.document['status_reports'], STATUS_SILENCE_NS)):
            if not rows:
                self.faults.append('recorder_' + name + '_missing'); return
            row = rows[-1]
            if (not 0 <= now_ns-row['mapped_stamp_ns']+self.mapping.max_error_ns <= maximum
                    or not 0 <= wall_ns-row['receipt_wall_ns'] <= RECEIPT_WALL_SILENCE_NS):
                self.faults.append('recorder_' + name + '_silent'); return


def write_ready(path, report):
    """Publish a complete handshake without overwriting another run's artifact."""
    path = Path(path)
    temporary = path.with_name(path.name + '.' + str(os.getpid()) + '.tmp')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, allow_nan=False)
        if os.name == 'nt':
            # Windows rename refuses an existing destination; hard links may
            # be unavailable inside the desktop filesystem sandbox.
            os.rename(temporary, path)
        else:
            os.link(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def load_contract(path, expected_sha256):
    raw = Path(path).read_bytes()
    if sha256(raw).hexdigest() != expected_sha256:
        raise ValueError('recorder contract digest mismatch')
    contract = json.loads(raw)
    if contract.get('review_status') != 'reviewed':
        raise ValueError('reviewed recorder contract required')
    topics = contract['topics']
    for role in ('vehicle_local_position', 'vehicle_status'):
        if not re.fullmatch(r'/fmu/out/' + role + r'(?:_v[0-9]+)?', topics[role]):
            raise ValueError('invalid reviewed PX4 topic')
    if not re.fullmatch(r'(/[A-Za-z_][A-Za-z_0-9]*)+', topics['events']):
        raise ValueError('invalid reviewed event topic')
    record = TrialRecording(contract['clock_id'], contract['frame_id'],
        clock_mapping=ClockMapping(**contract['clock_mapping']),
        trial_kind=contract['trial_kind'],
        **{key: contract[key] for key in ('trial_id','owner_id','profile_sha256')})
    record.document.update(contract_sha256=expected_sha256, topics=topics)
    return record, topics


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=90)
    parser.add_argument('--contract',type=Path,required=True)
    parser.add_argument('--contract-sha256',required=True)
    parser.add_argument('--ready-output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists() or args.ready_output.exists() or not 1 <= args.seconds <= 240:
        raise ValueError('new output and bounded duration required')
    record, topics = load_contract(args.contract, args.contract_sha256)
    import rclpy
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.executors import ExternalShutdownException
    from rclpy.impl.implementation_singleton import rclpy_implementation
    from rclpy.utilities import try_shutdown
    from rclpy.signals import SignalHandlerOptions
    from refnav.node import PX4_QOS
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'], signal_handler_options=SignalHandlerOptions.NO)
    stopping = [False]
    previous_handlers = {sig: signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
                         for sig in (signal.SIGINT, signal.SIGTERM)}
    node=rclpy.create_node('qualification_recorder')
    if not initial_clock_ready(lambda: node.get_clock().now().nanoseconds,
                               lambda: rclpy.spin_once(node, timeout_sec=.01)):
        record.faults.append('recorder_initial_clock_unavailable')
        with args.output.open('x') as stream:
            json.dump(record.result(), stream, indent=2, allow_nan=False)
        node.destroy_node()
        try_shutdown()
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        return 2
    position_subscription = node.create_subscription(VehicleLocalPosition,topics['vehicle_local_position'],
        lambda m: record.sample(m,node.get_clock().now().nanoseconds),PX4_QOS)
    def owner_event(message):
        receipt_ns = node.get_clock().now().nanoseconds
        try:
            publisher = event_publisher_identity(node, topics['events'], record.document['owner_id'])
        except (ValueError, TypeError, AttributeError):
            record.faults.append('unverified_event_publisher'); return
        record.event(message.data, receipt_ns, publisher=publisher)
    event_subscription = node.create_subscription(String,topics['events'],owner_event,10)
    status_subscription = node.create_subscription(VehicleStatus,topics['vehicle_status'],
        lambda m: record.status(m,node.get_clock().now().nanoseconds),PX4_QOS)
    node.create_subscription(Clock,'/clock',
        lambda m: record.clock(min(m.clock.sec*10**9+m.clock.nanosec,
                                  node.get_clock().now().nanoseconds)),qos_profile_sensor_data)
    try:
        deadline=monotonic()+args.seconds
        ready_written = False
        while rclpy.ok() and not stopping[0] and monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.01)
            now_ns = node.get_clock().now().nanoseconds
            record._expire_pending(now_ns, monotonic_ns())
            if ready_written:
                record.monitor(now_ns, monotonic_ns())
            if record.faults:
                break
            if not ready_written:
                ready = record.readiness(now_ns, pid=os.getpid())
                if ready is not None:
                    write_ready(args.ready_output, ready)
                    ready_written = True
                    record.monitor(now_ns, monotonic_ns())
    except (KeyboardInterrupt, ExternalShutdownException):
        record.document['shutdown_interrupted'] = True
    except rclpy_implementation.RCLError:
        if rclpy.ok():
            record.faults.append('recorder_runtime_error')
            raise
        record.document['shutdown_interrupted'] = True
    finally:
        if rclpy.ok():
            for subscription in (position_subscription, status_subscription, event_subscription):
                node.destroy_subscription(subscription)
            drain_clock_pending(record, lambda: node.get_clock().now().nanoseconds,
                                lambda: rclpy.spin_once(node, timeout_sec=.001))
        record.document['shutdown_requested'] = stopping[0]
        with args.output.open('x') as stream: json.dump(record.result(),stream,indent=2,allow_nan=False)
        try:
            node.destroy_node()
        finally:
            try_shutdown()
            for sig, handler in previous_handlers.items():
                signal.signal(sig, handler)
    return 0 if record.result()['recording_complete'] else 2

if __name__=='__main__': raise SystemExit(main())
