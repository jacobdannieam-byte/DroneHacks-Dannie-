"""Experimental-owner callback clock fence; original messages remain unchanged."""
import math
from time import perf_counter_ns

from onboard.diagnostics.phase1_probe import RESET_FIELDS
from refnav.core import NAV_OFFBOARD, Output

WAIT_NS = 50_000_000
CAPACITY = 32


class ExperimentalOwnerClockMixin:
    """Place before RefNav in the isolated experiment owner's MRO only."""

    def __init__(self, *args, **kwargs):
        self._owner_clock_pending = []
        self._owner_clock_seen = {}
        self._owner_clock_last = None
        self._owner_clock_fault = None
        self._owner_clock_audit = []
        super().__init__(*args, **kwargs)

    def _owner_clock_wall(self):
        return perf_counter_ns()

    def _owner_clock_clear(self, kind):
        key = {'lpos':'position', 'status':'authority', 'land':'landed'}.get(kind)
        if key:
            self._hybrid_reports.pop(key, None)
        if kind == 'lpos': self.v.pos_ok = False
        elif kind == 'status': self.v.armed = self.v.nav_state = None
        elif kind == 'land': self.v.landed = self.v.landed_capture_ns = None
        elif kind == 'att':
            self.v.q = self.v.attitude_capture_ns = self.v.attitude_received_ns = self.v.attitude_reset_counter = None
        elif kind == 'flags': self._yaw_align = (False, None)
        self._invalidate_hybrid()

    def _owner_clock_fail(self, reason):
        if self._owner_clock_fault is None:
            self._owner_clock_fault = reason
        self._owner_clock_pending.clear()
        for kind in ('lpos','status','land','att','flags'):
            self._owner_clock_clear(kind)

    def _owner_clock_identity(self, kind, message):
        if kind == 'status': return (message.arming_state == 2, int(message.nav_state))
        if kind == 'land': return bool(message.landed)
        if kind == 'lpos':
            if (not message.xy_valid or not message.z_valid
                    or not all(math.isfinite(x) for x in (message.x,message.y,message.z,message.heading))):
                raise ValueError('invalid_position')
            return tuple(getattr(message, key) for key in RESET_FIELDS)
        if kind == 'att':
            q = tuple(float(x) for x in message.q)
            if len(q) != 4 or not all(math.isfinite(x) for x in q) or abs(sum(x*x for x in q)-1.) > 1e-3:
                raise ValueError('invalid_attitude')
            return message.quat_reset_counter
        return bool(message.cs_yaw_align)

    def _owner_clock_drain(self):
        now, wall = self.get_clock().now().nanoseconds, self._owner_clock_wall()
        if self._owner_clock_fault:
            return
        if self._owner_clock_last is not None and (now < self._owner_clock_last[0] or wall < self._owner_clock_last[1]):
            self._owner_clock_fail('experimental_owner_clock_rewind')
            return
        self._owner_clock_last = (now, wall)
        pending = self._owner_clock_pending
        self._owner_clock_pending = []
        blocked = set()
        for row in pending:
            kind, message, callback, received, received_wall, target = row
            if wall-received_wall > WAIT_NS or now-received > WAIT_NS:
                self._owner_clock_fail('experimental_owner_clock_wait_expired')
                return
            if target > now or kind in blocked:
                self._owner_clock_pending.append(row)
                blocked.add(kind)
                continue
            callback(message)
            if len(self._owner_clock_audit) < 256:
                self._owner_clock_audit.append(dict(kind=kind, source_ready_ns=target,
                    original_receipt_ns=received, original_wall_ns=received_wall,
                    released_ros_ns=now, released_wall_ns=wall))

    def _owner_clock_receive(self, kind, message, callback):
        self._owner_clock_drain()
        if self._owner_clock_fault:
            return
        now, wall = self.get_clock().now().nanoseconds, self._owner_clock_wall()
        mapping = self._hybrid_runtime.loop.replay.hybrid.clock
        try:
            sample = getattr(message, 'timestamp_sample', None) if kind in ('lpos','att') else message.timestamp
            publish = message.timestamp
            if type(sample) is not int or type(publish) is not int or not 0 < sample <= publish:
                raise ValueError('invalid_source_time')
            capture = sample*1000+mapping.offset_ns
            target = publish*1000+mapping.offset_ns+mapping.max_error_ns
            if not mapping.valid_from_ns <= capture <= target <= mapping.valid_until_ns or target-now > WAIT_NS:
                raise ValueError('source_outside_clock_bound')
            identity = self._owner_clock_identity(kind, message)
            payload = ((message.x,message.y,message.z,message.heading,bool(message.xy_valid),bool(message.z_valid),
                        bool(message.heading_good_for_control)) if kind == 'lpos' else
                       tuple(message.q) if kind == 'att' else identity)
            previous = self._owner_clock_seen.get(kind)
            if previous and (capture < previous[0] or
                             (capture == previous[0] and (identity != previous[1] or payload != previous[2]))):
                raise ValueError('source_rewound_or_conflicting')
            # A newer mode/reset/contact report immediately revokes the old
            # admission even if its full payload must wait for delivered time.
            if previous and identity != previous[1]:
                self._owner_clock_clear(kind)
                self._owner_clock_pending = [row for row in self._owner_clock_pending if row[0] != kind]
            self._owner_clock_seen[kind] = (capture, identity, payload)
            if target > now or any(row[0] == kind for row in self._owner_clock_pending):
                if len(self._owner_clock_pending) >= CAPACITY:
                    raise ValueError('pending_capacity_exceeded')
                self._owner_clock_pending.append((kind,message,callback,now,wall,target))
            else:
                callback(message)
        except (ValueError, TypeError, AttributeError, OverflowError) as error:
            self._owner_clock_fail('experimental_owner_clock:'+str(error))

    def _on_lpos(self, message):
        self._owner_clock_receive('lpos', message, super()._on_lpos)

    def _on_status(self, message):
        self._owner_clock_receive('status', message, super()._on_status)

    def _on_land(self, message):
        self._owner_clock_receive('land', message, super()._on_land)

    def _on_att(self, message):
        self._owner_clock_receive('att', message, super()._on_att)

    def _on_est_flags(self, message):
        self._owner_clock_receive('flags', message, super()._on_est_flags)

    def _runtime_owner(self, now_ns):
        self._owner_clock_drain()
        if self._owner_clock_fault:
            return self.mission.yield_hybrid_authority(now_ns/1e9, self._owner_clock_fault)
        if getattr(self.mission, 'phase', None) == 'arm':
            status = [row[1] for row in self._owner_clock_pending if row[0] == 'status']
            if status and status[-1].nav_state == NAV_OFFBOARD and status[-1].arming_state in (1,2):
                # Expected acquisition transition, not continued authority: hold
                # all output until its original full report can be accepted.
                return Output('arm', stream=False)
        return super()._runtime_owner(now_ns)

    def _hybrid_watchdog_tick(self):
        self._owner_clock_drain()
        return super()._hybrid_watchdog_tick()
