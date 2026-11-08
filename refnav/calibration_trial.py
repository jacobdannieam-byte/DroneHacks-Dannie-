"""One-owner isolated SITL stopping calibration. No ROS or actuator access here."""

from dataclasses import dataclass
import math

from .core import NAV_AUTO_LAND, NAV_OFFBOARD


NS = 10**9
CLOCK_STALL_NS = 300_000_000
RECOVERY_LEASE_NS = 250_000_000
STOP_MPS = .05
ZERO = (0., 0., 0.)


def _numbers(values):
    return all(type(value) in (int, float) and math.isfinite(value) for value in values)


@dataclass(frozen=True)
class CalibrationPlan:
    """Reviewed absolute PX4 NED origin and relative vehicle-center corridor.

    Advance only in surveyed +NED north. The entire corridor must support
    normal landing. Geometry review reserves body, tracking and overshoot space
    outside this box; these limits do not establish that space themselves.
    """

    trial_kind: str
    origin_ned_m: tuple
    minimum_offset_m: tuple
    maximum_offset_m: tuple
    origin_tolerance_m: float
    height_m: float
    hover_tolerance_m: float
    north_brake_m: float
    transit_speed_mps: float
    climb_speed_mps: float
    actual_speed_limit_mps: float
    transit_timeout_s: float
    trial_ros_limit_s: float
    trial_wall_limit_s: float
    receipt_wall_limit_s: float
    settle_duration_s: float = .5
    brake_timeout_s: float | None = None

    def __post_init__(self):
        for name in ('origin_ned_m', 'minimum_offset_m', 'maximum_offset_m'):
            value = tuple(getattr(self, name))
            if len(value) != 3 or not _numbers(value):
                raise ValueError('Finite three-dimensional NED limits required')
            object.__setattr__(self, name, value)
        low, high = self.minimum_offset_m, self.maximum_offset_m
        scalars = (self.origin_tolerance_m, self.height_m, self.hover_tolerance_m, self.north_brake_m,
                   self.transit_speed_mps, self.climb_speed_mps, self.actual_speed_limit_mps,
                   self.transit_timeout_s, self.trial_ros_limit_s, self.trial_wall_limit_s,
                   self.receipt_wall_limit_s, self.settle_duration_s)
        if (self.trial_kind not in ('hover_land', 'stopping', 'authority_expiry') or not _numbers(scalars)
                or not all(-5 <= lo < 0 < hi <= 5 for lo, hi in zip(low, high))
                or not 0 < self.origin_tolerance_m <= .10
                or not 0 < self.hover_tolerance_m < self.height_m
                or not low[2] < -self.height_m - self.hover_tolerance_m < -self.origin_tolerance_m
                or not self.origin_tolerance_m < self.north_brake_m < high[0]
                or not .10 <= self.transit_speed_mps <= self.actual_speed_limit_mps <= .50
                or not 0 < self.climb_speed_mps <= self.transit_speed_mps
                or not 0 < self.transit_timeout_s < self.trial_ros_limit_s <= 60
                or not self.trial_ros_limit_s <= self.trial_wall_limit_s <= 120
                or not .30 <= self.receipt_wall_limit_s <= 3.
                or not .5 <= self.settle_duration_s <= 2.
                or (self.brake_timeout_s is not None and
                    (not _numbers((self.brake_timeout_s,)) or
                     not self.settle_duration_s < self.brake_timeout_s < self.trial_ros_limit_s))):
            raise ValueError('Invalid bounded calibration plan')


@dataclass(frozen=True)
class Observation:
    position: tuple
    velocity: tuple
    position_stamp_ns: int
    authority_stamp_ns: int
    armed: bool | None
    landed: bool
    nav_state: int


@dataclass(frozen=True)
class Intent:
    velocity_ned_mps: tuple | None = None
    command: str | None = None
    event: str | None = None
    recovery_lease: bool = False


class CalibrationTrial:
    """A one-shot northbound run from verified disarmed start to native landing."""

    def __init__(self, plan):
        self.plan = plan
        self.phase = 'wait'
        self.result = None
        self.recovery_reason = None
        self._start_wall = None
        self._start_ros = None
        self._clock = None
        self._phase_time = None
        self._settling = None
        self._last_position_stamp = None
        self.native_land_ack_ns = None
        self.native_disarm_ns = None
        self.max_observed_speed_mps = 0.
        self.recovery_lease = None
        self._pending = None
        self._command_stamp = None
        self._operator_land = False
        self.events = []

    def stop(self, reason):
        self.phase = 'done'
        self.result = reason
        self._pending = None

    def request_land(self):
        self._operator_land = True

    def _enter(self, phase, ros_ns, wall_ns):
        self.phase = phase
        self._phase_time = (ros_ns, wall_ns)
        self._settling = None

    def _elapsed(self, ros_ns, wall_ns):
        return ((ros_ns - self._phase_time[0]) / NS, (wall_ns - self._phase_time[1]) / NS)

    def _issue(self, intent, ros_ns):
        self._pending = (intent, ros_ns)
        return intent

    def published(self, intent, stamp_ns, px4_issue_us, *, wall_ns=None):
        """Acknowledge a returned intent only after the actual publisher accepts it."""
        if (self._pending is None or intent is not self._pending[0] or type(stamp_ns) is not int
                or type(px4_issue_us) is not int or px4_issue_us <= 0
                or not 0 <= stamp_ns - self._pending[1] <= CLOCK_STALL_NS):
            self.stop('invalid_publication_receipt')
            raise ValueError('Invalid calibration publication receipt')
        self._pending = None
        if intent.recovery_lease:
            if type(wall_ns) is not int or not 0 <= wall_ns - self._clock[1] < 50_000_000:
                self.stop('invalid_recovery_lease_clock')
                raise ValueError('Recovery lease needs its actual steady publication time')
            self.recovery_lease = dict(last_output_ros_ns=stamp_ns, last_output_px4_us=px4_issue_us,
                                       last_output_wall_ns=wall_ns, deadline_wall_ns=wall_ns + RECOVERY_LEASE_NS)
        if intent.command is not None:
            self._command_stamp = stamp_ns
        if intent.event is None:
            return None
        if self.events and stamp_ns <= self.events[-1]['stamp_ns']:
            self.stop('nonadvancing_publication_receipt')
            raise ValueError('Nonadvancing calibration event receipt')
        event = dict(event=intent.event, stamp_ns=stamp_ns, px4_issue_us=px4_issue_us)
        self.events.append(event)
        return event

    def _land(self, reason, ros_ns, wall_ns):
        self.recovery_reason = reason
        self._enter('land', ros_ns, wall_ns)
        return self._issue(Intent(command='land', event='land_command'), ros_ns)

    def _settled(self, obs, speed):
        stamp = obs.position_stamp_ns
        if speed > STOP_MPS:
            self._settling = None
            return False
        if self._settling is None:
            self._settling = stamp
        return stamp - self._settling >= round(self.plan.settle_duration_s * NS)

    def step(self, ros_ns, wall_ns, observation, *, admitted):
        if self.phase == 'done':
            return None
        if self._pending is not None:
            self.stop('publication_unconfirmed')
            return None
        if self._start_wall is None:
            self._start_wall = wall_ns
        if wall_ns - self._start_wall >= self.plan.trial_wall_limit_s * NS:
            self.stop('wall_deadline')
            return None
        if admitted is not True:
            self.stop('owner_admission_lost')
            return None
        if self._clock is not None:
            prior_ros, prior_wall = self._clock
            if ros_ns < prior_ros or not 0 <= wall_ns - prior_wall < CLOCK_STALL_NS:
                self.stop('clock_stalled_or_rewound')
                return None
            if ros_ns == prior_ros:
                return None
        if ros_ns <= 0:
            return None
        self._clock = (ros_ns, wall_ns)
        if self._start_ros is None:
            self._start_ros = ros_ns
        if ros_ns - self._start_ros >= self.plan.trial_ros_limit_s * NS:
            self.stop('ros_deadline')
            return None

        obs = observation
        if (obs is None or type(obs.armed) is not bool or type(obs.landed) is not bool
                or len(obs.position) != 3 or len(obs.velocity) != 3
                or not _numbers((*obs.position, *obs.velocity))
                or not 0 < obs.position_stamp_ns <= ros_ns
                or not 0 < obs.authority_stamp_ns <= ros_ns
                or (self._last_position_stamp is not None and obs.position_stamp_ns < self._last_position_stamp)):
            if self.phase != 'wait':
                self.stop('native_observation_lost')
            return None
        self._last_position_stamp = obs.position_stamp_ns
        offset = tuple(a - b for a, b in zip(obs.position, self.plan.origin_ned_m))
        speed = math.hypot(*obs.velocity)
        self.max_observed_speed_mps = max(self.max_observed_speed_mps, speed)
        in_lane = all(lo <= x <= hi for x, lo, hi in zip(offset, self.plan.minimum_offset_m,
                                                        self.plan.maximum_offset_m))
        if not in_lane:
            self.stop('lane_exceeded')
            return None

        if self.phase == 'land':
            if speed > self.plan.actual_speed_limit_mps:
                self.recovery_reason = self.recovery_reason or 'speed_limit_during_land'
            if max(self._elapsed(ros_ns, wall_ns)) >= 30.:
                self.stop('landing_deadline')
            elif not obs.armed:
                if self.native_land_ack_ns is None or obs.authority_stamp_ns <= self.native_land_ack_ns:
                    self.stop('disarmed_before_landing_ack')
                else:
                    self.native_disarm_ns = obs.authority_stamp_ns
                    self.stop((self.recovery_reason or 'complete') if obs.landed else 'disarmed_without_landed')
            elif obs.nav_state == NAV_AUTO_LAND and obs.authority_stamp_ns > self._command_stamp:
                if self.native_land_ack_ns is None:
                    self.native_land_ack_ns = obs.authority_stamp_ns
            elif obs.nav_state != NAV_OFFBOARD or self.native_land_ack_ns is not None:
                self.stop('native_authority_lost_during_land')
            return None

        if self.phase == 'authority_expiry':
            height = -offset[2]
            if (self._operator_land or not obs.armed or obs.landed or obs.nav_state != NAV_OFFBOARD
                    or speed > STOP_MPS or abs(height - self.plan.height_m) > self.plan.hover_tolerance_m):
                self.stop('recovery_window_invalidated')
            elif wall_ns >= self.recovery_lease['deadline_wall_ns']:
                self.recovery_lease.update(expiry_detected_ros_ns=ros_ns, expiry_detected_wall_ns=wall_ns)
                self.stop('planned_authority_expiry')
            return None  # This branch can never publish a renewal, including operator LAND.

        if self._operator_land:
            if obs.armed and obs.nav_state == NAV_OFFBOARD:
                return self._land('operator_land', ros_ns, wall_ns)
            self.stop('operator_cancelled_without_landing_authority')
            return None

        if self.phase in ('wait', 'prime', 'mode', 'arm'):
            if not obs.landed or math.hypot(*offset) > self.plan.origin_tolerance_m or speed > STOP_MPS:
                self.stop('start_not_stationary_on_reviewed_pad')
                return None
            if self.phase != 'arm' and obs.armed:
                self.stop('unexpected_arming')
                return None
            if self.phase == 'wait':
                if obs.nav_state == NAV_OFFBOARD:
                    self.stop('offboard_active_before_request')
                    return None
                self._enter('prime', ros_ns, wall_ns)
            elif self.phase == 'prime' and min(self._elapsed(ros_ns, wall_ns)) >= 1.:
                self._enter('mode', ros_ns, wall_ns)
                return self._issue(Intent(ZERO, 'offboard'), ros_ns)
            elif self.phase in ('mode', 'arm') and max(self._elapsed(ros_ns, wall_ns)) >= 3.:
                self.stop('command_ack_deadline')
                return None
            elif (self.phase == 'mode' and obs.nav_state == NAV_OFFBOARD
                  and obs.authority_stamp_ns > self._command_stamp):
                self._enter('arm', ros_ns, wall_ns)
                return self._issue(Intent(ZERO, 'arm'), ros_ns)
            elif self.phase == 'arm':
                if obs.nav_state != NAV_OFFBOARD:
                    self.stop('offboard_lost_before_takeoff')
                    return None
                if obs.armed and obs.authority_stamp_ns > self._command_stamp:
                    self._enter('climb', ros_ns, wall_ns)
            if self.phase != 'climb':
                return self._issue(Intent(ZERO), ros_ns)

        if obs.nav_state != NAV_OFFBOARD or not obs.armed:
            self.stop('native_authority_lost')
            return None
        if speed > self.plan.actual_speed_limit_mps:
            return self._land('speed_limit', ros_ns, wall_ns)
        height_error = self.plan.height_m + offset[2]
        if self.phase == 'climb':
            if abs(height_error) <= self.plan.hover_tolerance_m and not obs.landed:
                if self._settled(obs, speed):
                    if self.plan.trial_kind == 'hover_land':
                        return self._land(None, ros_ns, wall_ns)
                    if self.plan.trial_kind == 'authority_expiry':
                        self._enter('authority_expiry', ros_ns, wall_ns)
                        return self._issue(Intent(ZERO, recovery_lease=True), ros_ns)
                    self._enter('transit', ros_ns, wall_ns)
                else:
                    return self._issue(Intent(ZERO), ros_ns)
            elif height_error < -self.plan.hover_tolerance_m:
                return self._land('hover_height_exceeded', ros_ns, wall_ns)
            else:
                self._settling = None
                climb = min(self.plan.climb_speed_mps, max(0., height_error))
                return self._issue(Intent((0., 0., -climb)), ros_ns)
        if obs.landed or abs(height_error) > self.plan.hover_tolerance_m:
            return self._land('left_navigation_height_band', ros_ns, wall_ns)
        if self.phase == 'transit':
            if max(self._elapsed(ros_ns, wall_ns)) >= self.plan.transit_timeout_s:
                return self._land('transit_trigger_deadline', ros_ns, wall_ns)
            if offset[0] >= self.plan.north_brake_m:
                self._enter('brake', ros_ns, wall_ns)
                return self._issue(Intent(ZERO, event='brake_command'), ros_ns)
            return self._issue(Intent((self.plan.transit_speed_mps, 0., 0.)), ros_ns)
        if (self.plan.brake_timeout_s is not None
                and max(self._elapsed(ros_ns, wall_ns)) >= self.plan.brake_timeout_s):
            return self._land('brake_settle_deadline', ros_ns, wall_ns)
        if self._settled(obs, speed):
            return self._land(None, ros_ns, wall_ns)
        return self._issue(Intent(ZERO), ros_ns)
