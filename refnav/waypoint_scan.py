"""Bounded read-only waypoint scan scheduling over measured state and depth."""

import math

from onboard.adapters.geometry import R_ENU_NED, R_FRD_FLU, rot_z
from refnav.dynamics.validation import wrap_yaw
from refnav.trajectory_evaluation import state_capture_ns
from refnav.world_model.spatial_input import NS, stamp_ok


SECTORS = 24
SECTOR_RAD = math.tau / SECTORS
YAW_TOLERANCE_RAD = .04
MAX_YAW_RATE_RAD_S = .75
SETTLED_YAW_RATE_RAD_S = .10
MAX_DRIFT_M = .10
MAX_HEIGHT_DRIFT_M = .05
MAX_SPEED_MPS = .15
STABLE_NS = NS // 2
STABILIZE_TIMEOUT_NS = 3 * NS
SCAN_TIMEOUT_NS = 35 * NS
MAX_SAMPLE_GAP_NS = 350_000_000


class WaypointScan:
    """A single positive-NED-yaw revolution around one stationary anchor."""

    def __init__(self, scope, goal, arrival_m, now_ns, wall_ns, *, mode='full'):
        if mode not in ('full', 'partial'):
            raise ValueError('scan mode must be full or partial')
        self.mode = mode
        self.offsets = tuple((i + 1) * SECTOR_RAD for i in range(SECTORS)) if mode == 'full' else (-math.pi / 9, math.pi / 9, 0.)
        self.rate_limit = MAX_YAW_RATE_RAD_S if mode == 'full' else .25
        self.travel_rad = 0.
        self.scope, self.goal, self.arrival_m = scope, goal, arrival_m
        self._started = (now_ns, wall_ns)
        self._clock = self._started
        self._identity = None
        self._stable_since = self._last_sample = self._last_yaw = None
        self._anchor = self._start_yaw = None
        self._reviewed_at = self._settled_at = None
        self._seen = set()
        self.completed_sectors = 0
        self.measured_turn_rad = 0.
        self.last_observation_id = None
        self.last_observation_ns = None
        self.target = None
        self.state = 'SCAN_STABILIZE'
        self.reason = 'scan_waiting_for_stable_pose'

    def fail(self, reason):
        self.target = None
        self.state, self.reason = 'SCAN_REFUSED', reason

    def report(self):
        return dict(scope=self.scope, state=self.state, reason=self.reason, sectors=len(self.offsets), mode=self.mode,
                    measured_travel_rad=self.travel_rad,
                    completed_sectors=self.completed_sectors, measured_turn_rad=self.measured_turn_rad,
                    target_ned=self.target, observation_id=self.last_observation_id,
                    observation_capture_ns=self.last_observation_ns, command_authority=False)

    def step(self, now_ns, wall_ns, inputs, depth, check_clearance):
        self.target = None
        if self.state in ('SCAN_COMPLETE', 'SCAN_REFUSED'):
            return
        if (not stamp_ok(now_ns) or not stamp_ok(wall_ns) or now_ns < self._clock[0] or wall_ns < self._clock[1]):
            self.fail('scan_clock_rewind_or_invalid')
            return
        self._clock = (now_ns, wall_ns)
        elapsed = max(now_ns - self._started[0], wall_ns - self._started[1])
        if elapsed >= (SCAN_TIMEOUT_NS if self.mode == 'full' else 15 * NS) or (self._anchor is None and elapsed >= STABILIZE_TIMEOUT_NS):
            self.fail('scan_timeout' if self._anchor is not None else 'scan_stabilization_timeout')
            return

        state, alignment, metric = inputs['state'], inputs['alignment'], inputs['snapshot'].metric
        identity = (state.reference, alignment.map, alignment.map_from_enu_yaw_rad, alignment.ned_origin_in_map_m,
                    alignment.provenance, metric.calibration_id, inputs['estimator_reset_counter'])
        if (metric.reference != self.goal.reference or alignment.map != self.goal.reference
                or (self._identity is not None and identity != self._identity)):
            self.fail('scan_reference_changed')
            return
        self._identity = identity
        if (depth is None or depth.reference != metric.reference or depth.observation_id != metric.observation_id
                or depth.capture_ns != metric.capture_ns or depth.calibration.calibration_id != metric.calibration_id
                or not 0 <= now_ns - depth.capture_ns <= 250_000_000):
            self.fail('scan_capture_pose_unavailable')
            return

        sample = state_capture_ns(state, alignment)
        if not 0 <= now_ns - sample <= 100_000_000 or (self._last_sample is not None and sample < self._last_sample):
            self.fail('scan_state_stale_or_rewound')
            return
        if sample == self._last_sample:
            self.reason = 'scan_waiting_for_advancing_state'
            return
        transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
        map_position = transform @ state.position_m + alignment.ned_origin_in_map_m
        speed = math.hypot(*state.velocity_mps)
        yaw_rate = abs(state.yaw_rate_rad_s)
        sample_gap = 0 if self._last_sample is None else sample - self._last_sample
        if self._anchor is None:
            margin = inputs['vehicle'].state_position_error_m + alignment.position_error_m
            stable = (math.dist(map_position, self.goal.position_m) + margin <= self.arrival_m
                      and speed <= .05 and yaw_rate <= SETTLED_YAW_RATE_RAD_S)
            if not stable or sample_gap > MAX_SAMPLE_GAP_NS:
                self._stable_since = None
            if stable and self._stable_since is None:
                self._stable_since = sample
            self._last_sample, self._last_yaw = sample, state.yaw_rad
            self.reason = 'scan_waiting_for_stable_pose'
            if (self._stable_since is None or sample - self._stable_since < STABLE_NS
                    or depth.capture_ns <= self._stable_since):
                return
            self._anchor, self._start_yaw = state.position_m, state.yaw_rad
        else:
            delta = wrap_yaw(state.yaw_rad - self._last_yaw)
            desired = self.offsets[self.completed_sectors]
            wrong_direction = delta < -.03 if self.mode == 'full' else delta * (desired - self.measured_turn_rad) < -.003
            if (sample_gap > MAX_SAMPLE_GAP_NS or yaw_rate > self.rate_limit
                    or abs(delta) > self.rate_limit * sample_gap / NS + .015 or wrong_direction):
                self.fail('scan_yaw_rate_or_progress_unverified')
                return
            if (speed > MAX_SPEED_MPS or math.dist(state.position_m, self._anchor) > MAX_DRIFT_M
                    or abs(state.position_m[2] - self._anchor[2]) > MAX_HEIGHT_DRIFT_M):
                self.fail('scan_anchor_drift_or_speed')
                return
            self.measured_turn_rad += delta
            self.travel_rad += abs(delta)
            if self.mode == 'partial' and self.travel_rad > math.radians(80) + .10:
                self.fail('scan_partial_travel_exceeded')
                return
            self._last_sample, self._last_yaw = sample, state.yaw_rad

        desired_progress = self.offsets[self.completed_sectors]
        target_yaw = wrap_yaw(self._start_yaw + desired_progress)
        previous = 0. if self.completed_sectors == 0 else self.offsets[self.completed_sectors - 1]
        outside = (self.measured_turn_rad > desired_progress + YAW_TOLERANCE_RAD
                   or abs(wrap_yaw(target_yaw - state.yaw_rad)) > SECTOR_RAD + YAW_TOLERANCE_RAD) if self.mode == 'full' else (
                       not min(previous, desired_progress) - YAW_TOLERANCE_RAD <= self.measured_turn_rad <= max(previous, desired_progress) + YAW_TOLERANCE_RAD)
        if outside:
            self.fail('scan_sector_skipped_or_out_of_range')
            return
        target = (*self._anchor, target_yaw)
        refusal = check_clearance(target)
        if refusal is not None:
            self.fail(refusal)
            return
        self.target = target
        self.state, self.reason = 'SCAN_OBSERVE', 'scan_sector_reviewed'
        if self._reviewed_at is None:
            self._reviewed_at = sample
            return
        if abs(self.measured_turn_rad - desired_progress) > YAW_TOLERANCE_RAD or yaw_rate > SETTLED_YAW_RATE_RAD_S:
            self._settled_at = None
            return
        if self._settled_at is None:
            self._settled_at = sample
        if self.mode == 'partial' and sample - self._settled_at < 300_000_000:
            self.reason = 'scan_waiting_for_settled_dwell'
            return
        # A sector only counts once a fresh frame was actually captured facing it.
        optical_axis = tuple(row[2] for row in depth.calibration.R_body_optical)
        forward = transform @ rot_z(target_yaw) @ R_FRD_FLU @ optical_axis
        capture_forward = depth.pose.rotation @ optical_axis
        angle_error = wrap_yaw(math.atan2(capture_forward[1], capture_forward[0]) - math.atan2(forward[1], forward[0]))
        self.reason = 'scan_waiting_for_sector_depth'
        if (depth.capture_ns <= self._settled_at or depth.observation_id in self._seen
                or abs(angle_error) > YAW_TOLERANCE_RAD
                or min(math.hypot(*forward[:2]), math.hypot(*capture_forward[:2])) < .5
                or math.dist(depth.pose.position_m, transform @ self._anchor + alignment.ned_origin_in_map_m) > MAX_DRIFT_M):
            return
        self._seen.add(depth.observation_id)
        self.last_observation_id, self.last_observation_ns = depth.observation_id, depth.capture_ns
        self.completed_sectors += 1
        self._reviewed_at = self._settled_at = None
        self.target = None
        if self.completed_sectors == len(self.offsets):
            self.state, self.reason = 'SCAN_COMPLETE', 'scan_complete'
        else:
            self.reason = 'scan_sector_observed'
