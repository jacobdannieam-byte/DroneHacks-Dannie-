"""Join recorded depth and estimated pose into independent metric and semantic evidence."""

from dataclasses import dataclass
import math
from threading import RLock

import numpy as np

from .observations import Config, Observation, validate
from .spatial_input import DepthCalibration, DepthCapture, NS, PoseBuffer, PoseSample, SpatialReference, stamp_ok
from .spatial_volume import Cell, MapPolicy, MetricSnapshot, RollingVolume
from .state import WorldState


@dataclass(frozen=True)
class SpatialUpdate:
    accepted: bool
    reason: str
    pose_bracket_ns: tuple = ()
    semantic_events: tuple = ()


@dataclass(frozen=True)
class SpatialSnapshot:
    metric: MetricSnapshot
    semantic: dict
    semantic_time_origin_ns: int | None


class SpatialWorld:
    """One bounded observation owner. No ROS I/O or trajectory command authority."""

    def __init__(self, reference, calibration, *, policy=MapPolicy(),
                 max_pose_gap_ns=80_000_000, max_capture_age_ns=250_000_000,
                 linear_limit_mps=3.0, angular_limit_rad_s=4.0):
        if (not isinstance(reference, SpatialReference) or not isinstance(calibration, DepthCalibration)
                or calibration.body_frame != reference.body_frame or not isinstance(policy, MapPolicy)):
            raise ValueError("calibration and map must agree on body frame")
        if (not stamp_ok(max_pose_gap_ns) or not 0 < max_pose_gap_ns <= NS
                or not stamp_ok(max_capture_age_ns) or not 0 < max_capture_age_ns <= NS
                or not all(math.isfinite(v) and 0 < v <= 20 for v in (linear_limit_mps, angular_limit_rad_s))):
            raise ValueError("invalid synchronization or motion bounds")
        self._ref = reference
        self._calibration = calibration
        self._policy = policy
        self._max_age = max_capture_age_ns
        self._poses = PoseBuffer(max_pose_gap_ns, linear_limit_mps, angular_limit_rad_s)
        self._volume = RollingVolume(policy)
        config = Config(min_depth_m=policy.min_depth_m, max_depth_m=policy.max_depth_m,
                        max_age_s=max_capture_age_ns / NS, max_speed_mps=linear_limit_mps)
        self._world = WorldState(config, reference.frame_id, reference.epoch, reference.clock_id)
        self._lock = RLock()
        self._reset_local()

    def _reset_local(self):
        self._poses.clear()
        self._volume.clear()
        self._now = self._last_capture = self._last_id = self._time_origin = self._last_loss = None
        self._last_received = None
        self._ready = self._reset_required = False

    def reset_reference(self, reference):
        with self._lock:
            if (not isinstance(reference, SpatialReference) or reference.epoch <= self._ref.epoch
                    or reference.body_frame != self._calibration.body_frame):
                raise ValueError("reset requires a newer epoch and the calibrated body frame")
            self._world.reset_reference(reference.frame_id, reference.epoch, reference.clock_id)
            self._ref = reference
            self._reset_local()

    def _revoke(self):
        self._ready = False
        self._volume.cells[self._volume.cells == Cell.FREE] = Cell.UNKNOWN

    def push_pose(self, sample):
        if not isinstance(sample, PoseSample):
            raise ValueError("expected a PoseSample")
        with self._lock:
            if sample.reference != self._ref or self._reset_required:
                return False
            try:
                accepted = self._poses.push(sample)
            except ValueError:
                self._reset_required = True
                self._revoke()
                return False
            if not sample.healthy:
                # The health stream can turn false at the same clock tick as
                # the last pose. Revoke even if the pose buffer rejects a duplicate.
                self._last_loss = max(sample.capture_ns, self._last_loss or 0)
                self._revoke()
            return accepted

    def process(self, frame, *, now_ns):
        if not isinstance(frame, DepthCapture) or not stamp_ok(now_ns):
            raise ValueError("expected depth capture and valid processing time")
        with self._lock:
            if frame.reference != self._ref:
                # A delayed old epoch cannot clear the active map.
                return SpatialUpdate(False, "reference_mismatch")
            if self._now is not None and now_ns < self._now:
                self._reset_required = True
            if self._reset_required:
                self._revoke()
                return SpatialUpdate(False, "reference_reset_required")
            self._now = now_ns
            if (not stamp_ok(frame.capture_ns) or not stamp_ok(frame.received_ns)
                    or not frame.capture_ns <= frame.received_ns <= now_ns):
                self._revoke()
                return SpatialUpdate(False, "invalid_capture_time")
            if self._last_capture is not None and frame.capture_ns <= self._last_capture:
                return SpatialUpdate(False, "duplicate_or_out_of_order")
            if now_ns - frame.capture_ns > self._max_age:
                self._revoke()
                return SpatialUpdate(False, "stale_capture")
            if frame.camera_frame != self._calibration.camera_frame:
                self._revoke()
                return SpatialUpdate(False, "camera_frame_mismatch")
            if self._last_loss is not None and frame.capture_ns <= self._last_loss:
                return SpatialUpdate(False, "pose_health_loss")

            pose = self._poses.at(frame.capture_ns, now_ns)
            if pose is None:
                self._revoke()
                return SpatialUpdate(False, "missing_healthy_capture_pose")
            if self._last_loss is not None and pose.bracket_ns[0] <= self._last_loss:
                self._revoke()
                return SpatialUpdate(False, "pose_bracket_crosses_health_loss")
            depth = np.asarray(frame.depth_m)
            if (depth.shape != (self._calibration.height, self._calibration.width)
                    or depth.dtype.kind not in "fiu"):
                self._revoke()
                return SpatialUpdate(False, "invalid_depth_layout")
            depth = np.array(depth, dtype=np.float32, copy=True)
            valid = np.isfinite(depth) & (depth > self._policy.min_depth_m) & (depth < self._policy.max_depth_m)
            if not np.any(valid):
                self._revoke()
                return SpatialUpdate(False, "no_valid_depth")
            depth[~valid] = np.nan

            origin = frame.capture_ns if self._time_origin is None else self._time_origin
            capture_s = (frame.capture_ns - origin) / NS
            now_s = (now_ns - origin) / NS
            position_error = (pose.position_error_m + (self._calibration.extrinsic_position_error_m or 0.)
                              + self._calibration.depth_error_m)
            angle_error = pose.angle_error_rad + (self._calibration.extrinsic_angle_error_rad or 0.)
            obs = Observation(depth, self._calibration.K, self._calibration.R_body_optical,
                              self._calibration.t_body_optical_m, pose.rotation, pose.position_m,
                              capture_s, capture_s, frame.observation_id, frame_id=self._ref.frame_id,
                              epoch=self._ref.epoch, clock_id=self._ref.clock_id, source="rectified_depth",
                              position_sigma_m=position_error, angle_sigma_rad=angle_error)
            reason = validate(obs, now_s, self._world.config)
            if reason:
                self._revoke()
                return SpatialUpdate(False, reason)
            prepared = self._volume.prepare(depth, self._calibration, pose)
            semantic = self._world.process(obs, now_s)
            if not semantic["accepted"]:
                self._reset_required = semantic["rejection"] in (
                    "pose_jump_reset_required", "reference_reset_required")
                self._revoke()
                return SpatialUpdate(False, semantic["rejection"])
            self._volume.apply(prepared, frame.capture_ns)
            self._last_capture, self._last_id, self._time_origin = frame.capture_ns, frame.observation_id, origin
            self._last_received = frame.received_ns
            self._ready = True
            return SpatialUpdate(True, "accepted", pose.bracket_ns, tuple(semantic["events"]))

    def snapshot(self, *, now_ns):
        with self._lock:
            if not stamp_ok(now_ns) or (self._now is not None and now_ns < self._now):
                raise ValueError("snapshot time precedes the current reference")
            metric = self._volume.snapshot(self._ref, now_ns, self._last_capture, self._last_id,
                                           self._calibration.calibration_id, self._ready,
                                           received_ns=self._last_received)
            semantic_time = None if self._time_origin is None else (now_ns - self._time_origin) / NS
            return SpatialSnapshot(metric, self._world.semantic_snapshot(semantic_time), self._time_origin)
