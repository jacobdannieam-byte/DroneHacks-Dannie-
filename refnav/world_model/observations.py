"""ROS-independent depth/pose contracts. All times use one caller-owned clock."""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from refnav.depth import DepthFrame


@dataclass(frozen=True)
class Config:
    stride: int = 2
    max_pixels: int = 640 * 480
    max_rays: int = 5000
    min_depth_m: float = 0.3
    max_depth_m: float = 10.0
    max_age_s: float = 0.5
    pose_sync_s: float = 0.03
    max_pose_error_m: float = 0.25
    plane_tol_m: float = 0.045
    plane_trials: int = 64
    max_planes: int = 4
    min_plane_hits: int = 45
    bin_m: float = 0.08
    min_wall_width_m: float = 0.25
    min_wall_height_m: float = 0.5
    min_opening_m: float = 0.4
    max_opening_m: float = 3.0
    band_m: float = 0.25
    behind_m: float = 0.25
    min_coverage: float = 0.9
    association_m: float = 0.5
    stale_s: float = 5.0
    forget_s: float = 120.0
    max_entities: int = 128
    max_evidence: int = 8
    beam_frames: int = 3
    beam_age_s: float = 2.0
    jump_m: float = 1.0
    max_speed_mps: float = 3.0

    def __post_init__(self):
        integer_fields = ('stride', 'max_pixels', 'max_rays', 'plane_trials',
                          'max_planes', 'min_plane_hits', 'max_entities',
                          'max_evidence', 'beam_frames')
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f'{name} must be positive and finite')
            if name in integer_fields and (not isinstance(value, int) or isinstance(value, bool)):
                raise ValueError(f'{name} must be an integer')
        if self.min_depth_m >= self.max_depth_m or self.stale_s >= self.forget_s:
            raise ValueError('invalid depth or lifecycle interval')
        if self.min_opening_m >= self.max_opening_m or self.min_coverage > 1:
            raise ValueError('invalid opening/coverage limits')


@dataclass
class Observation:
    depth_m: np.ndarray
    K: tuple
    R_body_cam: np.ndarray
    t_body_cam: np.ndarray
    R_world_body: np.ndarray
    t_world_body: np.ndarray
    stamp: float
    pose_stamp: float
    observation_id: str
    frame_id: str = 'odom'
    epoch: int = 0
    clock_id: str = 'replay'
    source: str = 'depth'
    pose_valid: bool = True
    position_sigma_m: float = 0.02
    angle_sigma_rad: float = 0.005


def rotation_ok(value):
    try:
        a = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return False
    return (a.shape == (3, 3) and np.isfinite(a).all()
            and np.allclose(a.T @ a, np.eye(3), atol=1e-5)
            and abs(np.linalg.det(a) - 1) < 1e-5)


def validate(obs, now, config):
    """Return an explicit rejection reason; never repair unknown calibration or pose."""
    if not all(math.isfinite(t) for t in (now, obs.stamp, obs.pose_stamp)):
        return 'nonfinite_time'
    if obs.stamp > now:
        return 'future_capture'
    if now - obs.stamp > config.max_age_s:
        return 'stale_capture'
    if abs(obs.pose_stamp - obs.stamp) > config.pose_sync_s:
        return 'unsynchronized_pose'
    if not obs.pose_valid:
        return 'invalid_pose'
    if not rotation_ok(obs.R_world_body) or not rotation_ok(obs.R_body_cam):
        return 'invalid_rotation'
    for t in (obs.t_world_body, obs.t_body_cam):
        if np.shape(t) != (3,) or not np.isfinite(t).all():
            return 'invalid_translation'
    if (not math.isfinite(obs.position_sigma_m) or obs.position_sigma_m < 0
            or not math.isfinite(obs.angle_sigma_rad) or obs.angle_sigma_rad < 0):
        return 'invalid_uncertainty'
    if obs.position_sigma_m + config.max_depth_m * obs.angle_sigma_rad > config.max_pose_error_m:
        return 'pose_uncertainty'
    try:
        calibration = np.asarray(obs.K, dtype=float)
    except (TypeError, ValueError):
        return 'invalid_intrinsics'
    if calibration.shape != (4,) or not np.isfinite(calibration).all() or min(calibration[:2]) <= 0:
        return 'invalid_intrinsics'
    d = np.asarray(obs.depth_m)
    if d.ndim != 2 or not d.size or d.size > config.max_pixels:
        return 'invalid_image_size'
    if not (np.issubdtype(d.dtype, np.floating) or np.issubdtype(d.dtype, np.integer)):
        return 'invalid_depth_type'
    if not all(isinstance(v, str) and 0 < len(v) <= 128
               for v in (obs.frame_id, obs.clock_id, obs.source, obs.observation_id)):
        return 'invalid_provenance'
    if not isinstance(obs.epoch, int) or obs.epoch < 0:
        return 'invalid_epoch'
    return None


@dataclass
class Rays:
    origin: np.ndarray
    directions: np.ndarray
    depths: np.ndarray
    valid: np.ndarray

    def points(self):
        return self.origin + self.directions[self.valid] * self.depths[self.valid, None]


def project(obs, config):
    """Reuse DepthFrame's calibrated optical projection; the output frame is full odom."""
    stride = max(config.stride, math.ceil(math.sqrt(np.asarray(obs.depth_m).size / config.max_rays)))
    h, w = np.shape(obs.depth_m)
    while math.ceil(h / stride) * math.ceil(w / stride) > config.max_rays:
        stride += 1
    frame = DepthFrame(obs.depth_m, obs.K, obs.R_body_cam, obs.t_body_cam,
                       obs.R_world_body, stride=stride)
    valid = frame.valid & (frame.z > config.min_depth_m) & (frame.z < config.max_depth_m)
    return Rays(frame.origin + np.asarray(obs.t_world_body), frame.dirs, frame.z, valid)


@dataclass
class Detection:
    semantic_type: str
    center: list
    normal: list
    width_m: float
    height_m: float | None
    uncertainty_m: float
    geometric_confidence: float
    semantic_confidence: float
    missing_evidence: list = field(default_factory=list)
    support: dict = field(default_factory=dict)


@dataclass
class GeometryResult:
    detections: list[Detection]
    rays: Rays
    diagnostics: dict
