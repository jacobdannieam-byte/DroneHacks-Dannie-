"""Capture-time contracts for a rectified depth stream and estimated pose."""

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math

import numpy as np

from onboard.vio.frames import qnormalize
from refnav.depth import quat_to_mat
from .observations import rotation_ok

NS = 1_000_000_000
MAX_PIXELS = 640 * 480
MAX_LOCAL_M = 1000.0


def stamp_ok(stamp):
    return type(stamp) is int and 0 <= stamp < 2**63


def _name(value):
    return isinstance(value, str) and 0 < len(value.strip()) <= 128


def vector(value, size):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(f"expected {size} finite components")
    return tuple(map(float, result))


@dataclass(frozen=True)
class SpatialReference:
    """One gravity-aligned z-up map, named body FLU frame and caller-owned clock."""

    frame_id: str
    clock_id: str
    epoch: int
    body_frame: str = "base_link"

    def __post_init__(self):
        if not all(_name(v) for v in (self.frame_id, self.clock_id, self.body_frame)):
            raise ValueError("spatial references require named frames and clock")
        if type(self.epoch) is not int or self.epoch < 0:
            raise ValueError("reference epoch must be a nonnegative integer")


@dataclass(frozen=True)
class DepthCalibration:
    camera_frame: str
    body_frame: str
    width: int
    height: int
    K: tuple
    R_body_optical: tuple
    t_body_optical_m: tuple
    depth_error_m: float
    calibration_id: str
    validated_pixel_footprint: bool = False
    extrinsic_position_error_m: float | None = None
    extrinsic_angle_error_rad: float | None = None

    def __post_init__(self):
        if not all(_name(v) for v in (self.camera_frame, self.body_frame, self.calibration_id)):
            raise ValueError("calibration requires frame names and provenance")
        if (type(self.width) is not int or type(self.height) is not int
                or not (0 < self.width * self.height <= MAX_PIXELS and min(self.width, self.height) > 0)):
            raise ValueError("invalid depth calibration dimensions")
        intrinsics = vector(self.K, 4)
        if min(intrinsics[:2]) <= 0 or not rotation_ok(self.R_body_optical):
            raise ValueError("invalid rectified pinhole calibration")
        translation = vector(self.t_body_optical_m, 3)
        if math.hypot(*translation) > 2 or not math.isfinite(self.depth_error_m) or not 0 <= self.depth_error_m <= 1:
            raise ValueError("invalid extrinsic translation or depth error bound")
        if type(self.validated_pixel_footprint) is not bool:
            raise ValueError("pixel-footprint evidence must be declared explicitly")
        for error, limit in ((self.extrinsic_position_error_m, 1.), (self.extrinsic_angle_error_rad, .5)):
            if ((error is None and self.validated_pixel_footprint)
                    or (error is not None and (not math.isfinite(error) or not 0 <= error <= limit))):
                raise ValueError("free evidence requires valid explicit extrinsic error bounds")
        object.__setattr__(self, "K", intrinsics)
        object.__setattr__(self, "R_body_optical", tuple(vector(row, 3) for row in self.R_body_optical))
        object.__setattr__(self, "t_body_optical_m", translation)


@dataclass(frozen=True)
class PoseSample:
    reference: SpatialReference
    capture_ns: int
    received_ns: int
    healthy: bool
    position_m: tuple | None = None
    q_wxyz: tuple | None = None
    position_error_m: float | None = None
    angle_error_rad: float | None = None

    def __post_init__(self):
        if (not isinstance(self.reference, SpatialReference) or type(self.healthy) is not bool
                or not stamp_ok(self.capture_ns) or not stamp_ok(self.received_ns)
                or self.received_ns < self.capture_ns):
            raise ValueError("invalid pose provenance, health or time")
        if not self.healthy:
            return  # Explicit loss marker. Never used as a transform.
        position = vector(self.position_m, 3)
        quat = vector(self.q_wxyz, 4)
        if max(map(abs, position)) > MAX_LOCAL_M or not math.isclose(math.hypot(*quat), 1.0, abs_tol=.01):
            raise ValueError("invalid local position or quaternion")
        if not all(v is not None and math.isfinite(v) and 0 <= v <= limit
                   for v, limit in ((self.position_error_m, 1.0), (self.angle_error_rad, .5))):
            raise ValueError("healthy pose requires explicit metric and angular error bounds")
        object.__setattr__(self, "position_m", position)
        object.__setattr__(self, "q_wxyz", qnormalize(quat))


@dataclass(frozen=True)
class DepthCapture:
    reference: SpatialReference
    capture_ns: int
    received_ns: int
    camera_frame: str
    observation_id: str
    depth_m: np.ndarray


def decode_rectified_depth(data, *, encoding, width, height, step, bigendian=False):
    """Decode recorded sensor_msgs/Image layout. Zero and 65535 uint16 are no return."""
    code = {"16uc1": "u2", "32fc1": "f4"}.get(encoding.lower())
    if code is None:
        raise ValueError("expected 16UC1 millimeters or 32FC1 meters")
    if (not all(type(v) is int and v > 0 for v in (width, height, step))
            or width * height > MAX_PIXELS or height * step > 4 * MAX_PIXELS + 64 * 1024):
        raise ValueError("invalid depth image dimensions or byte budget")
    dtype = np.dtype((">" if bigendian else "<") + code)
    raw = memoryview(data).cast("B")
    if step < width * dtype.itemsize or len(raw) != height * step:
        raise ValueError("invalid depth image row layout")
    image = np.ndarray((height, width), dtype=dtype, buffer=raw, strides=(step, dtype.itemsize))
    meters = image.astype(np.float32)
    invalid = ~np.isfinite(meters) | (meters <= 0)
    if code == "u2":
        invalid |= image == 65535
        meters *= .001
    meters[invalid] = np.nan
    return meters


@dataclass(frozen=True)
class Alignment:
    position_m: tuple
    rotation: np.ndarray
    position_error_m: float
    angle_error_rad: float
    bracket_ns: tuple


@dataclass(frozen=True, eq=False)
class AlignedDepth:
    """Opt-in capture bundle emitted only after SpatialWorld accepts the input."""

    reference: SpatialReference
    capture_ns: int
    received_ns: int
    processed_ns: int
    observation_id: str
    calibration: DepthCalibration
    depth_m: np.ndarray
    pose: Alignment


class PoseBuffer:
    """Bounded, monotonic pose input. A later bracket must already have arrived."""

    def __init__(self, max_gap_ns, linear_limit, angular_limit):
        self._poses = deque(maxlen=64)
        self._max_gap = max_gap_ns
        self._linear = linear_limit
        self._angular = angular_limit

    def clear(self):
        self._poses.clear()

    def push(self, sample):
        if self._poses and (sample.capture_ns <= self._poses[-1].capture_ns
                            or sample.received_ns < self._poses[-1].received_ns):
            return False
        if self._poses and self._poses[-1].healthy and sample.healthy:
            before = self._poses[-1]
            dt = (sample.capture_ns - before.capture_ns) / NS
            dot = abs(float(np.dot(before.q_wxyz, sample.q_wxyz)))
            angle = 2 * math.acos(min(dot, 1.0))
            if (math.dist(before.position_m, sample.position_m) >
                    self._linear * dt + before.position_error_m + sample.position_error_m
                    or angle > self._angular * dt + before.angle_error_rad + sample.angle_error_rad):
                raise ValueError("implausible pose motion requires an explicit reference reset")
        self._poses.append(sample)
        return True

    def at(self, capture_ns, now_ns):
        poses = [p for p in self._poses if p.received_ns <= now_ns]
        times = [p.capture_ns for p in poses]
        i = bisect_left(times, capture_ns)
        if i < len(poses) and times[i] == capture_ns:
            p = poses[i]
            if not p.healthy:
                return None
            return Alignment(p.position_m, quat_to_mat(*p.q_wxyz), p.position_error_m, p.angle_error_rad,
                             (capture_ns, capture_ns))
        if i == 0 or i == len(poses):
            return None
        before, after = poses[i - 1], poses[i]
        gap = after.capture_ns - before.capture_ns
        if not before.healthy or not after.healthy or gap > self._max_gap:
            return None
        a, b = np.asarray(before.q_wxyz), np.asarray(after.q_wxyz)
        dot = float(a @ b)
        if dot < 0:
            b, dot = -b, -dot
        half_angle = math.acos(min(dot, 1.0))
        dt = gap / NS
        if (math.dist(before.position_m, after.position_m) >
                self._linear * dt + before.position_error_m + after.position_error_m
                or 2 * half_angle > self._angular * dt + before.angle_error_rad + after.angle_error_rad):
            return None
        fraction = (capture_ns - before.capture_ns) / gap
        if half_angle < 1e-6:
            quat = qnormalize((1 - fraction) * a + fraction * b)
        else:
            quat = qnormalize((math.sin((1 - fraction) * half_angle) * a
                               + math.sin(fraction * half_angle) * b) / math.sin(half_angle))
        position = tuple((1 - fraction) * x + fraction * y for x, y in zip(before.position_m, after.position_m))
        # Bound distance to the nearer endpoint for both true and interpolated pose.
        near = min(fraction, 1 - fraction)
        position_error = (max(before.position_error_m, after.position_error_m)
                          + near * (self._linear * dt + math.dist(before.position_m, after.position_m)))
        angle_error = (max(before.angle_error_rad, after.angle_error_rad)
                       + near * (self._angular * dt + 2 * half_angle))
        return Alignment(position, quat_to_mat(*quat), position_error, angle_error,
                         (before.capture_ns, after.capture_ns))
