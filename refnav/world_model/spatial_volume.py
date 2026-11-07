"""Bounded, conservative 3D evidence and a non-commanding swept-volume query."""

from dataclasses import dataclass
from enum import IntEnum
import math

import numpy as np

from .spatial_input import MAX_LOCAL_M, SpatialReference, stamp_ok

TILE = 8


class Cell(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


@dataclass(frozen=True)
class MapPolicy:
    resolution_m: float = .20
    shape: tuple = (40, 40, 24)
    free_age_ns: int = 500_000_000
    occupied_age_ns: int = 2_000_000_000
    min_depth_m: float = .30
    max_depth_m: float = 6.0

    def __post_init__(self):
        if (not .05 <= self.resolution_m <= 1 or not isinstance(self.shape, tuple)
                or len(self.shape) != 3 or not all(type(v) is int and 4 <= v <= 96 for v in self.shape)
                or math.prod(self.shape) > 65536):
            raise ValueError("invalid voxel resolution or cell budget")
        if (not stamp_ok(self.free_age_ns) or not stamp_ok(self.occupied_age_ns)
                or not 0 < self.free_age_ns <= self.occupied_age_ns <= 10_000_000_000
                or not .1 <= self.min_depth_m < self.max_depth_m <= 10):
            raise ValueError("invalid voxel freshness or depth range")


@dataclass(frozen=True)
class SweepResult:
    status: str
    reason: str
    occupied_cells: int = 0
    unknown_cells: int = 0
    outside_map: bool = False


@dataclass(frozen=True, eq=False)
class MetricSnapshot:
    reference: SpatialReference
    policy: MapPolicy
    origin_index: tuple
    query_ns: int
    capture_ns: int | None
    observation_id: str | None
    calibration_id: str
    available: bool
    _cells: np.ndarray
    _stamps: np.ndarray

    def cells_at(self, now_ns, reference):
        """Detached voxel classes. Missing, invalid and expired evidence is unknown."""
        if (reference != self.reference or not stamp_ok(now_ns) or now_ns < self.query_ns
                or not self.available):
            return np.zeros(self.policy.shape, dtype=np.uint8)
        age = now_ns - self._stamps
        fresh = ((self._cells == Cell.FREE) & (age <= self.policy.free_age_ns)
                 | (self._cells == Cell.OCCUPIED) & (age <= self.policy.occupied_age_ns))
        return np.where(fresh, self._cells, Cell.UNKNOWN).astype(np.uint8, copy=False)

    def check_sweep(self, points_m, *, reference, now_ns, vehicle_radius_m, margin_m):
        """Test a continuous polyline with an explicitly enclosing body sphere."""
        points = np.asarray(points_m, dtype=float)
        if (points.ndim != 2 or points.shape[1:] != (3,) or not 1 <= len(points) <= 64
                or not np.isfinite(points).all() or np.max(np.abs(points)) > MAX_LOCAL_M):
            raise ValueError("sweep needs 1 to 64 finite map-frame points")
        if (not math.isfinite(vehicle_radius_m) or not math.isfinite(margin_m)
                or not 0 < vehicle_radius_m <= 2 or not 0 <= margin_m <= 2):
            raise ValueError("explicit vehicle and margin bounds required")
        if reference != self.reference or not stamp_ok(now_ns) or now_ns < self.query_ns:
            return SweepResult("unknown", "reference_or_time_mismatch")
        if not self.available:
            return SweepResult("unknown", "no_valid_observation")

        cells = self.cells_at(now_ns, reference)
        seen = np.zeros(cells.shape, dtype=bool)
        radius = vehicle_radius_m + margin_m
        segments = zip(points[:-1], points[1:]) if len(points) > 1 else [(points[0], points[0])]
        outside = False
        for a, b in segments:
            # Each expanded segment box is a superset of the continuous swept sphere.
            lo = np.floor(np.nextafter((np.minimum(a, b) - radius) / self.policy.resolution_m, -np.inf))
            hi = np.floor(np.nextafter((np.maximum(a, b) + radius) / self.policy.resolution_m, np.inf)) + 1
            lo = lo.astype(int) - self.origin_index
            hi = hi.astype(int) - self.origin_index
            outside |= bool(np.any(lo < 0) or np.any(hi > self.policy.shape))
            lower = np.maximum(lo, 0)
            upper = np.minimum(hi, self.policy.shape)
            if np.all(lower < upper):
                seen[tuple(slice(a, b) for a, b in zip(lower, upper))] = True
        occupied = int(np.count_nonzero(seen & (cells == Cell.OCCUPIED)))
        unknown = int(np.count_nonzero(seen & (cells == Cell.UNKNOWN)))
        if occupied:
            return SweepResult("occupied", "occupied_voxel", occupied, unknown, outside)
        if unknown or outside:
            return SweepResult("unknown", "incomplete_fresh_volume", occupied, unknown, outside)
        return SweepResult("verified_free", "observed_swept_volume", occupied, unknown, outside)


def _tile_tables(depth, policy):
    """Power-of-two rectangle minima on a small conservative pixel tile grid."""
    h, w = depth.shape
    padded = np.full((math.ceil(h / TILE) * TILE, math.ceil(w / TILE) * TILE), np.nan, np.float32)
    padded[:h, :w] = depth
    valid = np.isfinite(padded) & (padded > policy.min_depth_m) & (padded < policy.max_depth_m)
    values = np.stack((np.where(valid, padded, -np.inf), np.where(valid, padded, np.inf),
                       np.where(valid, -padded, np.inf)), axis=-1)
    tiles = values.reshape(padded.shape[0] // TILE, TILE, padded.shape[1] // TILE, TILE, 3).min(axis=(1, 3))
    tables = {(0, 0): tiles}
    for y in range(tiles.shape[0].bit_length()):
        if y:
            prev = tables[y - 1, 0]
            offset = 1 << (y - 1)
            tables[y, 0] = np.minimum(prev[:-offset], prev[offset:])
        for x in range(1, tiles.shape[1].bit_length()):
            prev = tables[y, x - 1]
            offset = 1 << (x - 1)
            tables[y, x] = np.minimum(prev[:, :-offset], prev[:, offset:])
    return tables


def _rectangle_ranges(tables, x0, y0, x1, y1):
    powers = np.floor(np.log2(np.column_stack((y1 - y0 + 1, x1 - x0 + 1)))).astype(int)
    result = np.empty((len(x0), 3), np.float32)
    for y, x in np.unique(powers, axis=0):
        mask = (powers[:, 0] == y) & (powers[:, 1] == x)
        table = tables[y, x]
        a, b = y0[mask], y1[mask] - (1 << y) + 1
        c, d = x0[mask], x1[mask] - (1 << x) + 1
        result[mask] = np.minimum(np.minimum(table[a, c], table[a, d]),
                                  np.minimum(table[b, c], table[b, d]))
    return result


def classify(depth, calibration, pose, policy, indices):
    """Return per-voxel current evidence and the set affected by this view."""
    resolution = policy.resolution_m
    centers = (indices + .5) * resolution
    world_optical = pose.rotation @ np.asarray(calibration.R_body_optical)
    body_offset = np.asarray(calibration.t_body_optical_m)
    origin = np.asarray(pose.position_m) + pose.rotation @ body_offset
    camera = (centers - origin) @ world_optical
    distance = np.linalg.norm(centers - pose.position_m, axis=1) + math.sqrt(3) * resolution / 2
    extrinsic_angle = calibration.extrinsic_angle_error_rad or 0.
    uncertainty = (pose.position_error_m + 2 * math.sin(pose.angle_error_rad / 2) * distance
                   + (calibration.extrinsic_position_error_m or 0.)
                   + 2 * math.sin(extrinsic_angle / 2) * (distance + math.hypot(*body_offset)))
    half = (resolution / 2 + uncertainty[:, None]) * np.abs(world_optical).sum(axis=0)
    lo, hi = camera - half, camera + half
    near, far = lo[:, 2], hi[:, 2]
    result = np.zeros(len(indices), dtype=np.uint8)
    # Boxes straddling the optical plane cannot safely use a projected rectangle.
    touched = (near <= 0) & (far > 0)
    forward = np.flatnonzero((near > 0) & (far > policy.min_depth_m) & (near < policy.max_depth_m))
    if not len(forward):
        return result, touched

    fx, fy, cx, cy = calibration.K
    h, w = depth.shape
    # All extrema of x/z and y/z on the positive-z box occur at its corners.
    projected = np.stack((lo[forward, :2] / near[forward, None], lo[forward, :2] / far[forward, None],
                          hi[forward, :2] / near[forward, None], hi[forward, :2] / far[forward, None]))
    low = projected.min(axis=0) * (fx, fy) + (cx, cy)
    high = projected.max(axis=0) * (fx, fy) + (cx, cy)
    # One-pixel guard and outward tile rounding only add required observations.
    low -= 1.0
    high += 1.0
    intersects = np.all(high >= -.5, axis=1) & np.all(low <= (w - .5, h - .5), axis=1)
    forward, low, high = forward[intersects], low[intersects], high[intersects]
    if not len(forward):
        return result, touched
    touched[forward] = True
    entire = np.all(low >= -.5, axis=1) & np.all(high <= (w - .5, h - .5), axis=1)
    x0, y0 = (np.floor(np.clip(low + .5, 0, (w - 1, h - 1))).astype(int) // TILE).T
    x1, y1 = (np.floor(np.clip(high + .5, 0, (w - 1, h - 1))).astype(int) // TILE).T
    free_min, hit_min, neg_hit_max = _rectangle_ranges(_tile_tables(depth, policy), x0, y0, x1, y1).T
    error = calibration.depth_error_m
    free = (calibration.validated_pixel_footprint & entire
            & (near[forward] > policy.min_depth_m) & (far[forward] < policy.max_depth_m)
            & (free_min - error > far[forward]))
    # The min/max interval is a conservative occupied overestimate at depth discontinuities.
    occupied = (hit_min - error <= far[forward]) & (-neg_hit_max + error >= near[forward])
    result[forward[free]] = Cell.FREE
    result[forward[occupied]] = Cell.OCCUPIED
    return result, touched


class RollingVolume:
    def __init__(self, policy):
        self.policy = policy
        self._indices = np.indices(policy.shape, dtype=np.int32).reshape(3, -1).T
        self.origin = np.zeros(3, dtype=int)
        self.cells = np.zeros(policy.shape, dtype=np.uint8)
        self.stamps = np.zeros(policy.shape, dtype=np.int64)

    def clear(self):
        self.cells.fill(Cell.UNKNOWN)
        self.stamps.fill(0)

    def prepare(self, depth, calibration, pose):
        origin = np.floor(np.asarray(pose.position_m) / self.policy.resolution_m).astype(int) - np.asarray(self.policy.shape) // 2
        classified, touched = classify(depth, calibration, pose, self.policy, self._indices + origin)
        return origin, classified.reshape(self.policy.shape), touched.reshape(self.policy.shape)

    def apply(self, prepared, stamp_ns):
        origin, classified, touched = prepared
        delta = origin - self.origin
        if np.any(delta):
            cells, stamps = np.zeros_like(self.cells), np.zeros_like(self.stamps)
            low, high = np.maximum(delta, 0), np.minimum(np.asarray(self.policy.shape) + delta, self.policy.shape)
            if np.all(low < high):
                old = tuple(slice(a, b) for a, b in zip(low, high))
                new = tuple(slice(a - d, b - d) for a, b, d in zip(low, high, delta))
                cells[new], stamps[new] = self.cells[old], self.stamps[old]
            self.origin, self.cells, self.stamps = origin, cells, stamps

        # A fresh obstacle wins over a later apparent free return. Once expired it
        # needs new evidence. A foreground/invalid return revokes visible old free.
        occupied = (self.cells == Cell.OCCUPIED) & (stamp_ns - self.stamps <= self.policy.occupied_age_ns)
        update = touched & (~occupied | (classified == Cell.OCCUPIED))
        self.cells[update] = classified[update]
        self.stamps[update] = stamp_ns

    def snapshot(self, reference, now_ns, capture_ns, observation_id, calibration_id, available):
        arrays = [np.frombuffer(a.tobytes(), dtype=a.dtype).reshape(a.shape) for a in (self.cells, self.stamps)]
        return MetricSnapshot(reference, self.policy, tuple(map(int, self.origin)), now_ns, capture_ns,
                              observation_id, calibration_id, available, *arrays)
