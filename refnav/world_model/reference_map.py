"""Offline preprocessing and bounded indexing of static reference geometry."""

from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from .reference_ply import MAX_POINTS, read_ply

SCHEMA = 1
MAX_COORDINATE_M = 100_000.


@dataclass(frozen=True)
class ReferenceSpec:
    map_id: str
    frame_id: str
    axes: str
    source_unit: str
    meters_per_unit: float
    scale_evidence: str = ""
    expected_span_m: tuple | None = None

    def __post_init__(self):
        for value in (self.map_id, self.frame_id, self.axes, self.source_unit):
            if not isinstance(value, str) or not value.strip() or len(value) > 256:
                raise ValueError("reference map needs named frame, axes and source units")
        if not math.isfinite(self.meters_per_unit) or not 0 < self.meters_per_unit <= 1000:
            raise ValueError("invalid source-to-metric scale")
        if not isinstance(self.scale_evidence, str) or len(self.scale_evidence) > 512:
            raise ValueError("invalid scale evidence")
        if self.expected_span_m is not None:
            lo, hi = self.expected_span_m
            if not 0 < lo <= hi <= 2 * MAX_COORDINATE_M:
                raise ValueError("invalid expected metric span")
            object.__setattr__(self, "expected_span_m", (lo, hi))

    @property
    def reviewed_scale(self):
        return bool(self.scale_evidence.strip() and self.expected_span_m is not None)


@dataclass(frozen=True)
class PreprocessPolicy:
    voxel_m: float = .05
    max_points: int = 1_000_000
    outlier_method: str = "none"
    neighbors: int = 8
    radius_m: float = .20
    stddev_multiplier: float = 2.5

    def __post_init__(self):
        if (not 0 <= self.voxel_m <= 10 or 0 < self.voxel_m < .001
                or type(self.max_points) is not int or not 1 <= self.max_points <= MAX_POINTS
                or self.outlier_method not in ("none", "radius", "statistical")
                or type(self.neighbors) is not int or not 1 <= self.neighbors <= 32
                or not .001 <= self.radius_m <= 10 or not .1 <= self.stddev_multiplier <= 10):
            raise ValueError("invalid reference preprocessing policy")


def _canonical(data):
    return json.dumps(data, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")


def _digest(data):
    return hashlib.sha256(data).hexdigest()


def _immutable(array, dtype):
    canonical = np.ascontiguousarray(array, dtype=dtype)
    return np.frombuffer(canonical.tobytes(), dtype=dtype).reshape(canonical.shape)


class ReferenceMap:
    """Metric surface samples. The absence of samples carries no occupancy evidence."""

    def __init__(self, xyz, rgb, manifest):
        if (xyz.ndim != 2 or xyz.shape[1:] != (3,) or not 1 <= len(xyz) <= MAX_POINTS
                or not np.isfinite(xyz).all() or np.max(np.abs(xyz)) > MAX_COORDINATE_M
                or (rgb is not None and (rgb.shape != xyz.shape or rgb.dtype != np.uint8))):
            raise ValueError("invalid metric reference arrays")
        self._spec = ReferenceSpec(**manifest["reference"])
        self._points = _immutable(xyz, "<f8")
        self._rgb = None if rgb is None else _immutable(rgb, "u1")
        self._manifest = _canonical(manifest)
        self._version = _digest(self._manifest)
        self._tree = cKDTree(self._points)

    @property
    def spec(self):
        return self._spec

    @property
    def points(self):
        return self._points

    @property
    def rgb(self):
        return self._rgb

    @property
    def version(self):
        return self._version

    @property
    def manifest(self):
        return json.loads(self._manifest)

    def local_points(self, center_m, radius_m, max_points):
        """Return a bounded spherical crop, or None if the crop exceeds its budget."""
        center = np.asarray(center_m, dtype=float)
        if (center.shape != (3,) or not np.isfinite(center).all() or not 0 < radius_m <= 50
                or type(max_points) is not int or not 1 <= max_points <= 100_000):
            raise ValueError("invalid reference crop")
        distance, indices = self._tree.query(center, k=max_points + 1, distance_upper_bound=radius_m)
        if math.isfinite(distance[-1]):
            return None
        return self.points[np.sort(indices[np.isfinite(distance)])]


def _outlier_mask(xyz, policy):
    if policy.neighbors >= len(xyz):
        raise ValueError("outlier filter requires more points than neighbors")
    tree = cKDTree(xyz)
    if policy.outlier_method == "radius":
        count = tree.query_ball_point(xyz, policy.radius_m, return_length=True)
        return count > policy.neighbors
    mean_distance = np.empty(len(xyz))
    for start in range(0, len(xyz), 8192):
        batch = slice(start, start + 8192)
        distances, _ = tree.query(xyz[batch], k=policy.neighbors + 1)
        mean_distance[batch] = distances[:, 1:].mean(axis=1)
    return mean_distance <= mean_distance.mean() + policy.stddev_multiplier * mean_distance.std()


def preprocess(path, spec, policy=PreprocessPolicy()):
    """Filter finite vertices, keep the first RGB on duplicate XYZ, then voxelize."""
    if not isinstance(spec, ReferenceSpec) or not isinstance(policy, PreprocessPolicy):
        raise ValueError("expected a reference specification and preprocessing policy")
    xyz, rgb, fmt, comments, source_hash = read_ply(path, max_points=policy.max_points)
    original = len(xyz)
    valid = np.isfinite(xyz).all(axis=1)
    if not valid.any():
        raise ValueError("reference has no finite XYZ")
    xyz = xyz[valid]
    if np.max(np.abs(xyz)) > MAX_COORDINATE_M / spec.meters_per_unit:
        raise ValueError("reference exceeds metric coordinate bound; check source units")
    xyz *= spec.meters_per_unit
    if rgb is not None:
        rgb = rgb[valid]
    raw_low, raw_high = xyz.min(axis=0), xyz.max(axis=0)
    span = float(np.max(raw_high - raw_low))
    if spec.expected_span_m is not None and not spec.expected_span_m[0] <= span <= spec.expected_span_m[1]:
        raise ValueError("reference span disagrees with reviewed metric dimensions")
    xyz, first = np.unique(xyz, return_index=True, axis=0)
    if rgb is not None:
        rgb = rgb[first]
    unique = len(xyz)
    if policy.voxel_m:
        # Retain a measured surface sample. Centroids could fabricate points at corners.
        _, first = np.unique(np.floor(xyz / policy.voxel_m).astype(np.int64), return_index=True, axis=0)
        first.sort()
        xyz = xyz[first]
        if rgb is not None:
            rgb = rgb[first]
    voxel_count = len(xyz)
    if policy.outlier_method != "none":
        keep = _outlier_mask(xyz, policy)
        if not keep.any():
            raise ValueError("outlier filter removed all reference points")
        xyz = xyz[keep]
        if rgb is not None:
            rgb = rgb[keep]
    xyz = np.ascontiguousarray(xyz, dtype="<f8")
    manifest = {
        "schema": SCHEMA, "reference": asdict(spec), "policy": asdict(policy),
        "source": {"filename": Path(path).name, "sha256": source_hash, "format": fmt, "header": comments},
        "counts": {"original": original, "invalid_xyz": original - int(valid.sum()),
                   "duplicate_xyz": int(valid.sum()) - unique, "after_voxel": voxel_count,
                   "outliers_removed": voxel_count - len(xyz), "retained": len(xyz)},
        "source_bounds_m": [raw_low.tolist(), raw_high.tolist()],
        "bounds_m": [xyz.min(axis=0).tolist(), xyz.max(axis=0).tolist()],
        "xyz_sha256": _digest(memoryview(xyz)), "rgb_sha256": None if rgb is None else _digest(memoryview(rgb)),
    }
    return ReferenceMap(xyz, rgb, manifest)


def save_reference(reference, directory):
    """Write deterministic NumPy arrays and the canonical manifest into a new directory."""
    directory = Path(directory)
    directory.mkdir()
    np.save(directory / "xyz.npy", reference.points, allow_pickle=False)
    if reference.rgb is not None:
        np.save(directory / "rgb.npy", reference.rgb, allow_pickle=False)
    (directory / "manifest.json").write_bytes(_canonical(reference.manifest) + b"\n")


def _load_array(path, dtype, count):
    if path.stat().st_size > count * 3 * dtype.itemsize + 4096:
        raise ValueError("reference array exceeds storage budget")
    data = np.load(path, mmap_mode="r", allow_pickle=False, max_header_size=4096)
    if (not isinstance(data, np.memmap) or data.dtype != dtype
            or data.shape != (count, 3) or not data.flags.c_contiguous):
        raise ValueError("invalid reference array layout")
    return data


def load_reference(directory):
    """Rebuild the index after validating the versioned manifest and array digests."""
    directory = Path(directory)
    with (directory / "manifest.json").open("rb") as stream:
        raw = stream.read(256 * 1024 + 1)
    if len(raw) > 256 * 1024:
        raise ValueError("reference manifest exceeds storage budget")
    try:
        manifest = json.loads(raw)
        count = manifest["counts"]["retained"]
        if manifest["schema"] != SCHEMA or type(count) is not int or not 1 <= count <= MAX_POINTS:
            raise ValueError("unsupported reference schema or point budget")
        policy = PreprocessPolicy(**manifest["policy"])
        if count > policy.max_points:
            raise ValueError("reference exceeds preprocessing point budget")
        xyz = _load_array(directory / "xyz.npy", np.dtype("<f8"), count)
        rgb_hash = manifest["rgb_sha256"]
        rgb = None if rgb_hash is None else _load_array(directory / "rgb.npy", np.dtype("u1"), count)
        if (_digest(memoryview(xyz)) != manifest["xyz_sha256"]
                or (rgb is not None and _digest(memoryview(rgb)) != rgb_hash)):
            raise ValueError("reference array digest mismatch")
        return ReferenceMap(xyz, rgb, manifest)
    except (TypeError, KeyError, OverflowError) as error:
        raise ValueError("invalid reference manifest") from error
