"""Local, bounded point-to-point ICP with conservative geometric quality gates."""

from dataclasses import dataclass, replace
import math

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

from .observations import rotation_ok
from .spatial_input import vector


@dataclass(frozen=True)
class SE3:
    """Rigid target_from_source transform, with translation in the target frame."""

    rotation: tuple
    translation_m: tuple

    def __post_init__(self):
        if not rotation_ok(self.rotation):
            raise ValueError("SE(3) requires a proper, right-handed rotation")
        object.__setattr__(self, "rotation", tuple(vector(row, 3) for row in self.rotation))
        object.__setattr__(self, "translation_m", vector(self.translation_m, 3))

    def apply(self, points):
        return np.asarray(points) @ np.asarray(self.rotation).T + self.translation_m

    def compose(self, source):
        return SE3(np.asarray(self.rotation) @ source.rotation, self.apply(source.translation_m))

    def inverse(self):
        rotation = np.asarray(self.rotation).T
        return SE3(rotation, -rotation @ self.translation_m)


@dataclass(frozen=True)
class RegistrationPolicy:
    max_source_points: int = 1200
    max_crop_points: int = 30_000
    max_iterations: int = 20
    max_translation_m: float = .45
    max_rotation_rad: float = .16
    correspondence_m: float = .20
    min_inliers: int = 45
    min_overlap: float = .62
    min_spatial_coverage: float = .60
    max_rmse_m: float = .055
    min_observability: float = .018

    def __post_init__(self):
        integer_limits = ((self.max_source_points, 64, 4000), (self.max_crop_points, 64, 100_000),
                          (self.max_iterations, 1, 40), (self.min_inliers, 16, self.max_source_points))
        real_limits = ((self.max_translation_m, .01, 2), (self.max_rotation_rad, .01, .5),
                       (self.correspondence_m, .01, .5), (self.max_rmse_m, .001, self.correspondence_m),
                       (self.min_overlap, .5, 1), (self.min_spatial_coverage, .5, 1),
                       (self.min_observability, .001, .3))
        if (not all(type(value) is int and lo <= value <= hi for value, lo, hi in integer_limits)
                or not all(lo <= value <= hi for value, lo, hi in real_limits)):
            raise ValueError("invalid bounded registration policy")


@dataclass(frozen=True)
class RegistrationQuality:
    reason: str
    source_points: int
    crop_points: int = 0
    inliers: int = 0
    distinct_matches: int = 0
    overlap: float = 0.
    rmse_m: float | None = None
    spatial_coverage: float = 0.
    observability: float | None = None
    normal_support: float = 0.
    converged: bool = False
    competing_solutions: int = 0


@dataclass(frozen=True)
class RegistrationResult:
    reference_from_body: SE3 | None
    quality: RegistrationQuality


def _angle(rotation):
    return math.acos(float(np.clip((np.trace(rotation) - 1) / 2, -1, 1)))


def _matches(source, tree, rotation, offset, limit):
    transformed = source @ rotation.T + offset
    distances, indices = tree.query(transformed, distance_upper_bound=limit)
    matched = np.isfinite(distances)
    return transformed, matched, indices[matched], distances[matched]


def _fit(source, tree, initial_rotation, initial_position, seed, policy):
    # All rotations act about the source body origin, including seed perturbations.
    rotation, offset = seed
    converged = False
    for _ in range(policy.max_iterations):
        _, matched, indices, _ = _matches(source, tree, rotation, offset, policy.correspondence_m)
        if len(indices) < policy.min_inliers:
            break
        a, b = source[matched], tree.data[indices]
        center_a, center_b = a.mean(axis=0), b.mean(axis=0)
        u, _, vt = np.linalg.svd((a - center_a).T @ (b - center_b))
        vt[-1] *= np.linalg.det(vt.T @ u.T)
        next_rotation = vt.T @ u.T
        next_offset = center_b - next_rotation @ center_a
        if (np.linalg.norm(next_offset - initial_position) > policy.max_translation_m
                or _angle(next_rotation @ initial_rotation.T) > policy.max_rotation_rad):
            break
        converged = (np.linalg.norm(next_offset - offset) < .0005
                     and _angle(next_rotation @ rotation.T) < .0005)
        rotation, offset = next_rotation, next_offset
        if converged:
            break
    return rotation, offset, converged


def _seeds(rotation, offset, policy):
    yield rotation, offset
    for axis in np.eye(3):
        for sign in (-.6, .6):
            yield rotation, offset + axis * sign * policy.max_translation_m
            yield Rotation.from_rotvec(axis * sign * policy.max_rotation_rad).as_matrix() @ rotation, offset


def _surface_information(tree, indices, matched_points):
    # Inspect local tangent planes so a regular sample grid cannot make sliding
    # along a wall appear constrained. Discard neighborhoods with unstable normals.
    unique, first = np.unique(indices, return_index=True)
    if len(unique) < 16:
        return 0., 0.
    distances, neighbors = tree.query(tree.data[unique], k=9)
    samples = tree.data[neighbors]
    centered = samples - samples.mean(axis=1, keepdims=True)
    eigenvalues, axes = np.linalg.eigh(centered.transpose(0, 2, 1) @ centered)
    planar = ((eigenvalues[:, 1] > 1e-9) & (eigenvalues[:, 0] < .015 * eigenvalues[:, 1])
              & (eigenvalues[:, 1] > .08 * eigenvalues[:, 2]) & (distances[:, -1] <= .35))
    support = float(planar.mean())
    if planar.sum() < 16:
        return 0., support
    normals = axes[planar, :, 0]
    points = matched_points[first[planar]]
    points = points - points.mean(axis=0)
    scale = max(.1, float(np.sqrt(np.mean(np.sum(points**2, axis=1)))))
    jacobian = np.column_stack((normals, np.cross(points / scale, normals)))
    singular = np.linalg.svd(jacobian, compute_uv=False)
    return float(singular[-1] / singular[0]), support


def _quality(matched, indices, distances, bins, total_bins, crop_count, converged, policy):
    count = len(indices)
    unique = len(np.unique(indices))
    overlap = count / len(matched)
    rmse = float(np.sqrt(np.mean(distances**2))) if count else None
    coverage = len(np.unique(bins[matched])) / total_bins
    reason = "candidate"
    if count < policy.min_inliers or unique < max(16, count // 4):
        reason = "insufficient_distinct_matches"
    elif overlap < policy.min_overlap or coverage < policy.min_spatial_coverage:
        reason = "insufficient_overlap_or_coverage"
    elif rmse > policy.max_rmse_m:
        reason = "high_residual"
    elif not converged:
        reason = "not_converged_within_bounds"
    return RegistrationQuality(reason, len(matched), crop_count, count, unique, overlap, rmse, coverage,
                               converged=converged)


def register(reference, source_body_m, initial_reference_from_body, policy=RegistrationPolicy()):
    """Solve one keyframe. Return a pose only after every gate passes."""
    source = np.asarray(source_body_m, dtype=float)
    if (source.ndim != 2 or source.shape[1:] != (3,) or len(source) > policy.max_source_points
            or not np.isfinite(source).all()):
        raise ValueError("invalid or over-budget live point cloud")
    if len(source) < policy.min_inliers:
        return RegistrationResult(None, RegistrationQuality("insufficient_source", len(source)))
    if not reference.spec.reviewed_scale:
        return RegistrationResult(None, RegistrationQuality("unreviewed_reference_scale", len(source)))
    radius = float(np.linalg.norm(source, axis=1).max()) + policy.max_translation_m + policy.correspondence_m
    if radius > 50:
        return RegistrationResult(None, RegistrationQuality("depth_outside_crop_bounds", len(source)))
    initial = initial_reference_from_body
    rotation0, offset0 = np.array(initial.rotation), np.array(initial.translation_m)
    crop = reference.local_points(offset0, radius, policy.max_crop_points)
    if crop is None:
        return RegistrationResult(None, RegistrationQuality("crop_budget_exceeded", len(source)))
    if len(crop) < policy.min_inliers:
        return RegistrationResult(None, RegistrationQuality("insufficient_local_map", len(source), len(crop)))
    tree = cKDTree(crop)
    unique_bins, bins = np.unique(np.floor(source / .3).astype(np.int64), axis=0, return_inverse=True)
    candidates = []
    for seed in _seeds(rotation0, offset0, policy):
        rotation, offset, converged = _fit(source, tree, rotation0, offset0, seed, policy)
        points, matched, indices, distances = _matches(source, tree, rotation, offset, policy.correspondence_m)
        quality = _quality(matched, indices, distances, bins, len(unique_bins), len(crop), converged, policy)
        score = (quality.overlap - (quality.rmse_m or 0.) / (4 * policy.correspondence_m))
        candidates.append((score, rotation, offset, quality, points[matched], indices))
    candidates.sort(key=lambda c: (c[3].reason == "candidate", c[0]), reverse=True)
    _, rotation, offset, quality, points, indices = candidates[0]
    if quality.reason != "candidate":
        return RegistrationResult(None, quality)
    observability, normal_support = _surface_information(tree, indices, points)
    quality = replace(quality, observability=observability, normal_support=normal_support)
    if normal_support < .45 or observability < policy.min_observability:
        return RegistrationResult(None, replace(quality, reason="degenerate_geometry"))
    competitors = sum(
        other.reason == "candidate"
        and other.overlap >= quality.overlap - .06
        and other.rmse_m <= quality.rmse_m + .012
        and (np.linalg.norm(other_offset - offset) > .08 or _angle(other_rotation @ rotation.T) > .05)
        for _, other_rotation, other_offset, other, _, _ in candidates[1:]
    )
    if competitors:
        return RegistrationResult(None, replace(quality, reason="ambiguous_alignment",
                                                competing_solutions=competitors))
    return RegistrationResult(SE3(rotation, offset), replace(quality, reason="accepted"))
