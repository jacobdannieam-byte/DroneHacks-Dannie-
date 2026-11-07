"""Deterministic vertical-plane / bounded horizontal gap hypotheses, NumPy only.

No course coordinates enter inference. Height and topological connectivity of
openings are deliberately unconfirmed in this side-boundary prototype.
"""
from __future__ import annotations

import numpy as np

from .observations import Detection, GeometryResult, project


def window_evidence(rays, center, normal, width, band, margin=0.25):
    """Ray evidence at an entry cross-section (including no-return rays in denominator)."""
    center, normal = np.asarray(center), np.asarray(normal)
    tangent = np.array([-normal[1], normal[0], 0.])
    denom = rays.directions @ normal
    numer = float((center - rays.origin) @ normal)
    t = np.divide(numer, denom, out=np.full_like(denom, -1.), where=np.abs(denom) > 1e-8)
    crossing = rays.origin + rays.directions * t[:, None]
    selected = ((t > 0) & (np.abs((crossing - center) @ tangent) < width / 2)
                & (np.abs(crossing[:, 2] - center[2]) < band))
    n = int(selected.sum())
    # margin is a physical distance normal to the plane, not optical z.
    separation = (rays.depths - t) * np.abs(denom)
    behind = selected & rays.valid & (separation > margin)
    blocked = selected & rays.valid & (separation < 0.07)
    return {'rays': n, 'observed_fraction': float((selected & rays.valid).sum() / n) if n else 0.,
            'behind_fraction': float(behind.sum() / n) if n else 0.,
            'blocked_fraction': float(blocked.sum() / n) if n else 0.}


def _planes(points, config):
    """Gravity-constrained RANSAC, fixed seed and capped trials/planes/points."""
    remaining = points.copy()
    rng = np.random.default_rng(7)
    for _ in range(config.max_planes):
        if len(remaining) < config.min_plane_hits:
            break
        pairs = rng.integers(0, len(remaining), size=(config.plane_trials, 2))
        a, b = remaining[pairs[:, 0], :2], remaining[pairs[:, 1], :2]
        diff = b - a
        norms = np.linalg.norm(diff, axis=1)
        normals = np.column_stack((-diff[:, 1], diff[:, 0])) / np.maximum(norms[:, None], 1e-9)
        offsets = np.sum(normals * a, axis=1)
        distances = np.abs(remaining[:, :2] @ normals.T - offsets)
        counts = (distances < config.plane_tol_m).sum(axis=0)
        counts[norms < config.min_wall_width_m] = 0
        best = int(counts.argmax())
        if counts[best] < config.min_plane_hits:
            break
        mask = distances[:, best] < config.plane_tol_m
        inliers = remaining[mask]
        xy_mean = inliers[:, :2].mean(axis=0)
        _, _, vt = np.linalg.svd(inliers[:, :2] - xy_mean, full_matrices=False)
        n = np.r_[vt[-1], 0.]
        if n[np.argmax(np.abs(n))] < 0:
            n = -n
        offset = float(n[:2] @ xy_mean)
        mask = np.abs(remaining @ n - offset) < config.plane_tol_m
        inliers = remaining[mask]
        remaining = remaining[~mask]
        if len(inliers) >= config.min_plane_hits and np.ptp(inliers[:, 2]) >= config.min_wall_height_m:
            yield n, offset, inliers


def _runs(mask):
    padded = np.r_[False, mask, False].astype(int)
    return list(zip(np.flatnonzero(np.diff(padded) == 1), np.flatnonzero(np.diff(padded) == -1)))


def _corridors(walls, obs, rays, config):
    """Parallel side surfaces around the observer plus observed rays along their overlap.

    This is a corridor-like hypothesis, never a claim about full free volume or topology.
    """
    forward = np.asarray(obs.R_world_body)[:, 0].copy()
    forward[2] = 0.
    if np.linalg.norm(forward) < 0.5:
        return []
    forward /= np.linalg.norm(forward)
    candidates = []
    for i, left in enumerate(walls):
        normal = np.array(left.normal)
        if abs(normal @ forward) > .2:
            continue
        tangent = np.array([-normal[1], normal[0], 0.])
        a = float(np.dot(left.center, normal))
        for right in walls[i + 1:]:
            if abs(normal @ right.normal) < .98:
                continue
            b = float(np.dot(right.center, normal))
            origin_across = float(rays.origin @ normal)
            if not min(a,b) < origin_across < max(a,b):
                continue
            width = abs(b - a)
            if not config.min_opening_m < width < config.max_opening_m:
                continue
            ac, bc = float(np.dot(left.center, tangent)), float(np.dot(right.center, tangent))
            lo = max(ac - left.width_m / 2, bc - right.width_m / 2)
            hi = min(ac + left.width_m / 2, bc + right.width_m / 2)
            if hi - lo < 0.75:
                continue
            center = normal * ((a + b) / 2) + tangent * ((lo + hi) / 2)
            center[2] = rays.origin[2]
            if (center - rays.origin) @ forward <= 0:
                continue
            evidence = window_evidence(rays, center, forward, width - 2 * config.bin_m,
                                       config.band_m, config.behind_m)
            if evidence['rays'] < 12 or evidence['behind_fraction'] < config.min_coverage:
                continue
            candidates.append(Detection('corridor', center.tolist(), forward.tolist(), width, None,
                                        max(left.uncertainty_m, right.uncertainty_m), .65, .5,
                                        ['corridor_extent', 'full_volume_coverage', 'region_connectivity'],
                                        {**evidence, 'observed_length_m': hi - lo}))
    return candidates


def extract(obs, config):
    rays = project(obs, config)
    points = rays.points()
    detections = []
    rejected = 0
    for normal, offset, hits in _planes(points, config):
        tangent = np.array([-normal[1], normal[0], 0.])
        # Analyze side boundaries near the sensor's height, not gaps caused by floor returns.
        zc = float(rays.origin[2])
        band_hits = hits[np.abs(hits[:, 2] - zc) < config.band_m]
        if len(band_hits) < config.min_plane_hits:
            continue
        across = band_hits @ tangent
        start = np.floor(across.min() / config.bin_m) * config.bin_m
        bins = np.floor((across - start) / config.bin_m).astype(int)
        count = np.bincount(bins)
        runs = _runs(count >= 2)
        segments = []
        for lo, hi in runs:
            select = (bins >= lo) & (bins < hi)
            section = band_hits[select]
            # Observed surface extents only; these are not full physical wall dimensions.
            low, high = float((section @ tangent).min()), float((section @ tangent).max())
            if high - low < config.min_wall_width_m:
                continue
            all_section = hits[((hits @ tangent) >= low) & ((hits @ tangent) <= high)]
            if np.ptp(all_section[:, 2]) < config.min_wall_height_m:
                continue
            center = normal * offset + tangent * ((low + high) / 2)
            center[2] = zc
            error = obs.position_sigma_m + np.linalg.norm(center - rays.origin) * obs.angle_sigma_rad + config.bin_m
            detections.append(Detection('wall', center.tolist(), normal.tolist(), high - low,
                                        float(np.ptp(all_section[:, 2])), error, 0.8, 0.7,
                                        ['full_surface_extent'], {'points': len(all_section)}))
            segments.append((low, high))
        for left, right in zip(segments, segments[1:]):
            width = right[0] - left[1]
            if not config.min_opening_m <= width <= config.max_opening_m:
                continue
            center = normal * offset + tangent * ((left[1] + right[0]) / 2)
            center[2] = zc
            # Exclude uncertain edge pixels. Finite returns behind must span the interior.
            evidence = window_evidence(rays, center, normal, max(0.05, width - 2 * config.bin_m),
                                       config.band_m, config.behind_m)
            # Check subwindows too: a concentrated patch of valid pixels is insufficient.
            supported = evidence['rays'] >= 12 and evidence['behind_fraction'] >= config.min_coverage
            for sign in (-1, 1):
                tile_center = center + tangent * sign * width / 4
                tile = window_evidence(rays, tile_center, normal, width / 3,
                                       config.band_m, config.behind_m)
                supported &= tile['rays'] >= 4 and tile['behind_fraction'] >= config.min_coverage
            if not supported or evidence['blocked_fraction'] > 0:
                rejected += 1
                continue
            error = obs.position_sigma_m + np.linalg.norm(center - rays.origin) * obs.angle_sigma_rad + config.bin_m
            detections.append(Detection('opening', center.tolist(), normal.tolist(), float(width), None,
                                        error, 0.75, 0.6,
                                        ['upper_boundary', 'lower_boundary', 'full_volume_coverage',
                                         'region_connectivity'], evidence))
    detections += _corridors([d for d in detections if d.semantic_type == 'wall'], obs, rays, config)
    return GeometryResult(detections, rays, {'valid_rays': int(rays.valid.sum()),
                          'sampled_rays': len(rays.valid), 'rejected_gaps': rejected})
