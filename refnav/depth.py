"""Clearances from one depth frame. PURE (numpy only).

The frame arrives with its own calibration: the pinhole intrinsics from CameraInfo and the
camera's pose on the body from TF (base_link <- camera optical frame, REP-103: optical x right,
y down, z forward; body x forward, y left, z up). The vehicle attitude levels the points, so
"ahead" and "at flight height" mean the same thing while the airframe is tilted.

four_clearances() reports ahead / left / right / low from one frame. Each is None (UNKNOWN: not
in view, no depth returned, or occluded - never permission to move), inf (OBSERVED clear at the
judged cross-section) or a distance in metres. See Probe for what each value claims.
"""
from __future__ import annotations

import math
from typing import NamedTuple

import numpy as np

# body FLU -> body FRD (PX4's body frame)
_FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_mat(w, x, y, z):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def level_rotation(q_wxyz):
    """Rotation taking body-FLU vectors into the LEVEL heading frame (x forward horizontal,
    y right horizontal, z down), i.e. PX4's attitude with its yaw removed. q = PX4's
    vehicle_attitude quaternion (FRD body -> NED), w first."""
    R = quat_to_mat(*q_wxyz)                                   # NED <- FRD
    yaw = math.atan2(R[1, 0], R[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    Rz_inv = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    return Rz_inv @ R @ _FLU_TO_FRD


def decode_depth(encoding, height, width, step, data):
    """sensor_msgs/Image depth -> float32 metres (NaN = no return). 32FC1 is metres,
    16UC1 is millimetres (0 and 65535 = invalid: 65535 is the saturated / no-data code, not a
    65.5 m return); anything else raises ValueError."""
    enc = encoding.lower()
    if enc == "32fc1":
        a = np.frombuffer(data, np.float32).reshape(height, step // 4)[:, :width].astype(np.float32)
        a = a.copy()
        a[~np.isfinite(a) | (a <= 0.0)] = np.nan
        return a
    if enc in ("16uc1", "mono16"):
        raw = np.frombuffer(data, np.uint16).reshape(height, step // 2)[:, :width]
        a = raw.astype(np.float32) * 1e-3
        a[(raw == 0) | (raw == 0xFFFF)] = np.nan
        return a
    raise ValueError(f"unsupported depth encoding {encoding!r} (need 32FC1 metres or 16UC1 millimetres)")


class DepthFrame:
    """One depth frame, sampled every `stride` pixels inside a `border_px` margin, in the LEVEL
    heading frame (forward, right, down) relative to the body origin. Keeps, per sampled pixel, its
    ray and whether the camera returned ANY depth for it - so the frame can say what it observed
    (a return, near or far) apart from what it did not (no return: unknown, never free space)."""

    def __init__(self, depth_m, K, R_body_cam, t_body_cam, R_level_body, stride=8, border_px=0):
        fx, fy, cx, cy = K
        d = np.asarray(depth_m, np.float32)
        h, w = d.shape
        b = int(border_px)
        vs, us = np.mgrid[b:h - b:stride, b:w - b:stride] if b > 0 else np.mgrid[0:h:stride, 0:w:stride]
        z = d[vs, us].ravel()
        u, v = us.ravel().astype(np.float32), vs.ravel().astype(np.float32)
        rays_cam = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=1)    # optical z = 1
        R = np.asarray(R_level_body) @ np.asarray(R_body_cam)
        self.dirs = rays_cam @ R.T                     # a point at optical depth z is origin + z * dir
        self.origin = np.asarray(R_level_body) @ np.asarray(t_body_cam, float)
        self.z = z
        self.valid = np.isfinite(z) & (z > 0.0)
        self.valid_frac = float(self.valid.mean()) if z.size else 0.0
        # geometry of the sampled grid, for window_in_view (what the image can see, whatever it returned)
        self.K = (float(fx), float(fy), float(cx), float(cy))
        self.R_level_cam = R
        self.u_rng = (float(us.min()), float(us.max())) if us.size else None
        self.v_rng = (float(vs.min()), float(vs.max())) if vs.size else None

    def points(self, rmin=0.3, rmax=6.0):
        sel = self.valid & (self.z > rmin) & (self.z < rmax)
        return self.origin + self.dirs[sel] * self.z[sel, None]

    def corridor_coverage(self, bearing_rad, half_width_m, band_m, at_m, center_down_m=0.0, reach=False):
        """Fraction of the rays that cross the corridor's cross-section at at_m ahead (along the
        bearing, within +-half_width_m and +-band_m of center_down_m below the body) that returned
        depth. 0.0 when no ray of this camera crosses it (the corridor is out of view).
        reach=True counts a ray only when its return lies at or beyond the cross-section (it was
        not stopped short by something nearer, nor returned nearer than points() keeps).
        CAUTION: this is a fraction of the CROSSING rays, not of the window - a window mostly
        outside the image can still score 1.0. Pair it with window_in_view()."""
        c, s = math.cos(bearing_rad), math.sin(bearing_rad)
        along_dir = self.dirs[:, 0] * c + self.dirs[:, 1] * s
        ok = along_dir > 1e-6
        t = np.where(ok, (at_m - (self.origin[0] * c + self.origin[1] * s)) / np.where(ok, along_dir, 1.0), -1.0)
        p = self.origin + self.dirs * t[:, None]
        across = -p[:, 0] * s + p[:, 1] * c
        window = ok & (t > 0) & (np.abs(across) < half_width_m) & (np.abs(p[:, 2] - center_down_m) < band_m)
        n = int(window.sum())
        if not n:
            return 0.0, 0
        good = (self.valid & (self.z >= t)) if reach else self.valid
        return float(good[window].mean()), n

    def window_in_view(self, bearing_rad, half_width_m, band_m, at_m, center_down_m=0.0, n_across=13, n_down=9):
        """Fraction of an n_across x n_down grid of points on the corridor's cross-section at at_m
        that projects inside the sampled image (border excluded) in front of the camera. Geometry
        only: it says what the camera COULD see there, not what it returned. at_m may be an array."""
        scalar = np.ndim(at_m) == 0
        if self.u_rng is None or self.v_rng is None:
            return 0.0 if scalar else np.zeros(np.shape(at_m))
        fx, fy, cx, cy = self.K
        at = np.atleast_1d(np.asarray(at_m, float))
        A, Z = np.meshgrid(np.linspace(-half_width_m, half_width_m, n_across),
                           np.linspace(center_down_m - band_m, center_down_m + band_m, n_down))
        A, Z = A.ravel()[None, :], Z.ravel()[None, :]
        c, s = math.cos(bearing_rad), math.sin(bearing_rad)
        T = at[:, None]
        P = np.stack([T * c - A * s, T * s + A * c, np.broadcast_to(Z, (len(at), Z.shape[1]))], -1) - self.origin
        q = P @ self.R_level_cam                               # level -> optical (R_level_cam is orthonormal)
        okz = q[..., 2] > 1e-6
        zz = np.where(okz, q[..., 2], 1.0)
        u, v = fx * q[..., 0] / zz + cx, fy * q[..., 1] / zz + cy
        inside = okz & (u >= self.u_rng[0]) & (u <= self.u_rng[1]) & (v >= self.v_rng[0]) & (v <= self.v_rng[1])
        f = inside.mean(axis=1)
        return float(f[0]) if scalar else f

    def seen_from(self, bearing_rad, half_width_m, band_m, center_down_m=0.0, min_view=0.9, lo=0.3, hi=6.0,
                  step=0.05):
        """Nearest along-distance at which the corridor's cross-section is at least min_view in
        view (inf = never within lo..hi). Nearer than this the corridor is partly outside the image."""
        at = np.arange(lo, hi + 1e-9, step)
        ok = np.nonzero(self.window_in_view(bearing_rad, half_width_m, band_m, at, center_down_m) >= min_view)[0]
        return float(at[ok[0]]) if len(ok) else math.inf


def level_points(depth_m, K, R_body_cam, t_body_cam, R_level_body, stride=8, rmin=0.3, rmax=6.0, border_px=0):
    """Depth image -> (N,3) points in the level heading frame relative to the body origin:
    columns (forward, right, down) in metres. K = (fx, fy, cx, cy). border_px: ignore that many
    pixels at every image edge, where a stereo matcher has the least support (a real OAK-D and
    the simulator's SGBM both produce false near matches along the top rows)."""
    fx, fy, cx, cy = K
    d = np.array(depth_m, np.float32, copy=True)
    if border_px > 0:
        b = int(border_px)
        d[:b, :] = np.nan
        d[-b:, :] = np.nan
        d[:, :b] = np.nan
        d[:, -b:] = np.nan
    d = d[::stride, ::stride]
    vs, us = np.mgrid[0:d.shape[0], 0:d.shape[1]]
    u = us.astype(np.float32) * stride
    v = vs.astype(np.float32) * stride
    ok = np.isfinite(d) & (d > rmin) & (d < rmax)
    z = d[ok]
    p_cam = np.stack([(u[ok] - cx) / fx * z, (v[ok] - cy) / fy * z, z], axis=1)
    p_body = p_cam @ np.asarray(R_body_cam).T + np.asarray(t_body_cam)
    return p_body @ np.asarray(R_level_body).T


def clearance(points, bearing_rad=0.0, half_width_m=0.6, band_m=0.4, min_hits=3):
    """Distance to the nearest obstacle in a corridor of +-half_width_m around the bearing
    (relative to the body heading, positive to the right) and within +-band_m of the body's
    height: the min_hits-th nearest point along the bearing (single pixels are not walls).
    inf when fewer than min_hits points lie in the corridor."""
    if points is None or len(points) == 0:
        return math.inf
    c, s = math.cos(bearing_rad), math.sin(bearing_rad)
    along = points[:, 0] * c + points[:, 1] * s
    across = -points[:, 0] * s + points[:, 1] * c
    sel = (along > 0.0) & (np.abs(across) < half_width_m) & (np.abs(points[:, 2]) < band_m)
    hits = np.sort(along[sel])
    return float(hits[min_hits - 1]) if len(hits) >= min_hits else math.inf


def in_view(bearing_rad, K, width, margin_deg=5.0):
    """Is a horizontal bearing (relative to the camera's forward axis) inside the image?"""
    fx, _, cx, _ = K
    half = min(math.atan2(cx, fx), math.atan2(width - cx, fx))
    return abs(bearing_rad) < half - math.radians(margin_deg)


# ------------------------------------------------------------ four clearances from one frame
SIDE_RAD = math.radians(20.0)        # widest side corridor a ~70 deg HFOV forward camera fully sees at 1.9 m
SIDE_HALF_WIDTH_M = 0.4              # with a 0.4 m half width (seen from ~1.6 m). +-60 deg is never in view.


class Probe(NamedTuple):
    """One corridor, judged from one frame.
    dist_m: None = UNKNOWN (do not move on it; `why` says which gate failed) | inf = OBSERVED CLEAR:
            the cross-section at at_m was >= min_view in the image, >= min_coverage of the rays
            crossing it returned depth and reached it, and clearance() found fewer than min_hits
            returns in the band. It claims nothing nearer than seen_from_m (partly out of view) and
            no coverage check beyond at_m | float = OBSERVED obstacle distance (an upper bound when
            the obstacle extends into the blind zone nearer than seen_from_m).
    obstacle_m: the same clearance() reading without the gates - evidence a caller may STOP on,
            never permission to move (inf = nothing dense seen, NOT free)."""
    dist_m: float | None
    obstacle_m: float
    seen_from_m: float               # the cross-section is >= min_view in view from here outward (inf = never)
    at_m: float                      # where view / coverage / reach were judged (nan = not judged)
    view: float
    coverage: float
    reach: float
    n_rays: int
    why: str                         # "" when observed


class Clearances(NamedTuple):
    ahead: Probe
    left: Probe
    right: Probe
    low: Probe

    def values(self):
        """(ahead, left, right, low) as float | None."""
        return tuple(p.dist_m for p in self)


def _unknown(why):
    return Probe(None, math.inf, math.inf, math.nan, 0.0, 0.0, 0.0, 0, why)


def probe(frame, points, bearing_rad, half_width_m, band_m, at_m, center_down_m=0.0, *, min_hits=3,
          min_view=0.9, min_coverage=0.6, max_m=math.inf):
    """clearance() of a corridor whose band is centred center_down_m below the body (points are
    shifted so clearance() is reused unchanged), gated by what the frame actually observed at at_m.
    Gates in order: in view (window_in_view) -> rays returned depth (corridor_coverage) -> a reading
    nearer than at_m is reported (those rays stopped ON it) -> otherwise the rays must have reached
    at_m (reach), so an occluder or a return nearer than rmin is unknown, never clear."""
    pts = points if center_down_m == 0.0 else points - np.array([0.0, 0.0, center_down_m])
    obs = clearance(pts, bearing_rad, half_width_m, band_m, min_hits)
    if obs > max_m:
        obs = math.inf
    sf = frame.seen_from(bearing_rad, half_width_m, band_m, center_down_m, min_view)
    if not math.isfinite(at_m):
        return Probe(None, obs, sf, math.nan, 0.0, 0.0, 0.0, 0, "never in view within range")
    view = frame.window_in_view(bearing_rad, half_width_m, band_m, at_m, center_down_m)
    cov, n = frame.corridor_coverage(bearing_rad, half_width_m, band_m, at_m, center_down_m)
    rch, _ = frame.corridor_coverage(bearing_rad, half_width_m, band_m, at_m, center_down_m, reach=True)
    base = dict(obstacle_m=obs, seen_from_m=sf, at_m=at_m, view=view, coverage=cov, reach=rch, n_rays=n)
    if view < min_view:
        return Probe(None, why=f"{view:.0%} of the corridor at {at_m:.2f} m is in view (< {min_view:.0%})", **base)
    if cov < min_coverage:
        return Probe(None, why=f"{cov:.0%} of {n} rays returned depth (< {min_coverage:.0%})", **base)
    if obs < at_m:
        return Probe(obs, why="", **base)
    if rch < min_coverage:
        return Probe(None, why=f"only {rch:.0%} of rays reached {at_m:.2f} m (occluded, or a return nearer than rmin)",
                     **base)
    return Probe(obs, why="", **base)


def four_clearances(frame, floor_down_m=None, *, points=None, bearing_rad=0.0, side_rad=SIDE_RAD,
                    side_half_width_m=SIDE_HALF_WIDTH_M, at_m=1.9, half_width_m=0.6, band_m=0.4, min_hits=3,
                    min_view=0.9, min_coverage=0.6, min_valid_frac=0.3, rmin=0.3, rmax=6.0, floor_margin_m=0.15,
                    low_top_m=None, low_at_m=None, low_max_m=4.0):
    """Ahead, left, right and low clearances from ONE DepthFrame -> Clearances (see Probe).

    ahead: bearing_rad, +-half_width_m, +-band_m around body height, judged at at_m (1.9 m =
           core's stop_m + lead_max_m). Identical to core's clearance at mission height.
    left / right: bearing_rad -/+ side_rad (positive = right), +-side_half_width_m. Default 20 deg /
           0.4 m; side_rad=math.radians(60), side_half_width_m=0.6 is allowed but a forward camera
           never sees that corridor, so it always returns None (with obstacle_m as evidence).
    low:   from band_m (the flight band's bottom) down to floor_margin_m above the floor (contains
           18 in boulders), ahead. Needs floor_down_m = body height above the floor in metres (down
           in the level frame); None / non-finite / <= 0 -> low is UNKNOWN. Judged at low_at_m or,
           by default, where the low window first comes fully into view (>= at_m); obstacles beyond
           low_max_m are ignored (floor returns rise into the band with range under attitude error).
    points: pass frame.points(rmin, rmax) if already computed (core caches it).
    min_hits: an obstacle is at least this many samples (core passes max(3, min_obstacle_px / stride^2)).
    Missing depth stays unknown: no NaN pixel or out-of-image space ever becomes inf."""
    if frame.valid_frac < min_valid_frac:
        u = _unknown(f"depth on {frame.valid_frac:.0%} of samples (< {min_valid_frac:.0%})")
        return Clearances(u, u, u, u)
    pts = frame.points(rmin, rmax) if points is None else points
    floor_ok = floor_down_m is not None and math.isfinite(floor_down_m) and floor_down_m > 0
    top, bot = -band_m, band_m                             # flight band, down positive
    if floor_ok:
        bot = min(bot, floor_down_m - floor_margin_m)      # never reach into the floor when flying low
    c_f, h_f = (top + bot) / 2, (bot - top) / 2
    kw = dict(min_hits=min_hits, min_view=min_view, min_coverage=min_coverage)
    ahead = probe(frame, pts, bearing_rad, half_width_m, h_f, at_m, c_f, **kw)
    left = probe(frame, pts, bearing_rad - side_rad, side_half_width_m, h_f, at_m, c_f, **kw)
    right = probe(frame, pts, bearing_rad + side_rad, side_half_width_m, h_f, at_m, c_f, **kw)
    if not floor_ok:
        low = _unknown("no height above the floor")
    else:
        l_top = band_m if low_top_m is None else max(band_m, floor_down_m - low_top_m)
        l_bot = floor_down_m - floor_margin_m
        if l_bot <= l_top:                                 # flying low: the flight band already reaches the floor margin
            low = ahead._replace(why=ahead.why or "low band lies inside the flight band")
        else:
            c_l, h_l = (l_top + l_bot) / 2, (l_bot - l_top) / 2
            if low_at_m is None:
                a = max(at_m, frame.seen_from(bearing_rad, half_width_m, h_l, c_l, min_view))
                a = a if a <= low_max_m else math.inf
            else:
                a = low_at_m
            low = probe(frame, pts, bearing_rad, half_width_m, h_l, a, c_l, max_m=low_max_m, **kw)
    return Clearances(ahead, left, right, low)
