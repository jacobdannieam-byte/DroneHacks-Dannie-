"""refnav.depth.four_clearances on synthetic depth frames. Self-contained: its own ray-cast renderer
(floor, walls standing on the floor with a top, box boulders, optional ceiling) so it does not depend
on tests/test_refnav_core.py. Each case runs on the small test camera and on the 640x400 sim camera.

The invariant under test: missing depth (no return, out of the image, occluded) is UNKNOWN (None),
never free space (inf)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from refnav import depth as D  # noqa: E402

# ------------------------------------------------------------------ renderer
R_BODY_CAM = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])   # FLU <- optical, forward camera
T_BODY_CAM = np.array([0.16, 0.0, 0.0])
FLOOR_D = 0.18                       # floor below the body origin of a landed aircraft (as tests/test_refnav_core.py)
FLU2FRD = np.diag([1.0, -1.0, -1.0])

TEST = dict(W=160, H=100, K=(106.0, 106.0, 80.0, 50.0), stride=2, border=3, hits=5)
SIM = dict(W=640, H=400, K=(421.0, 421.0, 320.0, 200.0), stride=8, border=24, hits=19)
CAMS = pytest.mark.parametrize("cam", [TEST, SIM], ids=["test160", "sim640"])
CEIL = -3.82                                                   # 4 m ceiling
ROOM = [(0, 8.0, -8, 8), (0, -8.0, -8, 8), (1, 8.0, -8, 8), (1, -8.0, -8, 8)]
TOP68 = FLOOR_D - 1.727                                        # 68 in maze walls
MAZE = [(1, 0.7, -1, 12, TOP68), (1, -0.7, -1, 12, TOP68), (0, 6.0, -0.7, 0.7, TOP68)]
SIXTY = dict(side_rad=math.radians(60.0), side_half_width_m=0.6)


def quat(yaw=0.0, pitch=0.0, roll=0.0):
    """FRD -> NED attitude quaternion (w, x, y, z), ZYX; pitch + = nose up."""
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


def render(W, H, K, pos_ned, q=(1.0, 0, 0, 0), walls=(), boxes=(), ceiling_d=None):
    """Optical depth (m, NaN = no return). walls: (axis, value, lo, hi[, top_d]) vertical planes,
    axis 0: n = value, axis 1: e = value, from the floor up to top_d (default -2.5).
    boxes: (n0, n1, e0, e1, top_d) solids standing on the floor."""
    fx, fy, cx, cy = K
    vs, us = np.mgrid[0:H, 0:W].astype(np.float64)
    d_opt = np.stack([(us - cx) / fx, (vs - cy) / fy, np.ones_like(us)], -1)     # optical z = 1
    w, x, y, z = q
    R_ned_frd = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                          [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    d_ned = d_opt @ (R_ned_frd @ FLU2FRD @ R_BODY_CAM).T
    o = np.asarray(pos_ned, float) + R_ned_frd @ (FLU2FRD @ T_BODY_CAM)
    best = np.full((H, W), np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for plane_d in [FLOOR_D] + ([] if ceiling_d is None else [ceiling_d]):
            t = (plane_d - o[2]) / d_ned[..., 2]
            best = np.where(t > 0, np.minimum(best, t), best)
        for wall in walls:
            axis, val, lo, hi = wall[:4]
            top = wall[4] if len(wall) > 4 else -2.5
            t = (val - o[axis]) / d_ned[..., axis]
            p = o + d_ned * t[..., None]
            hit = (t > 0) & (p[..., 1 - axis] >= lo) & (p[..., 1 - axis] <= hi) & (p[..., 2] <= FLOOR_D) & (p[..., 2] >= top)
            best = np.where(hit, np.minimum(best, t), best)
        for n0, n1, e0, e1, top in boxes:
            t1 = (np.array([n0, e0, top]) - o) / d_ned
            t2 = (np.array([n1, e1, FLOOR_D]) - o) / d_ned
            tmin = np.nanmax(np.minimum(t1, t2), axis=-1)
            tmax = np.nanmin(np.maximum(t1, t2), axis=-1)
            hit = (tmax >= tmin) & (tmin > 0)
            best = np.where(hit, np.minimum(best, tmin), best)
    best[~np.isfinite(best)] = np.nan
    return best.astype(np.float32)


def boulder(n_face, e_c=0.0, size=0.457):
    """An 18 in cube on the floor, its near face at n_face."""
    return (n_face, n_face + size, e_c - size / 2, e_c + size / 2, FLOOR_D - size)


def rend(cam, h=1.2, q=(1.0, 0, 0, 0), walls=(), boxes=(), ceiling=CEIL, room=True):
    """Body at height h above its takeoff origin, i.e. h + FLOOR_D above the floor."""
    return render(cam["W"], cam["H"], cam["K"], (0, 0, -h), q, list(walls) + (ROOM if room else []), boxes, ceiling)


def frame(cam, img, q=(1.0, 0, 0, 0)):
    return D.DepthFrame(img, cam["K"], R_BODY_CAM, T_BODY_CAM, D.level_rotation(q), stride=cam["stride"],
                        border_px=cam["border"])


def fc(cam, img, floor, q=(1.0, 0, 0, 0), **kw):
    return D.four_clearances(frame(cam, img, q), floor, min_hits=cam["hits"], **kw)


def original_corridor_coverage(f, bearing_rad, half_width_m, band_m, at_m):
    """refnav/depth.py's corridor_coverage before four_clearances, frozen for the compatibility test."""
    c, s = math.cos(bearing_rad), math.sin(bearing_rad)
    along_dir = f.dirs[:, 0] * c + f.dirs[:, 1] * s
    ok = along_dir > 1e-6
    t = np.where(ok, (at_m - (f.origin[0] * c + f.origin[1] * s)) / np.where(ok, along_dir, 1.0), -1.0)
    p = f.origin + f.dirs * t[:, None]
    across = -p[:, 0] * s + p[:, 1] * c
    window = ok & (t > 0) & (np.abs(across) < half_width_m) & (np.abs(p[:, 2]) < band_m)
    n = int(window.sum())
    return (float(f.valid[window].mean()) if n else 0.0), n


def original_points(cam, img, q, rmin=0.3, rmax=6.0):
    """DepthFrame's sampling + points() before four_clearances, frozen for the compatibility test."""
    fx, fy, cx, cy = cam["K"]
    b, s = cam["border"], cam["stride"]
    vs, us = np.mgrid[b:cam["H"] - b:s, b:cam["W"] - b:s]
    z = img[vs, us].ravel()
    u, v = us.ravel().astype(np.float32), vs.ravel().astype(np.float32)
    R = D.level_rotation(q) @ R_BODY_CAM
    dirs = np.stack([(u - cx) / fx, (v - cy) / fy, np.ones_like(u)], axis=1) @ R.T
    origin = D.level_rotation(q) @ T_BODY_CAM
    sel = np.isfinite(z) & (z > 0) & (z > rmin) & (z < rmax)
    return origin + dirs[sel] * z[sel, None]


# ------------------------------------------------------------------ tests
@CAMS
def test_blank_frame_is_unknown_everywhere(cam):
    """The dark tunnel: a passive stereo camera returns nothing."""
    c = fc(cam, np.full((cam["H"], cam["W"]), np.nan, np.float32), 1.38)
    assert c.values() == (None, None, None, None)
    assert all(not math.isfinite(p.obstacle_m) for p in c)


def test_ahead_is_the_existing_rule_and_open_room_is_observed_clear():
    f = frame(TEST, rend(TEST, walls=[(0, 3.0, -5, 5)]))
    c = D.four_clearances(f, 1.38, min_hits=5)
    assert c.ahead.dist_m == D.clearance(f.points(0.3, 6.0), 0.0, 0.6, 0.4, 5) == pytest.approx(3.0, abs=0.05)
    assert c.ahead.coverage == original_corridor_coverage(f, 0.0, 0.6, 0.4, 1.9)[0]
    b = D.four_clearances(f, 1.38, min_hits=5, bearing_rad=0.2)          # centred on a leg bearing
    assert b.ahead.obstacle_m == D.clearance(f.points(0.3, 6.0), 0.2, 0.6, 0.4, 5)
    o = fc(TEST, rend(TEST), 1.38)
    assert o.ahead.dist_m == math.inf and o.ahead.view == 1.0 and 1.0 <= o.ahead.seen_from_m <= 1.2


@CAMS
def test_missing_depth_over_a_band_is_unknown_only_for_that_band(cam):
    img = rend(cam, h=0.8)
    H, W = cam["H"], cam["W"]
    mid = img.copy()
    mid[int(.3 * H):int(.7 * H), int(.375 * W):int(.625 * W)] = np.nan
    c = fc(cam, mid, 0.98)
    assert c.ahead.dist_m is None and c.ahead.coverage < 0.6 and c.ahead.view == 1.0   # in view, no data
    assert c.low.dist_m == math.inf
    low = img.copy()
    low[int(.6 * H):, :] = np.nan
    c = fc(cam, low, 0.98)
    assert c.low.dist_m is None and c.ahead.dist_m == math.inf


@CAMS
def test_sides_default_to_twenty_degrees_and_sixty_is_unknown_never_free(cam):
    f = frame(cam, rend(cam, walls=MAZE, room=False))
    c = D.four_clearances(f, 1.38, min_hits=cam["hits"])                 # default 20 deg / 0.4 m
    assert c.left.dist_m == pytest.approx(1.32, abs=0.1) and c.right.dist_m == pytest.approx(1.32, abs=0.1)
    o = fc(cam, rend(cam), 1.38)
    assert o.left.dist_m == o.right.dist_m == math.inf and 1.4 <= o.left.seen_from_m <= 1.7
    s = D.four_clearances(f, 1.38, min_hits=cam["hits"], **SIXTY)        # the 60 deg option
    assert s.left.dist_m is None and s.right.dist_m is None and s.left.seen_from_m == math.inf
    b = math.radians(60)
    cov, n = f.corridor_coverage(b, 0.6, 0.4, 0.6)                        # the trap the view gate closes
    assert n > 0 and cov == 1.0 and f.window_in_view(b, 0.6, 0.4, 0.6) < 0.15
    pn, pe = 0.8 * math.cos(b) - 0.3 * math.sin(b), 0.8 * math.sin(b) + 0.3 * math.cos(b)
    fp = frame(cam, rend(cam, boxes=[(pn - 0.1, pn + 0.1, pe - 0.1, pe + 0.1, -2.0)]))   # a post nobody imaged
    assert D.clearance(fp.points(), b, 0.6, 0.4, cam["hits"]) == math.inf and fp.corridor_coverage(b, 0.6, 0.4, 0.8)[0] == 1.0
    assert D.four_clearances(fp, 1.38, min_hits=cam["hits"], at_m=0.8, **SIXTY).right.dist_m is None


def test_evidence_without_observation_is_not_a_distance():
    c = fc(TEST, rend(TEST, walls=[(0, 1.0, 0.3, 3.0)]), 1.38, **SIXTY)   # wall face in the near-right sliver
    assert c.right.dist_m is None and c.right.obstacle_m == pytest.approx(0.97, abs=0.1)
    junk = np.full((TEST["H"], TEST["W"]), np.nan, np.float32)
    junk[44:56, 74:86] = 0.6                                              # 12x12 px of near junk, rest blank
    j = fc(TEST, junk, 1.38, min_valid_frac=0.0)
    assert D.clearance(frame(TEST, junk).points(), 0, 0.6, 0.4, 5) < 1.0
    assert j.ahead.dist_m is None and j.ahead.obstacle_m < 1.0


@CAMS
def test_a_return_nearer_than_rmin_is_never_clear(cam):
    f = frame(cam, rend(cam, walls=[(0, 0.41, -5, 5)]))                    # 0.25 m from the camera
    cov, _ = original_corridor_coverage(f, 0.0, 0.6, 0.4, 1.9)
    assert cov == 1.0 and D.clearance(f.points(0.3, 6.0), 0, 0.6, 0.4, cam["hits"]) == math.inf   # the old rule
    c = D.four_clearances(f, 1.38, min_hits=cam["hits"])
    assert c.ahead.dist_m is None and c.ahead.reach < 0.6


@CAMS
@pytest.mark.parametrize("pitch", [-6.0, 0.0, 6.0])
def test_low_sees_a_boulder_the_flight_band_misses(cam, pitch):
    q = quat(pitch=math.radians(pitch))
    c = fc(cam, rend(cam, h=0.8, q=q, boxes=[boulder(2.5)]), 0.98, q)
    assert c.low.dist_m == pytest.approx(2.5, abs=0.1) and c.ahead.dist_m == math.inf
    if pitch == 0.0:
        c = fc(cam, rend(cam, h=1.2, boxes=[boulder(3.4)]), 1.38)
        assert c.low.dist_m == pytest.approx(3.4, abs=0.1) and c.low.at_m == c.low.seen_from_m > 3.0


@CAMS
def test_the_low_blind_zone_is_reported_not_hidden(cam):
    img = rend(cam, h=1.2, boxes=[boulder(1.5)])
    c = fc(cam, img, 1.38)
    assert c.low.dist_m == math.inf and c.low.seen_from_m > 1.5 + 0.457 and c.low.at_m >= c.low.seen_from_m
    assert fc(cam, img, 1.38, low_at_m=1.9).low.dist_m is None            # judged where it is out of view


def test_low_needs_a_floor_height():
    img = rend(TEST)
    ref = fc(TEST, img, 1.38)
    for floor in (None, math.nan, 0.0):
        c = fc(TEST, img, floor)
        assert c.low.dist_m is None and "floor" in c.low.why
        assert (c.ahead.dist_m, c.left.dist_m, c.right.dist_m) == (ref.ahead.dist_m, ref.left.dist_m, ref.right.dist_m)


@CAMS
@pytest.mark.parametrize("h", [0.8, 1.2])
def test_the_floor_is_not_a_low_obstacle_within_the_error_budget(cam, h):
    img = rend(cam, h=h)
    agl = h + FLOOR_D
    for e in (-2.0, 0.0, 2.0):
        lo = fc(cam, img, agl, quat(pitch=math.radians(e))).low.dist_m
        assert lo is None or lo == math.inf, (e, lo)
    for err in (-0.1, 0.1):
        lo = fc(cam, img, agl + err).low.dist_m
        assert lo is None or lo == math.inf, (err, lo)
    # documented limit: 3 deg nose-up attitude error, or a 0.10 m margin at 2 deg, puts the floor in the band
    bad = [fc(cam, img, agl, quat(pitch=math.radians(3))).low.dist_m,
           fc(cam, img, agl, quat(pitch=math.radians(2)), floor_margin_m=0.10).low.dist_m]
    assert any(b is not None and math.isfinite(b) for b in bad)


@CAMS
def test_an_occluder_hides_the_low_band_and_is_reported_ahead(cam):
    c = fc(cam, rend(cam, h=0.8, walls=[(0, 0.6, -5, 5)]), 0.98)
    assert c.ahead.dist_m == pytest.approx(0.6, abs=0.05)
    assert c.low.dist_m is None and c.low.reach < 0.6


def test_existing_depth_frame_behaviour_is_unchanged():
    rng = np.random.default_rng(1)
    for cam in (TEST, SIM):
        for q in ((1.0, 0, 0, 0), quat(pitch=math.radians(3), roll=math.radians(-2))):
            img = rend(cam, walls=[(0, 3.0, -5, 5)], q=q)
            img[rng.random(img.shape) < 0.2] = np.nan
            f = frame(cam, img, q)
            assert np.array_equal(f.points(0.3, 6.0), original_points(cam, img, q))
            assert f.valid_frac == pytest.approx(float(np.isfinite(img[cam["border"]:cam["H"] - cam["border"]:cam["stride"],
                                                                    cam["border"]:cam["W"] - cam["border"]:cam["stride"]]).mean()))
            for b in (-0.6, 0.0, 0.3, 1.0):
                assert f.corridor_coverage(b, 0.6, 0.4, 1.9) == original_corridor_coverage(f, b, 0.6, 0.4, 1.9) \
                    == f.corridor_coverage(b, 0.6, 0.4, 1.9, center_down_m=0.0, reach=False)


def test_decode_16uc1_treats_zero_and_65535_as_no_return():
    mm = np.array([[0, 1000], [65534, 65535]], np.uint16)
    a = D.decode_depth("16UC1", 2, 2, 4, mm.tobytes())
    assert np.isnan(a[0, 0]) and np.isnan(a[1, 1])
    assert a[0, 1] == pytest.approx(1.0) and a[1, 0] == pytest.approx(65.534)
    b = D.decode_depth("mono16", 2, 2, 4, mm.tobytes())
    assert np.isnan(b[1, 1])
