"""refnav.stages with fake inputs (a 2-D world of wall segments, a fake line and pad detector, a
kinematic vehicle), then through refnav.core with the ray-cast camera and fake PX4 of
test_refnav_core. Every stage is walked; every failure rule is pinned."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from refnav import core as C  # noqa: E402
from refnav import depth as D  # noqa: E402
from refnav import stages as S  # noqa: E402
import test_refnav_core as T  # noqa: E402

INF = math.inf


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def probe(d):
    return D.Probe(d, INF if d is None else d, 0.5, 1.9, 1.0, 1.0, 1.0, 50, "fake unknown" if d is None else "")


def clr(ahead=INF, left=INF, right=INF, low=INF):
    return D.Clearances(probe(ahead), probe(left), probe(right), probe(low))


def ray(p, ang, segs, rmax=6.0):
    """Distance from p = (n, e) along heading ang to the nearest segment; inf beyond rmax."""
    dn, de = math.cos(ang), math.sin(ang)
    best = INF
    for (an, ae), (bn, be) in segs:
        sn, se = bn - an, be - ae
        den = dn * se - de * sn
        if abs(den) < 1e-12:
            continue
        qn, qe = an - p[0], ae - p[1]
        t, u = (qn * se - qe * sn) / den, (qn * de - qe * dn) / den
        if t > 0 and 0 <= u <= 1:
            best = min(best, t)
    return best if best <= rmax else INF


def ahead(p, ang, segs, half=0.3):
    """The ahead corridor: the nearest of three parallel rays, half m either side of the centre."""
    c, s = math.cos(ang), math.sin(ang)
    return min(ray((p[0] - s * o, p[1] + c * o), ang, segs) for o in (-half, 0.0, half))


def wall_distance(p, segs):
    best = INF
    for (an, ae), (bn, be) in segs:
        sn, se = bn - an, be - ae
        u = max(0.0, min(1.0, ((p[0] - an) * sn + (p[1] - ae) * se) / (sn * sn + se * se)))
        best = min(best, math.hypot(p[0] - an - u * sn, p[1] - ae - u * se))
    return best


class Sim:
    """Course + a vehicle that tracks the setpoint at <= 1 m/s and 60 deg/s; feed(sim, t) sets inputs."""

    def __init__(self, cp=None, feed=None, pos=(0.0, 0.0, -1.2), yaw=0.0):
        self.c = S.Course(cp or S.CourseParams())
        self.inp = S.Inputs()
        self.pos, self.yaw = np.array(pos, float), yaw
        self.feed = feed or (lambda sim, t: None)
        self.t, self.trace, self.ended = 0.0, [], None
        self.c.supervise(0.0, tuple(self.pos))
        self.c.start(0.0, (*pos, yaw))

    def tick(self, dt=0.05):
        self.feed(self, self.t)
        stop = self.c.supervise(self.t, tuple(self.pos))
        if stop is not None:
            self.ended = (self.t, stop)
            return
        d = self.c.step(self.t, dt, tuple(self.pos), self.yaw, self.inp)
        self.trace.append((self.t, d.stage, d.reason, d.land, self.pos.copy(), d.setpoint))
        if d.land:
            self.ended = (self.t, d.reason)
            return
        n, e, dd, yaw = d.setpoint
        step = np.array([n, e, dd]) - self.pos
        dist = np.linalg.norm(step)
        if dist > 1e-9:
            self.pos += step * min(1.0, 1.0 * dt / dist)
        self.yaw = wrap(self.yaw + max(-math.radians(60) * dt, min(math.radians(60) * dt, wrap(yaw - self.yaw))))
        self.t += dt

    def run(self, seconds, until=None):
        end = self.t + seconds
        while self.t < end and self.ended is None and not (until and until(self)):
            self.tick()
        return self

    def stages(self):
        out = []
        for _, st, *_r in self.trace:
            if not out or out[-1] != st:
                out.append(st)
        return out

    def reasons(self):
        return [r for _, _, r, *_x in self.trace]

    def first(self, reason):
        return next(t for t, _, r, *_x in self.trace if r == reason)


# ---------------------------------------------------------------- fake world and detectors
BARRIER_E, MOUTH = 5.0, (1.6, 2.4)                    # a barrier across e = 5 with a 0.8 m mouth at n = 2
WORLD = [((-5.0, BARRIER_E), (MOUTH[0], BARRIER_E)), ((MOUTH[1], BARRIER_E), (9.0, BARRIER_E)),
         ((MOUTH[0], BARRIER_E), (MOUTH[0], 7.0)), ((MOUTH[1], BARRIER_E), (MOUTH[1], 7.0)),   # the tunnel
         ((-5.0, 9.0), (2.3, 9.0)), ((3.9, 9.0), (9.0, 9.0))]                               # chicane wall, gate B
DARK = (5.6, 6.2)                                     # stretch of the tunnel where depth has no returns
PAD = (3.0, 11.5)                                     # behind gate B; the fake detector sees it < 2.5 m


def vision_and_height(sim, t):
    sim.inp.set_vision(t, True)
    sim.inp.set_height(t, 1.38)


def line_L(sim, t):
    """A line north from (0, 0) to (2, 0), then east to (2, 2) where it ends."""
    n, e, _ = sim.pos
    psi = sim.yaw
    if n < 1.7:
        sim.inp.set_line(t, True, -e * math.cos(psi), wrap(0.0 - psi), end=False)
    else:
        sim.inp.set_line(t, True, (2.0 - n) * -math.sin(psi), wrap(math.pi / 2 - psi), end=e >= 2.0)


def world_clearances(sim, t):
    n, e, _ = sim.pos
    psi = sim.yaw
    a = ahead((n, e), psi, WORLD)
    if DARK[0] <= e <= DARK[1] and MOUTH[0] < n < MOUTH[1]:
        a = None
    sim.inp.set_clearances(t, clr(a, ray((n, e), psi - math.radians(20), WORLD),
                                  ray((n, e), psi + math.radians(20), WORLD)))


def pad_detector(sim, t):
    n, e, _ = sim.pos
    dn, de = PAD[0] - n, PAD[1] - e
    rel = wrap(math.atan2(de, dn) - sim.yaw)
    dist = math.hypot(dn, de)
    if abs(rel) < math.radians(35) and 0.8 < dist < 2.5 and ray((n, e), math.atan2(de, dn), WORLD) > dist:
        sim.inp.set_pad(t, True, (dist * math.cos(rel), dist * math.sin(rel)))
    else:
        sim.inp.set_pad(t, False)


def course_world(sim, t):
    vision_and_height(sim, t)
    line_L(sim, t)
    world_clearances(sim, t)
    pad_detector(sim, t)


def open_line_north(end_at=None, valid=lambda t: True):
    """A straight line north from the start, in open space (all clear), optionally ending at n = end_at."""
    def feed(sim, t):
        vision_and_height(sim, t)
        n, e, _ = sim.pos
        sim.inp.set_line(t, valid(t), -e * math.cos(sim.yaw), wrap(-sim.yaw),
                         end=end_at is not None and n >= end_at)
        sim.inp.set_clearances(t, clr())
    return feed


# ---------------------------------------------------------------- walking the stages
def test_walks_all_eight_stages_with_the_tunnel_on():
    sim = Sim(S.CourseParams(tunnel_enabled=True, tunnel_len_m=4.0), course_world).run(150)
    assert sim.stages() == list(S.STAGES), (sim.stages(), sim.ended)
    assert sim.ended[1] == S.COURSE_COMPLETE
    assert math.hypot(sim.pos[0] - PAD[0], sim.pos[1] - PAD[1]) < 0.3
    assert all(r for r in sim.reasons())                                  # every decision has a reason
    logged = [(st, r) for _, st, r in sim.c.log]
    for want in (("follow_line", "turning onto the line"), ("tunnel_cross", "depth back after the dark stretch"),
                 ("chicane", "gate B: blocked ahead, turning left"), ("chicane", "pad in view")):
        assert want in logged, want
    assert {st for st, _ in logged} >= set(S.STAGES)
    # crossed the dark stretch on space it had observed free, and never came within stop_m of a wall
    assert any(r == "crossing the tunnel" and DARK[0] <= p[1] <= DARK[1] for _, st, r, _l, p, _s in sim.trace)
    assert min(wall_distance(p, WORLD) for *_x, p, _s in sim.trace) > 0.3


def test_defaults_bank_checkpoints_one_and_two_at_the_tunnel_mouth():
    sim = Sim(None, course_world).run(150)
    assert sim.stages() == ["follow_line", "gate_a", "tunnel_align"]
    assert sim.ended[1] == "tunnel disabled: landing at the tunnel mouth"
    hover = [p for t, _, r, _, p, _ in sim.trace if r == "tunnel disabled: hovering at the tunnel mouth"]
    assert len(hover) * 0.05 >= S.CourseParams().mouth_hover_s - 0.1
    assert np.ptp(np.array(hover)[:, :2], axis=0).max() < 0.15             # it held still while hovering
    assert sim.pos[1] < BARRIER_E - 0.9                                    # never went in


def test_chicane_turns_toward_the_open_side_and_window_says_it_uses_gate_b():
    for cp, prefix in ((S.CourseParams(), "gate B: "), (S.CourseParams(window_enabled=True),
                                                         "no window detector input, gate B: ")):
        for left, right, word, sign in ((INF, 1.2, "left", -1), (1.2, INF, "right", 1)):
            def feed(sim, t, left=left, right=right):
                vision_and_height(sim, t)
                sim.inp.set_clearances(t, clr(1.0, left, right))
            sim = Sim(cp, feed)
            sim.c._enter("chicane", 0.0)
            sim.run(1.0)
            assert sim.reasons()[-1] == prefix + f"blocked ahead, turning {word}"
            assert sign * sim.yaw > math.radians(20)                     # it turned that way
            assert np.linalg.norm(sim.pos[:2]) < 1e-9                     # without translating


# ---------------------------------------------------------------- failures
def test_line_lost_holds_then_lands():
    sim = Sim(None, open_line_north(valid=lambda t: t < 4.0)).run(30)
    t_lost = sim.first("line lost, holding")
    assert 3.9 < t_lost < 4.1
    held = [p for t, _, r, _, p, _ in sim.trace if r == "line lost, holding" and t > t_lost + 0.3]
    assert np.ptp(np.array(held)[:, :2], axis=0).max() < 1e-6              # stopped advancing
    assert sim.ended[1] == "line lost for 5 s, landing" and sim.ended[0] == pytest.approx(t_lost + 5.0, abs=0.06)


def test_line_back_within_the_hold_resumes():
    sim = Sim(None, open_line_north(valid=lambda t: not 4.0 <= t < 6.0)).run(12)
    assert sim.ended is None and sim.reasons()[-1] == "following the line"
    assert sim.pos[0] > 4.0


def test_stage_timeout_hovers_then_lands():
    sim = Sim(S.CourseParams(follow_line_timeout_s=10.0), open_line_north()).run(30)
    t0 = sim.first("follow_line timed out after 10 s, hovering")
    assert t0 == pytest.approx(10.0, abs=0.06)
    hover = [p for t, _, r, _, p, _ in sim.trace if r == "follow_line timed out, hovering" and t > t0 + 0.3]
    assert np.ptp(np.array(hover)[:, :2], axis=0).max() < 1e-6
    assert sim.ended[1] == "follow_line timed out; hovered 5 s, landing"
    assert sim.ended[0] == pytest.approx(t0 + 5.0, abs=0.06)


def test_an_exit_seen_during_the_fallback_hover_moves_on():
    sim = Sim(S.CourseParams(follow_line_timeout_s=10.0), open_line_north(end_at=0.0))
    sim.c._line_end_n = -10**6                                             # the end is not believed before the timeout
    sim.run(10.5)
    sim.c._line_end_n = 0
    sim.run(5, until=lambda s: s.c.stage == "gate_a")
    assert sim.c.stage == "gate_a" and "line end seen (during the fallback hover)" in sim.reasons()


def test_vision_lost_holds_and_stops_advancing_then_resumes():
    def feed(sim, t):
        open_line_north()(sim, t)
        sim.inp.set_vision(t, not 3.0 <= t < 5.0)
    sim = Sim(None, feed).run(8)
    during = [s for t, _, r, _, _, s in sim.trace if r == "vision lost, holding"]
    assert len(during) > 30 and len(set(during[1:])) == 1                  # setpoint (yaw too) frozen
    assert sim.ended is None and sim.reasons()[-1] == "following the line"


def test_vision_lost_too_long_lands():
    def feed(sim, t):
        open_line_north()(sim, t)
        sim.inp.set_vision(t, t < 3.0)
    sim = Sim(None, feed).run(20)
    assert sim.ended[1] == "vision lost for 5 s, landing"
    assert sim.ended[0] - sim.first("vision lost, holding") == pytest.approx(5.0, abs=0.06)


def test_pose_jump_latches_a_fault_drops_goals_and_never_resumes():
    sim = Sim(S.CourseParams(tunnel_enabled=True, tunnel_len_m=4.0), course_world)
    sim.run(150, until=lambda s: s.c.stage == "find_pad")
    assert sim.c._pad_ned is not None                                     # a world goal to drop
    sim.pos[:2] += (0.4, 0.0)                                             # 0.4 m: below the threshold
    sim.tick()
    assert sim.ended is None and sim.c.fault is None
    sim.pos[:2] += (0.0, 0.6)
    sim.tick()
    assert sim.ended[1] == "pose jump 0.60 m in one tick: world goals dropped, landing"
    assert sim.c.fault and sim.c._pad_ned is None and sim.c._free is None
    assert np.allclose(sim.c.sp[:3], sim.pos)                              # holds where it now is
    for k in range(5):                                                     # nothing resumes, even with good inputs
        course_world(sim, sim.t + 0.05 * k)
        d = sim.c.step(sim.t + 0.05 * k, 0.05, tuple(sim.pos), sim.yaw, sim.inp)
        assert d.land and "pose jump" in d.reason
        assert sim.c.supervise(sim.t + 0.05 * k, tuple(sim.pos + 3.0)) is None   # already latched


STALE = {
    "line stale": (lambda i, t: i.set_line(t - 1.0, True, 0.0, 0.0), "line stale, holding"),
    "line from the future": (lambda i, t: i.set_line(t + 1.0, True, 0.0, 0.0), "line from the future, holding"),
    "line missing": (lambda i, t: i.set_line(None, True, 0.0, 0.0), "line missing, holding"),
    "line NaN offset": (lambda i, t: i.set_line(t, True, math.nan, 0.0), "line lost, holding"),
    "clearances stale": (lambda i, t: i.set_clearances(t - 1.0, clr()), "depth clearances stale, holding"),
    "clearances missing": (lambda i, t: i.set_clearances(None, None), "depth clearances missing, holding"),
    "ahead unknown": (lambda i, t: i.set_clearances(t, clr(ahead=None)), "ahead unknown, holding"),
    "obstacle": (lambda i, t: i.set_clearances(t, clr(ahead=0.8)), "obstacle ahead, holding"),
    "vision stale": (lambda i, t: i.set_vision(t - 1.0, True), "vision stale, holding"),
    "vision missing": (lambda i, t: i.set_vision(None, True), "vision missing, holding"),
}


@pytest.mark.parametrize("case", list(STALE))
def test_stale_missing_or_blocked_input_never_moves_and_lands(case):
    spoil, reason = STALE[case]

    def feed(sim, t):
        open_line_north()(sim, t)
        spoil(sim.inp, t)
    sim = Sim(None, feed).run(20)
    assert sim.reasons()[0] == reason
    assert np.linalg.norm(sim.pos[:2]) < 1e-9
    assert sim.ended[1] == reason.replace(", holding", " for 5 s, landing")


def test_the_carrot_stops_short_of_an_observed_obstacle():
    wall = [((3.0, -5.0), (3.0, 5.0))]

    def feed(sim, t):
        open_line_north()(sim, t)
        n, e, _ = sim.pos
        sim.inp.set_clearances(t, clr(ray((n, e), sim.yaw, wall)))
    sim = Sim(None, feed).run(30)
    assert max(p[0] for *_x, p, _s in sim.trace) <= 3.0 - 0.9 + 0.05
    assert sim.ended[1] == "obstacle ahead for 5 s, landing"


def test_flight_limit_lands_at_170_s_whatever_it_is_doing():
    def feed(sim, t):                                                     # turning in place for ever: never a stall
        vision_and_height(sim, t)
        sim.inp.set_line(t, True, 0.0, 1.0)
        sim.inp.set_clearances(t, clr())
    sim = Sim(S.CourseParams(follow_line_timeout_s=1e9), feed).run(400)
    assert S.CourseParams().flight_limit_s == 170.0
    assert sim.ended[1] == "flight limit 170 s after takeoff, landing"
    assert sim.ended[0] == pytest.approx(170.0, abs=0.06)
    assert set(sim.reasons()) == {"turning onto the line"}


# ---------------------------------------------------------------- through refnav.core
def fly_course(cp, walls, seconds=120.0, dt=0.05, line_end_n=0.8, jump_after=None, line_angle=None):
    m = C.Mission(C.Params(use_stages=True, depth_border_px=3), course=cp)
    px4 = T.FakePX4()
    walls = list(walls) + T.ROOM
    t, next_depth, log, course_t, jumped = 0.0, 0.0, [], None, None
    while t < seconds and m.phase != "done":
        m.on_health(t, True)
        if t >= next_depth:
            img = T.render_depth(px4.pos, px4.yaw, walls)
            m.on_depth(t, img, T.K, T.R_BODY_CAM, T.T_BODY_CAM, T.q_yaw(px4.yaw), now=t)
            next_depth = t + 1 / 7.5
        n, e, d = px4.pos
        m.inputs.set_vision(t, True)
        m.inputs.set_height(t, -d + T.FLOOR_D)
        ang = wrap(-px4.yaw) if line_angle is None else line_angle
        m.inputs.set_line(t, True, -e * math.cos(px4.yaw), ang, end=n >= line_end_n)
        if m.phase == "course" and course_t is None:
            course_t = t
        if jump_after is not None and jumped is None and course_t is not None and t >= course_t + jump_after:
            px4.pos[0] += 0.6
            jumped = px4.pos.copy()
        out = m.step(t, px4.vehicle())
        log.append((t, out.phase, list(out.events), out.setpoint, px4.pos.copy()))
        px4.apply(t, out, dt)
        t += dt
    return m, px4, log, course_t, jumped


def test_core_without_use_stages_has_no_course():
    assert C.Params().use_stages is False
    assert C.Mission(C.Params(route=[(1.0, 0.0)])).course is None


def test_core_flies_the_stages_to_a_landing_at_the_tunnel_mouth():
    barrier = [(0, 4.0, -15.0, 15.0)]                                     # a wall across n = 4
    m, px4, log, course_t, _ = fly_course(None, barrier)
    assert m.result == "landed: tunnel disabled: landing at the tunnel mouth", (m.result, m.course.log[-5:])
    events = [e for *_x, ev, _s, _p in log for e in ev]
    for st in ("follow_line", "gate_a", "tunnel_align"):
        assert any(e.startswith(f"course {st}:") for e in events), st
    assert max(p[0] for *_x, p in log) < 4.0 - 0.9                         # never nearer than stop_m
    assert m.course.log[0][2] == "following the line" and not px4.armed and px4.landed
    assert m.status(log[-1][0])["course"]["stage"] == "tunnel_align"


def test_core_pose_jump_lands_drops_goals_and_never_resumes():
    m, px4, log, course_t, jumped = fly_course(None, [], jump_after=2.0)
    assert jumped is not None and m.result.startswith("landed: pose jump 0.6")       # 0.6 m + one tick of flight
    i = next(k for k, (t, ph, *_r) in enumerate(log) if ph == "land")
    assert np.allclose(log[i][3][:3], jumped, atol=1e-9)                  # the setpoint is where it now is
    assert all(ph != "course" for _, ph, *_r in log[i:]) and m.targets == []
    assert not px4.armed and px4.landed


def test_core_flight_limit_lands_whatever_the_stage_is_doing():
    m, px4, log, course_t, _ = fly_course(S.CourseParams(flight_limit_s=12.0, follow_line_timeout_s=1e9), [],
                                          line_end_n=1e9, line_angle=1.0)
    climb_t = next(t for t, ph, *_r in log if ph == "climb")
    land_t = next(t for t, ph, *_r in log if ph == "land")
    assert m.result == "landed: flight limit 12 s after takeoff, landing"
    assert land_t - climb_t == pytest.approx(12.0, abs=0.11)
