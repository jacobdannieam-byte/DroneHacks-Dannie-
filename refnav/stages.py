"""refnav.stages - the course as sensor-driven stages, flown by refnav.core. PURE: no ROS, no clock reads.

core.Mission hands over to Course after its climb when Params.use_stages is true (the route legs
otherwise). Every tick core passes its clock, PX4's position and heading and an Inputs record;
Course.step answers with a setpoint, a short reason and whether to land. core keeps every check it
already has (Offboard loss, estimator health, depth silence) and owns the landing itself.

    follow_line -> gate_a -> tunnel_align -> tunnel_cross -> tunnel_exit -> chicane -> find_pad -> land

Rules (each one is a test in tests/test_stages.py):
* every input group carries the capture stamp of its data; missing, stale (older than its
  *_stale_s) or future-stamped is invalid, the same as a detector reporting "not found";
* translation only into OBSERVED free space: the carrot leads by at most the depth ahead
  (refnav.depth.four_clearances) minus stop_m, unknown ahead holds; sideways and backward leads are
  capped at lat_lead_m. Rotation in place needs no clearance;
* vision_ok invalid -> hold and stop advancing (yaw frozen too);
* a hold that waits on an input (line lost, ahead unknown or blocked, vision) lands after
  hold_max_s; a stage past its timeout hovers fallback_hover_s (its exit may still fire), then lands;
* the position jumping more than pose_jump_m between two ticks latches a fault: every world-position
  goal is dropped, the aircraft lands where it is, nothing resumes this flight;
* flight_limit_s after takeoff it lands, whatever it is doing;
* every decision has a reason; each change of (stage, reason) is logged (Course.log, core's events).
Defaults bank checkpoints 1 and 2: tunnel_enabled false -> hover at the tunnel mouth, then land;
window_enabled false -> the chicane makes for Gate B.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

STAGES = ("follow_line", "gate_a", "tunnel_align", "tunnel_cross", "tunnel_exit", "chicane", "find_pad", "land")
COURSE_COMPLETE = "course complete: centred over the pad"


@dataclass
class CourseParams:
    tunnel_enabled: bool = False      # False: hover at the tunnel mouth, then land (banks checkpoints 1-2)
    window_enabled: bool = False      # False: Gate B. True has no window detector input yet (says so, uses Gate B)
    flight_limit_s: float = 170.0     # land this long after takeoff, whatever is happening (run limit 180 s)
    pose_jump_m: float = 0.5          # PX4 position moving more than this between two ticks latches a fault
    # inputs older than these are invalid
    line_stale_s: float = 0.5
    pad_stale_s: float = 0.5
    clear_stale_s: float = 0.5
    vision_stale_s: float = 0.5
    height_stale_s: float = 0.5
    future_tol_s: float = 0.05
    # motion
    speed_mps: float = 0.5
    slow_mps: float = 0.3
    stop_m: float = 0.9               # the carrot never comes nearer than this to observed depth ahead
    min_room_m: float = 0.1           # less forward room than this is blocked (a hold, not a crawl)
    lead_max_m: float = 1.0
    lat_lead_m: float = 0.3           # sideways / backward carrot lead (the forward camera does not see there)
    max_lat_mps: float = 0.2
    yaw_rate_dps: float = 45.0
    # holds and timeouts
    hold_max_s: float = 5.0
    fallback_hover_s: float = 5.0
    confirm_n: int = 3                # consecutive samples that confirm a detection
    follow_line_timeout_s: float = 60.0
    gate_a_timeout_s: float = 15.0
    tunnel_align_timeout_s: float = 20.0
    tunnel_cross_timeout_s: float = 15.0
    tunnel_exit_timeout_s: float = 10.0
    chicane_timeout_s: float = 40.0
    find_pad_timeout_s: float = 25.0
    land_timeout_s: float = 1e9       # core lands on the first tick of this stage
    # stage settings
    line_k_lat: float = 0.8           # lateral speed per metre of line offset (1/s)
    turn_in_place_deg: float = 30.0   # a line / pad bearing beyond this: rotate before moving
    gate_pass_m: float = 1.2          # distance past the line's end that counts as through Gate A
    centre_k: float = 0.5             # gate centring: lateral speed per metre of left/right imbalance
    side_near_m: float = 1.6          # both side probes nearer than this: an opening framed by the barrier
    standoff_m: float = 1.2           # the barrier this near ahead: at the mouth (> stop_m)
    square_tol_m: float = 0.1
    square_k: float = 0.5             # rad of yaw per metre of left/right difference
    mouth_hover_s: float = 3.0
    tunnel_len_m: float = 2.5         # never travel further than this inside the tunnel stage
    tunnel_min_m: float = 0.5
    exit_clear_m: float = 1.5         # observed depth this far ahead after a dark stretch: out of the tunnel
    exit_settle_s: float = 1.0
    chicane_turn_m: float = 1.5       # ahead nearer than this (or unknown): turn toward the more open side
    chicane_open_m: float = 1.8       # once turning, ahead must be this open to go again (hysteresis)
    chicane_turn_dir: int = -1        # -1 left, +1 right: when neither side is more open
    pad_alpha: float = 0.5            # smoothing of the pad's position on the floor
    pad_centre_m: float = 0.15
    pad_centre_s: float = 1.0
    face_min_m: float = 0.5           # nearer than this to the pad: stop turning to face it


@dataclass
class Inputs:
    """The latest detector outputs. Each group carries the capture stamp (mission clock) of the data
    it came from. Level body frame: forward, right; angles clockwise (NED yaw) positive."""
    line_t: float | None = None
    line_valid: bool = False
    line_offset: float = math.nan     # m, the line's lateral position from the body, + = right
    line_angle: float = math.nan      # rad, the line's direction from the body heading, + = right
    line_end: bool = False            # the line's end is seen
    pad_t: float | None = None
    pad_valid: bool = False
    pad_offset: tuple | None = None   # (forward, right) m from the body to the pad centre
    clear_t: float | None = None
    clearances: object = None         # refnav.depth.Clearances (core fills it from each usable frame)
    vision_t: float | None = None
    vision_ok: bool = False           # visual odometry tracking on features (not IMU only)
    height_t: float | None = None
    height: float | None = None       # m, body above the floor (downward rangefinder)

    def set_line(self, t, valid, offset=math.nan, angle=math.nan, end=False):
        self.line_t, self.line_valid, self.line_end = t, bool(valid), bool(end)
        self.line_offset, self.line_angle = float(offset), float(angle)

    def set_pad(self, t, valid, offset=None):
        self.pad_t, self.pad_valid, self.pad_offset = t, bool(valid), offset

    def set_clearances(self, t, clearances):
        self.clear_t, self.clearances = t, clearances

    def set_vision(self, t, ok):
        self.vision_t, self.vision_ok = t, ok is True

    def set_height(self, t, height):
        self.height_t, self.height = t, height


@dataclass
class Act:
    """One stage decision. fwd/right m/s in the body frame, or goal = an (n, e) to fly to; yaw = a
    heading to turn to (None keeps it). stall = a hold waiting on an input (lands after hold_max_s)."""
    reason: str
    fwd: float = 0.0
    right: float = 0.0
    goal: tuple | None = None
    yaw: float | None = None
    stall: bool = False
    exit: str | None = None
    land: bool = False
    blind_m: float = 0.0              # may move this far on space observed free earlier (ahead unknown now)

    def moves(self):
        return self.fwd != 0.0 or self.right != 0.0 or self.goal is not None


@dataclass
class Decision:
    stage: str
    reason: str
    setpoint: tuple                   # (n, e, d, yaw)
    land: bool = False
    events: list = field(default_factory=list)


@dataclass
class _View:
    inp: Inputs
    line_ok: bool
    line_state: str
    pad_ok: bool
    clr: object
    clr_state: str
    vision_ok: bool
    vision_state: str
    height: float | None
    height_state: str


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _clamp(x, lim):
    return max(-lim, min(lim, x))


def _fin(x):
    return x is not None and math.isfinite(x)


def _along(pos, start, heading):
    return (pos[0] - start[0]) * math.cos(heading) + (pos[1] - start[1]) * math.sin(heading)


class Course:
    def __init__(self, params: CourseParams | None = None):
        self.p = params or CourseParams()
        self.stage = None
        self.t_stage = None
        self.sp = None                # [n, e, d, yaw]
        self.reason = ""
        self.log = []                 # [(t, stage, reason)] at every change
        self.takeoff_t = None
        self.fault = None             # latched: nothing resumes this flight
        self.limit_hit = False
        self._landing = None
        self._last_pos = None
        self._fallback_since = None
        self._hold_since = None
        self._moving = False
        self._seen = {}               # the last stamp counted, per input group
        self._new_clr = False
        self._line_end_n = 0
        self._pad_n = 0
        self._pad_ned = None          # the pad on the floor, NED (n, e): a world-position goal
        self._free = None             # (n, e, cos, sin): observed free space ahead reached this point
        self._st = {}                 # the current stage's own state

    # ------------------------------------------------------------------ core's calls
    def start(self, now, sp):
        """After the climb: sp = core's hold setpoint (n, e, d, yaw)."""
        self.sp = [float(c) for c in sp]
        self._enter("follow_line", now)

    def supervise(self, now, pos):
        """Every airborne tick, the climb included (the first call is takeoff). -> a reason to land on
        the tick a stop is first raised, else None. A pose jump also latches self.fault."""
        p = self.p
        if self.takeoff_t is None:
            self.takeoff_t = now
        if self.fault is None and pos is not None:
            if self._last_pos is not None:
                jump = math.dist(pos, self._last_pos)
                if jump > p.pose_jump_m:
                    self.fault = f"pose jump {jump:.2f} m in one tick"
                    self._pad_ned = self._free = None             # drop every world-position goal
                    self._st = {}
                    if self.sp is not None:
                        self.sp[:3] = [float(c) for c in pos]
                    why = self.fault + ": world goals dropped, landing"
                    self._landing = why
                    self._note(now, why, [])
                    return why
            self._last_pos = tuple(pos)
        if self.fault is None and not self.limit_hit and now - self.takeoff_t >= p.flight_limit_s:
            self.limit_hit = True
            why = f"flight limit {p.flight_limit_s:g} s after takeoff, landing"
            self._landing = why
            self._note(now, why, [])
            return why
        return None

    def floor_down(self, now, inp):
        """The rangefinder height if valid (refnav.depth.four_clearances' floor_down_m), else None."""
        ok = self._state(inp.height_t, now, self.p.height_stale_s, _fin(inp.height) and inp.height > 0)
        return float(inp.height) if ok == "ok" else None

    def step(self, now, dt, pos, heading, inp) -> Decision:
        p = self.p
        v = self._view(now, inp)
        self._observe(now, pos, heading, v)
        stage = self.stage
        act = self._decide(now, pos, heading, v)
        room = 0.0
        if act.moves() and not act.land:
            room, why = self._room(v, act)
            if not room:
                act = Act(why, yaw=act.yaw, stall=True)           # rotation is still allowed
        if act.stall and not act.land:
            self._hold_since = now if self._hold_since is None else self._hold_since
            if now - self._hold_since >= p.hold_max_s:
                act = Act(f"{act.reason.removesuffix(', holding')} for {p.hold_max_s:g} s, landing", land=True)
        else:
            self._hold_since = None
        if act.land:
            self._landing = act.reason
            self._halt(pos)
        elif act.moves():
            self._move(pos, heading, dt, act, room)
            self._moving = True
        else:
            self._halt(pos)
        if act.yaw is not None and not act.land:
            err = _wrap(act.yaw - self.sp[3])
            lim = math.radians(p.yaw_rate_dps) * dt
            self.sp[3] = _wrap(self.sp[3] + _clamp(err, lim))
        events = []
        self._note(now, act.reason, events)
        if act.exit:
            self._enter(act.exit, now)
        return Decision(stage, act.reason, tuple(self.sp), act.land, events)

    def status(self, now):
        return {"stage": self.stage, "reason": self.reason, "fault": self.fault,
                "stage_s": None if self.t_stage is None else round(now - self.t_stage, 1),
                "fallback": self._fallback_since is not None,
                "flight_s": None if self.takeoff_t is None else round(now - self.takeoff_t, 1),
                "pad_ned": None if self._pad_ned is None else [round(c, 2) for c in self._pad_ned]}

    # ------------------------------------------------------------------ bookkeeping
    def _enter(self, stage, now):
        self.stage, self.t_stage, self._st = stage, now, {}
        self._fallback_since = self._hold_since = None

    def _note(self, now, reason, events):
        if reason != self.reason or not self.log or self.log[-1][1] != self.stage:
            self.log.append((now, self.stage, reason))
            events.append(f"{self.stage}: {reason}")
        self.reason = reason

    def _state(self, t, now, stale_s, valid):
        if t is None or not math.isfinite(t):
            return "missing"
        if t > now + self.p.future_tol_s:
            return "from the future"
        if now - t > stale_s:
            return "stale"
        return "ok" if valid else "lost"

    def _view(self, now, inp):
        p = self.p
        line = self._state(inp.line_t, now, p.line_stale_s,
                           inp.line_valid and _fin(inp.line_offset) and _fin(inp.line_angle))
        po = inp.pad_offset
        pad = self._state(inp.pad_t, now, p.pad_stale_s,
                          inp.pad_valid and po is not None and len(po) == 2 and all(_fin(c) for c in po))
        clr = self._state(inp.clear_t, now, p.clear_stale_s, inp.clearances is not None)
        vis = self._state(inp.vision_t, now, p.vision_stale_s, inp.vision_ok is True)
        hgt = self._state(inp.height_t, now, p.height_stale_s, _fin(inp.height) and inp.height > 0)
        return _View(inp, line == "ok", line, pad == "ok", inp.clearances if clr == "ok" else None, clr,
                     vis == "ok", vis, float(inp.height) if hgt == "ok" else None, hgt)

    def _observe(self, now, pos, heading, v):
        """Count each detector sample once (by its stamp), however many ticks it is seen for."""
        inp, p = v.inp, self.p
        c, s = math.cos(heading), math.sin(heading)
        if inp.line_t is not None and inp.line_t != self._seen.get("line"):
            self._seen["line"] = inp.line_t
            self._line_end_n = self._line_end_n + 1 if (v.line_ok and inp.line_end) else 0
        if inp.pad_t is not None and inp.pad_t != self._seen.get("pad"):
            self._seen["pad"] = inp.pad_t
            if v.pad_ok and self.fault is None:
                f, r = inp.pad_offset
                pn, pe = pos[0] + c * f - s * r, pos[1] + s * f + c * r
                a = p.pad_alpha
                old = self._pad_ned
                self._pad_ned = (pn, pe) if old is None else (a * pn + (1 - a) * old[0], a * pe + (1 - a) * old[1])
                self._pad_n += 1
            else:
                self._pad_n = 0
        self._new_clr = v.clr is not None and inp.clear_t != self._seen.get("clr")
        if self._new_clr:
            self._seen["clr"] = inp.clear_t
            a = v.clr.ahead
            if a.dist_m is not None:
                free = self._free_m(a)
                self._free = (pos[0] + c * free, pos[1] + s * free, c, s)

    @staticmethod
    def _free_m(probe):
        """How far ahead a probe OBSERVED free space: to the obstacle, or (clear) to where it was judged."""
        d = probe.dist_m
        if math.isinf(d):
            return probe.at_m if _fin(probe.at_m) else 0.0
        return d

    def _blind_room(self, pos):
        if self._free is None:
            return 0.0
        n, e, c, s = self._free
        return max(0.0, (n - pos[0]) * c + (e - pos[1]) * s - self.p.stop_m)

    def _room(self, v, act):
        """Forward room for the carrot from the latest clearances; (0, why) when it may not translate."""
        c = v.clr
        if c is not None and c.ahead.dist_m is not None:
            room = self._free_m(c.ahead) - self.p.stop_m
            return (room, "") if room > self.p.min_room_m else (0.0, "obstacle ahead, holding")
        if act.blind_m > self.p.min_room_m:
            return act.blind_m, ""
        return 0.0, ("ahead unknown, holding" if c is not None else f"depth clearances {v.clr_state}, holding")

    def _halt(self, pos):
        if self._moving:                              # stop where it is, not where the carrot was
            self.sp[0], self.sp[1] = float(pos[0]), float(pos[1])
        self._moving = False

    def _move(self, pos, heading, dt, act, room):
        p = self.p
        c, s = math.cos(heading), math.sin(heading)
        n, e = self.sp[0], self.sp[1]
        if act.goal is not None:
            dn, de = act.goal[0] - n, act.goal[1] - e
            dist = math.hypot(dn, de)
            k = min(1.0, p.slow_mps * dt / dist) if dist > 1e-9 else 0.0
            n, e = n + dn * k, e + de * k
        else:
            n += (c * act.fwd - s * act.right) * dt
            e += (s * act.fwd + c * act.right) * dt
        f = c * (n - pos[0]) + s * (e - pos[1])       # the carrot in the body frame, bounded
        r = -s * (n - pos[0]) + c * (e - pos[1])
        f = max(-p.lat_lead_m, min(f, p.lead_max_m, room))
        r = _clamp(r, p.lat_lead_m)
        self.sp[0], self.sp[1] = pos[0] + c * f - s * r, pos[1] + s * f + c * r

    # ------------------------------------------------------------------ decisions
    def _decide(self, now, pos, heading, v):
        p = self.p
        if self.fault is not None or self._landing is not None:
            return Act(self._landing or self.fault, land=True)
        if not v.vision_ok:
            return Act(f"vision {v.vision_state}, holding", stall=True)
        fn = getattr(self, "_" + self.stage)
        if self._fallback_since is not None:
            a = fn(now, pos, heading, v)
            if a.exit or a.land:
                return Act(f"{a.reason} (during the fallback hover)", exit=a.exit, land=a.land)
            if now - self._fallback_since >= p.fallback_hover_s:
                return Act(f"{self.stage} timed out; hovered {p.fallback_hover_s:g} s, landing", land=True)
            return Act(f"{self.stage} timed out, hovering")
        limit = getattr(p, f"{self.stage}_timeout_s")
        if now - self.t_stage > limit:
            self._fallback_since = now
            return Act(f"{self.stage} timed out after {limit:g} s, hovering")
        return fn(now, pos, heading, v)

    def _centre(self, c):
        """Lateral speed toward the middle of an opening from the side probes (+ = right)."""
        if c is None:
            return 0.0
        l, r, p = c.left.dist_m, c.right.dist_m, self.p
        if _fin(l) and _fin(r):
            return _clamp(p.centre_k * (r - l), p.max_lat_mps)
        if _fin(l) and l < p.side_near_m:
            return p.max_lat_mps / 2
        if _fin(r) and r < p.side_near_m:
            return -p.max_lat_mps / 2
        return 0.0

    def _follow_line(self, now, pos, heading, v):
        p = self.p
        if not v.line_ok:
            return Act(f"line {v.line_state}, holding", stall=True)
        if self._line_end_n >= p.confirm_n:
            return Act("line end seen", exit="gate_a")
        yaw = heading + v.inp.line_angle
        if abs(v.inp.line_angle) > math.radians(p.turn_in_place_deg):
            return Act("turning onto the line", yaw=yaw)
        return Act("following the line", fwd=p.speed_mps, yaw=yaw,
                   right=_clamp(p.line_k_lat * v.inp.line_offset, p.max_lat_mps))

    def _gate_a(self, now, pos, heading, v):
        st, p = self._st, self.p
        if "start" not in st:                         # the line's end, and its direction
            st["start"], st["heading"] = (pos[0], pos[1]), heading
        if _along(pos, st["start"], st["heading"]) >= p.gate_pass_m:
            return Act("through gate A", exit="tunnel_align")
        return Act("crossing gate A", fwd=p.slow_mps, right=self._centre(v.clr), yaw=st["heading"])

    def _tunnel_align(self, now, pos, heading, v):
        st, p = self._st, self.p
        st.setdefault("heading", heading)
        st.setdefault("n", 0)
        if "aligned_t" in st:                         # tunnel disabled: hover at the mouth, then land
            if now - st["aligned_t"] >= p.mouth_hover_s:
                return Act("tunnel disabled: landing at the tunnel mouth", land=True)
            return Act("tunnel disabled: hovering at the tunnel mouth")
        c = v.clr
        if c is None:
            return Act(f"depth clearances {v.clr_state}, holding", stall=True)
        a, l, r = c.ahead.dist_m, c.left.dist_m, c.right.dist_m
        facing = _fin(a) and a <= p.standoff_m                       # the barrier face ahead
        framed = _fin(l) and _fin(r) and max(l, r) <= p.side_near_m   # an opening framed by the barrier
        if not (facing or framed):
            return Act("approaching the barrier", fwd=p.slow_mps, yaw=st["heading"])
        if _fin(l) and _fin(r) and abs(l - r) > p.square_tol_m:
            st["n"] = 0                                # turn toward the nearer side until square
            return Act("squaring up to the barrier", yaw=heading + _clamp(p.square_k * (l - r), math.radians(10)))
        st["n"] += self._new_clr
        if st["n"] < p.confirm_n:
            return Act("at the tunnel mouth, settling")
        if p.tunnel_enabled:
            return Act("aligned at the tunnel mouth", exit="tunnel_cross")
        st["aligned_t"] = now
        return Act("tunnel disabled: hovering at the tunnel mouth")

    def _tunnel_cross(self, now, pos, heading, v):
        st, p = self._st, self.p
        if "start" not in st:
            st["start"], st["heading"], st["blind"] = (pos[0], pos[1]), heading, False
        if v.height is None:
            return Act(f"height {v.height_state}, holding", stall=True)
        gone = _along(pos, st["start"], st["heading"])
        a = v.clr.ahead.dist_m if v.clr is not None else None
        st["blind"] = st["blind"] or a is None
        if gone >= p.tunnel_len_m:
            return Act("tunnel length crossed", exit="tunnel_exit")
        if st["blind"] and gone >= p.tunnel_min_m and a is not None and a >= p.exit_clear_m:
            return Act("depth back after the dark stretch", exit="tunnel_exit")
        return Act("crossing the tunnel", fwd=p.slow_mps, yaw=st["heading"], blind_m=self._blind_room(pos))

    def _tunnel_exit(self, now, pos, heading, v):
        st, p = self._st, self.p                       # vision is already checked before every stage
        if v.clr is None or v.clr.ahead.dist_m is None:
            st.pop("ok_since", None)
            return Act("out of the tunnel, waiting for depth")
        st.setdefault("ok_since", now)
        if now - st["ok_since"] >= p.exit_settle_s:
            return Act("vision and depth back after the tunnel", exit="chicane")
        return Act("out of the tunnel, settling")

    def _chicane(self, now, pos, heading, v):
        st, p = self._st, self.p
        if self._pad_n >= p.confirm_n:
            return Act("pad in view", exit="find_pad")
        pre = "no window detector input, gate B: " if p.window_enabled else "gate B: "
        c = v.clr
        if c is None:
            return Act(f"depth clearances {v.clr_state}, holding", stall=True)
        a = c.ahead.dist_m
        if a is not None and a >= (p.chicane_open_m if st.get("turning") else p.chicane_turn_m):
            st["turning"] = False
            return Act(pre + "open ahead", fwd=p.speed_mps)
        st["turning"] = True

        def score(x):                                 # unknown < near < far < observed clear
            return -1.0 if x is None else (1e9 if math.isinf(x) else x)
        sl, sr = score(c.left.dist_m), score(c.right.dist_m)
        side = -1 if sl > sr + 0.2 else 1 if sr > sl + 0.2 else st.get("dir", p.chicane_turn_dir)
        st["dir"] = side
        return Act(pre + f"blocked ahead, turning {'right' if side > 0 else 'left'}",
                   yaw=heading + side * math.radians(30))

    def _find_pad(self, now, pos, heading, v):
        st, p = self._st, self.p
        if self._pad_ned is None:
            return Act("pad position unknown, holding", stall=True)
        dn, de = self._pad_ned[0] - pos[0], self._pad_ned[1] - pos[1]
        dist = math.hypot(dn, de)
        if dist <= p.pad_centre_m:
            st.setdefault("in_since", now)
            if now - st["in_since"] >= p.pad_centre_s:
                return Act("centred over the pad", exit="land")
            return Act("over the pad, settling")
        st.pop("in_since", None)
        bearing = math.atan2(de, dn)
        if dist > p.face_min_m and abs(_wrap(bearing - heading)) > math.radians(p.turn_in_place_deg):
            return Act("turning to the pad", yaw=bearing)
        return Act("flying to the pad", goal=self._pad_ned, yaw=bearing if dist > p.face_min_m else None)

    def _land(self, now, pos, heading, v):
        return Act(COURSE_COMPLETE, land=True)
