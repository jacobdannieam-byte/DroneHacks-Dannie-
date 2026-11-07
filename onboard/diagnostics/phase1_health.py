"""Analyze metadata from a read-only ROS 2 simulator observation."""

from bisect import bisect_right
from collections import Counter, defaultdict, deque
from dataclasses import dataclass
import math

from onboard.slam.stereo_pairing import add_half
from onboard.vio.frames import qconj, qmul, qnormalize, qrotate, quat_xyzw_to_wxyz

NS = 1_000_000_000
SCHEMA = "dronehacks-phase1-capture/1"
MAX_EVENTS = 250_000
COMMAND_TOPICS = ("/fmu/in/offboard_control_mode", "/fmu/in/trajectory_setpoint", "/fmu/in/vehicle_command")
ROS_CAPTURE = ("left", "right", "imu", "depth", "processing", "vio")
CAMERAS = ("left", "right", "depth")
CONTINUOUS = ("clock", *ROS_CAPTURE, "ev", "health", "px4_local", "px4_flags",
              "px4_status", "px4_landed", "px4_odom", "refnav_odom", "graph")
REQUIRED = (*CONTINUOUS, "left_info", "right_info", "depth_info", "tf_static")
SLOW_WALL_GAPS_S = {"graph": 1.0, "px4_status": 1.0, "px4_flags": 1.5, "px4_landed": 1.5}


@dataclass(frozen=True)
class Policy:
    """Provisional observation thresholds, independent of flight gating."""
    min_rtf: float = 0.95
    min_wall_s: float = 30.0
    max_wall_gap_s: float = 0.5
    max_capture_age_s: float = 0.3
    future_clock_skew_s: float = 0.05
    min_stereo_fraction: float = 0.90
    min_visual_fraction: float = 0.90
    min_ev_match_fraction: float = 0.90

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (
                self.min_rtf, self.min_wall_s, self.max_wall_gap_s, self.max_capture_age_s,
                self.future_clock_skew_s)):
            raise ValueError("time and RTF thresholds must be positive and finite")
        if not all(0 < v <= 1 for v in (
                self.min_stereo_fraction, self.min_visual_fraction, self.min_ev_match_fraction)):
            raise ValueError("fraction thresholds must be in (0, 1]")


def _quantiles(values):
    ordered = sorted(values)
    if not ordered:
        return None
    return {label: ordered[math.ceil(len(ordered) * q) - 1]
            for label, q in (("p50", .5), ("p95", .95), ("p99", .99), ("max", 1.0))}


def _interval_summary(values):
    if len(values) < 2:
        return None
    elapsed = (values[-1] - values[0]) / NS
    return (len(values) - 1) / elapsed if elapsed > 0 else None


def _stream(rows, end_ns):
    walls = [r["wall_ns"] for r in rows]
    stamps = [r["stamp_ns"] for r in rows if "stamp_ns" in r]
    gaps = [b - a for a, b in zip((0, *walls), (*walls, end_ns))]
    deltas = [b - a for a, b in zip(stamps, stamps[1:])]
    result = {
        "count": len(rows), "rate_wall_hz": _interval_summary(walls),
        "rate_stamp_hz": _interval_summary(stamps),
        "max_receive_gap_s": max(gaps) / NS,
        "invalid_count": sum(r.get("valid") is False for r in rows),
        "stamp_reversals": sum(delta < 0 for delta in deltas),
        "stamp_duplicates": sum(delta == 0 for delta in deltas),
    }
    for field in ("frame", "child_frame", "pose_frame", "velocity_frame"):
        values = {r[field] for r in rows if field in r}
        if values:
            result[field + "s"] = sorted(values)
    if any("reset" in r for r in rows):
        counters = [tuple(r["reset"]) for r in rows if "reset" in r]
        result["reset_changes"] = sum(a != b for a, b in zip(counters, counters[1:]))
    return result


def _clock(rows, end_ns):
    stamps = [r["stamp_ns"] for r in rows]
    walls = [r["wall_ns"] for r in rows]
    progress = [wall for wall, stamp, previous in zip(walls[1:], stamps[1:], stamps) if stamp > previous]
    stall = max(b - a for a, b in zip((0, *progress), (*progress, end_ns))) / NS
    if len(stamps) < 2 or any(b < a for a, b in zip(stamps, stamps[1:])):
        return {"rtf": None, "one_second_rtf_min": None, "max_progress_gap_s": stall}
    indices = [bisect_right(walls, t) - 1 for t in range(0, end_ns + 1, NS)]
    factors = [(stamps[b] - stamps[a]) / NS for a, b in zip(indices, indices[1:]) if a >= 0]
    return {"rtf": (stamps[-1] - stamps[0]) / end_ns, "one_second_rtf_min": min(factors, default=None),
            "max_progress_gap_s": stall}


def _stereo(events, grouped):
    pending = {}
    pairs = dropped = 0
    for event in events:
        if event["stream"] in ("left", "right") and event.get("valid") and event["stamp_ns"] > 0:
            pair, discarded = add_half(pending, event["stamp_ns"], event["stream"][0], True)
            pairs += pair is not None
            dropped += discarded
    denominator = max(len(grouped["left"]), len(grouped["right"]))
    return {"matched": pairs, "fraction": pairs / denominator if denominator else 0.0,
            "evicted_unmatched": dropped, "pending_unmatched": len(pending)}


def _static_tf(rows, base, frames, stereo_frames, issues):
    parents = {}
    edges = defaultdict(dict)
    for row in rows:
        parent, child = row["frame"], row["child_frame"]
        if not row.get("valid") or not parent or not child or parent == child:
            issues.add("tf_static:invalid")
            continue
        edge = (parent, tuple(row["translation_m"]), tuple(row["rotation_xyzw"]))
        previous = parents.setdefault(child, edge)
        same_rotation = previous[2] in (edge[2], tuple(-v for v in edge[2]))
        if previous[:2] != edge[:2] or not same_rotation:
            issues.add("tf_static:conflicting_child")
        position, rotation = edge[1], qnormalize(quat_xyzw_to_wxyz(*edge[2]))
        inverse = qconj(rotation)
        edges[parent][child] = (position, rotation)
        edges[child][parent] = (qrotate(inverse, tuple(-value for value in position)), inverse)
    reached = {base: ((0., 0., 0.), (1., 0., 0., 0.))}
    frontier = deque([base])
    while frontier:
        frame = frontier.popleft()
        origin, orientation = reached[frame]
        for next_frame in sorted(edges[frame].keys() - reached.keys()):
            position, rotation = edges[frame][next_frame]
            reached[next_frame] = (tuple(a + b for a, b in zip(origin, qrotate(orientation, position))),
                                   qmul(orientation, rotation))
            frontier.append(next_frame)
    missing = sorted(frames - reached.keys())
    if missing:
        issues.add("tf_static:sensor_frame_disconnected")
    # A static rig is a tree. A component with at least as many undirected
    # edges as vertices has a loop. A reciprocal edge is invalid too.
    if (sum(len(edges[frame]) for frame in reached) // 2 >= len(reached)
            or any(parents.get(parent, (None,))[0] == child for child, (parent, *_) in parents.items())):
        issues.add("tf_static:cycle")
    baseline = None
    graph_invalid = bool(issues.intersection({"tf_static:invalid", "tf_static:conflicting_child", "tf_static:cycle"}))
    if not graph_invalid and len(stereo_frames) == 2 and all(frame in reached for frame in stereo_frames):
        left, right = stereo_frames
        baseline = math.dist(reached[left][0], reached[right][0])
        if baseline <= 1e-6:
            issues.add("tf_static:degenerate_stereo_baseline")
    return {"base_frame": base, "sensor_frames": sorted(frames), "disconnected": missing,
            "stereo_baseline_m": baseline,
            "transforms": [{"parent": edge[0], "child": child, "translation_m": edge[1], "rotation_xyzw": edge[2]}
                           for child, edge in sorted(parents.items())]}


def _vision(grouped):
    vio = {r["stamp_ns"] // 1000 for r in grouped["vio"] if r.get("valid") and r["stamp_ns"] > 0}
    ev = [r["stamp_ns"] // 1000 for r in grouped["ev"] if r.get("valid") and r["stamp_ns"] > 0]
    # Boundary samples may miss their corresponding VIO record. Include them
    # in the denominator so clock domains with little overlap cannot pass.
    lo, hi = min(vio, default=0), max(vio, default=0)
    eligible = [stamp for stamp in ev if lo <= stamp <= hi]
    matched = sum(stamp in vio for stamp in eligible)
    return {"matched_sample_stamps": matched, "ev_samples": len(ev), "ev_in_vio_capture_span": len(eligible),
            "stamp_match_fraction": matched / len(ev) if ev else 0.0}


def _owners(rows, issues):
    maximum = dict.fromkeys(COMMAND_TOPICS, 0)
    ev_counts = []
    seen = set()
    sources = {name: [] for name in ("clock", "vio", "health")}
    for row in rows:
        commands = row["command_publishers"]
        ev_counts.append(len(row["ev_publishers"]))
        for name, counts in sources.items():
            counts.append(row["source_counts"][name])
        active = set()
        for topic in COMMAND_TOPICS:
            publishers = commands[topic]
            maximum[topic] = max(maximum[topic], len(publishers))
            active.update(publishers)
        seen.update(active)
        if len(active) > 1 or any(len(commands[topic]) > 1 for topic in COMMAND_TOPICS):
            issues.add("graph:multiple_command_owners")
    if ev_counts and (ev_counts[-1] != 1 or max(ev_counts) > 1):
        issues.add("graph:vision_owner_count")
    if len(seen) > 1:
        issues.add("graph:command_owner_changed")
    for name, counts in sources.items():
        if counts and (counts[-1] != 1 or max(counts) > 1):
            issues.add(f"graph:{name}_publisher_count")
    return {"max_command_publishers": maximum, "command_nodes_seen": sorted(seen),
            "ev_publisher_counts": ev_counts, "source_publisher_counts": sources}


def analyze(capture, policy=Policy()):
    """Report observer evidence. A passing smoke window is never a flight qualification."""
    if not isinstance(capture, dict) or capture.get("schema") != SCHEMA:
        raise ValueError("unsupported Phase 1 capture schema")
    end_ns = capture["duration_ns"]
    if type(end_ns) is not int or not 0 < end_ns <= 130 * NS:
        raise ValueError("capture wall duration is outside the supported range")
    events = capture["events"]
    if not isinstance(events, list) or len(events) > MAX_EVENTS:
        raise ValueError("capture events must be a list within the event budget")
    grouped = {name: [] for name in REQUIRED}
    ages = defaultdict(list)
    latest_clock = None
    previous_wall = 0
    for row in events:
        wall, name = row["wall_ns"], row["stream"]
        if type(wall) is not int or not previous_wall <= wall <= end_ns or name not in grouped:
            raise ValueError("invalid stream or monotonic receive time in capture")
        previous_wall = wall
        if name in (*ROS_CAPTURE, "clock", "ev", "px4_local", "px4_odom", "refnav_odom"):
            if type(row.get("stamp_ns")) is not int:
                raise ValueError(f"missing integer stamp for {name}")
        grouped[name].append(row)
        if name == "clock":
            latest_clock = row["stamp_ns"]
        elif latest_clock is not None and name in (*ROS_CAPTURE, "refnav_odom"):
            ages[name].append((latest_clock - row["stamp_ns"]) / NS)

    issues = set()
    if capture.get("complete") is not True:
        issues.add("capture:incomplete")
    if end_ns / NS < policy.min_wall_s:
        issues.add("capture:too_short")
    streams = {name: _stream(rows, end_ns) for name, rows in grouped.items()}
    for name, stat in streams.items():
        if not stat["count"]:
            issues.add(f"{name}:missing")
        elif name in CONTINUOUS and stat["max_receive_gap_s"] > SLOW_WALL_GAPS_S.get(name, policy.max_wall_gap_s):
            issues.add(f"{name}:receive_gap")
        if stat["invalid_count"]:
            issues.add(f"{name}:invalid_or_unhealthy")
        if stat["stamp_reversals"] or (name in (*ROS_CAPTURE, "ev") and stat["stamp_duplicates"]):
            issues.add(f"{name}:stamp_discontinuity")
        if name in (*ROS_CAPTURE, "ev", "px4_local", "px4_odom", "refnav_odom") and any(
                row["stamp_ns"] <= 0 for row in grouped[name]):
            issues.add(f"{name}:invalid_stamp")
        if name != "tf_static" and any(
                len(stat.get(field, [])) > 1 for field in ("frames", "child_frames", "pose_frames", "velocity_frames")):
            issues.add(f"{name}:frame_changed")
        if stat.get("reset_changes"):
            issues.add(f"{name}:reference_reset")

    clock = _clock(grouped["clock"], end_ns)
    if clock["rtf"] is None or clock["rtf"] < policy.min_rtf:
        issues.add("clock:rtf_below_target_or_unknown")
    if clock["max_progress_gap_s"] > policy.max_wall_gap_s:
        issues.add("clock:stalled")
    for name in (*ROS_CAPTURE, "refnav_odom"):
        stat, values = streams[name], ages[name]
        stat["age_to_latest_observed_clock_s"] = _quantiles(values)
        stat["ahead_of_latest_clock_count"] = sum(a < -policy.future_clock_skew_s for a in values)
        stat["old_capture_count"] = sum(a > policy.max_capture_age_s for a in values)
        if stat["count"] and not values:
            issues.add(f"{name}:unmapped_ros_capture_clock")
        if stat["ahead_of_latest_clock_count"] or stat["old_capture_count"]:
            issues.add(f"{name}:capture_clock_skew")

    frames = set(streams["imu"].get("frames", []))
    for name in CAMERAS:
        image, info = grouped[name], grouped[f"{name}_info"]
        frames.update(row["frame"] for row in image)
        observed = {(r["frame"], r["width"], r["height"]) for r in image}
        calibrated = {(r["frame"], r["width"], r["height"]) for r in info if r.get("valid")}
        if len(observed) > 1 or len(calibrated) > 1 or (observed and observed != calibrated):
            issues.add(f"{name}:camera_info_mismatch")
    base_frame = capture["base_frame"]
    for name in ("vio", "refnav_odom"):
        if grouped[name] and streams[name].get("child_frames") != [base_frame]:
            issues.add(f"{name}:unexpected_child_frame")
    stereo_frames = [grouped[name][0]["frame"] for name in ("left", "right") if grouped[name]]
    static = _static_tf(grouped["tf_static"], base_frame, frames, stereo_frames, issues)
    stereo = _stereo(events, grouped)
    if stereo["fraction"] < policy.min_stereo_fraction:
        issues.add("stereo:pair_fraction_below_target")

    processing = grouped["processing"]
    results = Counter(row["result"] for row in processing)
    visual_fraction = results["visual"] / len(processing) if processing else 0.0
    if visual_fraction < policy.min_visual_fraction:
        issues.add("processing:visual_fraction_below_target")
    if results["error"] or results["exception"] or any(row["covariance_rejected"] for row in processing):
        issues.add("processing:tracking_or_covariance_failure")
    vision = _vision(grouped)
    if vision["stamp_match_fraction"] < policy.min_ev_match_fraction:
        issues.add("ev:vio_sample_stamp_mismatch")
    graph = _owners(grouped["graph"], issues)
    return {
        "schema": "dronehacks-phase1-report/1", "passed_smoke_check": not issues, "issues": sorted(issues),
        "duration_wall_s": end_ns / NS, "window_label": capture["window_label"], "host": capture.get("host"),
        "topics": capture.get("topics"),
        "thresholds": {**policy.__dict__, "slow_stream_wall_gaps_s": SLOW_WALL_GAPS_S}, "streams": streams,
        "clock": clock, "static_tf": static, "stereo": stereo, "external_vision": vision,
        "processing": {"outcomes": dict(results), "visual_fraction": visual_fraction,
                       "duration_ms": _quantiles([r["duration_ms"] for r in processing if r["duration_ms"] is not None])},
        "px4": {"armed_states": sorted({r["armed"] for r in grouped["px4_status"]}),
                "nav_states": sorted({r["nav_state"] for r in grouped["px4_status"]}),
                "landed_states": sorted({r["landed"] for r in grouped["px4_landed"]}),
                "ev_position_intended_count": sum(r["ev_position_intended"] for r in grouped["px4_flags"]),
                "dead_reckoning_count": sum(r["dead_reckoning"] for r in grouped["px4_flags"]),
                "gnss_position_intended_count": sum(r["gnss_position_intended"] for r in grouped["px4_flags"]),
                "max_observed_speed_mps": max((r["speed_mps"] for r in grouped["px4_local"]
                                               if r["speed_mps"] is not None), default=None),
                "heading_good_for_control_count": sum(r["heading_good"] for r in grouped["px4_local"])},
        "graph": graph,
    }
