import json
import math
from types import SimpleNamespace as Message

import pytest

from onboard.diagnostics.phase1_health import COMMAND_TOPICS, NS, SCHEMA, analyze
from onboard.diagnostics.phase1_probe import main, payloads, px4_topic
from onboard.vio.frames import POSE_FRAME_FRD, POSE_FRAME_NED, odometry_to_px4, quat_from_yaw, yaw_of
from onboard.vio.validation import CaptureGate


def sample_capture(rtf=1.0):
    """Synthetic evidence, including low-rate PX4 health/landing status and a different boot clock."""
    rows = []
    frames = {"left": "oak_left", "right": "oak_right", "depth": "oak_depth", "imu": "oak_imu"}

    def add(stream, wall_ns, **kwargs):
        rows.append(dict(stream=stream, wall_ns=wall_ns, **kwargs))

    for name, frame in frames.items():
        add("tf_static", 0, frame="base_link", child_frame=frame, valid=True,
            translation_m=[0.075 if name == "right" else 0.0, 0.0, 0.0], rotation_xyzw=[0, 0, 0, 1])
        if name != "imu":
            add(name + "_info", 0, frame=frame, width=640, height=400, valid=True)

    for i in range(300):
        wall = i * 100_000_000
        stamp = 5 * NS + round(wall * rtf)
        add("clock", wall, stamp_ns=stamp, valid=True)
        for name, frame in frames.items():
            extra = {} if name == "imu" else {"width": 640, "height": 400}
            add(name, wall, stamp_ns=stamp, frame=frame, valid=True, **extra)
        add("processing", wall, stamp_ns=stamp, valid=True, result="visual",
            duration_ms=14.0, covariance_rejected=False)
        add("vio", wall, stamp_ns=stamp, frame="odom", child_frame="base_link", valid=True)
        add("refnav_odom", wall, stamp_ns=stamp, frame="odom", child_frame="base_link", valid=True)
        add("ev", wall, stamp_ns=(stamp // 1000) * 1000, pose_frame=2, velocity_frame=3, reset=[2], valid=True)
        add("px4_local", wall, stamp_ns=500 * NS + wall, reset=[0] * 5, valid=True, heading_good=True, speed_mps=0.0)
        add("px4_odom", wall, stamp_ns=500 * NS + wall, pose_frame=1, velocity_frame=1, reset=[0], valid=True)
        if i % 2 == 0:
            add("health", wall, valid=True)
        if i % 5 == 0:
            add("px4_status", wall, armed=False, nav_state=0)
            add("graph", wall, command_publishers={topic: [] for topic in COMMAND_TOPICS},
                ev_publishers=["/px4_vision_bridge"], source_counts={"clock": 1, "vio": 1, "health": 1})
        if i % 10 == 0:
            add("px4_flags", wall, valid=True, ev_position_intended=True, yaw_aligned=True,
                dead_reckoning=False, gnss_position_intended=False, fake_position=False)
            add("px4_landed", wall, landed=True)
    return {"schema": SCHEMA, "complete": True, "duration_ns": 30 * NS, "base_frame": "base_link",
            "window_label": "synthetic", "events": rows}


def test_settled_metadata_passes_without_treating_px4_time_as_ros_time():
    report = analyze(sample_capture())
    assert report["passed_smoke_check"], report["issues"]
    assert report["clock"]["rtf"] == pytest.approx(29.9 / 30)
    assert report["streams"]["left"]["rate_stamp_hz"] == pytest.approx(10.0)
    assert report["streams"]["px4_flags"]["rate_wall_hz"] == pytest.approx(1.0)
    assert report["streams"]["processing"]["old_capture_count"] == 0
    assert "age_to_latest_observed_clock_s" not in report["streams"]["px4_local"]
    assert report["external_vision"]["stamp_match_fraction"] == 1.0
    assert report["stereo"]["matched"] == 300
    assert report["static_tf"]["disconnected"] == []
    assert report["static_tf"]["stereo_baseline_m"] == pytest.approx(.075)
    assert report["processing"]["duration_ms"]["p95"] == 14.0
    assert report["px4"]["armed_states"] == [False]
    assert report["graph"]["command_nodes_seen"] == []


def test_slow_clock_fails_even_when_all_inputs_are_continuous():
    report = analyze(sample_capture(.53))
    assert report["clock"]["rtf"] == pytest.approx(.53, abs=.002)
    assert report["issues"] == ["clock:rtf_below_target_or_unknown"]


def test_clock_stall_is_visible_even_with_continuous_clock_messages():
    capture = sample_capture()
    clock = [row for row in capture["events"] if row["stream"] == "clock"]
    for row in clock[-10:]:
        row["stamp_ns"] = clock[-11]["stamp_ns"]
    report = analyze(capture)
    assert "clock:stalled" in report["issues"]
    assert report["streams"]["clock"]["max_receive_gap_s"] == .1


def test_clock_rewind_prevents_publishing_a_misleading_rtf():
    capture = sample_capture()
    for row in capture["events"]:
        if row["stream"] == "clock" and row["wall_ns"] >= 15 * NS:
            row["stamp_ns"] -= 10 * NS
    report = analyze(capture)
    assert report["clock"]["rtf"] is None
    assert "clock:stamp_discontinuity" in report["issues"]


def test_missing_stale_and_out_of_order_sources_are_reported_separately():
    capture = sample_capture()
    capture["events"] = [row for row in capture["events"] if row["stream"] != "imu"]
    for row in capture["events"]:
        if row["stream"] == "depth" and 10 * NS <= row["wall_ns"] < 11 * NS:
            row["stamp_ns"] -= 2 * NS
        if row["stream"] == "vio" and row["wall_ns"] == 5 * NS:
            row["frame"] = "other_odom"
    report = analyze(capture)
    assert {"imu:missing", "depth:stamp_discontinuity", "depth:capture_clock_skew",
            "vio:frame_changed"} <= set(report["issues"])


def test_stereo_calibration_transform_and_bridge_stamp_mismatches_fail_closed():
    capture = sample_capture()
    capture["events"] = [row for row in capture["events"]
                         if not (row["stream"] == "tf_static" and row["child_frame"] == "oak_right")]
    for row in capture["events"]:
        if row["stream"] == "right":
            row["stamp_ns"] += 2_000_000
        if row["stream"] == "left_info":
            row["height"] = 480
        if row["stream"] == "ev":
            row["stamp_ns"] += 5 * NS
    report = analyze(capture)
    assert {"left:camera_info_mismatch", "tf_static:sensor_frame_disconnected",
            "stereo:pair_fraction_below_target", "ev:vio_sample_stamp_mismatch"} <= set(report["issues"])


def test_invalid_tracking_and_estimator_resets_are_reported():
    capture = sample_capture()
    for row in capture["events"]:
        if row["stream"] == "px4_local" and row["wall_ns"] >= 14 * NS:
            row["reset"] = [0, 0, 0, 0, 1]
        if row["stream"] == "health" and row["wall_ns"] == 14 * NS:
            row["valid"] = False
        if row["stream"] == "px4_flags" and row["wall_ns"] == 14 * NS:
            row.update(valid=False, ev_position_intended=False, dead_reckoning=True)
        if row["stream"] == "processing" and row["wall_ns"] == 14 * NS:
            row["covariance_rejected"] = True
    report = analyze(capture)
    assert {"px4_local:reference_reset", "health:invalid_or_unhealthy", "px4_flags:invalid_or_unhealthy",
            "processing:tracking_or_covariance_failure"} <= set(report["issues"])
    assert report["px4"]["dead_reckoning_count"] == 1


def test_static_rig_rejects_coincident_cameras():
    capture = sample_capture()
    right = next(row for row in capture["events"] if row.get("child_frame") == "oak_right")
    right["translation_m"] = [0, 0, 0]
    report = analyze(capture)
    assert report["static_tf"]["stereo_baseline_m"] == 0
    assert "tf_static:degenerate_stereo_baseline" in report["issues"]


def test_different_nodes_across_command_topics_and_multiple_vision_sources_fail():
    capture = sample_capture()
    graph = next(row for row in capture["events"] if row["stream"] == "graph")
    graph["command_publishers"][COMMAND_TOPICS[0]] = ["/mission"]
    graph["command_publishers"][COMMAND_TOPICS[1]] = ["/native_example"]
    graph["ev_publishers"].append("/competing_bridge")
    graph["source_counts"]["vio"] = 2
    issues = analyze(capture)["issues"]
    assert {"graph:multiple_command_owners", "graph:vision_owner_count", "graph:vio_publisher_count"} <= set(issues)


def test_incomplete_capture_and_invalid_event_order():
    capture = sample_capture()
    capture["complete"] = False
    assert "capture:incomplete" in analyze(capture)["issues"]
    capture["events"][-1]["wall_ns"] = -1
    with pytest.raises(ValueError, match="monotonic receive time"):
        analyze(capture)


def test_processing_decoder_pins_report_fields_and_rejects_inconsistent_stamps():
    header = Message(stamp=Message(sec=1, nanosec=500), frame_id="odom")
    frame = Message(name="visual_slam/frame", message="inertial", values=[
        Message(key="capture_ns", value=str(NS + 500)), Message(key="duration_ms", value="12.5"),
    ])
    row, = payloads("processing", Message(header=header, status=[frame]))
    assert row["valid"] and row["duration_ms"] == 12.5 and row["result"] == "inertial"
    frame.values[0].value = "4"
    row, = payloads("processing", Message(header=header, status=[frame]))
    assert not row["valid"] and row["duration_ms"] is None


def test_image_decoder_checks_layout_and_drops_payload():
    header = Message(stamp=Message(sec=1, nanosec=0), frame_id="oak_left")
    image = Message(header=header, encoding="mono8", width=2, height=2, step=4, data=bytes(8))
    row, = payloads("left", image)
    assert row["valid"] and "data" not in row
    image.data = bytes(5)
    row, = payloads("left", image)
    assert not row["valid"]


def test_ev_frame_and_native_topic_contract():
    args = dict(stamp_sample_us=900, now_us=950, position=(1, 2, 3),
                q_wxyz=quat_from_yaw(0), velocity=(1, 2, 3))
    unknown = odometry_to_px4(**args, world="flu")
    aligned = odometry_to_px4(**args, world="flu", heading_enu_rad=0)
    assert unknown.pose_frame == POSE_FRAME_FRD and unknown.position == (1, -2, -3)
    assert aligned.pose_frame == POSE_FRAME_NED and aligned.position == (2, 1, -3)
    assert yaw_of(aligned.q) == pytest.approx(math.pi / 2)
    assert aligned.velocity == (1, -2, -3)
    row, = payloads("ev", aligned)
    assert row["stamp_ns"] == 900_000 and row["publish_stamp_us"] == 950
    assert px4_topic("vehicle_local_position", Message(MESSAGE_VERSION=1)) == "/fmu/out/vehicle_local_position_v1"


def test_existing_capture_gate_rejects_stale_and_future_on_shared_clock():
    gate = CaptureGate()
    assert gate.check(1_000_000, 1_020_000)[:2] == (True, "ok")
    assert gate.check(1_001_000, 1_400_000)[:2] == (False, "stale")
    assert gate.check(1_030_000, 1_020_000)[:2] == (False, "future")
    gate.reset()
    assert gate.check(100_000, 101_000)[:2] == (True, "ok")


@pytest.mark.parametrize(("rtf", "code"), [(1.0, 0), (.53, 1)])
def test_replay_cli_produces_machine_readable_report(tmp_path, rtf, code):
    source = tmp_path / "capture.json"
    output = tmp_path / "report.json"
    source.write_text(json.dumps(sample_capture(rtf)), encoding="utf-8")
    assert main(["replay", str(source), "--output", str(output)]) == code
    assert json.loads(output.read_text())["passed_smoke_check"] == (code == 0)
