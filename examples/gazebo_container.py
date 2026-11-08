"""Preflight, read-only evidence, and optional reviewed flight inside the isolated Gazebo image."""

import argparse
from collections import deque
from datetime import datetime, timezone
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import re
import socket
import sys
from time import monotonic

import numpy as np

from examples.gazebo_sensor_contract import OPTICAL_TO_FLU, SIM_BINDINGS, SIM_ESTIMATOR, read_camera_rig
from examples.h1_observation import PX4_TYPES, TF, TRACKING
from examples.hybrid_launch import configured, run_reviewed_sitl
from examples.spatial_bag_input import TOPICS
from examples.gazebo_hybrid_adapter import IsolationApproval
from onboard.diagnostics.phase1_probe import px4_topic, payloads
from onboard.diagnostics.sitl_graph import inspect


BASE = Path(__file__).resolve().parents[1]
ROLE_TYPES = {**PX4_TYPES, "vehicle_attitude": "px4_msgs/msg/VehicleAttitude",
              "vehicle_status": "px4_msgs/msg/VehicleStatus",
              "vehicle_land_detected": "px4_msgs/msg/VehicleLandDetected"}


def sha(path):
    return sha256(Path(path).read_bytes()).hexdigest()


def isolated():
    domain = os.environ.get("ROS_DOMAIN_ID", "")
    return (Path("/.dockerenv").is_file() and {n for _, n in socket.if_nameindex()} == {"lo"}
            and not any(Path("/dev").glob("ttyUSB*")) and not any(Path("/dev").glob("ttyACM*"))
            and not Path("/dev/serial").exists() and os.environ.get("ROS_LOCALHOST_ONLY") == "1"
            and bool(re.fullmatch(r"[1-9][0-9]?", domain)))


def _json_value(value):
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: _json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    return value


def _ports():
    from rosidl_runtime_py.utilities import get_message
    return {role: px4_topic(role, get_message(kind)) for role, kind in ROLE_TYPES.items()}


def graph_topics():
    ports = _ports()
    topics = {**TOPICS, TF: "tf2_msgs/msg/TFMessage", TRACKING: "diagnostic_msgs/msg/DiagnosticArray"}
    return {**{SIM_BINDINGS.get(name, name): kind for name, kind in topics.items()},
            **{ports[role]: kind for role, kind in ROLE_TYPES.items()}}


def probe(seconds, output):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.utilities import try_shutdown
    from rosidl_runtime_py.utilities import get_message

    topics, ports = graph_topics(), _ports()
    last, counts = {}, {}
    px4_samples = deque(maxlen=96)
    first_clock = None
    duration = max(seconds, 4)
    rclpy.init(args=[])
    node = rclpy.create_node("gazebo_disarmed_probe", enable_rosout=False, start_parameter_services=False,
                             parameter_overrides=[Parameter("use_sim_time", value=True)])
    try:
        def cb(topic):
            def receive(msg):
                nonlocal first_clock
                now = node.get_clock().now().nanoseconds
                counts[topic] = counts.get(topic, 0) + 1
                if topic == "/clock":
                    last[topic] = {"sim_ns": msg.clock.sec * 10**9 + msg.clock.nanosec, "receipt_ns": now}
                    if first_clock is None:
                        first_clock = last[topic]["sim_ns"], monotonic()
                elif topic == SIM_BINDINGS["/camera/depth/camera_info"]:
                    last[topic] = {"frame": msg.header.frame_id, "width": msg.width, "height": msg.height,
                                   "k": list(msg.k), "p": list(msg.p), "d": list(msg.d), "receipt_ns": now}
                elif topic == SIM_BINDINGS["/camera/depth/image_raw"]:
                    last[topic] = {"frame": msg.header.frame_id, "encoding": msg.encoding, "step": msg.step,
                                   "width": msg.width, "height": msg.height, "receipt_ns": now,
                                   "capture_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec}
                elif topic in (ports["vehicle_status"], ports["vehicle_land_detected"],
                               ports["vehicle_local_position"], ports["vehicle_odometry"],
                               ports["estimator_status_flags"], "/fmu/in/vehicle_visual_odometry", TF):
                    key = {ports["vehicle_status"]: "px4_status", ports["vehicle_land_detected"]: "px4_landed",
                           ports["vehicle_local_position"]: "px4_local", ports["vehicle_odometry"]: "px4_odom",
                           ports["estimator_status_flags"]: "px4_flags",
                           "/fmu/in/vehicle_visual_odometry": "ev", TF: "tf_static"}[topic]
                    last[topic] = {"evidence": list(payloads(key, msg)), "receipt_ns": now,
                                   "px4_publish_us": getattr(msg, "timestamp", None)}
                    if topic == ports["vehicle_odometry"]:
                        last[topic].update(position=list(msg.position), q=list(msg.q),
                                           position_variance=list(msg.position_variance))
                        px4_samples.append(last[topic])
                elif topic == SIM_BINDINGS["/visual_slam/tracking/odometry"]:
                    p, q = msg.pose.pose.position, msg.pose.pose.orientation
                    last[topic] = {"header_frame": msg.header.frame_id, "body_frame": msg.child_frame_id,
                                   "position": [p.x, p.y, p.z], "q_xyzw": [q.x, q.y, q.z, q.w],
                                   "covariance": list(msg.pose.covariance), "receipt_ns": now,
                                   "capture_ns": msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec}
                elif topic == SIM_BINDINGS["/estimator/healthy"]:
                    last[topic] = {"healthy": bool(msg.data), "receipt_ns": now}
            return receive

        sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        static = QoSProfile(depth=16, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        for topic, kind in topics.items():
            node.create_subscription(get_message(kind), topic, cb(topic), static if topic == TF else sensor)
        start = monotonic()
        while rclpy.ok() and monotonic() - start < duration:
            rclpy.spin_once(node, timeout_sec=.05)
        graph = inspect(node, strict_motion=True)
        last_clock = last.get("/clock", {}).get("sim_ns")
        advancing = first_clock is not None and last_clock is not None and last_clock > first_clock[0]
        now = node.get_clock().now().nanoseconds

        def fresh(topic):
            row = last.get(topic, {})
            return row if 0 <= now - row.get("receipt_ns", -10**9) <= 300_000_000 else {}

        disarmed = fresh(ports["vehicle_status"]).get("evidence", [{}])[0].get("armed") is False
        landed = fresh(ports["vehicle_land_detected"]).get("evidence", [{}])[0].get("landed") is True
        healthy = fresh(SIM_BINDINGS["/estimator/healthy"]).get("healthy") is True
        missing = sorted(set(topics) - set(counts))
        evidence = {"tracking_source": SIM_ESTIMATOR, "graph": graph, "advancing_clock": advancing,
                    "ros_vs_wall_rtf": (last_clock - first_clock[0]) / 1e9 / (monotonic() - first_clock[1]) if advancing else None,
                    "px4_clock_samples": list(px4_samples),
                    "topics": {name: [{"node": item.node_name, "type": item.topic_type,
                                      "reliability": item.qos_profile.reliability.name,
                                      "durability": item.qos_profile.durability.name}
                                     for item in node.get_publishers_info_by_topic(name)] for name in topics},
                    "sample_counts": counts, "last_observations": last, "missing": missing,
                    "wall_seconds": monotonic() - start, "disarmed": disarmed,
                    "landed": landed, "healthy": healthy}
        with Path(output).open("x", encoding="utf-8") as file:
            json.dump(_json_value(evidence), file, indent=2, allow_nan=False)
            file.write("\n")
        return disarmed and landed and healthy and advancing and not graph["issues"] and not missing
    finally:
        node.destroy_node()
        try_shutdown()


def verify_profile(path, mode):
    strategy, runtime = configured(path)
    if strategy != mode or not runtime.adapter.simulated_estimator:
        raise ValueError("use the explicit Gazebo simulated-estimator profile for this mode")
    adapter = runtime.adapter
    rig = read_camera_rig(BASE / "simulator/gazebo_trial/models")
    h1 = runtime.loop.replay.hybrid
    cal = h1.spatial.calibration
    if ((cal.width, cal.height, cal.camera_frame, cal.body_frame) !=
            (rig.width, rig.height, "oak_depth_optical_frame", "base_link") or
            h1.spatial.reference.frame_id != "odom" or h1.spatial.reference.body_frame != "base_link" or
            cal.R_body_optical != OPTICAL_TO_FLU or
            any(abs(a - b) > 1e-6 for a, b in zip(cal.t_body_optical_m, rig.translation)) or
            h1.px4_topics != {role: _ports()[role] for role in PX4_TYPES}):
        raise ValueError("reviewed H1 profile differs from the imported rig or pinned PX4 messages")
    if mode == "hybrid_sitl" and adapter.owner_topics != {role: _ports()[role] for role in ROLE_TYPES.keys() - PX4_TYPES.keys()}:
        raise ValueError("owner PX4 schemas do not match the installed simulator messages")
    return runtime


def load_approval(path, profile, *, mode, world, revision):
    approval = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(approval, dict):
        raise ValueError("approval must be an object")
    expires = datetime.fromisoformat(approval["expires_utc"].replace("Z", "+00:00"))
    if (expires.tzinfo is None or not datetime.now(timezone.utc) < expires
            or approval["mode"] != mode or approval["revision"] != revision or approval["world_sha256"] != sha(world)
            or str(approval["ros_domain_id"]) != os.environ.get("ROS_DOMAIN_ID")
            or not isinstance(approval["reviewed_by"], str) or not approval["reviewed_by"].strip()
            or not isinstance(approval["trial_id"], str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", approval["trial_id"])):
        raise ValueError("no current approval for this isolated trial, source, world, and ROS domain")
    if mode == "hybrid_sitl":
        config = json.loads(Path(profile).read_text(encoding="utf-8"))
        inputs = {"launcher": profile, "h1": Path(profile).parent / config["h1_profile"],
                  "course": Path(profile).parent / config["mission_profile"]}
        if (approval["profiles"] != {key: sha(value) for key, value in inputs.items()}
                or not all(approval.get(key) is True for key in ("native_shadow_accepted", "fallback_reviewed",
                                                                 "isolation_reviewed", "course_prior_permitted"))):
            raise ValueError("approval does not cover the exact profiles and native acceptance gates")
    return approval


def request_land(trial_id):
    """Operator request to the current mission only. This creates no PX4 publisher."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.utilities import try_shutdown
    from std_msgs.msg import String

    approval = json.loads(Path("/evidence/trial_approval.json").read_text(encoding="utf-8"))
    if approval.get("mode") != "hybrid_sitl" or approval.get("trial_id") != trial_id:
        raise ValueError("landing request requires this container's exact active trial")
    rclpy.init(args=[])
    node = rclpy.create_node("gazebo_operator_land", enable_rosout=False, start_parameter_services=False)
    try:
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pub = node.create_publisher(String, "/gazebo_hybrid/control", qos)
        until = monotonic() + 4.
        while rclpy.ok() and monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
            if pub.get_subscription_count() == 1:
                if inspect(node, "/refnav", strict_motion=True)["issues"]:
                    raise ValueError("operator request refused: command owner is ambiguous")
                pub.publish(String(data=json.dumps({"run_id": trial_id, "request": "land", "why": "operator"})))
                rclpy.spin_once(node, timeout_sec=.3)
                print("Landing requested through RefNav; verify landed/disarmed in telemetry")
                return 0
        raise ValueError("operator request refused: expected mission subscriber was unavailable")
    finally:
        node.destroy_node()
        try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    preflight = commands.add_parser("probe")
    preflight.add_argument("--seconds", type=int, default=8, choices=range(4, 61))
    preflight.add_argument("--output", type=Path, required=True)
    commands.add_parser("topics")
    commands.add_parser("land").add_argument("--trial-id", required=True)
    for name in ("authorize", "fly"):
        p = commands.add_parser(name)
        p.add_argument("--mode", choices=("classical", "hybrid_sitl"), required=True)
        p.add_argument("--profile", type=Path)
        p.add_argument("--approval", type=Path, required=True)
        p.add_argument("--world", type=Path, required=True)
        p.add_argument("--revision", required=True)
        p.add_argument("--output", type=Path)
    p = commands.add_parser("verify-shadow")
    p.add_argument("--profile", type=Path, required=True)
    args = parser.parse_args()
    try:
        if not isolated():
            raise ValueError("Gazebo launcher requires a loopback-only container without hardware devices")
        if args.command == "land":
            return request_land(args.trial_id)
        if args.command == "topics":
            print(*sorted(graph_topics()), sep="\n")
            return 0
        if args.command == "probe":
            return int(not probe(args.seconds, args.output))
        if args.command == "verify-shadow":
            verify_profile(args.profile, "hybrid_shadow")
            return 0
        approval = load_approval(args.approval, args.profile, mode=args.mode, world=args.world, revision=args.revision)
        if args.command == "authorize":
            return 0
        if args.mode != "hybrid_sitl" or args.output is None:
            raise ValueError("classical uses the imported route runner")
        runtime = verify_profile(args.profile, args.mode)
        permit = IsolationApproval(runtime.adapter.profile_id, int(os.environ["ROS_DOMAIN_ID"]),
                                   approval["trial_id"], lambda *_: isolated())
        return run_reviewed_sitl(runtime, permit, output=args.output,
                                 ros_args=["refnav", "--ros-args", "-p", "use_sim_time:=true",
                                           "-p", "height_m:=0.8",
                                           "-p", "control_topic:=/gazebo_hybrid/control",
                                           "-p", f"run_id:={approval['trial_id']}"])
    except (ValueError, OSError, KeyError, RuntimeError, TypeError, AttributeError) as error:
        print(f"gazebo_container: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
