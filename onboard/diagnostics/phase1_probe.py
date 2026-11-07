"""Capture and check a native DroneHacks ROS 2 graph without commanding it."""

import argparse
import json
import math
import os
from pathlib import Path
import platform
import time

from onboard.diagnostics.phase1_health import COMMAND_TOPICS, MAX_EVENTS, NS, Policy, SCHEMA, analyze
from onboard.diagnostics.sitl_graph import inspect as inspect_command_graph
from onboard.vio.frames import POSE_FRAME_FRD, POSE_FRAME_NED, VELOCITY_FRAME_NED
from onboard.vio.validation import sample_problem

MAX_CAPTURE_BYTES = 160_000_000
RESET_FIELDS = ("xy_reset_counter", "z_reset_counter", "vxy_reset_counter", "vz_reset_counter", "heading_reset_counter")


def _ns(stamp):
    return int(stamp.sec) * NS + int(stamp.nanosec)


def _xyz(vector):
    return vector.x, vector.y, vector.z


def _quaternion(rotation):
    return rotation.x, rotation.y, rotation.z, rotation.w


def _finite(values):
    return all(math.isfinite(value) for value in values)


def _pose_ok(position, rotation):
    return sample_problem({"position", "orientation"}, position, rotation, ()) is None


def _header(message):
    return {"stamp_ns": _ns(message.header.stamp), "frame": message.header.frame_id}


def _image(stream, message):
    encoding = message.encoding.lower()
    sizes = {"16uc1": 2, "32fc1": 4} if stream == "depth" else {"mono8": 1, "8uc1": 1}
    valid = (encoding in sizes and message.width > 0 and message.height > 0
             and message.step >= sizes[encoding] * message.width and len(message.data) == message.height * message.step
             and bool(message.header.frame_id))
    return {**_header(message), "width": message.width, "height": message.height,
            "encoding": encoding, "valid": valid}


def _info(stream, message):
    projected = stream != "depth_info" and any(message.p)
    matrix = message.p if projected else message.k
    fx, fy = matrix[0], matrix[5 if projected else 4]
    valid = (_finite(matrix) and fx > 0 and fy > 0 and message.width > 0 and message.height > 0
             and bool(message.header.frame_id))
    return {"frame": message.header.frame_id, "width": message.width, "height": message.height,
            "focal_px": [v if math.isfinite(v) else None for v in (fx, fy)], "valid": valid}


def _processing(message):
    frame = next((status for status in message.status if status.name == "visual_slam/frame"), None)
    details = {item.key: item.value for item in frame.values} if frame else {}
    stamp = _ns(message.header.stamp)
    try:
        duration = float(details["duration_ms"])
        valid = math.isfinite(duration) and duration >= 0 and int(details["capture_ns"]) == stamp
    except (KeyError, ValueError):
        valid, duration = False, None
    return {"stamp_ns": stamp, "result": frame.message if frame else "unknown", "valid": valid,
            "covariance_rejected": "covariance_rejected" in details,
            "duration_ms": duration if valid else None}


def _px4_odometry(stream, message):
    position_ok = _pose_ok(message.position, message.q)
    if stream == "ev":
        frame_ok = message.pose_frame in (POSE_FRAME_NED, POSE_FRAME_FRD)
    else:
        frame_ok = (message.pose_frame == POSE_FRAME_NED and message.velocity_frame == VELOCITY_FRAME_NED
                    and _finite(message.velocity))
    return {"stamp_ns": int(message.timestamp_sample) * 1000, "publish_stamp_us": int(message.timestamp),
            "pose_frame": int(message.pose_frame), "velocity_frame": int(message.velocity_frame),
            "reset": [int(message.reset_counter)], "valid": bool(position_ok and frame_ok)}


def _px4_local(message):
    flags = [bool(getattr(message, field)) for field in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid")]
    values = [getattr(message, field) for field in ("x", "y", "z", "vx", "vy", "vz", "heading")]
    counters = [getattr(message, field, None) for field in RESET_FIELDS]
    valid = all(flags) and _finite(values) and all(counter is not None for counter in counters)
    return {"stamp_ns": int(message.timestamp_sample) * 1000, "publish_stamp_us": int(message.timestamp),
            "valid": valid, "reset": counters, "heading_good": bool(message.heading_good_for_control),
            "speed_mps": math.hypot(*values[3:6]) if _finite(values[3:6]) else None}


def _tf_rows(message):
    for transform in message.transforms:
        translation = _xyz(transform.transform.translation)
        rotation = _quaternion(transform.transform.rotation)
        valid = _finite(translation) and _finite(rotation) and abs(math.hypot(*rotation) - 1) <= .01
        yield {"frame": transform.header.frame_id, "child_frame": transform.child_frame_id, "valid": valid,
               "translation_m": translation if valid else None, "rotation_xyzw": rotation if valid else None}


def payloads(stream, message):
    """Normalize only the fields needed by the health report. Image pixels stay out."""
    if stream == "tf_static":
        yield from _tf_rows(message)
        return
    if stream == "clock":
        stamp = _ns(message.clock)
        value = {"stamp_ns": stamp, "valid": stamp >= 0}
    elif stream in ("left", "right", "depth"):
        value = _image(stream, message)
    elif stream.endswith("_info"):
        value = _info(stream, message)
    elif stream == "imu":
        valid = (bool(message.header.frame_id) and _finite(_xyz(message.linear_acceleration))
                 and _finite(_xyz(message.angular_velocity)))
        value = {**_header(message), "valid": valid}
    elif stream == "processing":
        value = _processing(message)
    elif stream in ("vio", "refnav_odom"):
        pose = message.pose.pose
        value = {**_header(message), "child_frame": message.child_frame_id,
                 "valid": bool(message.header.frame_id and message.child_frame_id
                               and _pose_ok(_xyz(pose.position), _quaternion(pose.orientation)))}
    elif stream in ("ev", "px4_odom"):
        value = _px4_odometry(stream, message)
    elif stream == "px4_local":
        value = _px4_local(message)
    elif stream == "health":
        value = {"valid": bool(message.data)}
    elif stream == "px4_flags":
        ev_intended = bool(getattr(message, "cs_ev_pos", False))
        yaw = bool(getattr(message, "cs_yaw_align", False))
        dead_reckoning = bool(getattr(message, "cs_inertial_dead_reckoning", False))
        gnss = bool(getattr(message, "cs_gnss_pos", False))
        fake_position = bool(getattr(message, "cs_fake_pos", False))
        value = {"valid": ev_intended and yaw and not (dead_reckoning or gnss or fake_position),
                 "ev_position_intended": ev_intended, "yaw_aligned": yaw,
                 "dead_reckoning": dead_reckoning, "gnss_position_intended": gnss,
                 "fake_position": fake_position}
    elif stream == "px4_status":
        value = {"armed": message.arming_state == message.ARMING_STATE_ARMED, "nav_state": int(message.nav_state)}
    elif stream == "px4_landed":
        value = {"landed": bool(message.landed)}
    else:
        raise ValueError(f"unknown capture stream {stream}")
    yield value


def px4_topic(name, message_type):
    version = int(getattr(message_type, "MESSAGE_VERSION", 0))
    return f"/fmu/out/{name}" + (f"_v{version}" if version else "")


def _topic_map(overrides):
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import EstimatorStatusFlags, VehicleLandDetected, VehicleLocalPosition, VehicleOdometry, VehicleStatus
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import Bool
    from tf2_msgs.msg import TFMessage

    topics = {
        "clock": ("/clock", Clock),
        "left": ("/stereo/left/image", Image), "right": ("/stereo/right/image", Image),
        "left_info": ("/stereo/left/camera_info", CameraInfo), "right_info": ("/stereo/right/camera_info", CameraInfo),
        "imu": ("/stereo/imu", Imu), "depth": ("/camera/depth/image_raw", Image),
        "depth_info": ("/camera/depth/camera_info", CameraInfo),
        "processing": ("/visual_slam/processing_status", DiagnosticArray),
        "vio": ("/visual_slam/tracking/odometry", Odometry),
        "ev": ("/fmu/in/vehicle_visual_odometry", VehicleOdometry), "health": ("/estimator/healthy", Bool),
        "px4_local": (px4_topic("vehicle_local_position", VehicleLocalPosition), VehicleLocalPosition),
        "px4_flags": (px4_topic("estimator_status_flags", EstimatorStatusFlags), EstimatorStatusFlags),
        "px4_status": (px4_topic("vehicle_status", VehicleStatus), VehicleStatus),
        "px4_landed": (px4_topic("vehicle_land_detected", VehicleLandDetected), VehicleLandDetected),
        "px4_odom": (px4_topic("vehicle_odometry", VehicleOdometry), VehicleOdometry),
        "refnav_odom": ("/refnav/odom", Odometry), "tf_static": ("/tf_static", TFMessage),
    }
    overridden = set()
    for item in overrides:
        stream, sep, name = item.partition("=")
        if not sep or stream not in topics or not name.startswith("/") or stream in overridden:
            raise ValueError(f"invalid or repeated topic override {item!r}, expected stream=/topic")
        if name in COMMAND_TOPICS:
            raise ValueError("command topics are observed through graph discovery only")
        overridden.add(stream)
        topics[stream] = (name, topics[stream][1])
    if len({name for name, _ in topics.values()}) != len(topics):
        raise ValueError("capture stream topics must be distinct")
    return topics


def capture_ros(seconds, base_frame, window_label, overrides):
    """Observe the existing graph for a bounded wall window. Create subscriptions only."""
    import rclpy
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    topics = _topic_map(overrides)
    rclpy.init(args=[])
    node = executor = None
    events = []
    complete = False
    try:
        node = rclpy.create_node("dronehacks_phase1_probe", enable_rosout=False, start_parameter_services=False)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        sensor = QoSProfile(depth=64, reliability=ReliabilityPolicy.BEST_EFFORT)
        static = QoSProfile(depth=64, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        def add(stream, data, wall):
            if len(events) < MAX_EVENTS:
                events.append({"stream": stream, "wall_ns": wall, **data})

        def callback(stream):
            def observe(message):
                wall = time.monotonic_ns() - start
                for data in payloads(stream, message):
                    add(stream, data, wall)
            return observe

        def publishers(topic):
            return sorted(f"{item.node_namespace.rstrip('/')}/{item.node_name}"
                          for item in node.get_publishers_info_by_topic(topic))

        def graph():
            endpoints = inspect_command_graph(node)["endpoints"]
            data = {"command_publishers": {topic: [item["owner"] for item in endpoints[topic]]
                                           for topic in COMMAND_TOPICS},
                    "alternate_command_topics": sorted(topic for topic, items in endpoints.items()
                                                       if topic not in COMMAND_TOPICS and items),
                    "ev_publishers": publishers(topics["ev"][0]),
                    "source_counts": {stream: len(publishers(topics[stream][0]))
                                      for stream in ("clock", "vio", "health")}}
            add("graph", data, time.monotonic_ns() - start)

        for stream, (name, message_type) in topics.items():
            node.create_subscription(message_type, name, callback(stream), static if stream == "tf_static" else sensor)
        start = time.monotonic_ns()
        deadline = start + int(seconds * NS)
        next_graph = start
        try:
            while rclpy.ok() and len(events) < MAX_EVENTS and time.monotonic_ns() < deadline:
                now = time.monotonic_ns()
                if now >= next_graph:
                    graph()
                    next_graph = now + NS // 2
                executor.spin_once(timeout_sec=max(0.0, min(.05, (deadline - time.monotonic_ns()) / NS)))
            complete = rclpy.ok() and len(events) < MAX_EVENTS and time.monotonic_ns() >= deadline
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        end = time.monotonic_ns() - start
        return {"schema": SCHEMA, "complete": complete, "duration_ns": end, "base_frame": base_frame,
                "window_label": window_label, "events": events,
                "topics": {stream: name for stream, (name, _) in topics.items()},
                "discovered_topics": dict(node.get_topic_names_and_types()) if rclpy.ok() else None,
                "host": {"platform": platform.platform(), "machine": platform.machine(),
                         "ros_distro": os.environ.get("ROS_DISTRO"), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                         "rmw": os.environ.get("RMW_IMPLEMENTATION")}}
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def _write(path, payload):
    with path.open("x", encoding="utf-8") as stream:
        json.dump(payload, stream, allow_nan=False, separators=(",", ":") if path.name == "capture.json" else None,
                  indent=None if path.name == "capture.json" else 2)
        stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    live = commands.add_parser("capture", help="observe an already running ROS 2 simulator")
    live.add_argument("--output", type=Path, required=True, help="new evidence directory")
    live.add_argument("--seconds", type=int, default=30, choices=range(30, 121), metavar="30..120")
    live.add_argument("--window", default="settled", help="label the observed interval")
    live.add_argument("--base-frame", default="base_link")
    live.add_argument("--topic", action="append", default=[], metavar="STREAM=/TOPIC")
    replay = commands.add_parser("replay", help="check saved observation metadata without ROS")
    replay.add_argument("input", type=Path)
    replay.add_argument("--output", type=Path, help="new report file, otherwise print JSON")
    for command in (live, replay):
        command.add_argument("--min-rtf", type=float, default=Policy().min_rtf)
        command.add_argument("--require-idle", action="store_true",
                             help="require landed/disarmed reports and no PX4 command publishers throughout")
    args = parser.parse_args(argv)
    try:
        policy = Policy(min_rtf=args.min_rtf, require_idle=args.require_idle)
        if args.command == "capture":
            if args.output.exists():
                raise ValueError("capture output directory already exists")
            if not args.base_frame or not args.window:
                raise ValueError("base frame and capture window label must be nonempty")
            record = capture_ros(args.seconds, args.base_frame, args.window, args.topic)
            args.output.mkdir(parents=True)
            _write(args.output / "capture.json", record)
            report = analyze(record, policy)
            _write(args.output / "report.json", report)
            print(f"Phase 1 smoke check: {'PASS' if report['passed_smoke_check'] else 'NO-GO'}")
            print(f"Evidence: {args.output}")
        else:
            if args.input.stat().st_size > MAX_CAPTURE_BYTES:
                raise ValueError("capture exceeds the file size budget")
            record = json.loads(args.input.read_text(encoding="utf-8"))
            report = analyze(record, policy)
            if args.output:
                _write(args.output, report)
            else:
                print(json.dumps(report, indent=2, allow_nan=False))
    except ImportError as error:
        parser.exit(2, f"ROS 2/PX4 runtime unavailable: {error}\n")
    except (OSError, ValueError, TypeError, KeyError) as error:
        parser.exit(2, f"Phase 1 observation failed: {error}\n")
    return 0 if report["passed_smoke_check"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
