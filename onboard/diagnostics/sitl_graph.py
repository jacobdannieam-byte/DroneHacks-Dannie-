
"""Inspect ROS command endpoint ownership without creating application publishers."""

import argparse
from collections import Counter
import json
import re
from time import monotonic

COMMAND_TOPICS = {
    "/fmu/in/offboard_control_mode": "px4_msgs/msg/OffboardControlMode",
    "/fmu/in/trajectory_setpoint": "px4_msgs/msg/TrajectorySetpoint",
    "/fmu/in/vehicle_command": "px4_msgs/msg/VehicleCommand",
}


def _name(namespace, name):
    return namespace.rstrip("/") + "/" + name


def inspect(node, expected_owner=None, *, strict_motion=False):
    counts = Counter(_name(namespace, name) for name, namespace in node.get_node_names_and_namespaces())
    issues, endpoints = [], {}
    if expected_owner is not None and counts[expected_owner] != 1:
        issues.append("expected_owner_missing_or_ambiguous")
    command_types = set(COMMAND_TOPICS.values())
    alternates = {name for name, types in node.get_topic_names_and_types()
                  if name.startswith("/fmu/in/") and name not in COMMAND_TOPICS
                  and command_types.intersection(types)}
    if strict_motion:
        other_motion = {"manual_control_input", "goto_setpoint", "vehicle_attitude_setpoint",
                        "vehicle_rates_setpoint", "vehicle_thrust_setpoint", "vehicle_torque_setpoint",
                        "actuator_motors", "actuator_servos", "vehicle_command_mode_executor"}
        alternates.update(name for name, _ in node.get_topic_names_and_types()
                          if name.startswith("/fmu/in/") and
                          re.sub(r"_v[0-9]+$", "", name.split("/")[-1]) in other_motion)
    for topic in sorted(set(COMMAND_TOPICS) | alternates):
        message_type = COMMAND_TOPICS.get(topic)
        entries = sorted((_name(info.node_namespace, info.node_name), info.topic_type, bytes(info.endpoint_gid).hex())
                         for info in node.get_publishers_info_by_topic(topic))
        endpoints[topic] = [{"owner": owner, "type": kind, "gid": gid} for owner, kind, gid in entries]
        if expected_owner is None or topic in alternates:
            if entries:
                issues.append(f"unexpected_command_publisher:{topic}")
        elif len(entries) != 1 or entries[0][:2] != (expected_owner, message_type):
            issues.append(f"unexpected_command_ownership:{topic}")
    return {"expected_owner": expected_owner, "endpoints": endpoints, "issues": issues}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-owner", help="Full ROS node name. Default is an idle graph with zero publishers.")
    parser.add_argument("--discovery-seconds", type=int, choices=range(1, 16), default=4, metavar="1..15")
    args = parser.parse_args(argv)
    if args.expected_owner and not args.expected_owner.startswith("/"):
        parser.error("--expected-owner must be a full ROS node name")

    import rclpy

    rclpy.init(args=[])
    try:
        node = rclpy.create_node("dronehacks_sitl_graph_review", enable_rosout=False,
                                 start_parameter_services=False)
        try:
            deadline = monotonic() + args.discovery_seconds
            while monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=min(.2, max(0., deadline - monotonic())))
            report = inspect(node, args.expected_owner)
            print(json.dumps(report, indent=2))
            return int(bool(report["issues"]))
        finally:
            node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
