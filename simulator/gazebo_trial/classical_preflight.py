"""Observe the classical handoff inputs before allowing a simulator mission to start."""

from dataclasses import dataclass
import json
from pathlib import Path
import sys
from time import monotonic

from examples.gazebo_container import isolated


# Deadlines use elapsed time so the check can stop even if Gazebo pauses.
# The land and estimator reports from the pinned PX4 build are nominally 1 Hz.
MAX_RECEIVE_AGE_S = {"clock": .35, "radio": .35, "vision": .35, "position": .35,
                     "interface": .35, "status": 1., "land": 1.5, "estimator": 1.5, "failsafe": 1.}
COMMAND_OWNERS = {
    "/fmu/in/vehicle_command": {"flight_interface", "pretend_remote_control"},
    "/fmu/in/offboard_control_mode": {"flight_interface"},
    "/fmu/in/trajectory_setpoint": {"flight_interface"},
}


@dataclass(frozen=True)
class Sample:
    received_s: float
    source_us: int
    healthy: bool


def readiness(samples, now_s):
    issues = []
    for role, maximum_age in MAX_RECEIVE_AGE_S.items():
        sample = samples.get(role)
        if sample is None:
            issues.append(f"{role}: missing")
        elif now_s - sample.received_s > maximum_age:
            issues.append(f"{role}: stale")
        elif not sample.healthy or sample.source_us <= 0:
            issues.append(f"{role}: invalid")
    clock = samples.get("clock")
    if clock is not None:
        for role in ("vision", "radio", "position", "interface"):
            sample = samples.get(role)
            if sample is not None and not -.02 <= (clock.source_us - sample.source_us) / 1e6 <= .3:
                issues.append(f"{role}: stamp outside Gazebo clock window")
    return issues


def capture(node, observe, seconds):
    from px4_msgs.msg import EstimatorStatusFlags, FailsafeFlags, ManualControlSetpoint
    from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleOdometry, VehicleStatus
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String

    def interface(message):
        state = json.loads(message.data)
        healthy = (state["px4_link"] and state["landed"] is True and state["armed"] is False
                   and state["local_enu"] is not None and state["odom_enu"] is not None
                   and state["envelope_ok"] and not state["offboard_active"])
        return round(state["stamp"] * 1e6), healthy

    streams = {
        "clock": ("/clock", Clock, lambda m: (m.clock.sec * 10**6 + m.clock.nanosec // 1000, True)),
        "radio": ("/fmu/in/manual_control_input", ManualControlSetpoint, lambda m: (m.timestamp, m.valid)),
        "vision": ("/fmu/in/vehicle_visual_odometry", VehicleOdometry,
                   lambda m: (m.timestamp, m.pose_frame == VehicleOdometry.POSE_FRAME_NED)),
        "position": ("/fmu/out/vehicle_local_position", VehicleLocalPosition,
                     lambda m: (m.timestamp, m.xy_valid and m.z_valid and m.heading_good_for_control)),
        "interface": ("/skybeta/state", String, interface),
        "status": ("/fmu/out/vehicle_status_v1", VehicleStatus,
                   lambda m: (m.timestamp, m.arming_state == VehicleStatus.ARMING_STATE_DISARMED)),
        "land": ("/fmu/out/vehicle_land_detected", VehicleLandDetected, lambda m: (m.timestamp, m.landed)),
        "estimator": ("/fmu/out/estimator_status_flags", EstimatorStatusFlags,
                      lambda m: (m.timestamp, m.cs_yaw_align and m.cs_ev_pos and m.cs_ev_yaw
                                 and not m.cs_ev_yaw_fault)),
        "failsafe": ("/fmu/out/failsafe_flags", FailsafeFlags,
                     lambda m: (m.timestamp, not (m.manual_control_signal_lost or m.local_position_invalid))),
    }
    samples, counts, max_gap_s = {}, {}, {}
    faults = set()

    def receive(role, read):
        def update(message):
            stamp, valid = read(message)
            now = monotonic()
            if role in samples:
                previous = samples[role]
                max_gap_s[role] = max(max_gap_s.get(role, 0.), now - previous.received_s)
                if role == "clock" and stamp < previous.source_us:
                    faults.add("clock: rewound")
            if role == "status" and not valid:
                faults.add("status: not disarmed")
            samples[role] = Sample(now, stamp, valid)
            counts[role] = counts.get(role, 0) + 1
        return update

    for role, (topic, kind, read) in streams.items():
        node.create_subscription(kind, topic, receive(role, read), qos_profile_sensor_data)

    deadline = monotonic() + 180
    healthy_since = first_clock_us = None
    owners = {}
    issues = []
    next_check = 0.
    while rclpy.ok() and monotonic() < deadline and not faults:
        rclpy.spin_once(node, timeout_sec=.02)
        now = monotonic()
        if now < next_check:
            continue
        next_check = now + .02
        issues = readiness(samples, now)
        owners = {topic: {p.node_name for p in node.get_publishers_info_by_topic(topic)}
                  for topic in COMMAND_OWNERS}
        for topic, expected in COMMAND_OWNERS.items():
            if owners[topic] != (set() if observe else expected):
                issues.append(f"{topic}: unexpected command publishers")
        if issues:
            healthy_since = first_clock_us = None
        elif healthy_since is None:
            healthy_since, first_clock_us = now, samples["clock"].source_us
        elif now - healthy_since >= seconds:
            elapsed_s = now - healthy_since
            return {"passed": True, "continuous_healthy_s": elapsed_s,
                    "rtf": (samples["clock"].source_us - first_clock_us) / 1e6 / elapsed_s,
                    "counts": counts, "max_receive_gap_s": max_gap_s,
                    "command_publishers": {k: sorted(v) for k, v in owners.items()}}
    return {"passed": False, "issues": sorted(set(issues) | faults), "counts": counts,
            "max_receive_gap_s": max_gap_s,
            "command_publishers": {k: sorted(v) for k, v in owners.items()}}


def main():
    import rclpy
    from rclpy.utilities import try_shutdown

    if not isolated():
        raise SystemExit("Classical preflight requires the isolated simulator")
    observe, seconds, output = sys.argv[1:]
    rclpy.init(args=[])
    node = rclpy.create_node("classical_gazebo_preflight")
    try:
        report = capture(node, observe == "true", int(seconds))
    finally:
        node.destroy_node()
        try_shutdown()
    Path(output).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
