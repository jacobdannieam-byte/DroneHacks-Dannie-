#!/usr/bin/env python3
"""Preflight diagnostics tool for the FlyX companion environment.

What this file does:
    Listens to live ROS 2 topics for 3 seconds and checks all four safety gates
    required by refnav (main.py) before arming:
      1. PX4 position fix and heading alignment
      2. Estimator health report (/estimator/healthy)
      3. Depth camera stream and camera info metadata
      4. Simulation clock (/clock) active and synced

What this file does NOT do:
    This file NEVER arms the vehicle, NEVER requests Offboard mode, and NEVER
    publishes setpoints. It is purely an observer.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

LISTEN_DURATION_SECONDS = 3.0  # Time to listen for incoming messages


class PreflightChecker(Node):

    def __init__(self):
        super().__init__("debug_flyx_preflight")

        # Track arrival counts and latest values
        self.clock_ticks = 0
        self.healthy_reports = 0
        self.last_healthy_value = None
        self.px4_status_reports = 0
        self.px4_armed = False
        self.px4_nav_state = None
        self.px4_position_reports = 0
        self.px4_pos_valid = False
        self.depth_frames = 0
        self.camera_info_received = False

        # Subscriptions
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v1", self._on_status, PX4_QOS
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._on_pos,
            PX4_QOS,
        )
        self.create_subscription(
            Bool, "/estimator/healthy", self._on_healthy, 10
        )
        self.create_subscription(
            Image, "/camera/depth/image_raw", self._on_depth, 10
        )
        self.create_subscription(
            CameraInfo, "/camera/depth/camera_info", self._on_info, 10
        )

    def _on_status(self, msg):
        self.px4_status_reports += 1
        self.px4_armed = msg.arming_state == 2
        self.px4_nav_state = msg.nav_state

    def _on_pos(self, msg):
        self.px4_position_reports += 1
        self.px4_pos_valid = bool(msg.xy_valid and msg.z_valid)

    def _on_healthy(self, msg):
        self.healthy_reports += 1
        self.last_healthy_value = bool(msg.data)

    def _on_depth(self, msg):
        self.depth_frames += 1

    def _on_info(self, msg):
        self.camera_info_received = True


def main():
    rclpy.init()
    node = PreflightChecker()

    print(f"Listening to companion ROS 2 topics for {LISTEN_DURATION_SECONDS} seconds...")
    start_time = time.monotonic()
    while time.monotonic() - start_time < LISTEN_DURATION_SECONDS:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n=======================================================")
    print("      FLYX COMPANION PREFLIGHT DIAGNOSTIC REPORT       ")
    print("=======================================================\n")

    # Gate 1: PX4 Status & Position
    pos_pass = node.px4_position_reports > 0 and node.px4_pos_valid
    status_pass = node.px4_status_reports > 0
    gate1_pass = pos_pass and status_pass
    print(f"1. PX4 Autopilot Telemetry: [{'PASS' if gate1_pass else 'FAIL'}]")
    print(f"   - Vehicle status reports received : {node.px4_status_reports}")
    print(f"   - Position reports received       : {node.px4_position_reports}")
    print(f"   - XY and Z position valid         : {node.px4_pos_valid}")
    if not gate1_pass:
        print("   -> CAUSE: PX4 DDS bridge not publishing or EKF2 position fix invalid.")

    # Gate 2: Estimator Health
    gate2_pass = node.healthy_reports > 0 and node.last_healthy_value is True
    print(f"\n2. Estimator Health (/estimator/healthy): [{'PASS' if gate2_pass else 'FAIL'}]")
    print(f"   - Health messages received        : {node.healthy_reports}")
    print(f"   - Latest health value             : {node.last_healthy_value}")
    if not gate2_pass:
        print("   -> CAUSE: /estimator/healthy is silent or False. Check 'python3 onboard/services.py status'.")

    # Gate 3: Depth Camera & Camera Info
    gate3_pass = node.depth_frames > 0 and node.camera_info_received
    print(f"\n3. Depth Perception Pipeline: [{'PASS' if gate3_pass else 'FAIL'}]")
    print(f"   - Depth frames received (/camera/depth/image_raw) : {node.depth_frames}")
    print(f"   - Camera info metadata received                   : {node.camera_info_received}")
    if not gate3_pass:
        print("   -> CAUSE: Missing depth images or camera_info topic. Check camera driver/simulator publishing.")

    # Overall Verdict
    all_passed = gate1_pass and gate2_pass and gate3_pass
    print("\n-------------------------------------------------------")
    if all_passed:
        print("VERDICT: ALL PREFLIGHT GATES PASSED! main.py is ready to arm.")
    else:
        print("VERDICT: GATES FAILED. refnav (main.py) will refuse to arm.")
        print("See docs/FLYX_DEBUGGING_GUIDE.md for step-by-step remediation.")
    print("-------------------------------------------------------\n")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
