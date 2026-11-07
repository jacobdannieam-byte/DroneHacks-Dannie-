"""Gazebo-only ROS observations. No control, raw truth subscription or synthesized VIO claim."""

from pathlib import Path
import sys

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from gz.msgs10.clock_pb2 import Clock as GazeboClock
from gz.msgs10.image_pb2 import R_FLOAT32
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from px4_msgs.msg import EstimatorStatusFlags
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import try_shutdown
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from examples.gazebo_sensor_contract import (
    OPTICAL_FRAME, OPTICAL_TO_FLU, SIM_ESTIMATOR, SIM_HEALTH, SIM_ODOM, SIM_PROCESSING, SIM_STATUS,
    TRACKING_MAX_AGE_NS, capture_ns, depth_layout, read_camera_rig, ros_clock_parts, simulated_health,
)
from onboard.adapters.geometry import mat_to_quat
from onboard.diagnostics.phase1_probe import payloads, px4_topic
from simulator.gazebo_trial.gazebo_camera_to_ros import GazeboCameraToRos


class HybridSensorBridge(GazeboCameraToRos):
    def __init__(self):
        self.rig = read_camera_rig(Path(__file__).parent / "models")
        if sys.byteorder != "little":
            raise RuntimeError("this Gazebo camera bridge requires a little-endian host")
        super().__init__()
        self.sample = self.status = self.flags = None
        self.clock_pub = self.create_publisher(Clock, "/clock", qos_profile_sensor_data)
        self.health_pub = self.create_publisher(Bool, SIM_HEALTH, 10)
        self.processing = self.create_publisher(DiagnosticArray, SIM_PROCESSING, qos_profile_sensor_data)
        if not self.gazebo_node.subscribe(GazeboClock, "/world/dronehacks_course/clock", self.on_clock):
            raise RuntimeError("Gazebo world clock subscription failed")
        self.create_subscription(Odometry, SIM_ODOM, self.on_odometry, qos_profile_sensor_data)
        self.create_subscription(VisualSlamStatus, SIM_STATUS, self.on_status, qos_profile_sensor_data)
        self.create_subscription(EstimatorStatusFlags, px4_topic("estimator_status_flags", EstimatorStatusFlags),
                                 self.on_flags, qos_profile_sensor_data)
        self.create_timer(.05, self.on_health)
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = "base_link", OPTICAL_FRAME
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = (
            self.rig.translation)
        q = mat_to_quat(OPTICAL_TO_FLU)
        rotation = transform.transform.rotation
        rotation.x, rotation.y, rotation.z, rotation.w = map(float, q)
        self.tf.sendTransform(transform)

    def on_gazebo_depth_picture(self, picture):
        if depth_layout(picture, self.rig, float32_enum=R_FLOAT32):
            super().on_gazebo_depth_picture(picture)

    def on_gazebo_camera_info(self, info):
        if (info.width, info.height) == (self.rig.width, self.rig.height):
            super().on_gazebo_camera_info(info)

    def on_clock(self, clock):
        result = Clock()
        try:
            result.clock.sec, result.clock.nanosec = ros_clock_parts(clock)
        except ValueError:
            return
        self.clock_pub.publish(result)

    def on_status(self, message):
        self.status = capture_ns(message), message.vo_state == 1

    def on_flags(self, message):
        report, = payloads("px4_flags", message)
        self.flags = self.get_clock().now().nanoseconds, report["valid"]

    def on_odometry(self, message):
        now = self.get_clock().now().nanoseconds
        stamp = capture_ns(message)
        if (not 0 < stamp <= now or (self.sample is not None and stamp <= self.sample)
                or message.header.frame_id != "odom" or message.child_frame_id != "base_link"):
            return
        self.sample = stamp
        tracked = (self.status is not None and self.status[1]
                   and 0 <= now - self.status[0] <= TRACKING_MAX_AGE_NS
                   and now - stamp <= TRACKING_MAX_AGE_NS)
        result = DiagnosticArray()
        result.header = message.header
        status = DiagnosticStatus()
        # The name is H1's existing diagnostic envelope; the result explicitly
        # distinguishes the simulated sensor from genuine camera tracking.
        status.name = "visual_slam/frame"
        status.message = SIM_ESTIMATOR if tracked else "simulated_tracking_unavailable"
        status.level = DiagnosticStatus.OK if tracked else DiagnosticStatus.WARN
        status.values = [KeyValue(key="capture_ns", value=str(stamp)),
                         KeyValue(key="duration_ms", value=str((now - stamp) / 1e6))]
        result.status = [status]
        self.processing.publish(result)

    def on_health(self):
        self.health_pub.publish(Bool(data=simulated_health(now_ns=self.get_clock().now().nanoseconds,
                                                          sample_ns=self.sample, status=self.status, flags=self.flags)))


def main():
    rclpy.init()
    node = HybridSensorBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        try_shutdown()


if __name__ == "__main__":
    main()
