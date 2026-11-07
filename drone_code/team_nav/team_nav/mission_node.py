"""The ROS 2 program that flies the mission on the real drone (drone 16).

How it fits: this file only moves messages. It hands every report it hears to mission.py, and
twenty times a second publishes whatever mission.py decides. It also starts camera_exposure.py,
which caps the camera's automatic exposure. All flight decisions live in mission.py,
course_stages.py and landing.py, which do not need ROS, so they can be tested on a laptop.

Topics:
  hears  /visual_slam/tracking/odometry    camera tracking position and heading (nav_msgs/Odometry)
         /skybeta/state                    the organizers' flight state, as JSON text (std_msgs/String)
         /fmu/out/vehicle_local_position   PX4's position report; we read only dist_bottom, the
                                           bottom laser distance. Read only.
  sends  /skybeta/setpoint/pose            position targets while waiting, climbing and cruising
         /skybeta/setpoint/twist           velocity commands while landing (world directions)
  sets   the camera driver's exposure parameters (camera_exposure.py)

It never publishes to /fmu/in/*: the organizers' rules say only their flight interface talks to
PX4. It does not arm the drone, ask for Offboard, or call /skybeta/hold.

team_nav.launch.py starts it on the drone, next to the organizers' flight interface. To run it by
hand instead:  ros2 run team_nav mission
"""
import json
import math
from dataclasses import fields

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from .camera_exposure import CameraExposureCap
from .mission import Mission
from .mission_settings import MissionSettings

# From the team plan (appendix) and the organizers' code.
DEFAULT_POSITION_TOPIC = "/visual_slam/tracking/odometry"
DEFAULT_FLIGHT_STATE_TOPIC = "/skybeta/state"
# Measured on drone 16: this topic has no version suffix there (vehicle_status does: _v1).
DEFAULT_PX4_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
POSE_TARGET_TOPIC = "/skybeta/setpoint/pose"
VELOCITY_COMMAND_TOPIC = "/skybeta/setpoint/twist"
# The camera tracking frame. The organizers' code ignores the frame named in a message (organizer
# code audit): positions are always camera tracking ENU, and velocities always world ENU (x East,
# y North, z Up), whatever way the nose points. We fill it in only so the messages read correctly.
TARGET_FRAME_ID = "odom"


def heading_from_quaternion(orientation):
    """Heading (anticlockwise from East, radians) of a ROS orientation quaternion."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def px4_local_position_topic(requested_topic):
    """The topic to read PX4's position report from. By default the name measured on drone 16.
    PX4 adds a version suffix such as _v1 to some topic names, depending on its version; if the
    parameter is set empty, ask the organizers' code, which reads the rule from the installed PX4
    messages, and use the plain name if that code is not installed."""
    if requested_topic:
        return requested_topic
    try:
        from skybeta_flight_interface.topics import fmu_topic
        return fmu_topic("out", "vehicle_local_position")
    except ImportError:
        return DEFAULT_PX4_LOCAL_POSITION_TOPIC


def setting_value(name, default, value):
    """The parameter value converted to the type of the setting's default. A launch file writes
    numbers without a type, so 1 must become 1.0 for a setting in metres."""
    if isinstance(default, bool):
        if isinstance(value, bool):
            return value
        if str(value).lower() in ("true", "false"):
            return str(value).lower() == "true"
    else:
        try:
            return type(default)(value)
        except (TypeError, ValueError):
            pass
    raise ValueError(f"The ROS parameter {name} is {value!r}, which cannot be used as a "
                     f"{type(default).__name__} like its default {default!r}. Fix it in the launch command.")


class MissionNode(Node):
    def __init__(self):
        super().__init__("team_nav_mission")
        self.mission = Mission(self.read_settings())
        position_topic = self.declare_parameter("position_topic", DEFAULT_POSITION_TOPIC).value
        flight_state_topic = self.declare_parameter("flight_state_topic", DEFAULT_FLIGHT_STATE_TOPIC).value
        local_position_topic = px4_local_position_topic(
            self.declare_parameter("px4_local_position_topic", DEFAULT_PX4_LOCAL_POSITION_TOPIC).value)
        # PX4's reports arrive "best effort" and keep the latest message for late listeners; a
        # listener that asks for anything else never receives them.
        px4_reports = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, position_topic, self.on_position_report, 10)
        self.create_subscription(String, flight_state_topic, self.on_flight_state, 10)
        self.create_subscription(VehicleLocalPosition, local_position_topic, self.on_px4_local_position, px4_reports)
        self.pose_publisher = self.create_publisher(PoseStamped, POSE_TARGET_TOPIC, 10)
        self.twist_publisher = self.create_publisher(TwistStamped, VELOCITY_COMMAND_TOPIC, 10)
        self.create_timer(1.0 / self.mission.settings.steps_per_second, self.on_step)
        self.camera_exposure = None
        settings = self.mission.settings
        if settings.cap_camera_exposure:
            self.camera_exposure = CameraExposureCap(self, settings.camera_node_name,
                                                     settings.camera_auto_exposure_limit_microseconds)
            self.camera_exposure.check()
            self.create_timer(settings.camera_check_seconds, self.camera_exposure.check)
        self.get_logger().info(
            f"team_nav mission ready. Hears {position_topic}, {flight_state_topic}, {local_position_topic}; "
            f"sends {POSE_TARGET_TOPIC} and {VELOCITY_COMMAND_TOPIC}; cruise height "
            f"{self.mission.settings.cruise_height_metres:g} m. It waits for Offboard before flying.")

    def read_settings(self):
        """Every MissionSettings value becomes a ROS parameter with the same name and default.
        Dynamic typing lets a launch value like 1 stand for 1.0; setting_value converts it."""
        settings = MissionSettings()
        any_type = ParameterDescriptor(dynamic_typing=True)
        for setting in fields(MissionSettings):
            default = getattr(settings, setting.name)
            value = self.declare_parameter(setting.name, default, any_type).value
            setattr(settings, setting.name, setting_value(setting.name, default, value))
        return settings

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ what we hear
    def on_position_report(self, message):
        point = message.pose.pose.position
        position = (float(point.x), float(point.y), float(point.z))
        heading = heading_from_quaternion(message.pose.pose.orientation)
        if not all(math.isfinite(value) for value in position + (heading,)):
            return
        self.mission.observe_position(self.now_seconds(), position, heading)

    def on_flight_state(self, message):
        try:
            state = json.loads(message.data)
        except ValueError:
            self.get_logger().warn("The flight state on /skybeta/state was not valid JSON, so it was ignored.")
            return
        if not isinstance(state, dict):
            return
        # The organizers' code repeats its last known values here, so this can only say what PX4's
        # mode was, not that position reports are still arriving: mission.py checks those itself.
        self.mission.observe_flight_state(self.now_seconds(), state.get("offboard_active"), state.get("armed"),
                                          state.get("landed"), state.get("px4_link"))

    def on_px4_local_position(self, message):
        # PX4 also sends dist_bottom_valid, but on drone 16 it stays false for the whole flight
        # (docs/ARCHITECTURE.md, section 7), so it is ignored on purpose.
        self.mission.observe_px4_position_report(self.now_seconds(), float(message.dist_bottom))

    # ------------------------------------------------------------------ what we send
    def on_step(self):
        command = self.mission.step(self.now_seconds())
        for line in command.log_lines:
            self.get_logger().info(line)
        if command.kind == "position":
            self.publish_pose_target(command.position, command.heading_radians)
        elif command.kind == "velocity":
            if command.position_before_velocity is not None:
                # Sent first, so that if the velocity commands ever stop, the organizers' code holds
                # the drone here rather than flying back to an older position target.
                hold_point = command.position_before_velocity
                self.publish_pose_target(hold_point[:3], hold_point[3])
            self.publish_velocity_command(command.velocity)

    def publish_pose_target(self, position, heading_radians):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = TARGET_FRAME_ID
        message.pose.position.x, message.pose.position.y, message.pose.position.z = (float(v) for v in position)
        # A rotation about the vertical axis only, written as a quaternion.
        message.pose.orientation.z = math.sin(heading_radians / 2.0)
        message.pose.orientation.w = math.cos(heading_radians / 2.0)
        self.pose_publisher.publish(message)

    def publish_velocity_command(self, velocity):
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = TARGET_FRAME_ID
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = (float(v) for v in velocity)
        # No turning while landing: PX4 only counts the drone as landed if it is not rotating.
        message.twist.angular.z = 0.0
        self.twist_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
