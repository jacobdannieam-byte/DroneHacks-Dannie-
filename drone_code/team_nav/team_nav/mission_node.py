"""The ROS 2 program that flies the mission on the real drone (drone 16).

How it fits: this file only moves messages. It hands every report it hears to mission.py, and
twenty times a second publishes whatever mission.py decides. It also starts camera_exposure.py,
which caps the camera's automatic exposure. All flight decisions live in mission.py and the files
it uses, which do not need ROS, so they can be tested on a laptop.

Topics:
  hears  /visual_slam/tracking/odometry    camera tracking position, heading and timestamp (nav_msgs/Odometry)
         /skybeta/state                    the organizers' flight state, as JSON text (std_msgs/String)
         /fmu/out/vehicle_local_position   PX4's position report: dist_bottom, the bottom laser
                                           distance, and its reset counters. Read only.
         /fmu/out/vehicle_status_v1        PX4's status: nav_state, its current mode. Read only.
         /visual_slam/status               camera tracking's own status (vo_state): logged only, never
                                           used to land. On drone 16 it stayed 1 ("tracking") while the
                                           camera was covered.
         depth_topic (a parameter)         only with drift_correction:=true: depth points
                                           (sensor_msgs/PointCloud2, x y z = forward left up in metres
                                           from the drone's centre). In the simulator, Kenny's pretend
                                           depth camera; on the drone, a depth program of ours that
                                           does not exist yet. depth_ahead.py reads them.
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
import os
from dataclasses import fields

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from .camera_exposure import CameraExposureCap
from .course_map import load_course_map
from .depth_ahead import distance_straight_ahead, points_from_cloud
from .drift_correction import MapRanging
from .mission import Mission
from .mission_settings import MissionSettings
from .start_frame import StartFrame

try:
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
except ImportError:
    # Its status is only logged, so the mission must not fail to start without this package.
    VisualSlamStatus = None

# From the team plan (appendix) and the organizers' code.
DEFAULT_POSITION_TOPIC = "/visual_slam/tracking/odometry"
DEFAULT_FLIGHT_STATE_TOPIC = "/skybeta/state"
# Measured on drone 16: this topic has no version suffix there.
DEFAULT_PX4_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
# Measured on drone 16, and in the simulator: PX4's status message is at version 1, so its topic has
# the suffix _v1.
DEFAULT_PX4_STATUS_TOPIC = "/fmu/out/vehicle_status_v1"
DEFAULT_VISUAL_SLAM_STATUS_TOPIC = "/visual_slam/status"
# From the message definition read on drone 16 (VisualSlamStatus.msg).
VO_STATE_NAMES = {0: "unknown", 1: "tracking", 2: "lost"}
# The route files installed with this package live in its missions folder.
MISSIONS_FOLDER_IN_PACKAGE = "missions"
DEFAULT_ROUTE_FILE = "route_waypoints.json"
# The course maps installed with this package (Kenny's, copied unchanged from origin/classical).
COURSE_MAPS_FOLDER_IN_PACKAGE = "course_maps"
DEFAULT_COURSE_MAP_FILE = "organizers_drawing.json"
POSE_TARGET_TOPIC = "/skybeta/setpoint/pose"
VELOCITY_COMMAND_TOPIC = "/skybeta/setpoint/twist"
# SIMULATOR ONLY. The simulator publishes no camera images, so nothing reports camera tracking
# health and every stage would hold for it. simulator/run_team_nav.sh launches with
# vision_check:=false to treat it as healthy. This program accepts that only when both of these
# show it is that script, inside the simulator box, so the check can never be switched off on the
# drone: the script sets this environment variable, and the simulator box shares the repository at
# /repository, a folder the drone does not have.
SIMULATOR_RUN_VARIABLE = "TEAM_NAV_STARTED_BY"
SIMULATOR_RUN_VALUE = "simulator/run_team_nav.sh"
SIMULATOR_FOLDER = "/repository/simulator"

# The camera tracking frame. The organizers' code ignores the frame named in a message (organizer
# code audit): positions are always camera tracking ENU, and velocities always world ENU (x East,
# y North, z Up), whatever way the nose points. We fill it in only so the messages read correctly.
TARGET_FRAME_ID = "odom"


def heading_from_quaternion(orientation):
    """Heading (anticlockwise from East, radians) of a ROS orientation quaternion."""
    x, y, z, w = orientation.x, orientation.y, orientation.z, orientation.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def px4_topic(requested_topic, uorb_name, plain_topic):
    """The topic to read a PX4 report from: the parameter's value, by default the name measured on
    drone 16. PX4 adds a version suffix such as _v1 to some topic names, depending on its version;
    if the parameter is set empty, ask the organizers' code, which reads the rule from the installed
    PX4 messages, and use the plain name if that code is not installed."""
    if requested_topic:
        return requested_topic
    try:
        from skybeta_flight_interface.topics import fmu_topic
        return fmu_topic("out", uorb_name)
    except ImportError:
        return plain_topic


def file_in_package(file_name, folder):
    """A bare file name means that file in this package's folder; a path is used as it is."""
    if os.path.dirname(file_name) != "":
        return file_name
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("team_nav"), folder, file_name)


def read_route_waypoints(route_file):
    """The route mission's waypoints, [(forward, left) or (forward, left, height), ...] in metres,
    from a JSON file. A bare file name, such as route_waypoints_scan.json, means that file in this
    package's missions folder; a path is used as it is. With no file named, the default route."""
    route_file = file_in_package(route_file or DEFAULT_ROUTE_FILE, MISSIONS_FOLDER_IN_PACKAGE)
    try:
        with open(route_file, encoding="utf-8") as opened:
            waypoints = json.load(opened)["waypoints_forward_left_metres"]
    except (OSError, ValueError, KeyError) as problem:
        raise ValueError(f"Could not read the route's waypoints from {route_file} ({problem}). Check the "
                         "route_file parameter, or that the package was built with its missions folder.")
    route = []
    for number, waypoint in enumerate(waypoints, start=1):
        if not isinstance(waypoint, list) or len(waypoint) not in (2, 3):
            raise ValueError(f"Waypoint {number} in {route_file} is {waypoint!r}; each waypoint must be [forward, left] "
                             "or [forward, left, height] in metres. Fix the route file.")
        route.append(tuple(float(value) for value in waypoint))
    return route


def read_map_ranging(course_map_file, start_heading_degrees):
    """What the drift correction measures the map with: the course map, and where the drone starts
    on it (its launch mark, facing start_heading_degrees)."""
    course_map_file = file_in_package(course_map_file or DEFAULT_COURSE_MAP_FILE, COURSE_MAPS_FOLDER_IN_PACKAGE)
    try:
        course_map = load_course_map(course_map_file)
    except (OSError, ValueError) as problem:
        raise ValueError(f"Could not read the course map {course_map_file} ({problem}). Check the course_map_file "
                         "parameter, or launch with drift_correction:=false.")
    start_frame = StartFrame(course_map.launch_mark_centre_metres, start_heading_degrees)
    return MapRanging(course_map, start_frame), course_map_file


def xyz_offsets(message):
    """The byte offsets of the x, y and z fields of a PointCloud2, or None if any is missing or is
    not a 32-bit float."""
    offsets = {}
    for point_field in message.fields:
        if point_field.datatype == PointField.FLOAT32:
            offsets[point_field.name] = point_field.offset
    if not all(name in offsets for name in ("x", "y", "z")):
        return None
    return [offsets["x"], offsets["y"], offsets["z"]]


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


def vision_check_may_be_switched_off():
    started_by_the_simulator_script = os.environ.get(SIMULATOR_RUN_VARIABLE) == SIMULATOR_RUN_VALUE
    return started_by_the_simulator_script and os.path.isdir(SIMULATOR_FOLDER)


class MissionNode(Node):
    def __init__(self):
        super().__init__("team_nav_mission")
        settings = self.read_settings()
        mission_kind = self.declare_parameter("mission_kind", "stages").value
        route_file = self.declare_parameter("route_file", DEFAULT_ROUTE_FILE).value
        route_waypoints = read_route_waypoints(route_file) if mission_kind == "route" else None
        course_map_file = self.declare_parameter("course_map_file", DEFAULT_COURSE_MAP_FILE).value
        map_ranging = None
        if settings.drift_correction:
            map_ranging, course_map_file = read_map_ranging(course_map_file, settings.start_heading_degrees)
        self.mission = Mission(settings, vision_check=self.read_vision_check(), mission_kind=mission_kind,
                               route_waypoints=route_waypoints, map_ranging=map_ranging)
        self.last_vo_state = None
        self.depth_format_warned = False
        heard = self.listen_to_reports()
        heard += self.listen_to_depth(course_map_file)
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
        if mission_kind == "route":
            self.get_logger().info(f"The route comes from {route_file}: {len(route_waypoints)} waypoints.")
        self.get_logger().info(
            f"team_nav mission ready: the {mission_kind} mission. Hears {', '.join(heard)}; sends "
            f"{POSE_TARGET_TOPIC} and {VELOCITY_COMMAND_TOPIC}; cruise height "
            f"{self.mission.settings.cruise_height_metres:g} m. It waits for Offboard before flying.")

    def listen_to_reports(self):
        """Subscribe to every report the mission uses. Returns the topic names, for the log."""
        position_topic = self.declare_parameter("position_topic", DEFAULT_POSITION_TOPIC).value
        flight_state_topic = self.declare_parameter("flight_state_topic", DEFAULT_FLIGHT_STATE_TOPIC).value
        local_position_topic = px4_topic(self.declare_parameter("px4_local_position_topic",
                                                                DEFAULT_PX4_LOCAL_POSITION_TOPIC).value,
                                         "vehicle_local_position", DEFAULT_PX4_LOCAL_POSITION_TOPIC)
        status_topic = px4_topic(self.declare_parameter("px4_status_topic", DEFAULT_PX4_STATUS_TOPIC).value,
                                 "vehicle_status", DEFAULT_PX4_STATUS_TOPIC)
        visual_slam_status_topic = self.declare_parameter("visual_slam_status_topic",
                                                          DEFAULT_VISUAL_SLAM_STATUS_TOPIC).value
        # PX4's reports arrive "best effort" and keep the latest message for late listeners; a
        # listener that asks for anything else never receives them.
        px4_reports = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                                 history=QoSHistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(Odometry, position_topic, self.on_position_report, 10)
        self.create_subscription(String, flight_state_topic, self.on_flight_state, 10)
        self.create_subscription(VehicleLocalPosition, local_position_topic, self.on_px4_local_position, px4_reports)
        self.create_subscription(VehicleStatus, status_topic, self.on_px4_status, px4_reports)
        heard = [position_topic, flight_state_topic, local_position_topic, status_topic]
        if VisualSlamStatus is None:
            self.get_logger().warn("The package isaac_ros_visual_slam_interfaces is not installed, so "
                                   f"{visual_slam_status_topic} is not logged. Nothing else depends on it.")
        else:
            self.create_subscription(VisualSlamStatus, visual_slam_status_topic, self.on_visual_slam_status, 10)
            heard.append(f"{visual_slam_status_topic} (logged only)")
        return heard

    def listen_to_depth(self, course_map_file):
        """With drift_correction:=true, subscribe to the depth points. Returns the topic, for the log."""
        depth_topic = self.declare_parameter("depth_topic", "").value
        if not self.mission.settings.drift_correction:
            return []
        if not depth_topic:
            self.get_logger().warn("drift_correction is true but depth_topic is empty, so every correction will be "
                                   "skipped for want of a depth reading. Give depth_topic:=<topic of the depth points>.")
            return []
        # Best effort accepts a sender of either kind; only the latest depth matters.
        latest_only = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, depth_topic, self.on_depth_points, latest_only)
        self.get_logger().info(f"Drift correction on: depth from {depth_topic}, map {course_map_file}, start heading "
                               f"{self.mission.settings.start_heading_degrees:g} deg from the map's east.")
        return [f"{depth_topic} (drift correction)"]

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

    def read_vision_check(self):
        """True unless this is a simulator run that asked for vision_check:=false."""
        any_type = ParameterDescriptor(dynamic_typing=True)
        wanted = setting_value("vision_check", True, self.declare_parameter("vision_check", True, any_type).value)
        if wanted:
            return True
        if vision_check_may_be_switched_off():
            self.get_logger().warn("SIMULATOR ONLY: vision_check is false, so camera tracking health is treated as "
                                   "healthy. This must never happen on the drone.")
            return False
        self.get_logger().error("vision_check:=false is only for the simulator, started by "
                                "simulator/run_team_nav.sh. This is not that run, so the vision check stays on.")
        return True

    def now_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ what we hear
    def on_position_report(self, message):
        point = message.pose.pose.position
        position = (float(point.x), float(point.y), float(point.z))
        quaternion = message.pose.pose.orientation
        orientation = (float(quaternion.x), float(quaternion.y), float(quaternion.z), float(quaternion.w))
        heading = heading_from_quaternion(quaternion)
        if not all(math.isfinite(value) for value in position + orientation):
            return
        stamp = message.header.stamp
        stamp_seconds = None if stamp is None else stamp.sec + stamp.nanosec * 1e-9
        self.mission.observe_position(self.now_seconds(), position, heading, stamp_seconds, orientation)

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
        self.mission.observe_px4_position_report(self.now_seconds(), float(message.dist_bottom),
                                                 message.xy_reset_counter, message.heading_reset_counter)

    def on_px4_status(self, message):
        self.mission.observe_px4_status(self.now_seconds(), message.nav_state)

    def on_depth_points(self, message):
        offsets = xyz_offsets(message)
        if offsets is None:
            if not self.depth_format_warned:
                self.depth_format_warned = True
                self.get_logger().warn("The depth points have no float32 x, y and z fields, so they are ignored and "
                                       "the drift correction is skipped. Check what publishes on depth_topic.")
            return
        points = points_from_cloud(message.data, message.point_step, offsets, little_endian=not message.is_bigendian)
        settings = self.mission.settings
        distance, _ = distance_straight_ahead(points, settings.drift_depth_window_degrees,
                                              settings.drift_depth_least_points)
        self.mission.observe_depth_ahead(self.now_seconds(), distance)

    def on_visual_slam_status(self, message):
        vo_state = int(message.vo_state)
        if vo_state == self.last_vo_state:
            return
        self.last_vo_state = vo_state
        self.get_logger().info(f"camera tracking status: vo_state {vo_state} ({VO_STATE_NAMES.get(vo_state, 'other')}). "
                               "Logged only: it never makes the drone land.")

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
