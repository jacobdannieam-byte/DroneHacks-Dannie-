"""Listens to the drone for the hand tests, in the background, while the person moves it.

REAL DRONE CODE. It runs on the drone, inside the organizers' software container where ROS and
the drone's message types are installed, and in our simulator.

It ONLY LISTENS. It creates no publishers and calls no service that changes anything, so it can
never arm the drone, spin a motor or send a target. The one service it calls,
/flight_interface/get_parameters, only reads settings. test/test_hand_tests_are_read_only.py
scans this folder to keep it that way.

What it gives the tests
-----------------------
  - reports: the shared DroneReports (drone_reports.py), which keeps the latest of each PX4
    report and of the organizers' summary /skybeta/state.
  - tracking_history and tracking_status_history: every camera tracking report and status
    report, with its arrival time. "Camera tracking" is
    the organizers' program (cuVSLAM) that works out how the drone has moved from the front
    cameras' pictures. It is the drone's only source of sideways position.
  - laser_history: every raw reading of the bottom laser, with its arrival time. On 2026-11-07
    drone 16 did NOT send this topic (/fmu/out/distance_sensor), so it may stay empty.
  - px4_height_history: every PX4 position report's height numbers (dist_bottom, the distance
    to the ground below, and z), each with PX4's flags saying whether it is using the laser.
    This is how the laser tests see the laser when its raw reading is missing.
  - attitude_now: which way the drone is tilted and how fast it turns, with the arrival time of
    the report it came from. DroneReports reads vehicle_odometry, or vehicle_attitude and
    sensor_combined when the drone doesn't send vehicle_odometry, as drone 16 didn't on 2026-11-07.
  - count_messages: how many messages arrive on a list of topics in a given time.
  - read_parameters and node_names: what the organizers' interface program is set to, and which
    programs are running.

A background thread receives the messages, so a test can wait for the person to press Enter
without missing any.
"""
import math
import threading
import time
from dataclasses import dataclass

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import DistanceSensor, VehicleLocalPosition
from rcl_interfaces.srv import GetParameters
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

from team_nav.drone_reports import PX4_REPORT_SETTINGS, DroneReports
from team_nav.hand_tests.hand_test_tools import yaw_degrees_from_quaternion

# From the organizers' code (flight_interface.py's default "odometry_topic").
CAMERA_TRACKING_TOPIC = "/visual_slam/tracking/odometry"
# From the organizers' code (px4_odom_bridge.py reads it). Its vo_state field means
# 0 = unknown, 1 = tracking fine, 2 = tracking lost.
CAMERA_TRACKING_STATUS_TOPIC = "/visual_slam/status"
CAMERA_TRACKING_STATUS_TYPE = "isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus"

# Our own choice: how long to let ROS find the drone's programs before a test starts.
DISCOVERY_SECONDS = 2.0
# Our own choice: how often a waiting test wakes up to look at the readings.
POLLING_PERIOD_SECONDS = 0.02


@dataclass
class TrackingReport:
    """One camera tracking report, in the directions tracking started with: forward, left, up."""
    arrival_seconds: float
    forward_metres: float
    left_metres: float
    up_metres: float
    heading_degrees: float   # Grows when the drone turns left.


@dataclass
class LaserReading:
    arrival_seconds: float
    distance_metres: float   # Measured along the beam.
    signal_quality_percent: int   # -1 means the sensor does not say.


@dataclass
class Px4HeightSample:
    """The height numbers of one PX4 position report, and PX4's latest estimator flags. Each flag
    is None until PX4's estimator flags first arrive (about once or twice a second)."""
    arrival_seconds: float
    distance_to_ground_metres: float   # dist_bottom: from the drone's centre to the ground below.
    z_metres: float                    # PX4's height, pointing DOWN: climbing makes it smaller.
    height_is_valid: bool              # z_valid
    distance_to_ground_is_valid: bool  # dist_bottom_valid; drone 16's settings keep it false.
    uses_laser_for_height: bool        # cs_rng_hgt
    uses_laser_for_ground: bool        # cs_rng_terrain: the laser measures the ground's height.
    laser_declared_faulty: bool        # cs_rng_fault


@dataclass
class TopicCount:
    exists: bool
    message_count: int
    seconds_since_last: float   # Infinity if nothing arrived.


class DroneListener:

    def __init__(self, node_name="team_hand_test"):
        # The simulator's wrapper starts ROS itself, for its own flying node, before this.
        self._started_ros = not rclpy.ok()
        if self._started_ros:
            rclpy.init()
        self.node = Node(node_name)
        self.reports = DroneReports(self.node)
        self.tracking_history = []
        self.laser_history = []
        self.tracking_status_history = []    # (arrival time in seconds, vo_state) pairs.
        self.px4_height_history = []
        self.px4_topic_names = self.reports.topic_names   # Our report name -> topic name.
        self._message_counts = {}
        self._last_arrival_seconds = {}
        self._listen_to_tracking_and_laser()
        self._listen_to_px4_height()
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self.wait(DISCOVERY_SECONDS)

    def _listen_to_tracking_and_laser(self):
        # "Sensor data" settings receive from both reliable and best-effort senders.
        self.node.create_subscription(Odometry, CAMERA_TRACKING_TOPIC, self._on_tracking, qos_profile_sensor_data)
        self.node.create_subscription(
            DistanceSensor, self.reports.topic_names["laser"], self._on_laser, PX4_REPORT_SETTINGS)
        self._can_read_tracking_status = False
        try:
            status_type = get_message(CAMERA_TRACKING_STATUS_TYPE)
        except (AttributeError, ModuleNotFoundError, ValueError):
            print("NOTE: camera tracking's status message type is not installed here, so the tests "
                  "can't read whether tracking says it is lost. Run inside the organizers' container.")
            return
        self.node.create_subscription(status_type, CAMERA_TRACKING_STATUS_TOPIC, self._on_tracking_status,
                                      qos_profile_sensor_data)
        self._can_read_tracking_status = True

    def _listen_to_px4_height(self):
        # DroneReports keeps only the latest position report; the laser tests need every one.
        self.node.create_subscription(VehicleLocalPosition, self.px4_topic_names["px4_position"],
                                      self._on_px4_position, PX4_REPORT_SETTINGS)

    def _on_px4_position(self, message):
        flags = self.reports.latest.get("estimator_flags")
        self.px4_height_history.append(Px4HeightSample(
            time.monotonic(), float(message.dist_bottom), float(message.z), bool(message.z_valid),
            bool(message.dist_bottom_valid),
            None if flags is None else bool(flags.cs_rng_hgt),
            None if flags is None else bool(flags.cs_rng_terrain),
            None if flags is None else bool(flags.cs_rng_fault)))

    def _on_tracking(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        heading = yaw_degrees_from_quaternion(orientation.w, orientation.x, orientation.y, orientation.z)
        self.tracking_history.append(
            TrackingReport(time.monotonic(), position.x, position.y, position.z, heading))

    def _on_laser(self, message):
        self.laser_history.append(
            LaserReading(time.monotonic(), float(message.current_distance), int(message.signal_quality)))

    def _on_tracking_status(self, message):
        self.tracking_status_history.append((time.monotonic(), int(message.vo_state)))

    # ---- Time ---------------------------------------------------------------------------------

    def now(self):
        """The same clock DroneReports stamps its reports with."""
        return time.monotonic()

    def wait(self, seconds):
        """Waits while the background thread keeps receiving."""
        time.sleep(seconds)

    # ---- Histories ----------------------------------------------------------------------------

    def tracking_since(self, start_seconds):
        return [report for report in list(self.tracking_history) if report.arrival_seconds >= start_seconds]

    def laser_since(self, start_seconds):
        return [reading for reading in list(self.laser_history) if reading.arrival_seconds >= start_seconds]

    def px4_heights_since(self, start_seconds):
        return [sample for sample in list(self.px4_height_history) if sample.arrival_seconds >= start_seconds]

    def tracking_states_since(self, start_seconds):
        """Camera tracking's own status reports (vo_state) since then, oldest first."""
        return [state for arrival, state in list(self.tracking_status_history) if arrival >= start_seconds]

    def can_read_tracking_status(self):
        return self._can_read_tracking_status

    # ---- Attitude -----------------------------------------------------------------------------

    def attitude_now(self):
        """(arrival time, tilt in degrees, rotation speed in degrees per second) from PX4's
        latest attitude report, or None if none has arrived. The rotation speed is 0 if PX4 has
        sent no turn rate."""
        reports = self.reports
        if not reports.has_attitude():
            return None
        source = "px4_odometry" if reports.has("px4_odometry") else "px4_attitude"
        rotation = reports.rotation_degrees_per_second() if reports.has_turn_rate() else 0.0
        return reports.arrival_seconds[source], reports.tilt_degrees(), rotation

    def tilt_degrees_now(self):
        """The latest tilt, or 0 if PX4 has sent no attitude report."""
        attitude = self.attitude_now()
        return 0.0 if attitude is None else attitude[1]

    # ---- Counting messages --------------------------------------------------------------------

    def count_messages(self, topic_names, seconds):
        """Listens to each topic for this long. Returns topic name -> TopicCount. A topic that no
        program offers is reported as not existing."""
        offered_types = dict(self.node.get_topic_names_and_types())
        for topic_name in topic_names:
            if topic_name in offered_types and topic_name not in self._message_counts:
                self._count_topic(topic_name, offered_types[topic_name][0])
        start_counts = dict(self._message_counts)
        self.wait(seconds)
        counts = {}
        for topic_name in topic_names:
            if topic_name not in self._message_counts:
                counts[topic_name] = TopicCount(False, 0, math.inf)
                continue
            arrived = self._message_counts[topic_name] - start_counts.get(topic_name, 0)
            last = self._last_arrival_seconds.get(topic_name)
            since_last = math.inf if last is None else time.monotonic() - last
            counts[topic_name] = TopicCount(True, arrived, since_last)
        return counts

    def _count_topic(self, topic_name, type_name):
        self._message_counts[topic_name] = 0

        def on_message(message):
            self._message_counts[topic_name] += 1
            self._last_arrival_seconds[topic_name] = time.monotonic()

        self.node.create_subscription(get_message(type_name), topic_name, on_message, qos_profile_sensor_data)

    # ---- Other programs -----------------------------------------------------------------------

    def node_names(self):
        """The names of every ROS program running on this ROS network number."""
        return list(self.node.get_node_names())

    def read_parameters(self, node_name, parameter_names, longest_wait_seconds=5.0):
        """Reads settings of another ROS program through its read-only parameter service.
        Returns setting name -> value, or None if the program did not answer."""
        client = self.node.create_client(GetParameters, f"/{node_name}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=longest_wait_seconds):
                return None
            request = GetParameters.Request()
            request.names = list(parameter_names)
            future = client.call_async(request)
            give_up_at = time.monotonic() + longest_wait_seconds
            while not future.done() and time.monotonic() < give_up_at:
                self.wait(POLLING_PERIOD_SECONDS)
            if not future.done() or future.result() is None:
                return None
            values = {}
            for name, value in zip(parameter_names, future.result().values):
                values[name] = _python_value(value)
            return values
        finally:
            self.node.destroy_client(client)

    def close(self):
        """Stops the background thread and this node. Stops ROS too if this listener started it."""
        self._executor.shutdown()
        # Our own choice: long enough for the executor to notice it was shut down.
        self._thread.join(timeout=2.0)
        self.node.destroy_node()
        if self._started_ros and rclpy.ok():
            rclpy.shutdown()


def _python_value(parameter_value):
    """A ROS parameter value as plain Python. Type numbers from rcl_interfaces/ParameterType:
    0 not set, 1 bool, 2 integer, 3 double, 4 string."""
    if parameter_value.type == 1:
        return bool(parameter_value.bool_value)
    if parameter_value.type == 2:
        return int(parameter_value.integer_value)
    if parameter_value.type == 3:
        return float(parameter_value.double_value)
    if parameter_value.type == 4:
        return str(parameter_value.string_value)
    return None
