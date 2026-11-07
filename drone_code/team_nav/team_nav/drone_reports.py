"""Listens to everything the drone reports, and keeps the latest report of each kind together with
the time it arrived.

REAL DRONE CODE. The flight programs and the hardware checks read the drone through this file.
It only listens: it has no publishers, so it can never move the drone. Sending targets is
another file's job.

What it listens to
------------------
  - /skybeta/state: the organizers' flight_interface program's summary, as JSON text. It says
    whether PX4 is connected, armed and landed, whether control has been handed to our code
    ("Offboard"), and where camera tracking says the drone is ("odom_enu", in metres, as
    forward, left, up from where tracking started).
  - Reports straight from PX4, the flight controller's software:
      vehicle_local_position   PX4's own position and speed estimate, and its distance to the
                               ground below ("dist_bottom"). PX4's directions are north, east,
                               down, so a larger z means lower.
      vehicle_status           PX4's mode, armed state and pre-flight checks.
      vehicle_land_detected    whether PX4 believes the drone is on the ground.
      vehicle_odometry         PX4's attitude (which way the drone is tilted) and turn rate.
      vehicle_attitude         PX4's attitude on its own. The real drone sends this one; on
                               2026-11-07 a check of drone 16 did not list vehicle_odometry.
      sensor_combined          the flight controller's raw motion sensors, including its gyro
                               (turn rate). Used for the turn rate when vehicle_odometry is missing.
      distance_sensor          the bottom laser's raw reading, measured along its beam.
      estimator_status_flags   which sensors PX4's position estimator ("EKF2") is using now,
                               for example whether it still uses the laser for height.
      battery_status           the battery's voltage and how much charge is left.

PX4 topic names
---------------
PX4 1.16 adds "_v1" to some topic names, and which ones depends on the message versions
installed on that computer. The real drone and our simulator may differ. So, like the
organizers' own flight_interface, we build each name with their fmu_topic function rather than
writing it by hand. Every name in use is listed by topic_names, so programs can log them.

Message settings
----------------
PX4's reports are sent "best effort": a lost message is not sent again. We ask for best-effort
delivery and only for messages sent after we start listening ("volatile"). These are the most
permissive settings, so they match PX4's sender whatever it uses.
"""
import json
import time

from px4_msgs.msg import (
    BatteryStatus, DistanceSensor, EstimatorStatusFlags, SensorCombined, VehicleAttitude, VehicleLandDetected,
    VehicleLocalPosition, VehicleOdometry, VehicleStatus,
)
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String

from team_nav.height_check import laser_height_above_launch_metres
from team_nav.touch_detector import rotation_degrees_per_second, tilt_degrees

# From the organizers' code (flight_interface.py's default "state_topic").
ORGANIZERS_STATE_TOPIC = "/skybeta/state"

PX4_REPORT_SETTINGS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=5,
)

# (our name for the report, PX4's topic name without "/fmu/out/", message type)
PX4_REPORTS = [
    ("px4_position", "vehicle_local_position", VehicleLocalPosition),
    ("px4_status", "vehicle_status", VehicleStatus),
    ("px4_land_detected", "vehicle_land_detected", VehicleLandDetected),
    ("px4_odometry", "vehicle_odometry", VehicleOdometry),
    ("px4_attitude", "vehicle_attitude", VehicleAttitude),
    ("motion_sensors", "sensor_combined", SensorCombined),
    ("laser", "distance_sensor", DistanceSensor),
    ("estimator_flags", "estimator_status_flags", EstimatorStatusFlags),
    ("battery", "battery_status", BatteryStatus),
]


def px4_topic_name(base_name, message_type):
    """The name PX4 publishes this report under on this computer, for example
    /fmu/out/vehicle_status_v1. Uses the organizers' function, which reads the installed
    message definitions."""
    from skybeta_flight_interface.topics import fmu_topic  # Installed with the organizers' software.
    return fmu_topic("out", base_name, message_type.__name__)


class DroneReports:
    """Subscribes on the given ROS node and keeps the latest of each report. It never sends."""

    def __init__(self, node):
        self.latest = {}             # Our report name -> the latest message.
        self.arrival_seconds = {}    # Our report name -> time.monotonic() when it arrived.
        self.topic_names = {"state": ORGANIZERS_STATE_TOPIC}
        node.create_subscription(String, ORGANIZERS_STATE_TOPIC, self._on_state, 10)
        for report_name, base_name, message_type in PX4_REPORTS:
            topic_name = px4_topic_name(base_name, message_type)
            self.topic_names[report_name] = topic_name
            node.create_subscription(message_type, topic_name, self._make_receiver(report_name), PX4_REPORT_SETTINGS)

    def _on_state(self, message):
        self._store("state", json.loads(message.data))

    def _make_receiver(self, report_name):
        def receive(message):
            self._store(report_name, message)
        return receive

    def _store(self, report_name, message):
        self.latest[report_name] = message
        self.arrival_seconds[report_name] = time.monotonic()

    # ---- Freshness ------------------------------------------------------------------------------

    def has(self, report_name):
        return report_name in self.latest

    def seconds_since(self, report_name):
        """How long ago this report last arrived, or infinity if it never has."""
        if report_name not in self.arrival_seconds:
            return float("inf")
        return time.monotonic() - self.arrival_seconds[report_name]

    def reports_older_than(self, report_names, longest_age_seconds):
        """Which of these reports have not arrived within longest_age_seconds."""
        old_reports = []
        for report_name in report_names:
            if self.seconds_since(report_name) > longest_age_seconds:
                old_reports.append(report_name)
        return old_reports

    # ---- The organizers' summary ----------------------------------------------------------------

    def state(self):
        """The organizers' latest summary as a dictionary, or None before the first one."""
        return self.latest.get("state")

    def tracked_position(self):
        """Camera tracking's position as [forward, left, up] in metres from where tracking
        started, or None if tracking has not reported yet."""
        state = self.state()
        if state is None:
            return None
        return state["odom_enu"]

    def is_armed(self):
        state = self.state()
        return state is not None and bool(state["armed"])

    def control_is_ours(self):
        """True while PX4 is following our targets ("Offboard" mode)."""
        state = self.state()
        return state is not None and bool(state["offboard_active"])

    # ---- PX4's reports --------------------------------------------------------------------------

    def px4_height_estimate_is_working(self):
        position = self.latest.get("px4_position")
        return position is not None and bool(position.z_valid)

    def px4_distance_to_ground_metres(self):
        """PX4's estimate of the distance to the ground below, based on the laser, or None.
        simulator/test_flights/simulator_test_flight.py explains why we ignore the flag
        dist_bottom_valid next to it on drone 16."""
        if not self.px4_height_estimate_is_working():
            return None
        return float(self.latest["px4_position"].dist_bottom)

    def px4_z_metres(self):
        """PX4's height estimate as its z, which points DOWN, so climbing makes it smaller."""
        return float(self.latest["px4_position"].z)

    def px4_downward_speed_metres_per_second(self):
        return float(self.latest["px4_position"].vz)  # PX4's z points down.

    def raw_laser_reading_metres(self):
        """The bottom laser's latest raw reading along its beam, or None. Unlike PX4's estimate,
        it stays right even when PX4 stops trusting the laser."""
        laser = self.latest.get("laser")
        return None if laser is None else float(laser.current_distance)

    def laser_height_risen_since_launch_metres(self, launch_raw_laser_reading_metres):
        """How far the laser has risen since the launch mark, from its raw reading allowing for
        tilt, or None if the laser or the attitude has not reported."""
        reading = self.raw_laser_reading_metres()
        if reading is None or not self.has_attitude():
            return None
        return laser_height_above_launch_metres(reading, launch_raw_laser_reading_metres, self.tilt_degrees())

    def has_attitude(self):
        return self.has("px4_odometry") or self.has("px4_attitude")

    def has_turn_rate(self):
        return self.has("px4_odometry") or self.has("motion_sensors")

    def tilt_degrees(self):
        """How far the drone leans from level. Uses vehicle_odometry, or vehicle_attitude when the
        drone doesn't send vehicle_odometry. Both store the attitude as (w, x, y, z)."""
        if self.has("px4_odometry"):
            return tilt_degrees(self.latest["px4_odometry"].q)
        return tilt_degrees(self.latest["px4_attitude"].q)

    def rotation_degrees_per_second(self):
        """How fast the drone is turning. Uses PX4's filtered estimate in vehicle_odometry, or the
        gyro's raw reading in sensor_combined when the drone doesn't send vehicle_odometry. The raw
        gyro is noisier, especially with motors spinning; we have not yet seen how much on drone 16."""
        if self.has("px4_odometry"):
            return rotation_degrees_per_second(self.latest["px4_odometry"].angular_velocity)
        return rotation_degrees_per_second(self.latest["motion_sensors"].gyro_rad)

    def px4_uses_laser_for_height(self):
        """True if PX4's estimator is using the laser for height right now, None if unknown."""
        flags = self.latest.get("estimator_flags")
        return None if flags is None else bool(flags.cs_rng_hgt)

    def battery_volts(self):
        battery = self.latest.get("battery")
        return None if battery is None else float(battery.voltage_v)
