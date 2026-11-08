"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

The bench's pretend ROS node: the one place where our REAL flight code meets the pretend drone.

How it fits in
--------------
The real flight code talks to the drone through flight_steps.DroneLink, which only does three
things with ROS: it subscribes to topics, publishes targets, and calls rclpy.spin_once() to wait.
So the bench does not fake DroneLink at all. It gives the REAL DroneLink this pretend node:
  - subscriptions are remembered by topic name, and the bench calls them with pretend messages
    (camera tracking, PX4's reports, the organizers' summary, depth points);
  - published targets go to the pretend drone (pretend_drone.py);
  - rclpy.spin_once(node, timeout) (see ros_stand_ins.py) moves the pretend world forward by
    that much simulated time. DroneLink.wait_one_cycle() calls it until 0.1 s has passed, so
    the real 10 Hz loop runs unchanged, only faster than real time.
Our code reads the clock with time.monotonic(). use_simulated_time() swaps the "time" module
inside every team_nav module for one whose monotonic() reads the simulated clock.

It also plays the teammate (arms and hands over control once targets arrive) and calls the
referee (bench_referee.py), and stops the trial when the referee says the run is over.

What it does not do: decide anything for the flight program, or use any real network.

The exact places this file depends on the real code (update here if they change):
  - topic names: flight_steps.CAMERA_TRACKING_POSITION_TOPIC, target_sender.POSE_TARGET_TOPIC and
    SPEED_TARGET_TOPIC, depth_camera.OBSTACLE_POINTS_TOPIC, drone_reports.ORGANIZERS_STATE_TOPIC
    and drone_reports.PX4_REPORTS (as "/fmu/out/<name>");
  - message fields read by drone_reports.py, flight_steps.py, target_sender.py and
    depth_camera.points_from_message (float32 x, y, z in message.data).
"""
import heapq
import json
import math
import sys
import time as real_time

import numpy as np

from bench_referee import BenchReferee
from pretend_drone import PretendDrone
from pretend_tracking import PretendTracking
from ros_stand_ins import PretendMessage

# ---- Topic names (copied from the real code; see the header) -------------------------------------
CAMERA_TRACKING_TOPIC = "/visual_slam/tracking/odometry"
POSE_TARGET_TOPIC = "/skybeta/setpoint/pose"
SPEED_TARGET_TOPIC = "/skybeta/setpoint/twist"
DEPTH_POINTS_TOPIC = "/team_nav/obstacle_points"
ORGANIZERS_STATE_TOPIC = "/skybeta/state"
PX4_TOPIC_PREFIX = "/fmu/out/"

# ---- Simulated time ------------------------------------------------------------------------------
# Our own choice: physics steps of 20 ms; fine enough for a drone moving under 1 m/s.
PHYSICS_STEP_SECONDS = 0.02
# Our own choice: the simulated clock starts here, like a computer that has been on a while.
CLOCK_START_SECONDS = 100.0
# Our own choice: a trial is stopped whatever happens after this much simulated time.
LONGEST_TRIAL_SECONDS = 330.0

# ---- How often each pretend report arrives (per second) ------------------------------------------
# Measured on drone 16: camera tracking 30, the organizers' summary 5, estimator flags about 2.
# Our own choice for the rest: slower than on the drone (PX4 sends its position 100 times a
# second) to save computer time; our program only reads them 10 times a second.
REPORTS_PER_SECOND = {"camera_tracking": 30.0, "state": 5.0, "vehicle_local_position": 20.0,
                      "vehicle_status": 5.0, "vehicle_land_detected": 5.0, "vehicle_attitude": 20.0,
                      "sensor_combined": 20.0, "estimator_status_flags": 2.0, "battery_status": 1.0}

# ---- Depth pictures -------------------------------------------------------------------------------
# Measured on drone 16 (10:39-10:54): about 8.6 pictures a second, 15 % of gaps doubled (a missed
# picture), computing takes 59-75 ms.
DEPTH_PICTURE_GAP_SECONDS = 0.1
DEPTH_PICTURE_MISSED_FRACTION = 0.15
DEPTH_SHORTEST_DELAY_SECONDS = 0.055
DEPTH_LONGEST_DELAY_SECONDS = 0.075

# ---- Heights --------------------------------------------------------------------------------------
# Measured on drone 16 on 2026-11-08: on the launch mark PX4 said z = -0.15 m (its zero is
# 15 cm below the floor), camera tracking reads 2-6 cm below PX4's height, and the bottom laser
# reads 0.10 m on the floor.
PX4_HEIGHT_OF_FLOOR_METRES = 0.15
CAMERA_HEIGHT_READS_LOW_METRES = 0.04
LASER_READING_ON_FLOOR_METRES = 0.10
# Estimated: the gyro's noise in flight, 1.5-4.9 deg/s measured.
GYRO_NOISE_RADIANS_PER_SECOND = 0.05

# ---- Start --------------------------------------------------------------------------------------
# Estimated: how well a person places the drone on the launch mark, and points it north.
LAUNCH_PLACEMENT_METRES = 0.03
LAUNCH_HEADING_ERROR_DEGREES = 3.0
# From course_run.DEFAULT_START_HEADING_DEGREES: the program assumes the nose points north.
ASSUMED_START_HEADING_DEGREES = 90.0
# Our own choice: the pretend teammate arms this long after our program starts sending targets.
TEAMMATE_ARMS_AFTER_SECONDS = 2.0
# Our own choice: after PX4 disarms, the program gets this long to finish on its own.
SECONDS_TO_FINISH_AFTER_DISARM = 5.0
# PX4 nav_state numbers, from PX4 source code (vehicle_status.msg).
NAV_STATE_POSITION = 2
NAV_STATE_OFFBOARD = 14


class BenchRunIsOver(BaseException):
    """Raised inside rclpy.spin_once when the referee ends the run. It is a BaseException on
    purpose, so the flight program's own "except Exception" does not catch it and land again."""


class SimulatedClock:
    def __init__(self):
        self.seconds = CLOCK_START_SECONDS


class SimulatedTimeModule:
    """Looks like Python's time module, but monotonic() reads the simulated clock."""

    def __init__(self, clock):
        self.clock = clock

    def monotonic(self):
        return self.clock.seconds

    def __getattr__(self, name):
        return getattr(real_time, name)


def use_simulated_time(clock):
    """Swaps the "time" module inside every loaded team_nav module for the simulated one. This
    catches any time.monotonic() call, including ones added to the code later."""
    for module_name, module in list(sys.modules.items()):
        if not module_name.startswith("team_nav"):
            continue
        current = getattr(module, "time", None)
        if current is real_time or isinstance(current, SimulatedTimeModule):
            module.time = SimulatedTimeModule(clock)


class BenchNode:
    """The pretend ROS node, and the pretend world behind it."""

    def __init__(self, world, random, tracking_starts_bad, speck_setting, trace_rows=None):
        self.world = world
        self.random = random
        self.speck_setting = speck_setting
        self.clock = SimulatedClock()
        self.drone = PretendDrone(random)
        self.tracking = PretendTracking(random, tracking_starts_bad, CLOCK_START_SECONDS, LONGEST_TRIAL_SECONDS)
        self.referee = BenchReferee(world)
        self.subscriptions = {}           # topic -> list of callbacks
        self.waiting_messages = []        # heap of (delivery time, number, topic, message, picture time)
        self.message_number = 0
        self.next_report_seconds = {name: CLOCK_START_SECONDS for name in REPORTS_PER_SECOND}
        self.next_depth_picture_seconds = CLOCK_START_SECONDS
        self.offboard = False
        self.first_target_seconds = None
        self.disarmed_after_flight_seconds = None
        self.trace_rows = trace_rows
        self.next_trace_seconds = CLOCK_START_SECONDS
        self.last_target_on_course = None
        self.link = None                  # Set by run_one_flight.py, only for the trace's stage column.
        self._place_on_launch_mark()

    def _place_on_launch_mark(self):
        """The true launch position and heading on the course. Camera tracking's frame ("odom")
        starts here: forward along the true nose direction."""
        self.launch_on_course = self.world.launch_mark + self.random.normal(0.0, LAUNCH_PLACEMENT_METRES, size=2)
        error_degrees = self.random.normal(0.0, LAUNCH_HEADING_ERROR_DEGREES)
        self.launch_heading_radians = math.radians(ASSUMED_START_HEADING_DEGREES + error_degrees)

    # ---- Converting between camera tracking's frame and the course ------------------------------

    def course_from_odom(self, odom_metres):
        cosine, sine = math.cos(self.launch_heading_radians), math.sin(self.launch_heading_radians)
        east = self.launch_on_course[0] + cosine * odom_metres[0] - sine * odom_metres[1]
        north = self.launch_on_course[1] + sine * odom_metres[0] + cosine * odom_metres[1]
        return np.array([east, north, odom_metres[2]])

    def true_position_on_course(self):
        return self.course_from_odom(self.drone.true_position())

    def true_heading_on_course(self):
        return self.launch_heading_radians + self.drone.heading_radians

    # ---- The ROS node interface the real code uses ----------------------------------------------

    def create_subscription(self, _message_type, topic, callback, _settings):
        self.subscriptions.setdefault(topic, []).append(callback)

    def create_publisher(self, _message_type, topic, _queue_length):
        return BenchPublisher(self, topic)

    def get_clock(self):
        return BenchRosClock(self.clock)

    def destroy_node(self):
        pass

    def spin_once(self, timeout_seconds):
        """Moves the pretend world forward by timeout_seconds, delivering what falls due."""
        end_seconds = self.clock.seconds + timeout_seconds
        self._deliver_due_messages()
        while self.clock.seconds < end_seconds - 1e-9:
            step_seconds = min(PHYSICS_STEP_SECONDS, end_seconds - self.clock.seconds)
            self._step(step_seconds)
        # Land exactly on the end time, so the caller's "time left" can't stay a tiny positive
        # rounding error for ever.
        self.clock.seconds = max(self.clock.seconds, end_seconds)

    # ---- Targets from our program -----------------------------------------------------------------

    def receive_published(self, topic, message):
        now = self.clock.seconds
        if topic == POSE_TARGET_TOPIC:
            position = message.pose.position
            target = np.array([position.x, position.y, position.z - PX4_HEIGHT_OF_FLOOR_METRES])
            self.drone.receive_position_target(now, target, _heading_from_target(message.pose.orientation))
            self.last_target_on_course = self.course_from_odom(target)
            if self.first_target_seconds is None:
                self.first_target_seconds = now
        elif topic == SPEED_TARGET_TOPIC:
            speed = message.twist.linear
            self.drone.receive_speed_target(now, (speed.x, speed.y, speed.z))

    # ---- One physics step ---------------------------------------------------------------------------

    def _step(self, step_seconds):
        # Drift grows with the distance really flown, so the hover wander's jitter is left out.
        before = self.drone.flown_position.copy()
        self.clock.seconds += step_seconds
        now = self.clock.seconds
        self._play_the_teammate(now)
        self.tracking.update_px4(now, step_seconds)
        self.drone.step(now, step_seconds, self.tracking.estimate_error_metres())
        self.tracking.add_flown_distance(float(np.linalg.norm(self.drone.flown_position[:2] - before[:2])))
        self._take_camera_tracking_pictures(now)
        self._take_depth_pictures(now)
        self._send_px4_reports(now)
        self._deliver_due_messages()
        self._record_trace(now)
        self._ask_the_referee(now)

    def _play_the_teammate(self, now):
        """Arms and switches to Offboard once our program has sent targets for a while."""
        if self.drone.armed or self.disarmed_after_flight_seconds is not None or self.first_target_seconds is None:
            return
        if now - self.first_target_seconds >= TEAMMATE_ARMS_AFTER_SECONDS:
            self.drone.arm(now)
            self.offboard = True

    def _ask_the_referee(self, now):
        reason = self.referee.update(now, self.true_position_on_course(), self.drone.armed, self.drone.velocity[2])
        if self.drone.armed is False and self.offboard:
            self.offboard = False
            self.disarmed_after_flight_seconds = now
        if now - CLOCK_START_SECONDS > LONGEST_TRIAL_SECONDS:
            raise BenchRunIsOver("The trial ran out of simulated time.")
        if reason is None:
            return
        if self.drone.armed:
            # The referee ended the run while flying (crash, 3 minutes, motor cut).
            self.drone.cut_motors()
            raise BenchRunIsOver(reason)
        if now - self.disarmed_after_flight_seconds > SECONDS_TO_FINISH_AFTER_DISARM:
            raise BenchRunIsOver(reason)

    # ---- Pretend sensors ----------------------------------------------------------------------------

    def _take_camera_tracking_pictures(self, now):
        if now < self.next_report_seconds["camera_tracking"]:
            return
        self.next_report_seconds["camera_tracking"] += 1.0 / REPORTS_PER_SECOND["camera_tracking"]
        position = self.tracking.reported_position(self.drone.true_position())
        position[2] -= CAMERA_HEIGHT_READS_LOW_METRES
        message = PretendMessage()
        _stamp(message.header.stamp, now)
        message.pose.pose.position.x, message.pose.pose.position.y, message.pose.pose.position.z = position
        orientation = message.pose.pose.orientation
        orientation.x, orientation.y = 0.0, 0.0
        orientation.z, orientation.w = math.sin(self.drone.heading_radians / 2), math.cos(self.drone.heading_radians / 2)
        self._send_later(self.tracking.delivery_seconds(now), CAMERA_TRACKING_TOPIC, message, picture_seconds=now)

    def _take_depth_pictures(self, now):
        if now < self.next_depth_picture_seconds:
            return
        self.next_depth_picture_seconds += DEPTH_PICTURE_GAP_SECONDS
        if self.random.random() < DEPTH_PICTURE_MISSED_FRACTION:
            return
        if not self.subscriptions.get(DEPTH_POINTS_TOPIC):
            return      # Nobody listens yet (the live map starts after the climb): save the work.
        points = self.world.depth_points_on_body(self.true_position_on_course(), self.true_heading_on_course(),
                                                 self.speck_setting)
        message = PretendMessage(data=points.astype(np.float32).tobytes())
        _stamp(message.header.stamp, now)
        delay = self.random.uniform(DEPTH_SHORTEST_DELAY_SECONDS, DEPTH_LONGEST_DELAY_SECONDS)
        self._send_later(now + delay, DEPTH_POINTS_TOPIC, message)

    def _send_px4_reports(self, now):
        for name in REPORTS_PER_SECOND:
            if name == "camera_tracking" or now < self.next_report_seconds[name]:
                continue
            self.next_report_seconds[name] += 1.0 / REPORTS_PER_SECOND[name]
            if name == "state":
                self._deliver(ORGANIZERS_STATE_TOPIC, self._organizers_summary())
            else:
                self._deliver(PX4_TOPIC_PREFIX + name, self._px4_report(name))

    def _organizers_summary(self):
        summary = {"px4_link": True, "armed": self.drone.armed, "landed": self.drone.landed,
                   "offboard_active": self.offboard and self.drone.armed,
                   "nav_state_name": "OFFBOARD" if self.offboard else "POSCTL",
                   "odom_enu": self.drone.true_position().tolist()}
        return PretendMessage(data=json.dumps(summary))

    def _px4_report(self, name):
        drone = self.drone
        if name == "vehicle_local_position":
            estimate = drone.true_position() + self.tracking.estimate_error_metres()
            return PretendMessage(x=estimate[0], y=estimate[1], z=-(estimate[2] + PX4_HEIGHT_OF_FLOOR_METRES),
                                  vx=drone.velocity[0], vy=drone.velocity[1], vz=-drone.velocity[2],
                                  dist_bottom=estimate[2] + LASER_READING_ON_FLOOR_METRES,
                                  xy_valid=True, z_valid=True, heading=-drone.heading_radians)
        if name == "vehicle_status":
            nav_state = NAV_STATE_OFFBOARD if self.offboard else NAV_STATE_POSITION
            return PretendMessage(nav_state=nav_state, arming_state=2 if drone.armed else 1)
        if name == "vehicle_land_detected":
            return PretendMessage(landed=drone.landed)
        if name == "vehicle_attitude":
            half_tilt = math.radians(drone.tilt_degrees()) / 2
            return PretendMessage(q=[math.cos(half_tilt), math.sin(half_tilt), 0.0, 0.0])
        if name == "sensor_combined":
            noise = self.random.normal(0.0, GYRO_NOISE_RADIANS_PER_SECOND, size=3)
            return PretendMessage(gyro_rad=[noise[0], noise[1], drone.turn_rate_radians_per_second + noise[2]])
        if name == "estimator_status_flags":
            uses_camera = self.tracking.px4_uses_camera
            return PretendMessage(cs_ev_pos=uses_camera, cs_ev_yaw=uses_camera, cs_rng_hgt=True,
                                  cs_rng_fault=False, cs_mag_hdg=False)
        return PretendMessage(voltage_v=15.0, remaining=0.8)    # battery_status

    # ---- Delivering messages ------------------------------------------------------------------------

    def _send_later(self, delivery_seconds, topic, message, picture_seconds=None):
        self.message_number += 1
        heapq.heappush(self.waiting_messages, (delivery_seconds, self.message_number, topic, message, picture_seconds))

    def _deliver_due_messages(self):
        while self.waiting_messages and self.waiting_messages[0][0] <= self.clock.seconds:
            _due, _number, topic, message, picture_seconds = heapq.heappop(self.waiting_messages)
            if topic == CAMERA_TRACKING_TOPIC:
                self.tracking.px4_receives_report(self.clock.seconds, picture_seconds)
            self._deliver(topic, message)

    def _deliver(self, topic, message):
        for callback in self.subscriptions.get(topic, []):
            callback(message)

    # ---- Trace --------------------------------------------------------------------------------------

    def _record_trace(self, now):
        if self.trace_rows is None or now < self.next_trace_seconds:
            return
        self.next_trace_seconds += 0.1
        position = self.true_position_on_course()
        target = self.last_target_on_course if self.last_target_on_course is not None else [np.nan] * 3
        self.trace_rows.append({
            "seconds": round(now - CLOCK_START_SECONDS, 2),
            "stage": None if self.link is None else self.link.stage,
            "true_east_metres": round(position[0], 3), "true_north_metres": round(position[1], 3),
            "true_height_metres": round(position[2], 3),
            "true_heading_degrees": round(math.degrees(self.true_heading_on_course()), 1),
            "target_east_metres": round(target[0], 3), "target_north_metres": round(target[1], 3),
            "px4_uses_camera": self.tracking.px4_uses_camera,
            "checkpoints": len(self.referee.checkpoint_seconds), "touches": self.referee.touches,
        })


class BenchPublisher:
    def __init__(self, node, topic):
        self.node = node
        self.topic = topic

    def publish(self, message):
        self.node.receive_published(self.topic, message)


class BenchRosClock:
    """node.get_clock(): now().nanoseconds, and now().to_msg() with sec and nanosec. The ROS
    clock and the simulated clock are the same here, so picture stamps and ages agree."""

    def __init__(self, clock):
        self.clock = clock

    def now(self):
        return self

    @property
    def nanoseconds(self):
        return int(round(self.clock.seconds * 1e9))

    def to_msg(self):
        stamp = PretendMessage()
        _stamp(stamp, self.clock.seconds)
        return stamp


def _stamp(stamp, seconds):
    stamp.sec = int(seconds)
    stamp.nanosec = int(round((seconds - int(seconds)) * 1e9))


def _heading_from_target(orientation):
    """The heading in a pose target, or None when all-zero (flight_interface keeps the heading)."""
    w = orientation.w if isinstance(orientation.w, (int, float)) else 0.0
    z = orientation.z if isinstance(orientation.z, (int, float)) else 0.0
    if w == 0.0 and z == 0.0:
        return None
    return 2.0 * math.atan2(z, w)
