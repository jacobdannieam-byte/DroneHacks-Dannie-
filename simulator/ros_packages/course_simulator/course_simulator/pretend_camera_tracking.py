"""Pretends to be the real camera tracking program (cuVSLAM).

SIMULATOR ONLY.

What the real program does
--------------------------
On the drone, cuVSLAM follows visual details from one camera picture to the next, and
works out how the drone has moved. This is the drone's only horizontal position source.
It publishes on /visual_slam/tracking/odometry, and the organizers' bridge program
passes that on to PX4.

What this file does instead
---------------------------
We have no camera pictures in this simulator, so this program takes the drone's true
position and makes it worse in the same ways the real tracking is imperfect:

  - It arrives late, because processing pictures takes time.
  - It has small random jitter.
  - It slowly drifts, by an amount that grows with the distance flown. Normally only
    sideways. A flight can add a height drift too, with run_test_flight.sh's
    --camera-height-drift, which sets SIMULATOR_CAMERA_HEIGHT_DRIFT_PER_METRE_FLOWN.
    SIMULATOR_CAMERA_DRIFT_PER_METRE_FLOWN and SIMULATOR_CAMERA_DRIFT_DIRECTION_DEGREES set the
    sideways drift's size and direction (fly_course_in_gazebo.sh's --tracking-drift-per-metre
    and --tracking-drift-direction-degrees).
  - If the drone was put down turned a little from the heading the flight program assumes
    (SIMULATOR_LAUNCH_HEADING_ERROR_DEGREES, fly_course_in_gazebo.sh's
    --launch-heading-error-degrees), it reports in the drone's real starting directions, as the
    real program would: it measures from wherever the drone pointed when it started.

It publishes the same message on the same topic, in the same directions, so the
organizers' code and our flight code cannot tell the difference.

What it does not copy: the real program can lose track, for example when facing a blank
wall up close. Here, tracking only fails if you ask it to, with the
"lose_tracking_after_seconds" setting. The simulator cannot predict when real tracking
fails, because that depends on what the real walls look like.
"""
import os
from collections import deque

import numpy as np
import rclpy
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node

from course_simulator.true_drone_state import (
    DEFAULT_START_HEADING_DEGREES,
    TrueDroneState,
    rotation_matrix_from_quaternion,
)

# The real camera produces 30 pictures per second (read from drone 16).
REPORTS_PER_SECOND = 30.0

# Our own choice: how often this program says whether tracking is working.
STATUS_REPORTS_PER_SECOND = 10.0

# How late each report is. It matches the drone's setting for this delay, EKF2_EV_DELAY.
# The organizers' documents suggest the true value could be up to about 106 ms.
REPORT_DELAY_SECONDS = 0.060

# Estimated: about 5 mm of jitter, from the drone at rest moving 0.3 to 2 mm in its reports.
# We use a larger value because a moving drone should jitter more than a still one.
POSITION_JITTER_METRES = 0.005

# Estimated: an error of 1% of the distance flown. We have not measured this yet. A flight can
# change it, for example 0 for no drift.
DRIFT_METRES_PER_METRE_FLOWN = float(os.environ.get("SIMULATOR_CAMERA_DRIFT_PER_METRE_FLOWN", "0.01"))
# Empty means a direction chosen from drift_seed, as before. Otherwise degrees from the start
# frame's forward towards its left.
DRIFT_DIRECTION_DEGREES_TEXT = os.environ.get("SIMULATOR_CAMERA_DRIFT_DIRECTION_DEGREES", "").strip()
# Zero unless a flight sets it: how far the drone was put down turned from the assumed heading,
# in degrees, positive towards the left (anticlockwise seen from above).
LAUNCH_HEADING_ERROR_DEGREES = float(os.environ.get("SIMULATOR_LAUNCH_HEADING_ERROR_DEGREES", "0"))
# Normally zero: the reported height doesn't drift. Nobody has measured the real camera
# tracking's height drift. A flight can set this, to see what a drift would do, for example 0.01
# for 1 cm higher per metre flown (negative means lower).
HEIGHT_DRIFT_METRES_PER_METRE_FLOWN = float(os.environ.get("SIMULATOR_CAMERA_HEIGHT_DRIFT_PER_METRE_FLOWN", "0"))

# The size of error the program claims for itself, as a variance (error squared). The
# real values were not recorded, so this assumes a claimed error of about 2 cm.
CLAIMED_POSITION_VARIANCE = 0.02 ** 2
CLAIMED_ANGLE_VARIANCE = 0.01 ** 2
CLAIMED_SPEED_VARIANCE = 0.05 ** 2

# Positions in the 6x6 covariance table used by ROS messages, for the values on its
# diagonal: the three positions followed by the three angles.
COVARIANCE_DIAGONAL_POSITIONS = (0, 7, 14, 21, 28, 35)

# The real tracking program's codes for "tracking is fine" and "tracking is lost". From the
# organizers' code, px4_odom_bridge.py.
VISUAL_ODOMETRY_STATE_TRACKING = 1
VISUAL_ODOMETRY_STATE_LOST = 2


class PretendCameraTracking(Node):

    def __init__(self):
        super().__init__("pretend_camera_tracking")
        self.declare_parameter("start_heading_degrees", DEFAULT_START_HEADING_DEGREES)
        self.declare_parameter("drift_seed", 1)
        # Set above zero to test what happens when tracking is lost. Zero means never.
        self.declare_parameter("lose_tracking_after_seconds", 0.0)

        self.true_state = TrueDroneState(self, self.get_parameter("start_heading_degrees").value)
        self.random = np.random.default_rng(self.get_parameter("drift_seed").value)

        # The drift grows in one fixed direction. Real drift wanders, but one direction
        # is the worst case for reaching a gap, and it is easy to reason about.
        random_angle = self.random.uniform(0.0, 2.0 * np.pi)
        if DRIFT_DIRECTION_DEGREES_TEXT:
            random_angle = np.radians(float(DRIFT_DIRECTION_DEGREES_TEXT))
        self.drift_direction = np.array([np.cos(random_angle), np.sin(random_angle), 0.0])
        self.get_logger().info(f"Drift {DRIFT_METRES_PER_METRE_FLOWN} m per metre flown, towards "
                               f"{np.degrees(random_angle):.1f} degrees; launch heading error "
                               f"{LAUNCH_HEADING_ERROR_DEGREES} degrees.")
        self.distance_flown_metres = 0.0
        self.previous_true_position = None

        self.start_time_seconds = self.now_in_seconds()
        self.waiting_reports = deque()

        self.odometry_publisher = self.create_publisher(Odometry, "/visual_slam/tracking/odometry", 10)
        self.status_publisher = self.create_publisher(VisualSlamStatus, "/visual_slam/status", 10)
        self.create_timer(1.0 / REPORTS_PER_SECOND, self.take_new_measurement_and_publish_due_ones)
        self.create_timer(1.0 / STATUS_REPORTS_PER_SECOND, self.publish_status)

    def now_in_seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def tracking_is_lost(self):
        lose_after_seconds = self.get_parameter("lose_tracking_after_seconds").value
        if lose_after_seconds <= 0.0:
            return False
        return self.now_in_seconds() - self.start_time_seconds > lose_after_seconds

    def take_new_measurement_and_publish_due_ones(self):
        if not self.true_state.has_data:
            return
        self.add_distance_flown()
        self.waiting_reports.append(self.build_report())
        self.publish_reports_that_are_old_enough()

    def add_distance_flown(self):
        position = self.true_state.position_in_start_frame_metres
        if self.previous_true_position is not None:
            self.distance_flown_metres += float(np.linalg.norm(position - self.previous_true_position))
        self.previous_true_position = position.copy()

    def build_report(self):
        drift_metres = self.drift_direction * self.distance_flown_metres * DRIFT_METRES_PER_METRE_FLOWN
        drift_metres[2] += self.distance_flown_metres * HEIGHT_DRIFT_METRES_PER_METRE_FLOWN
        jitter_metres = self.random.normal(0.0, POSITION_JITTER_METRES, 3)
        true_position = turned_into_the_drones_start_directions(self.true_state.position_in_start_frame_metres)
        reported_position = true_position + drift_metres + jitter_metres

        report = Odometry()
        # The time stamp is when the measurement was taken, not when it gets published.
        report.header.stamp = self.get_clock().now().to_msg()
        report.header.frame_id = "odom"
        report.child_frame_id = "base_link"

        report.pose.pose.position.x = float(reported_position[0])
        report.pose.pose.position.y = float(reported_position[1])
        report.pose.pose.position.z = float(reported_position[2])

        w, x, y, z = rotation_turned_into_the_drones_start_directions(self.true_state.body_rotation_in_start_frame)
        report.pose.pose.orientation.w = float(w)
        report.pose.pose.orientation.x = float(x)
        report.pose.pose.orientation.y = float(y)
        report.pose.pose.orientation.z = float(z)

        self.fill_in_velocity(report)
        self.fill_in_claimed_errors(report)
        return report

    def fill_in_velocity(self, report):
        # The real program reports speed in body directions, so we convert from the start frame.
        body_directions = rotation_matrix_from_quaternion(self.true_state.body_rotation_in_start_frame)
        velocity_along_body = body_directions.T @ self.true_state.velocity_in_start_frame_metres_per_second
        report.twist.twist.linear.x = float(velocity_along_body[0])
        report.twist.twist.linear.y = float(velocity_along_body[1])
        report.twist.twist.linear.z = float(velocity_along_body[2])

    def fill_in_claimed_errors(self, report):
        positions = COVARIANCE_DIAGONAL_POSITIONS
        for table_position in positions[:3]:
            report.pose.covariance[table_position] = CLAIMED_POSITION_VARIANCE
            report.twist.covariance[table_position] = CLAIMED_SPEED_VARIANCE
        for table_position in positions[3:]:
            report.pose.covariance[table_position] = CLAIMED_ANGLE_VARIANCE
            report.twist.covariance[table_position] = CLAIMED_ANGLE_VARIANCE

    def publish_reports_that_are_old_enough(self):
        now = self.get_clock().now()
        while self.waiting_reports:
            oldest_report = self.waiting_reports[0]
            age_seconds = (now - rclpy.time.Time.from_msg(oldest_report.header.stamp)).nanoseconds * 1e-9
            if age_seconds < REPORT_DELAY_SECONDS:
                return
            self.waiting_reports.popleft()
            if not self.tracking_is_lost():
                self.odometry_publisher.publish(oldest_report)

    def publish_status(self):
        status = VisualSlamStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.vo_state = VISUAL_ODOMETRY_STATE_LOST if self.tracking_is_lost() else VISUAL_ODOMETRY_STATE_TRACKING
        self.status_publisher.publish(status)


def turned_into_the_drones_start_directions(position_metres):
    """The start frame here is the assumed one. If the drone really started turned by
    LAUNCH_HEADING_ERROR_DEGREES, real tracking would measure along its real directions, so turn
    the position the other way by that angle."""
    angle = -np.radians(LAUNCH_HEADING_ERROR_DEGREES)
    turned = np.array(position_metres, dtype=float)
    turned[0] = np.cos(angle) * position_metres[0] - np.sin(angle) * position_metres[1]
    turned[1] = np.sin(angle) * position_metres[0] + np.cos(angle) * position_metres[1]
    return turned


def rotation_turned_into_the_drones_start_directions(rotation_wxyz):
    """The same turn for the drone's rotation (w, x, y, z): a turn about "up" applied first.
    The usual formula for multiplying two quaternions."""
    half_angle = -np.radians(LAUNCH_HEADING_ERROR_DEGREES) / 2.0
    turn_w, turn_z = np.cos(half_angle), np.sin(half_angle)
    w, x, y, z = rotation_wxyz
    return np.array([turn_w * w - turn_z * z, turn_w * x - turn_z * y, turn_w * y + turn_z * x, turn_w * z + turn_z * w])


def main():
    rclpy.init()
    rclpy.spin(PretendCameraTracking())
