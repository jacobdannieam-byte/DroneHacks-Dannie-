"""Puts the depth camera's obstacle points on the live map, during the flight.

REAL DRONE CODE, also used unchanged in the simulator.

How it fits in
--------------
depth_camera.py (its own program) publishes the points it sees, in the drone's own directions:
forward, left and up from the drone's centre, stamped with the time the pictures were taken.
This file listens to them inside the flight program. For each message it:
  1. finds where the drone was, and how it was turned, when the pictures were taken
     (flight_steps.DroneLink.tracked_pose_at);
  2. turns the points into camera tracking's frame ("odom"), using the drone's full rotation, so
     a tilt while flying doesn't make the floor look like a wall;
  3. moves them from camera tracking's frame to the course (east, north, height);
  4. hands the wall points to wall_alignment.py, which measures camera tracking's drift, and
     stores its correction in the start frame, so the next picture and everything else use it;
  5. hands them, with the camera's position, to the live map (live_map.LiveMap.add_points).

What it does not do: decide anything about where to fly (live_navigation.py does that).
"""
import time

import numpy as np

from team_nav.depth_camera import CAMERA_FORWARD_OF_CENTRE_METRES, OBSTACLE_POINTS_TOPIC, points_from_message
from team_nav.flight_steps import rotation_matrix
from team_nav.wall_alignment import WallAlignment

# Our own choice. Depth messages are only used while this many or fewer are waiting; the
# newest picture matters most.
POINTS_QUEUE_LENGTH = 2


class DepthToMap:

    def __init__(self, link, start_frame, live_map, course_map):
        from sensor_msgs.msg import PointCloud2
        self.link = link
        self.start_frame = start_frame
        self.live_map = live_map
        self.wall_alignment = WallAlignment(course_map)
        self.pictures_used = 0
        self.pictures_without_pose = 0
        self.last_picture_monotonic_seconds = None    # When the last picture was put on the map.
        link.node.create_subscription(PointCloud2, OBSTACLE_POINTS_TOPIC, self.on_obstacle_points, POINTS_QUEUE_LENGTH)

    def on_obstacle_points(self, message):
        if self.link.launch is None:
            return
        stamp_seconds = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        pose = self.link.tracked_pose_at(stamp_seconds)
        if pose is None:
            self.pictures_without_pose += 1
            return
        position_odom, rotation = pose
        turn = rotation_matrix(rotation)
        points_odom = points_from_message(message).astype(float) @ turn.T + position_odom
        camera_odom = position_odom + turn @ np.array([CAMERA_FORWARD_OF_CENTRE_METRES, 0.0, 0.0])
        points_on_course = self.course_from_odom(points_odom)
        camera_on_course = self.course_from_odom(camera_odom[None, :])[0]
        self.correct_drift(camera_on_course, points_on_course)
        self.live_map.add_points(camera_on_course, points_on_course)
        self.pictures_used += 1
        self.last_picture_monotonic_seconds = time.monotonic()

    def correct_drift(self, camera_on_course, points_on_course):
        """Lines up the points at flying height with the trusted walls, and stores the new
        correction in the start frame."""
        in_band = ((points_on_course[:, 2] >= self.live_map.lowest_counted_height_metres)
                   & (points_on_course[:, 2] <= self.live_map.highest_counted_height_metres))
        before = self.wall_alignment.correction_metres.copy()
        self.wall_alignment.add_points(camera_on_course, points_on_course[in_band])
        change = self.wall_alignment.correction_metres - before
        self.start_frame.drift_correction_metres = self.wall_alignment.correction_metres.copy()
        # The points and the camera were placed with the old correction; move them by the change too.
        points_on_course[:, :2] += change
        camera_on_course[:2] += change

    def seconds_since_last_picture(self):
        if self.last_picture_monotonic_seconds is None:
            return float("inf")
        return time.monotonic() - self.last_picture_monotonic_seconds

    def course_from_odom(self, points_odom):
        """N x 3 points in camera tracking's frame -> N x 3 (east, north, height) on the course.
        The same sum as start_frame.course_from_start_frame, for many points at once."""
        from_launch = points_odom - self.link.launch.tracked_position_metres
        on_floor = (self.start_frame.launch_mark_centre_metres
                    + from_launch[:, 0:1] * self.start_frame.forward_on_course
                    + from_launch[:, 1:2] * self.start_frame.left_on_course
                    + self.start_frame.drift_correction_metres)
        return np.column_stack([on_floor, from_launch[:, 2]])
