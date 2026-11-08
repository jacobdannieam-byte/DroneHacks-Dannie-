"""The depth camera program: turns the drone's stereo pictures into obstacle points, for the
flight program to build its live map from.

REAL DRONE CODE, also used unchanged in the simulator. Start it with `ros2 run team_nav
depth_camera` (the test_flights and course_run launch files start it).

What it does, about 10 times a second
-------------------------------------
1. Takes the newest left and right pictures that were taken at the same moment (it keeps the
   last few of each eye, so one late picture doesn't lose the pair).
2. Works out the distance to everything in them (stereo_depth.py).
3. Turns the distances into 3D points in the drone's own directions, measured from the drone's
   centre: forward, left and up, in metres.
4. Publishes the points on OBSTACLE_POINTS_TOPIC, stamped with the time the pictures were taken.
   The flight program places them on the course using where the drone was at that time.
5. Writes one line per picture pair to a records file, and saves a small depth picture once a
   second, so we can study real flights afterwards.

What it does not do: decide anything, or send anything to PX4. It only reads pictures.
"""
import csv
import os
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField

from team_nav import stereo_depth
from team_nav.run_log import records_folder

# From the organizers' drone software (the live topic list of drone 16, 2026-11-07).
LEFT_PICTURE_TOPIC = "/oak/left/image_raw"
RIGHT_PICTURE_TOPIC = "/oak/right/image_raw"
LEFT_CALIBRATION_TOPIC = "/oak/left/camera_info"
RIGHT_CALIBRATION_TOPIC = "/oak/right/camera_info"

# Our own name. The flight program listens here.
OBSTACLE_POINTS_TOPIC = "/team_nav/obstacle_points"
OBSTACLE_POINTS_FRAME = "drone_body_forward_left_up"

# From the organizers' launch line on drone 16 (oakd_vslam.launch.py camera_x:=0.1053
# camera_z:=0.0304): the camera sits 10.5 cm in front of the drone's centre and 3 cm above it.
CAMERA_FORWARD_OF_CENTRE_METRES = 0.1053
CAMERA_ABOVE_CENTRE_METRES = 0.0304

# Measured on drone 16 on 2026-11-08: one depth picture takes about 30 ms on its GPU, so 10 a
# second leaves the GPU free most of the time.
# Our own choice. 10 depth pictures a second is plenty at the drone's 0.3 m/s (3 cm between
# pictures), and leaves the processors time for everything else.
LONGEST_RATE_PER_SECOND = 10.0

# Our own choice. Two pictures count as a pair if taken within this time of each other. The
# real camera takes both at once; the drone sees them arrive up to ~40 ms apart (measured
# 2026-11-07: 25 pictures a second per eye).
PAIR_TOLERANCE_SECONDS = 0.02

# Our own choice. Beyond this, stereo distances are too rough to plan with (at 3.5 m, one
# pixel of shift error is about 0.6 m of distance error at 512 pixels wide).
FARTHEST_USEFUL_METRES = 3.5

# Our own choice. The last few pictures of each eye are kept, so a pair can be found even when
# the two eyes' pictures arrive in a different order or one is late. In the simulator on
# 2026-11-08, pairing only the newest two lost most pictures in flight.
PICTURES_KEPT_PER_EYE = 5

# Our own choice: every 4th pixel in each direction, about 10,000 points a picture at most.
EVERY_NTH_PIXEL = 4

# Our own choice: one saved depth picture a second is enough to see what the drone saw.
SECONDS_BETWEEN_SAVED_PICTURES = 1.0


class DepthCamera(Node):

    def __init__(self):
        super().__init__("team_nav_depth_camera")
        pictures = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.recent_left = deque(maxlen=PICTURES_KEPT_PER_EYE)
        self.recent_right = deque(maxlen=PICTURES_KEPT_PER_EYE)
        self.last_used_stamp_seconds = 0.0
        self.calibration = None
        self.eye_spacing_metres = None
        self.last_depth_seconds = 0.0
        self.last_saved_picture_seconds = 0.0
        self.create_subscription(Image, LEFT_PICTURE_TOPIC, self.on_left_picture, pictures)
        self.create_subscription(Image, RIGHT_PICTURE_TOPIC, self.on_right_picture, pictures)
        self.create_subscription(CameraInfo, LEFT_CALIBRATION_TOPIC, self.on_left_calibration, 10)
        self.create_subscription(CameraInfo, RIGHT_CALIBRATION_TOPIC, self.on_right_calibration, 10)
        self.publisher = self.create_publisher(PointCloud2, OBSTACLE_POINTS_TOPIC, 2)
        self.records = DepthRecords()
        self.get_logger().info(f"Depth camera started. Calculator: {stereo_depth.which_calculator()}. "
                               f"Records in {self.records.folder}.")

    # ---- Receiving -------------------------------------------------------------------------------

    def on_left_calibration(self, message):
        self.calibration = message
        self.note_eye_spacing(message)

    def on_right_calibration(self, message):
        self.note_eye_spacing(message)

    def note_eye_spacing(self, message):
        """One camera's projection holds -(focal length x eye spacing) in its 4th number; the
        other's holds 0. ROS usually puts it on the right camera, but drone 16 puts it on the
        left (measured 2026-11-08: left P[3] = -47.689, right P[3] = 0). So take whichever has it."""
        focal_length = message.p[0]
        if focal_length > 0 and message.p[3] != 0:
            self.eye_spacing_metres = abs(message.p[3]) / focal_length

    def on_left_picture(self, message):
        self.recent_left.append(message)
        self.make_depth_if_pair_is_ready()

    def on_right_picture(self, message):
        self.recent_right.append(message)
        self.make_depth_if_pair_is_ready()

    # ---- Depth ---------------------------------------------------------------------------------

    def make_depth_if_pair_is_ready(self):
        if self.calibration is None or self.eye_spacing_metres is None:
            return
        now = time.monotonic()
        if now - self.last_depth_seconds < 1.0 / LONGEST_RATE_PER_SECOND:
            return
        pair = self.newest_unused_pair()
        if pair is None:
            return
        self.last_depth_seconds = now
        self.last_used_stamp_seconds = _stamp_seconds(pair[0])
        self.make_depth(*pair)

    def newest_unused_pair(self):
        """The newest (left, right) pictures taken within PAIR_TOLERANCE_SECONDS of each other,
        newer than the last pair used, or None."""
        best = None
        for left in self.recent_left:
            left_seconds = _stamp_seconds(left)
            if left_seconds <= self.last_used_stamp_seconds:
                continue
            for right in self.recent_right:
                if abs(left_seconds - _stamp_seconds(right)) > PAIR_TOLERANCE_SECONDS:
                    continue
                if best is None or left_seconds > _stamp_seconds(best[0]):
                    best = (left, right)
        return best

    def make_depth(self, left_message, right_message):
        start = time.monotonic()
        left, factor = stereo_depth.shrink_to_working_width(stereo_depth.gray_from_colour(_picture(left_message)))
        right, _ = stereo_depth.shrink_to_working_width(stereo_depth.gray_from_colour(_picture(right_message)))
        focal_length = self.calibration.k[0] / factor
        distances = stereo_depth.distances_metres(left, right, focal_length, self.eye_spacing_metres)
        distances[distances > FARTHEST_USEFUL_METRES] = np.nan
        points = stereo_depth.points_ahead_of_camera(distances, focal_length, self.calibration.k[2] / factor,
                                                     self.calibration.k[5] / factor, EVERY_NTH_PIXEL)
        points = left_eye_points_to_body(points, self.eye_spacing_metres)
        self.publisher.publish(_point_message(points, left_message.header.stamp))
        calculation_seconds = time.monotonic() - start
        self.records.write_line(_stamp_seconds(left_message), calculation_seconds, distances, len(points))
        if start - self.last_saved_picture_seconds >= SECONDS_BETWEEN_SAVED_PICTURES:
            self.last_saved_picture_seconds = start
            self.records.save_picture(_stamp_seconds(left_message), distances)


def left_eye_points_to_body(points, eye_spacing_metres):
    """Apply the calibrated eye spacing and the documented midpoint mount translation."""
    # Reconstruction is relative to the left eye, not the midpoint of the stereo pair.
    # CameraInfo supplies eye spacing; the mount constants describe that midpoint.
    offset = np.array([CAMERA_FORWARD_OF_CENTRE_METRES, eye_spacing_metres/2.0,
                       CAMERA_ABOVE_CENTRE_METRES])
    return np.asarray(points) + offset


class DepthRecords:
    """depth_camera.csv (one line per picture pair) and a small depth picture each second."""

    def __init__(self):
        self.folder = os.path.join(records_folder(), time.strftime("%Y%m%d_%H%M%S") + "_depth_camera")
        os.makedirs(os.path.join(self.folder, "depth_pictures"), exist_ok=True)
        self.file = open(os.path.join(self.folder, "depth_camera.csv"), "w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["picture_time_seconds", "calculation_seconds", "trusted_fraction",
                              "nearest_ahead_metres", "points_published", "calculator"])
        self.calculator = stereo_depth.which_calculator()

    def write_line(self, picture_seconds, calculation_seconds, distances, point_count):
        height, width = distances.shape
        # The middle third of the picture: what is straight ahead of the drone.
        ahead = distances[height // 3: 2 * height // 3, width // 3: 2 * width // 3]
        nearest = float(np.nanmin(ahead)) if np.isfinite(ahead).any() else ""
        self.writer.writerow([f"{picture_seconds:.3f}", f"{calculation_seconds:.4f}",
                              f"{float(np.isfinite(distances).mean()):.3f}", nearest, point_count, self.calculator])
        self.file.flush()

    def save_picture(self, picture_seconds, distances):
        """A grey picture, near = white, far or unknown = black, as a PGM file (no extra library)."""
        brightness = np.nan_to_num(np.clip(255.0 * (1.0 - distances / FARTHEST_USEFUL_METRES), 0, 255)).astype(np.uint8)
        path = os.path.join(self.folder, "depth_pictures", f"{picture_seconds:.3f}.pgm")
        with open(path, "wb") as picture_file:
            picture_file.write(f"P5 {brightness.shape[1]} {brightness.shape[0]} 255\n".encode())
            picture_file.write(brightness.tobytes())


def _stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def _picture(message):
    """The picture's pixels as a NumPy array, for the two encodings the cameras use."""
    pixels = np.frombuffer(message.data, dtype=np.uint8)
    if message.encoding in ("mono8", "8UC1"):
        return pixels.reshape(message.height, message.step)[:, :message.width]
    rows = pixels.reshape(message.height, message.step)[:, :message.width * 3]
    picture = rows.reshape(message.height, message.width, 3)
    if message.encoding == "rgb8":
        return picture[..., ::-1]
    return picture


def _point_message(points, stamp):
    message = PointCloud2()
    message.header.stamp = stamp
    message.header.frame_id = OBSTACLE_POINTS_FRAME
    message.height = 1
    message.width = len(points)
    message.fields = [PointField(name=name, offset=4 * number, datatype=PointField.FLOAT32, count=1)
                      for number, name in enumerate(("x", "y", "z"))]
    message.is_bigendian = False
    message.point_step = 12
    message.row_step = 12 * len(points)
    message.is_dense = True
    message.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    return message


def points_from_message(message):
    """The (forward, left, up) points in a message published by this program, as an N x 3 array."""
    return np.frombuffer(message.data, dtype=np.float32).reshape(-1, 3)


def main():
    rclpy.init()
    node = DepthCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
