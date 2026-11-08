"""Publishes Gazebo's two pretend camera eyes on the OAK-D's real ROS 2 topics. SIMULATOR ONLY.

What the real camera does
-------------------------
On drone 16, the OAK-D driver publishes the left and right cameras' straightened ("rectified")
pictures, so a point in the world appears on the same picture row in both. The team's stereo
depth code finds how far each detail shifts sideways between the two pictures (the "disparity")
and turns that into a distance. Topics, all seen on drone 16:

  /oak/left/image_raw, /oak/right/image_raw      sensor_msgs/Image, "bgr8", 1024 x 640, ~25/s
  /oak/left/camera_info, /oak/right/camera_info  sensor_msgs/CameraInfo

What this file does instead
---------------------------
Gazebo draws the two eyes of models/drone_16_with_oak_d_stereo/model.sdf, at 512 x 320 and 10
pictures per second. This program listens to them with Gazebo's own Python library (the usual
Gazebo-to-ROS bridge package isn't published for our setup; simulator/gazebo_trial's
gazebo_camera_to_ros.py says more), turns Gazebo's red-green-blue order into the driver's
blue-green-red order, and publishes them under the real names, with camera_info holding drone
16's calibration scaled to the smaller pictures.

The topic names are the real ones, not /simulator_only/ ones, on purpose: the team's flight code
must find the pictures exactly where it finds them on the drone. The node's name says it is
pretend.

Time stamps: each picture pair is stamped with this computer's clock when its first picture
arrives, like every other pretend sensor in our simulator. Gazebo's own clock starts at zero, so
its stamps would look 56 years old next to everything else.

Both pictures of a pair get the SAME stamp. Gazebo draws both eyes at the same simulated moment
and says so in its own stamp, so we look that up: the first eye to arrive for a Gazebo moment
sets the stamp, and the other eye reuses it. The real OAK-D also takes both pictures at the same
instant. An earlier version stamped each eye on arrival; in flight the two arrived 30 to 40 ms
apart (median, measured on 2026-11-07 in run 20261107_193106), more than the 20 ms within which
the team's depth program pairs pictures, so it found almost no pairs once the drone took off.

Every few seconds it prints how many picture pairs it passed on per second of real time.

What it does not do
-------------------
It adds no camera noise, blur or exposure changes, and the pictures are half the real size.
"""
import array
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from gz.msgs10.image_pb2 import Image as GazeboImage
from gz.transport13 import Node as GazeboNode

# Set in models/drone_16_with_oak_d_stereo/model.sdf.
GAZEBO_LEFT_TOPIC = "/simulator_only/oak_left/image"
GAZEBO_RIGHT_TOPIC = "/simulator_only/oak_right/image"

# The real drone's topic names (seen on drone 16).
LEFT_PICTURE_TOPIC = "/oak/left/image_raw"
RIGHT_PICTURE_TOPIC = "/oak/right/image_raw"
LEFT_CAMERA_INFO_TOPIC = "/oak/left/camera_info"
RIGHT_CAMERA_INFO_TOPIC = "/oak/right/camera_info"

# The real drone's frame names (from drone 16's camera_info). "Optical frame" is the ROS custom
# for camera directions: z out of the lens, x to the right of the picture, y down it.
LEFT_FRAME_NAME = "oak_left_camera_optical_frame"
RIGHT_FRAME_NAME = "oak_right_camera_optical_frame"

# Drone 16's left camera calibration at 1024 x 640 (read from drone 16's camera_info).
REAL_FOCAL_LENGTH_X_PIXELS = 634.544
REAL_FOCAL_LENGTH_Y_PIXELS = 634.556
REAL_PICTURE_CENTRE_X_PIXELS = 507.673
REAL_PICTURE_CENTRE_Y_PIXELS = 314.526
# The right camera's P[3]: minus the focal length times the eye spacing (0.07515 m).
REAL_RIGHT_CAMERA_P3_PIXELS = -47.689
REAL_PICTURE_WIDTH_PIXELS = 1024

# Set in model.sdf: the pretend pictures are 512 x 320.
PRETEND_PICTURE_WIDTH_PIXELS = 512
PRETEND_PICTURE_HEIGHT_PIXELS = 320
# Halving the picture halves every lens number in pixels. (Exactly, the centre would move by a
# quarter of a pixel more, because pixel centres shift too. We ignore that; so does the model.)
SCALE = PRETEND_PICTURE_WIDTH_PIXELS / REAL_PICTURE_WIDTH_PIXELS

BYTES_PER_PIXEL = 3

# Our own choice: how many recent Gazebo moments to remember the shared stamp for. The two eyes
# of one moment arrive well within one picture interval (0.1 s) of each other, so a few is ample.
REMEMBERED_MOMENTS = 20

# Our own choice: how often to print the measured picture rate.
SECONDS_BETWEEN_RATE_REPORTS = 5.0


def camera_info_message(frame_name, p3_pixels):
    """drone 16's calibration, scaled to the pretend picture size."""
    focal_x = REAL_FOCAL_LENGTH_X_PIXELS * SCALE
    focal_y = REAL_FOCAL_LENGTH_Y_PIXELS * SCALE
    centre_x = REAL_PICTURE_CENTRE_X_PIXELS * SCALE
    centre_y = REAL_PICTURE_CENTRE_Y_PIXELS * SCALE
    message = CameraInfo()
    message.header.frame_id = frame_name
    message.width = PRETEND_PICTURE_WIDTH_PIXELS
    message.height = PRETEND_PICTURE_HEIGHT_PIXELS
    message.distortion_model = "plumb_bob"
    message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    message.k = [focal_x, 0.0, centre_x, 0.0, focal_y, centre_y, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    message.p = [focal_x, 0.0, centre_x, p3_pixels * SCALE, 0.0, focal_y, centre_y, 0.0, 0.0, 0.0, 1.0, 0.0]
    return message


class PretendOakStereoCameras(Node):

    def __init__(self):
        super().__init__("pretend_oak_stereo_cameras")
        self.left_picture_publisher = self.create_publisher(Image, LEFT_PICTURE_TOPIC, 10)
        self.right_picture_publisher = self.create_publisher(Image, RIGHT_PICTURE_TOPIC, 10)
        self.left_info_publisher = self.create_publisher(CameraInfo, LEFT_CAMERA_INFO_TOPIC, 10)
        self.right_info_publisher = self.create_publisher(CameraInfo, RIGHT_CAMERA_INFO_TOPIC, 10)
        self.left_info = camera_info_message(LEFT_FRAME_NAME, 0.0)
        self.right_info = camera_info_message(RIGHT_FRAME_NAME, REAL_RIGHT_CAMERA_P3_PIXELS)

        # Gazebo calls our functions from its own thread (possibly one per eye), and ROS runs the
        # rate report from this one, so the counter and the shared stamps are protected by a lock.
        self.counter_lock = threading.Lock()
        # Gazebo's simulated time of a picture, in nanoseconds -> the ROS stamp both eyes get.
        self.stamp_for_gazebo_moment = {}
        self.left_pictures_since_last_report = 0
        self.last_report_wall_seconds = time.monotonic()

        # The Gazebo listener must stay alive as long as this program runs, so we keep it.
        self.gazebo_node = GazeboNode()
        subscribed_left = self.gazebo_node.subscribe(GazeboImage, GAZEBO_LEFT_TOPIC, self.on_left_picture)
        subscribed_right = self.gazebo_node.subscribe(GazeboImage, GAZEBO_RIGHT_TOPIC, self.on_right_picture)
        if not (subscribed_left and subscribed_right):
            raise RuntimeError(
                f"Could not listen to Gazebo's topics {GAZEBO_LEFT_TOPIC} and {GAZEBO_RIGHT_TOPIC}. "
                "Check that this runs in the drone-simulator-gazebo-full box, where Gazebo's Python library is installed."
            )
        self.create_timer(SECONDS_BETWEEN_RATE_REPORTS, self.report_picture_rate)
        self.get_logger().info(f"Passing Gazebo's two camera eyes on to ROS as {LEFT_PICTURE_TOPIC} and {RIGHT_PICTURE_TOPIC}.")

    def on_left_picture(self, gazebo_picture):
        self.publish_picture(gazebo_picture, LEFT_FRAME_NAME, self.left_picture_publisher,
                             self.left_info, self.left_info_publisher)
        with self.counter_lock:
            self.left_pictures_since_last_report += 1

    def on_right_picture(self, gazebo_picture):
        self.publish_picture(gazebo_picture, RIGHT_FRAME_NAME, self.right_picture_publisher,
                             self.right_info, self.right_info_publisher)

    def shared_stamp(self, gazebo_picture):
        """The same ROS stamp for both eyes' pictures of one Gazebo moment."""
        gazebo_moment_nanoseconds = gazebo_picture.header.stamp.sec * 1_000_000_000 + gazebo_picture.header.stamp.nsec
        with self.counter_lock:
            stamp = self.stamp_for_gazebo_moment.get(gazebo_moment_nanoseconds)
            if stamp is None:
                stamp = self.get_clock().now().to_msg()
                self.stamp_for_gazebo_moment[gazebo_moment_nanoseconds] = stamp
                while len(self.stamp_for_gazebo_moment) > REMEMBERED_MOMENTS:
                    oldest_moment = min(self.stamp_for_gazebo_moment)
                    del self.stamp_for_gazebo_moment[oldest_moment]
        return stamp

    def publish_picture(self, gazebo_picture, frame_name, picture_publisher, info, info_publisher):
        stamp = self.shared_stamp(gazebo_picture)
        red_green_blue = np.frombuffer(gazebo_picture.data, dtype=np.uint8).reshape(
            gazebo_picture.height, gazebo_picture.width, BYTES_PER_PIXEL)
        # Reversing the last axis turns red-green-blue into blue-green-red, the driver's order.
        blue_green_red = np.ascontiguousarray(red_green_blue[:, :, ::-1])

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = frame_name
        message.height = gazebo_picture.height
        message.width = gazebo_picture.width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = gazebo_picture.width * BYTES_PER_PIXEL
        # Wrapping the bytes in an array is for speed: given plain bytes, ROS's Python library
        # checks each byte separately (measured in the Gazebo trial: 45% of a core for depth).
        message.data = array.array("B", blue_green_red.tobytes())
        picture_publisher.publish(message)

        info.header.stamp = stamp
        info_publisher.publish(info)

    def report_picture_rate(self):
        now_wall_seconds = time.monotonic()
        with self.counter_lock:
            picture_count = self.left_pictures_since_last_report
            self.left_pictures_since_last_report = 0
        seconds_passed = now_wall_seconds - self.last_report_wall_seconds
        self.last_report_wall_seconds = now_wall_seconds
        self.get_logger().info(f"Passed on {picture_count / seconds_passed:.1f} picture pairs per second of real time.")


def main():
    rclpy.init()
    rclpy.spin(PretendOakStereoCameras())


if __name__ == "__main__":
    main()
