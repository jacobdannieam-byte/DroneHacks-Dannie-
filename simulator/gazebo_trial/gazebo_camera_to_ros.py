"""Passes the pretend depth camera's pictures from Gazebo to ROS 2. TRIAL ONLY, SIMULATOR ONLY.

Why this file exists
--------------------
Gazebo and ROS 2 are separate programs, and each has its own way of sending messages. The usual
go-between, a package called ros_gz_bridge, is not available ready-made for our combination:
ROS 2 Humble with Gazebo Harmonic on the Mac's arm64 processor. Instead of compiling it, this
short program listens on Gazebo's side with Gazebo's own Python library, and repeats every
picture on ROS 2.

What it publishes
-----------------
  /simulator_only/oak/depth/image         sensor_msgs/Image, encoding "32FC1": one 4-byte
                                          decimal number per pixel, the distance in metres
                                          straight forward from the camera. A pixel that sees
                                          nothing within 10 m holds +infinity, and one closer
                                          than 0.35 m holds -infinity.
  /simulator_only/oak/depth/camera_info   sensor_msgs/CameraInfo: the picture size and the
                                          focal length, so a program can turn pixels into
                                          3D points.

The topic names start with /simulator_only/ because this camera does not exist on the drone.

Each message is stamped with Gazebo's simulated clock at the moment the picture was taken. PX4
runs in step with that same clock in this trial, so PX4's times and these times agree.

Every few seconds it also prints how many pictures it passed on per second of real time, which
is how we measured the camera's speed.

What it does not do
-------------------
It adds no errors to the pictures. The real camera's distances are imperfect; see
oak_d_s2_camera/model.sdf for how.
"""
import array
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from gz.msgs10.camera_info_pb2 import CameraInfo as GazeboCameraInfo
from gz.msgs10.image_pb2 import Image as GazeboImage
from gz.transport13 import Node as GazeboNode

# Set in models/oak_d_s2_camera/model.sdf. Gazebo also publishes the camera's settings on a
# topic next to the pictures, named camera_info.
GAZEBO_DEPTH_TOPIC = "/oak_d_s2/depth"
GAZEBO_CAMERA_INFO_TOPIC = "/oak_d_s2/camera_info"

ROS_DEPTH_TOPIC = "/simulator_only/oak/depth/image"
ROS_CAMERA_INFO_TOPIC = "/simulator_only/oak/depth/camera_info"

# The name of the camera's own set of directions. Our own choice. "Optical frame" is the ROS
# custom for camera directions: z points out of the lens, x to the right of the picture, and
# y down the picture.
CAMERA_FRAME_NAME = "oak_depth_optical_frame"

# Our own choice: how often to print the measured picture rate.
SECONDS_BETWEEN_RATE_REPORTS = 5.0

BYTES_PER_DEPTH_PIXEL = 4


class GazeboCameraToRos(Node):

    def __init__(self):
        super().__init__("simulator_only_gazebo_camera_to_ros")
        self.depth_publisher = self.create_publisher(Image, ROS_DEPTH_TOPIC, 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, ROS_CAMERA_INFO_TOPIC, 10)

        # Gazebo calls our functions from its own thread, while ROS runs the rate report from
        # this one, so the picture counter is protected by a lock.
        self.counter_lock = threading.Lock()
        self.pictures_since_last_report = 0
        self.last_report_wall_seconds = time.monotonic()

        # The Gazebo listener must stay alive for as long as this program runs, so we keep it.
        self.gazebo_node = GazeboNode()
        subscribed_to_depth = self.gazebo_node.subscribe(GazeboImage, GAZEBO_DEPTH_TOPIC, self.on_gazebo_depth_picture)
        subscribed_to_info = self.gazebo_node.subscribe(GazeboCameraInfo, GAZEBO_CAMERA_INFO_TOPIC, self.on_gazebo_camera_info)
        if not (subscribed_to_depth and subscribed_to_info):
            raise RuntimeError(
                f"Could not listen to Gazebo's topics {GAZEBO_DEPTH_TOPIC} and {GAZEBO_CAMERA_INFO_TOPIC}. "
                "Check that Gazebo's Python library is installed (this runs in the drone-simulator-gazebo box)."
            )
        self.create_timer(SECONDS_BETWEEN_RATE_REPORTS, self.report_picture_rate)
        self.get_logger().info(f"Passing Gazebo's {GAZEBO_DEPTH_TOPIC} on to ROS as {ROS_DEPTH_TOPIC}.")

    def on_gazebo_depth_picture(self, gazebo_picture):
        message = Image()
        message.header.stamp.sec = gazebo_picture.header.stamp.sec
        message.header.stamp.nanosec = gazebo_picture.header.stamp.nsec
        message.header.frame_id = CAMERA_FRAME_NAME
        message.height = gazebo_picture.height
        message.width = gazebo_picture.width
        message.encoding = "32FC1"
        message.is_bigendian = 0
        message.step = gazebo_picture.width * BYTES_PER_DEPTH_PIXEL
        # Wrapping the bytes in an array first is for speed, and was measured. Given plain
        # bytes, ROS's Python library checks every one of the 655,360 bytes separately, which
        # kept 45% of a processor core busy at 10 pictures per second.
        message.data = array.array("B", gazebo_picture.data)
        self.depth_publisher.publish(message)
        with self.counter_lock:
            self.pictures_since_last_report += 1

    def on_gazebo_camera_info(self, gazebo_info):
        message = CameraInfo()
        message.header.stamp.sec = gazebo_info.header.stamp.sec
        message.header.stamp.nanosec = gazebo_info.header.stamp.nsec
        message.header.frame_id = CAMERA_FRAME_NAME
        message.width = gazebo_info.width
        message.height = gazebo_info.height
        # The pretend camera has a perfect lens, so there is no distortion to describe.
        message.distortion_model = "plumb_bob"
        message.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        # k holds the focal lengths and the picture centre, in pixels; p is the same for a
        # camera whose pictures are already straightened; r is "no rotation".
        message.k = list(gazebo_info.intrinsics.k)
        message.p = list(gazebo_info.projection.p)
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.camera_info_publisher.publish(message)

    def report_picture_rate(self):
        now_wall_seconds = time.monotonic()
        with self.counter_lock:
            picture_count = self.pictures_since_last_report
            self.pictures_since_last_report = 0
        seconds_passed = now_wall_seconds - self.last_report_wall_seconds
        self.last_report_wall_seconds = now_wall_seconds
        self.get_logger().info(
            f"Passed on {picture_count / seconds_passed:.1f} depth pictures per second of real time."
        )


def main():
    rclpy.init()
    rclpy.spin(GazeboCameraToRos())


if __name__ == "__main__":
    main()
