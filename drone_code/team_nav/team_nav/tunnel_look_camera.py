"""Collects the left camera's pictures, its lens numbers and the stereo depth points, for the
tunnel_first mission's LOOK step and the tunnel_look hand test.

REAL DRONE CODE, also used unchanged in the simulators. It only listens: it publishes nothing.

What it listens to
------------------
  /oak/left/image_raw and /oak/left/camera_info      the left picture and its lens numbers (fx,
                                                     cx come from camera_info's K matrix, never
                                                     assumed). On drone 16 and in Gazebo the left
                                                     picture is already rectified.
  /team_nav/obstacle_points                          depth_camera.py's stereo points, in the
                                                     drone's own directions from its centre.
The depth points are turned back to the camera's position with depth_camera.py's numbers. The
camera sits on the drone's centre line, so there is no sideways offset to add (from the organizers'
launch line on drone 16: camera_x 0.1053 m, camera_z 0.0304 m, no camera_y).

What it does not do: decide anything (tunnel_look.py), or wait for a picture forever. A missing
picture simply means no look this cycle.
"""
from collections import deque

import numpy as np

from team_nav import stereo_depth
from team_nav.depth_camera import (CAMERA_ABOVE_CENTRE_METRES, CAMERA_FORWARD_OF_CENTRE_METRES, LEFT_CALIBRATION_TOPIC,
                                   LEFT_PICTURE_TOPIC, OBSTACLE_POINTS_TOPIC, _picture, _stamp_seconds, points_from_message)

# From the organizers' launch line (see above): no sideways offset between camera and drone centre.
CAMERA_LEFT_OF_CENTRE_METRES = 0.0
# Our own choice: keep the last few pictures, so a look can use the newest one.
PICTURES_KEPT = 3


class TunnelLookCamera:

    def __init__(self, node):
        from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from sensor_msgs.msg import CameraInfo, Image, PointCloud2
        # Pictures are sent "best effort" by the camera driver; ask for the same, keep only the newest.
        pictures = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=2)
        self.pictures = deque(maxlen=PICTURES_KEPT)
        self.calibration = None
        self.latest_points_from_camera = None
        self.latest_points_stamp_seconds = None
        node.create_subscription(Image, LEFT_PICTURE_TOPIC, self.pictures.append, pictures)
        node.create_subscription(CameraInfo, LEFT_CALIBRATION_TOPIC, self._on_calibration, 10)
        node.create_subscription(PointCloud2, OBSTACLE_POINTS_TOPIC, self._on_points, 2)

    def _on_calibration(self, message):
        self.calibration = message

    def _on_points(self, message):
        points = np.array(points_from_message(message), dtype=float)
        points[:, 0] -= CAMERA_FORWARD_OF_CENTRE_METRES
        points[:, 1] -= CAMERA_LEFT_OF_CENTRE_METRES
        points[:, 2] -= CAMERA_ABOVE_CENTRE_METRES
        self.latest_points_from_camera = points
        self.latest_points_stamp_seconds = _stamp_seconds(message)

    def lens(self):
        """(fx, cx) in pixels from camera_info, or None before it arrives."""
        if self.calibration is None or self.calibration.k[0] <= 0:
            return None
        return float(self.calibration.k[0]), float(self.calibration.k[2])

    def newest_picture(self):
        """(stamp seconds, brightness array, the ROS message), or None."""
        if not self.pictures:
            return None
        message = self.pictures[-1]
        return _stamp_seconds(message), stereo_depth.gray_from_colour(_picture(message)), message
