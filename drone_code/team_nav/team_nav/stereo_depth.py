"""Turns a pair of pictures from the OAK-D's two front cameras into depth points: where things are,
in metres forward, left and up from the drone's centre.

How it fits: depth_points_node.py builds a StereoDepth from the two cameras' lens numbers
(camera_info) and hands it every pair of pictures; it publishes the points in the same format as
the simulator's pretend depth camera, for the drift correction (drift_correction.py). This file
uses OpenCV but never talks to ROS.

How stereo depth works: the two cameras sit side by side, a "baseline" apart (7.5 cm on the
OAK-D-S2). Something near looks shifted sideways between the left and right pictures more than
something far: depth = focal length x baseline / shift. The shift, in pixels, is the "disparity".
OpenCV's StereoSGBM ("semi-global block matching") finds it for every pixel.

The steps, each with the numbers measured on drone 16 earlier (the team, 2026-11-07):
  1. "Rectify" both pictures with the cameras' lens numbers, so a point appears on the same row in
     both, straightening lens bending ("distortion") on the way, and shrink them to half size:
     1024 x 640 becomes 512 x 320, where the focal length is about 317.25 pixels. Depth in metres
     was then 23.79 / disparity (317.25 px x 0.075 m).
  2. Match with StereoSGBM, numDisparities 64 (about 15 ms on the drone). 64 pixels of disparity is
     23.79 / 64 = 0.37 m, the nearest depth it can see.
  3. Keep depths from 0.37 m to 4.0 m, at every 4th pixel across and down.
  4. Turn each kept pixel into a point: the camera's right/down/forward directions become the
     drone's forward/left/up, plus where the camera sits on the drone.

What it deliberately does not do: correct for the camera being tilted on the drone. It assumes the
camera looks straight along the drone's nose; the organizers measured the real angles
(CAMERA_ROLL/PITCH/YAW in /etc/skybeta/drone.env) and they are small.
"""
from dataclasses import dataclass

import numpy as np

# Measured earlier on drone 16 (the team): half-size pictures, 64 disparities.
SHRINK_FACTOR = 2
NUMBER_OF_DISPARITIES = 64
# Our own choice, OpenCV's usual values for a 5 x 5 matching block on grey pictures.
BLOCK_SIZE_PIXELS = 5
SMOOTHNESS_SMALL = 8 * BLOCK_SIZE_PIXELS * BLOCK_SIZE_PIXELS
SMOOTHNESS_LARGE = 32 * BLOCK_SIZE_PIXELS * BLOCK_SIZE_PIXELS
UNIQUENESS_PERCENT = 10
SPECKLE_WINDOW_PIXELS = 100
SPECKLE_RANGE_PIXELS = 2
LEFT_RIGHT_DIFFERENCE_PIXELS = 1
# StereoSGBM gives disparity in 16ths of a pixel.
DISPARITY_STEPS_PER_PIXEL = 16.0


@dataclass
class CameraLens:
    """One camera's numbers from its sensor_msgs/CameraInfo: picture size, lens matrix k (3 x 3),
    distortion d, rectifying rotation r (3 x 3) and rectified projection p (3 x 4)."""
    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    r: np.ndarray
    p: np.ndarray

    @staticmethod
    def from_camera_info(message):
        return CameraLens(int(message.width), int(message.height), np.asarray(message.k, float).reshape(3, 3),
                          np.asarray(message.d, float), np.asarray(message.r, float).reshape(3, 3),
                          np.asarray(message.p, float).reshape(3, 4))


@dataclass
class DepthSettings:
    nearest_metres: float = 0.37        # from the team: 64 disparities at half size
    farthest_metres: float = 4.0        # from the team
    every_nth_pixel: int = 4            # from the team
    camera_forward_metres: float = 0.116    # from the team: the left camera, from the drone's centre
    camera_left_metres: float = 0.037
    camera_up_metres: float = 0.029
    baseline_if_unknown_metres: float = 0.075   # from docs/ARCHITECTURE.md: the OAK-D-S2's baseline


def rectified_projection(lens):
    """The 3 x 3 projection of the rectified picture: p's left part, or k if p was not filled in."""
    projection = lens.p[:, :3]
    if not projection.any():
        return lens.k.copy()
    return projection.copy()


def baseline_metres(left, right, settings):
    """(baseline, where it came from). The right camera's p holds -focal length x baseline in its
    fourth column; without that, the OAK-D-S2's known baseline."""
    focal = right.p[0, 0]
    if focal > 0.0 and right.p[0, 3] != 0.0:
        return abs(right.p[0, 3]) / focal, "camera_info"
    return settings.baseline_if_unknown_metres, "the default (camera_info has no baseline)"


class StereoDepth:
    def __init__(self, cv2, left, right, settings):
        self.cv2 = cv2
        self.settings = settings
        width, height = left.width // SHRINK_FACTOR, left.height // SHRINK_FACTOR
        self.size = (width, height)
        self.left_maps = self.rectify_maps(left)
        self.right_maps = self.rectify_maps(right)
        projection = rectified_projection(left) / SHRINK_FACTOR
        self.focal_pixels = projection[0, 0]
        self.focal_vertical_pixels = projection[1, 1]
        self.centre_column = projection[0, 2]
        self.centre_row = projection[1, 2]
        self.baseline_metres, self.baseline_source = baseline_metres(left, right, settings)
        self.matcher = cv2.StereoSGBM_create(
            minDisparity=0, numDisparities=NUMBER_OF_DISPARITIES, blockSize=BLOCK_SIZE_PIXELS,
            P1=SMOOTHNESS_SMALL, P2=SMOOTHNESS_LARGE, disp12MaxDiff=LEFT_RIGHT_DIFFERENCE_PIXELS,
            uniquenessRatio=UNIQUENESS_PERCENT, speckleWindowSize=SPECKLE_WINDOW_PIXELS,
            speckleRange=SPECKLE_RANGE_PIXELS)

    def rectify_maps(self, lens):
        """Where each pixel of the rectified half-size picture comes from in the full-size one."""
        projection = rectified_projection(lens) / SHRINK_FACTOR
        projection[2, 2] = 1.0
        rotation = lens.r if lens.r.any() else np.eye(3)
        return self.cv2.initUndistortRectifyMap(lens.k, lens.d, rotation, projection, self.size, self.cv2.CV_32FC1)

    def describe(self):
        return (f"half-size pictures {self.size[0]} x {self.size[1]}, focal length {self.focal_pixels:.2f} px, "
                f"baseline {self.baseline_metres:.4f} m from {self.baseline_source}, so depth = "
                f"{self.focal_pixels * self.baseline_metres:.2f} / disparity")

    def disparity_pixels(self, left_grey, right_grey):
        cv2 = self.cv2
        left = cv2.remap(left_grey, *self.left_maps, cv2.INTER_LINEAR)
        right = cv2.remap(right_grey, *self.right_maps, cv2.INTER_LINEAR)
        return self.matcher.compute(left, right).astype(np.float32) / DISPARITY_STEPS_PER_PIXEL

    def points(self, left_grey, right_grey):
        """N x 3 float32 points (forward, left, up) in metres from the drone's centre."""
        settings = self.settings
        step = settings.every_nth_pixel
        disparity = self.disparity_pixels(left_grey, right_grey)[::step, ::step]
        rows, columns = np.mgrid[0:disparity.shape[0], 0:disparity.shape[1]] * step
        seen = disparity > 0.0
        depth = np.zeros_like(disparity)
        depth[seen] = self.focal_pixels * self.baseline_metres / disparity[seen]
        kept = seen & (depth >= settings.nearest_metres) & (depth <= settings.farthest_metres)
        depth, rows, columns = depth[kept], rows[kept], columns[kept]
        # The camera's own directions: right, down and forward ("optical" frame).
        right_metres = (columns - self.centre_column) * depth / self.focal_pixels
        down_metres = (rows - self.centre_row) * depth / self.focal_vertical_pixels
        forward = depth + settings.camera_forward_metres
        left = -right_metres + settings.camera_left_metres
        up = -down_metres + settings.camera_up_metres
        return np.stack([forward, left, up], axis=1).astype(np.float32)
