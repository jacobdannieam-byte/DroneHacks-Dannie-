"""Checks once that the pretend stereo cameras give sensible depth. SIMULATOR ONLY.

What it does
------------
  1. Takes one left and one right picture from /oak/left/image_raw and /oak/right/image_raw,
     and the lens numbers from /oak/left/camera_info and /oak/right/camera_info, exactly as the
     team's depth code would on the drone.
  2. Matches the two pictures with OpenCV's StereoSGBM, a common stereo matcher. For each pixel
     it finds how many pixels a detail moved sideways between the two pictures (the "disparity").
     The distance straight ahead is then focal length x eye spacing / disparity.
  3. Works out the true distance for a grid of pixels by following a ray from the left eye
     through the simulator's 3D model of the world map (course_simulator's CourseModel, the same
     one the referee uses), from the drone's true position.
  4. Saves a picture: the left picture, the stereo depth, and the true depth, side by side, and
     prints a short table of stereo depth against true depth.

fly_course_in_gazebo.sh runs it once, while the drone still stands on the launch mark:

    python3 /repository/simulator/gazebo_full/check_stereo_depth.py OUTPUT_FOLDER

A true depth of "none" (infinity in the table) means the ray met only the safety net or nothing.
The referee's model leaves the net out on purpose, because the real net is see-through, but
Gazebo draws it as a solid textured sheet, so stereo does see it there.

What it does not do
-------------------
It is not the team's depth code and does not test it. The matcher settings are ordinary
starting values, not tuned. The Gazebo course sits 4 cm lower than the referee's model (see
simulator/gazebo_trial/make_gazebo_world.py), so near the floor the two can differ a little.
"""
import math
import os
import sys
import time

import cv2
import matplotlib

matplotlib.use("Agg")  # Draw straight to a file, with no window.
import matplotlib.pyplot as plt  # noqa: E402  (must come after choosing the drawing method)
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402

from course_simulator.course_model import CourseModel  # noqa: E402
from course_simulator.true_drone_state import TrueDroneState  # noqa: E402
from course_simulator.world import course_model_file_and_tables_folder  # noqa: E402

# Our own choice: how long to wait for the pictures and the true position before giving up.
LONGEST_WAIT_SECONDS = 120.0
# Our own choice: the two pictures of a pair must be stamped this close together.
LARGEST_STAMP_DIFFERENCE_SECONDS = 0.05

# Where the left eye sits on the drone, as (forward, left, up) from its centre, in metres. From
# models/drone_16_with_oak_d_stereo/model.sdf, which says where each number comes from.
LEFT_EYE_ON_BODY_METRES = (0.1053, 0.037577, 0.0304)

# Our own choices for the matcher: ordinary starting values for 512-pixel-wide pictures.
# 96 disparities reach down to about 0.25 m at this eye spacing and focal length.
MATCHER_DISPARITY_COUNT = 96
MATCHER_BLOCK_SIZE_PIXELS = 7
# OpenCV gives disparities in sixteenths of a pixel.
OPENCV_DISPARITY_SCALE = 16.0

# Our own choice: compare at a grid of this many pixels across and down.
COMPARISON_GRID_COLUMNS = 7
COMPARISON_GRID_ROWS = 5
# Our own choice: rays farther than this count as "nothing seen".
LONGEST_RAY_METRES = 9.0
# Our own choice. CourseModel.cast_rays gives up after a fixed number of steps, and a ray that
# grazes the floor takes small steps, so a long ray can come back as "nothing seen". Following
# each ray in pieces of this length, each a fresh cast, avoids that.
RAY_PIECE_METRES = 0.5

# From simulator/gazebo_trial/make_drone_16_model.py: the drone's centre is 4 cm above the floor
# while it stands. The referee's model counts heights from the drone's centre at the start,
# while Gazebo's floor is really 4 cm below it (gazebo_trial/make_gazebo_world.py), so in the
# referee's model every Gazebo point is this much higher than it is reported.
GAZEBO_FLOOR_BELOW_REFEREE_FLOOR_METRES = 0.04


class PictureCatcher(Node):
    """Keeps the newest picture and lens numbers from each eye."""

    def __init__(self):
        super().__init__("simulator_only_check_stereo_depth")
        self.left_picture = None
        self.right_picture = None
        self.left_info = None
        self.right_info = None
        self.create_subscription(Image, "/oak/left/image_raw", self.on_left_picture, 10)
        self.create_subscription(Image, "/oak/right/image_raw", self.on_right_picture, 10)
        self.create_subscription(CameraInfo, "/oak/left/camera_info", self.on_left_info, 10)
        self.create_subscription(CameraInfo, "/oak/right/camera_info", self.on_right_info, 10)
        self.true_state = TrueDroneState(self)

    def on_left_picture(self, message):
        self.left_picture = message

    def on_right_picture(self, message):
        self.right_picture = message

    def on_left_info(self, message):
        self.left_info = message

    def on_right_info(self, message):
        self.right_info = message

    def has_a_matching_pair(self):
        if None in (self.left_picture, self.right_picture, self.left_info, self.right_info):
            return False
        if not self.true_state.has_data:
            return False
        return abs(stamp_seconds(self.left_picture) - stamp_seconds(self.right_picture)) < LARGEST_STAMP_DIFFERENCE_SECONDS


def stamp_seconds(message):
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


def picture_as_array(message):
    if message.encoding != "bgr8":
        raise ValueError(f"Expected bgr8 pictures, as the OAK-D driver sends, but got {message.encoding}.")
    return np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(message.height, message.width, 3)


def stereo_depth_metres(left_bgr, right_bgr, focal_length_pixels, eye_spacing_metres):
    """Distance straight ahead for every pixel of the left picture, NaN where matching failed."""
    matcher = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=MATCHER_DISPARITY_COUNT,
        blockSize=MATCHER_BLOCK_SIZE_PIXELS,
        P1=8 * MATCHER_BLOCK_SIZE_PIXELS ** 2,
        P2=32 * MATCHER_BLOCK_SIZE_PIXELS ** 2,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2,
    )
    left_grey = cv2.cvtColor(left_bgr, cv2.COLOR_BGR2GRAY)
    right_grey = cv2.cvtColor(right_bgr, cv2.COLOR_BGR2GRAY)
    disparity_pixels = matcher.compute(left_grey, right_grey).astype(np.float32) / OPENCV_DISPARITY_SCALE
    depth = np.full(disparity_pixels.shape, np.nan, dtype=np.float32)
    matched = disparity_pixels > 0.0
    depth[matched] = focal_length_pixels * eye_spacing_metres / disparity_pixels[matched]
    return depth


def true_depth_metres(catcher, course, info, pixel_columns, pixel_rows):
    """The true distance straight ahead at each given pixel, by following rays through the world
    map's 3D model from the left eye's true position."""
    focal_x, centre_x = info.k[0], info.k[2]
    focal_y, centre_y = info.k[4], info.k[5]
    # A ray through a pixel, in the camera's "optical" directions: x right, y down, z forward.
    optical_x = (pixel_columns - centre_x) / focal_x
    optical_y = (pixel_rows - centre_y) / focal_y
    # The same ray in the drone's body directions: forward = z, left = -x, up = -y.
    body_directions = np.stack([np.ones_like(optical_x), -optical_x, -optical_y], axis=1)
    lengths = np.linalg.norm(body_directions, axis=1)
    body_directions = body_directions / lengths[:, np.newaxis]
    course_directions = body_directions @ catcher.true_state.body_directions_on_course().T
    eye_position = catcher.true_state.body_points_on_course_metres(np.array([LEFT_EYE_ON_BODY_METRES]))[0]
    eye_position[2] += GAZEBO_FLOOR_BELOW_REFEREE_FLOOR_METRES
    starts = np.repeat(eye_position[np.newaxis, :], len(course_directions), axis=0)
    ray_lengths_metres = cast_rays_in_pieces(course, starts, course_directions)
    # Straight-ahead distance is the ray length times the ray's forward part.
    return ray_lengths_metres / lengths


def cast_rays_in_pieces(course, starts, directions):
    """Like CourseModel.cast_rays, but follows each ray in short pieces (see RAY_PIECE_METRES)."""
    ray_lengths_metres = np.full(len(starts), np.inf)
    distance_so_far_metres = 0.0
    while distance_so_far_metres < LONGEST_RAY_METRES:
        still_looking = ~np.isfinite(ray_lengths_metres)
        if not np.any(still_looking):
            break
        piece_starts = starts[still_looking] + directions[still_looking] * distance_so_far_metres
        piece_lengths = course.cast_rays(piece_starts, directions[still_looking], RAY_PIECE_METRES)
        ray_lengths_metres[still_looking] = distance_so_far_metres + piece_lengths
        distance_so_far_metres += RAY_PIECE_METRES
    return ray_lengths_metres


def wait_for_pictures(catcher):
    end_seconds = time.monotonic() + LONGEST_WAIT_SECONDS
    while not catcher.has_a_matching_pair():
        if time.monotonic() > end_seconds:
            raise RuntimeError(
                f"No matching left and right pictures, camera info and true position arrived within "
                f"{LONGEST_WAIT_SECONDS:.0f} s. Check px4_and_gazebo/pretend_oak_stereo_cameras.txt in the run folder."
            )
        rclpy.spin_once(catcher, timeout_sec=0.1)


def save_picture(path, left_bgr, stereo_depth, grid_columns, grid_rows, true_depths):
    figure, (left_axes, depth_axes) = plt.subplots(1, 2, figsize=(14, 5))
    left_axes.imshow(left_bgr[:, :, ::-1])
    left_axes.set_title("Left eye picture (/oak/left/image_raw)")
    shown = depth_axes.imshow(stereo_depth, cmap="viridis_r", vmin=0.0, vmax=4.0)
    depth_axes.set_title("Stereo depth from StereoSGBM, metres (white = no match)\nnumbers: true depth from the world map ('net': only the net, which the map's model leaves out)")
    for column, row, true_depth in zip(grid_columns, grid_rows, true_depths):
        label = "net" if not math.isfinite(true_depth) else f"{true_depth:.2f}"
        depth_axes.text(column, row, label, color="red", fontsize=7, ha="center", va="center")
    figure.colorbar(shown, ax=depth_axes, label="metres straight ahead")
    figure.tight_layout()
    figure.savefig(path, dpi=110)
    plt.close(figure)


def run_check(output_folder):
    os.makedirs(output_folder, exist_ok=True)
    catcher = PictureCatcher()
    course = CourseModel(*course_model_file_and_tables_folder())
    wait_for_pictures(catcher)
    left_bgr = picture_as_array(catcher.left_picture)
    right_bgr = picture_as_array(catcher.right_picture)
    focal_length_pixels = catcher.left_info.k[0]
    # The right camera's P[3] is minus focal length times eye spacing.
    eye_spacing_metres = -catcher.right_info.p[3] / catcher.right_info.p[0]
    stereo_depth = stereo_depth_metres(left_bgr, right_bgr, focal_length_pixels, eye_spacing_metres)

    height, width = stereo_depth.shape
    column_choices = np.linspace(width * 0.15, width * 0.85, COMPARISON_GRID_COLUMNS)
    row_choices = np.linspace(height * 0.15, height * 0.85, COMPARISON_GRID_ROWS)
    grid_columns, grid_rows = np.meshgrid(column_choices, row_choices)
    grid_columns = grid_columns.ravel()
    grid_rows = grid_rows.ravel()
    true_depths = true_depth_metres(catcher, course, catcher.left_info, grid_columns, grid_rows)

    lines = [
        f"Pictures: {width} x {height}, focal length {focal_length_pixels:.3f} pixels, eye spacing {eye_spacing_metres:.5f} m.",
        f"Drone's true position on the course: {np.round(catcher.true_state.position_on_course_metres(), 3).tolist()} m.",
        f"Pixels with a stereo match: {100.0 * np.mean(np.isfinite(stereo_depth)):.0f}%.",
        "column  row  stereo_depth_m  true_depth_m",
    ]
    errors = []
    for column, row, true_depth in zip(grid_columns, grid_rows, true_depths):
        # The median of a small patch, because single pixels can be unmatched.
        patch = stereo_depth[int(row) - 2:int(row) + 3, int(column) - 2:int(column) + 3]
        stereo_value = float(np.nanmedian(patch)) if np.any(np.isfinite(patch)) else float("nan")
        lines.append(f"{column:6.0f} {row:4.0f} {stereo_value:14.2f} {true_depth:13.2f}")
        if math.isfinite(stereo_value) and math.isfinite(true_depth):
            errors.append(abs(stereo_value - true_depth))
    if errors:
        lines.append(f"Grid points compared: {len(errors)}. Median error {np.median(errors):.3f} m, "
                     f"largest {np.max(errors):.3f} m.")
    text = "\n".join(lines)
    print(text, flush=True)
    with open(os.path.join(output_folder, "stereo_depth_check.txt"), "w", encoding="utf-8") as text_file:
        text_file.write(text + "\n")
    cv2.imwrite(os.path.join(output_folder, "left_eye.png"), left_bgr)
    cv2.imwrite(os.path.join(output_folder, "right_eye.png"), right_bgr)
    save_picture(os.path.join(output_folder, "stereo_depth_check.png"), left_bgr, stereo_depth,
                 grid_columns, grid_rows, true_depths)


def main():
    if len(sys.argv) != 2:
        print("Give one output folder, for example: check_stereo_depth.py /repository/simulator/generated_files/x")
        sys.exit(2)
    rclpy.init()
    run_check(sys.argv[1])


if __name__ == "__main__":
    main()
