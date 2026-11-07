"""Listens to the pretend depth camera on ROS 2, measures its rate, and saves a picture.
TRIAL ONLY, SIMULATOR ONLY.

What it does
------------
It listens to /simulator_only/oak/depth/image (published by gazebo_camera_to_ros.py) for
LISTENING_SECONDS, then prints:

  - how many depth pictures arrived per second of real time;
  - how many arrived per second of simulated time, worked out from the pictures' own time
    stamps (Gazebo's clock);
  - what share of the last picture's pixels saw something, and the nearest and farthest
    distances in it.

It saves the last picture as a coloured image in simulator/gazebo_trial/generated/pictures/,
so a person can check that the camera sees the course where it should.

This is a plain ROS 2 listener, like the drone's own programs, so it also proves that the
pictures really arrive on ROS 2 and not only inside Gazebo.

Run it inside the drone-simulator-gazebo box while run_px4_in_gazebo.sh is running.
"""
import argparse
import os
import time

import matplotlib
matplotlib.use("Agg")  # Draw to a file. The box has no screen.
import matplotlib.pyplot as pyplot  # noqa: E402
import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402

FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
FOLDER_FOR_PICTURES = os.path.join(FOLDER_OF_THIS_FILE, "generated", "pictures")

DEPTH_TOPIC = "/simulator_only/oak/depth/image"

# Our own choice: long enough to average out a few slow pictures.
LISTENING_SECONDS = 20.0


class DepthPictureListener(Node):

    def __init__(self):
        super().__init__("simulator_only_save_depth_picture")
        self.arrival_wall_seconds = []
        self.stamp_simulated_seconds = []
        self.last_picture_metres = None
        self.create_subscription(Image, DEPTH_TOPIC, self.on_depth_picture, 10)

    def on_depth_picture(self, message):
        self.arrival_wall_seconds.append(time.monotonic())
        self.stamp_simulated_seconds.append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        self.last_picture_metres = np.frombuffer(bytes(message.data), dtype=np.float32).reshape(message.height, message.width)


def listen(node, listening_seconds):
    end_wall_seconds = time.monotonic() + listening_seconds
    while time.monotonic() < end_wall_seconds:
        rclpy.spin_once(node, timeout_sec=0.1)


def print_rates(node):
    picture_count = len(node.arrival_wall_seconds)
    if picture_count < 2:
        raise SystemExit(
            f"Only {picture_count} depth pictures arrived on {DEPTH_TOPIC}. Check that run_px4_in_gazebo.sh is "
            "running and that gazebo_camera_to_ros.py printed no errors."
        )
    wall_seconds = node.arrival_wall_seconds[-1] - node.arrival_wall_seconds[0]
    simulated_seconds = node.stamp_simulated_seconds[-1] - node.stamp_simulated_seconds[0]
    gaps = picture_count - 1
    print(f"Depth pictures received: {picture_count}")
    print(f"  per second of real time:      {gaps / wall_seconds:.1f}")
    print(f"  per second of simulated time: {gaps / simulated_seconds:.1f}")
    print(f"  simulated seconds per real second while listening: {simulated_seconds / wall_seconds:.2f}")


def describe_and_save_picture(picture_metres, label):
    saw_something = np.isfinite(picture_metres)
    print(f"Last picture: {picture_metres.shape[1]} x {picture_metres.shape[0]} pixels, "
          f"{100.0 * saw_something.mean():.0f}% saw something within range")
    if saw_something.any():
        print(f"  nearest {picture_metres[saw_something].min():.2f} m, farthest {picture_metres[saw_something].max():.2f} m")

    os.makedirs(FOLDER_FOR_PICTURES, exist_ok=True)
    path = os.path.join(FOLDER_FOR_PICTURES, f"depth_{label}.png")
    figure, axes = pyplot.subplots(figsize=(8, 5.5))
    # Pixels that saw nothing are shown in white.
    shown = np.where(saw_something, picture_metres, np.nan)
    # The colours run from zero to this picture's own farthest distance, so that a picture of
    # nearby walls still shows clear differences.
    farthest_shown_metres = np.nanmax(shown) if saw_something.any() else 1.0
    drawing = axes.imshow(shown, cmap="viridis_r", vmin=0.0, vmax=farthest_shown_metres)
    figure.colorbar(drawing, ax=axes, label="distance straight ahead (m)")
    axes.set_title(f"Pretend OAK-D S2 depth picture ({label})")
    axes.set_axis_off()
    figure.savefig(path, dpi=100, bbox_inches="tight")
    print(f"Saved the picture as {os.path.relpath(path)}")


def main():
    parser = argparse.ArgumentParser(description="Measures the pretend depth camera's rate on ROS 2 and saves a picture.")
    parser.add_argument("--label", default="latest", help="A word for the saved picture's file name, for example 'hovering'.")
    parser.add_argument("--seconds", type=float, default=LISTENING_SECONDS, help="How long to listen.")
    arguments = parser.parse_args()

    rclpy.init()
    node = DepthPictureListener()
    listen(node, arguments.seconds)
    print_rates(node)
    describe_and_save_picture(node.last_picture_metres, arguments.label)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
