"""Listens to the depth points for a few seconds and prints how often they come and how far away
the wall straight ahead is, to compare with a tape measure. Run on the drone by
drone_code/depth_check_16.sh, or by hand inside the stack:  ros2 run team_nav depth_check --seconds 5

How it fits: it reads /team/depth_points (depth_points_node.py) and uses the drift correction's own
"straight ahead" (depth_ahead.py: the median of the points within 3 degrees of the nose), so it
measures exactly what the drift correction would use. It sends nothing and changes nothing.
"""
import argparse
import statistics
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import PointCloud2

from .depth_ahead import distance_straight_ahead, points_from_cloud
from .mission_settings import MissionSettings
from .stereo_depth import DepthSettings


def summary_lines(clouds, seconds, window_degrees, least_points, camera_forward_metres):
    """The lines to print, from [(points in the cloud, depth straight ahead or None)]."""
    if not clouds:
        return [f"No depth points in {seconds:g} s. Is depth_points running? The stack's competitor window says why."]
    ahead = [depth for _, depth in clouds if depth is not None]
    lines = [f"{len(clouds) / seconds:.1f} clouds per second, {statistics.median(n for n, _ in clouds):.0f} points per "
             f"cloud (median)"]
    if not ahead:
        lines.append(f"No good depth straight ahead (fewer than {least_points} points within {window_degrees:g} deg "
                     "of the nose): too far (over 4 m), too near (under 0.37 m), or a blank surface.")
        return lines
    median = statistics.median(ahead)
    lines.append(f"Straight ahead: {median:.3f} m from the drone's centre = {median - camera_forward_metres:.3f} m from "
                 f"the camera (median of {len(ahead)} clouds, spread {min(ahead):.3f} to {max(ahead):.3f} m)")
    return lines


def main(args=None):
    parser = argparse.ArgumentParser(description="Depth points: rate, and depth straight ahead.")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--topic", default="/team/depth_points")
    options = parser.parse_args(remove_ros_args(args=None)[1:])
    settings = MissionSettings()
    rclpy.init(args=args)
    node = Node("team_depth_check")
    clouds = []

    def on_cloud(message):
        offsets = [field.offset for name in ("x", "y", "z") for field in message.fields if field.name == name]
        points = points_from_cloud(message.data, message.point_step, offsets, not message.is_bigendian)
        depth, _ = distance_straight_ahead(points, settings.drift_depth_window_degrees, settings.drift_depth_least_points)
        clouds.append((len(points), depth))
    latest_only = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT, history=QoSHistoryPolicy.KEEP_LAST, depth=5)
    node.create_subscription(PointCloud2, options.topic, on_cloud, latest_only)
    end = time.monotonic() + options.seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    for line in summary_lines(clouds, options.seconds, settings.drift_depth_window_degrees,
                              settings.drift_depth_least_points, DepthSettings().camera_forward_metres):
        print(line)
    node.destroy_node()
    rclpy.shutdown()
