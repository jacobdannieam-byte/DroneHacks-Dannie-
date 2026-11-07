"""How far away the wall straight ahead is, from the depth points.

How it fits: mission_node.py hands every depth message to points_from_cloud, then to
distance_straight_ahead, and passes the answer to mission.py, which uses it to correct camera
tracking's drift at route corners (drift_correction.py). This file never talks to ROS.

The depth points: a list of points, each (forward, left, up) in metres from the drone's centre,
in a sensor_msgs/PointCloud2 message with float32 fields x, y, z. In the simulator, Kenny's
pretend depth camera publishes them on /simulator_only/depth_points (64 x 40 rays, with
stereo-like errors and missing points). On the drone, our own depth program must publish the same
format; the launch argument depth_topic names its topic.

"Straight ahead" is a small cone around the drone's forward direction: points whose sideways and
up-down offsets are both within drift_depth_window_degrees of it. Their forward distances are
the depth straight ahead; the median keeps one bad point from mattering. With too few points in
the cone (a plain wall gives a stereo camera nothing to match), there is no good reading.
"""
import math

import numpy as np


def points_from_cloud(data, point_step, field_offsets, little_endian=True):
    """The points of a PointCloud2 as an N x 3 array (forward, left, up). field_offsets gives the
    byte offset of x, y and z inside each point."""
    data = bytes(data)
    if point_step <= 0 or len(data) < point_step:
        return np.zeros((0, 3))
    count = len(data) // point_step
    order = "<" if little_endian else ">"
    rows = np.frombuffer(data[:count * point_step], dtype=np.uint8).reshape(count, point_step)
    columns = []
    for offset in field_offsets:
        columns.append(rows[:, offset:offset + 4].copy().view(f"{order}f4").reshape(count))
    return np.stack(columns, axis=1).astype(float)


def distance_straight_ahead(points, window_degrees, least_points):
    """(median forward distance in metres, number of points used), or (None, number of points)
    when fewer than least_points lie in the cone straight ahead."""
    points = np.asarray(points, dtype=float)
    if len(points) == 0:
        return None, 0
    forward, left, up = points[:, 0], points[:, 1], points[:, 2]
    slope = math.tan(math.radians(window_degrees))
    in_cone = (forward > 0.0) & np.isfinite(forward) & (np.abs(left) <= forward * slope) & (np.abs(up) <= forward * slope)
    used = int(np.count_nonzero(in_cone))
    if used < least_points:
        return None, used
    return float(np.median(forward[in_cone])), used
