"""depth_ahead.py: reading depth points out of a PointCloud2's bytes, and the median depth in a small
cone straight ahead, or no reading when too few points lie in it."""
import math

import numpy as np
import pytest

from team_nav.depth_ahead import distance_straight_ahead, points_from_cloud

WINDOW_DEGREES = 3.0
LEAST_POINTS = 5


def packed(points, point_step=12, offsets=(0, 4, 8), byte_order="<"):
    """Points packed as a PointCloud2 packs them: float32 x, y, z at the given offsets."""
    rows = np.zeros((len(points), point_step), dtype=np.uint8)
    for column, offset in enumerate(offsets):
        values = np.ascontiguousarray(np.asarray(points, dtype=f"{byte_order}f4")[:, column])
        rows[:, offset:offset + 4] = values.view(np.uint8).reshape(len(points), 4)
    return rows.tobytes()


def test_points_come_back_out_of_the_bytes():
    points = [(1.0, 0.5, -0.25), (2.0, -1.0, 0.125)]
    assert [tuple(row) for row in points_from_cloud(packed(points), 12, [0, 4, 8])] == points


def test_points_come_back_with_padding_and_another_byte_order():
    points = [(1.5, 0.25, 0.75), (3.0, -0.5, -0.5)]
    data = packed(points, point_step=16, offsets=(4, 8, 12), byte_order=">")
    assert [tuple(row) for row in points_from_cloud(data, 16, [4, 8, 12], little_endian=False)] == points


def test_an_empty_cloud_has_no_points():
    assert points_from_cloud(b"", 12, [0, 4, 8]).shape == (0, 3)
    assert distance_straight_ahead(np.zeros((0, 3)), WINDOW_DEGREES, LEAST_POINTS) == (None, 0)


def wall_ahead(distance_metres, count=9):
    """count points on a wall straight ahead, spread inside a 2 degree cone."""
    spread = distance_metres * math.tan(math.radians(2.0))
    offsets = np.linspace(-spread, spread, count)
    return [(distance_metres, left, up) for left, up in zip(offsets, offsets[::-1])]


def test_the_median_of_the_points_straight_ahead_is_the_depth():
    points = wall_ahead(1.5)
    points[0] = (0.3, 0.0, 0.0)                                            # one bad point does not matter
    distance, used = distance_straight_ahead(points, WINDOW_DEGREES, LEAST_POINTS)
    assert distance == pytest.approx(1.5)
    assert used == 9


def test_points_outside_the_cone_or_behind_are_left_out():
    sideways = [(1.0, 1.0 * math.tan(math.radians(10.0)), 0.0)] * 20      # 10 degrees to the left
    above = [(1.0, 0.0, 0.2)] * 20                                         # 11 degrees up
    behind = [(-1.0, 0.0, 0.0)] * 20
    distance, used = distance_straight_ahead(wall_ahead(2.0) + sideways + above + behind, WINDOW_DEGREES, LEAST_POINTS)
    assert (distance, used) == (pytest.approx(2.0), 9)


def test_too_few_points_straight_ahead_is_no_reading():
    assert distance_straight_ahead(wall_ahead(2.0, count=4), WINDOW_DEGREES, LEAST_POINTS) == (None, 4)
