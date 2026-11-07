"""stereo_depth.py, depth_points_node.py and depth_check_node.py with synthetic stereo pairs: a
textured wall at a known distance, shifted between the left and right pictures by the disparity
that distance gives. Needs OpenCV; skipped where it is not installed."""
import importlib
import sys
import types

import numpy as np
import pytest

cv2 = pytest.importorskip("cv2")

import fake_ros  # noqa: E402
from team_nav.depth_ahead import distance_straight_ahead, points_from_cloud  # noqa: E402
from team_nav.stereo_depth import CameraLens, DepthSettings, StereoDepth, baseline_metres  # noqa: E402

# The drone 16 camera as measured: 1024 x 640, focal length 634.5 px, baseline 0.075 m.
WIDTH, HEIGHT = 1024, 640
FOCAL_PIXELS = 634.5
BASELINE_METRES = 0.075
SETTINGS = DepthSettings()


def camera_info(baseline=BASELINE_METRES, right=False, with_projection=True):
    message = fake_ros.CameraInfo()
    message.width, message.height = WIDTH, HEIGHT
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    message.k = [FOCAL_PIXELS, 0.0, WIDTH / 2, 0.0, FOCAL_PIXELS, HEIGHT / 2, 0.0, 0.0, 1.0]
    message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    fourth = -FOCAL_PIXELS * baseline if right else 0.0
    message.p = ([FOCAL_PIXELS, 0.0, WIDTH / 2, fourth, 0.0, FOCAL_PIXELS, HEIGHT / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
                 if with_projection else [0.0] * 12)
    return message


def stereo_pair(full_size_disparity_pixels, seed=1):
    """A random texture seen by both cameras. A point at column c in the left picture is at column
    c - disparity in the right one, as for a flat wall facing the cameras."""
    texture = (np.random.default_rng(seed).random((HEIGHT, WIDTH + 200)) * 255).astype(np.uint8)
    left = np.ascontiguousarray(texture[:, :WIDTH])
    right = np.ascontiguousarray(texture[:, full_size_disparity_pixels:full_size_disparity_pixels + WIDTH])
    return left, right


def depth_for(full_size_disparity_pixels):
    """The camera-to-wall distance a disparity means: focal length x baseline / disparity."""
    return FOCAL_PIXELS * BASELINE_METRES / full_size_disparity_pixels


def stereo():
    return StereoDepth(cv2, CameraLens.from_camera_info(camera_info()),
                       CameraLens.from_camera_info(camera_info(right=True)), SETTINGS)


def test_the_measured_camera_numbers_come_out_of_camera_info():
    depth = stereo()
    assert depth.size == (512, 320)
    assert depth.focal_pixels == pytest.approx(317.25)                      # measured earlier: about 317.25
    assert depth.focal_pixels * depth.baseline_metres == pytest.approx(23.79, abs=0.01)   # depth = 23.79 / disparity
    assert "camera_info" in depth.describe()


def test_without_a_baseline_in_camera_info_the_oak_d_s2_baseline_is_used():
    left, right = (CameraLens.from_camera_info(camera_info(right=side)) for side in (False, False))
    assert baseline_metres(left, right, SETTINGS) == (0.075, "the default (camera_info has no baseline)")
    depth = StereoDepth(cv2, CameraLens.from_camera_info(camera_info(with_projection=False)),
                        CameraLens.from_camera_info(camera_info(with_projection=False)), SETTINGS)
    assert depth.focal_pixels == pytest.approx(317.25)                      # from k when p is empty


@pytest.mark.parametrize("full_size_disparity", [24, 40, 80])
def test_a_wall_ahead_is_found_at_its_distance_plus_the_camera_offset(full_size_disparity):
    points = stereo().points(*stereo_pair(full_size_disparity))
    expected = depth_for(full_size_disparity) + SETTINGS.camera_forward_metres     # 2.0, 1.2, 0.6 m to the camera
    assert len(points) > 3000                                                # 128 x 80 samples, most matched
    assert np.median(points[:, 0]) == pytest.approx(expected, rel=0.02)
    ahead, used = distance_straight_ahead(points, 3.0, 5)                    # what the drift correction reads
    assert ahead == pytest.approx(expected, rel=0.02) and used >= 5


def test_points_beyond_4_metres_are_dropped():
    assert len(stereo().points(*stereo_pair(10))) == 0                      # 4.8 m away


def test_each_pixel_becomes_forward_left_up_from_the_drone_centre():
    depth = stereo()
    depth.disparity_pixels = lambda left, right: np.full((320, 512), 20.0, dtype=np.float32)   # 1.19 m everywhere
    points = depth.points(None, None)
    distance = 317.25 * 0.075 / 20.0
    assert points.shape == (80 * 128, 3)
    assert np.allclose(points[:, 0], distance + 0.116, atol=1e-4)
    centre = points[np.argmin(np.abs(points[:, 1] - 0.037) + np.abs(points[:, 2] - 0.029))]
    assert centre[1:] == pytest.approx((0.037, 0.029), abs=1e-4)             # the picture's centre: straight ahead
    first = points[0]                                                         # top-left pixel: up and to the left
    assert first[1] == pytest.approx(256 * distance / 317.25 + 0.037, rel=1e-4)
    assert first[2] == pytest.approx(160 * distance / 317.25 + 0.029, rel=1e-4)


# ---------------------------------------------------------------- the program
def image_message(pixels, seconds):
    message = fake_ros.Image()
    message.header.stamp = types.SimpleNamespace(sec=int(seconds), nanosec=int(round((seconds % 1) * 1e9)))
    message.height, message.width = pixels.shape
    message.encoding, message.step, message.data = "mono8", pixels.shape[1], pixels.tobytes()
    return message


@pytest.fixture
def depth_node():
    fake_ros.install()
    sys.modules.pop("team_nav.depth_points_node", None)
    module = importlib.import_module("team_nav.depth_points_node")
    fake_ros.Node.parameter_overrides = {}
    yield module.DepthPointsNode(cv2)
    sys.modules.pop("team_nav.depth_points_node", None)
    fake_ros.uninstall()


def deliver(node, topic, message):
    node.subscriptions[topic][1](message)


def give_lenses(node):
    deliver(node, "/oak/left/camera_info", camera_info())
    deliver(node, "/oak/right/camera_info", camera_info(right=True))


def test_the_program_publishes_points_in_the_simulators_format(depth_node):
    assert set(depth_node.subscriptions) == {"/oak/left/image_raw", "/oak/right/image_raw",
                                             "/oak/left/camera_info", "/oak/right/camera_info"}
    give_lenses(depth_node)
    left, right = stereo_pair(40)
    deliver(depth_node, "/oak/left/image_raw", image_message(left, 3.5))
    deliver(depth_node, "/oak/right/image_raw", image_message(right, 3.5))
    make_points = depth_node.timers[0][1]
    make_points()
    make_points()                                                             # the same pair only once
    sent = depth_node.publishers["/team/depth_points"].sent
    assert len(sent) == 1
    cloud = sent[0]
    assert cloud.header.frame_id == "base_link" and cloud.point_step == 12
    assert [(field.name, field.offset, field.datatype) for field in cloud.fields] == [("x", 0, 7), ("y", 4, 7), ("z", 8, 7)]
    points = points_from_cloud(cloud.data, cloud.point_step, [0, 4, 8])
    assert len(points) == cloud.width
    ahead, _ = distance_straight_ahead(points, 3.0, 5)
    assert ahead == pytest.approx(depth_for(40) + 0.116, rel=0.02)
    assert depth_node.timers[0][0] == pytest.approx(0.1)                     # about 10 times a second


def test_nothing_is_published_without_camera_info_or_a_matching_pair(depth_node):
    left, right = stereo_pair(40)
    deliver(depth_node, "/oak/left/image_raw", image_message(left, 3.5))
    deliver(depth_node, "/oak/right/image_raw", image_message(right, 3.5))
    depth_node.timers[0][1]()
    assert depth_node.publishers["/team/depth_points"].sent == []            # no camera_info yet
    give_lenses(depth_node)
    deliver(depth_node, "/oak/right/image_raw", image_message(right, 3.6))
    depth_node.timers[0][1]()
    assert depth_node.publishers["/team/depth_points"].sent == []            # 0.1 s apart: not a pair


def test_pictures_of_the_wrong_size_are_skipped_with_one_warning(depth_node):
    give_lenses(depth_node)
    for seconds in (1.0, 2.0):
        small = np.zeros((320, 512), np.uint8)
        deliver(depth_node, "/oak/left/image_raw", image_message(small, seconds))
        deliver(depth_node, "/oak/right/image_raw", image_message(small, seconds))
        depth_node.timers[0][1]()
    assert depth_node.publishers["/team/depth_points"].sent == []
    assert sum("camera_info says 1024 x 640" in text for _, text in depth_node.log_lines) == 1


def test_the_depth_check_reports_the_rate_and_the_depth_straight_ahead():
    fake_ros.install()
    try:
        sys.modules.pop("team_nav.depth_check_node", None)
        summary_lines = importlib.import_module("team_nav.depth_check_node").summary_lines
    finally:
        sys.modules.pop("team_nav.depth_check_node", None)
        fake_ros.uninstall()
    lines = summary_lines([(5000, 1.616), (5100, 1.62), (4900, None), (5050, 1.61)], 2.0, 3.0, 5, 0.116)
    assert lines[0] == "2.0 clouds per second, 5025 points per cloud (median)"
    assert lines[1] == ("Straight ahead: 1.616 m from the drone's centre = 1.500 m from the camera (median of 3 clouds, "
                        "spread 1.610 to 1.620 m)")
    assert "No depth points in 5 s" in summary_lines([], 5.0, 3.0, 5, 0.116)[0]
    assert "No good depth straight ahead" in summary_lines([(10, None)], 1.0, 3.0, 5, 0.116)[1]
