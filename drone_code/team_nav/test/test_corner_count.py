"""fast_corners.py, camera_images.py and corner_count_node.py: FAST corners in a blank picture and a
textured one, pictures read from every accepted format, and the node publishing its count."""
import importlib
import sys

import numpy as np
import pytest

import fake_ros
from team_nav.camera_images import grey_from_image_message, shrink_by_whole_factor
from team_nav.fast_corners import count_fast_corners

THRESHOLD = 20


def checkerboard(height=320, width=512, square=16):
    rows, columns = np.indices((height, width))
    return (((rows // square) + (columns // square)) % 2 * 200 + 25).astype(np.uint8)


def scattered_squares(height=320, width=512, count=60, side=12, seed=1):
    """Bright squares on a dark background: each square's four outer corners are FAST corners.
    (A checkerboard is not: where four squares meet, no 9 neighbours in a row are all on one side.)"""
    random = np.random.default_rng(seed)
    picture = np.full((height, width), 40, dtype=np.uint8)
    for _ in range(count):
        row, column = random.integers(4, height - side - 4), random.integers(4, width - side - 4)
        picture[row:row + side, column:column + side] = 210
    return picture


def test_a_blank_picture_has_no_corners():
    assert count_fast_corners(np.full((320, 512), 90, dtype=np.uint8), THRESHOLD) == 0


def test_a_smooth_gradient_has_no_corners():
    gradient = np.tile(np.linspace(0, 255, 512), (320, 1)).astype(np.uint8)
    assert count_fast_corners(gradient, THRESHOLD) == 0


def test_a_single_bright_dot_is_a_corner():
    picture = np.full((40, 40), 50, dtype=np.uint8)
    picture[20, 20] = 200
    assert count_fast_corners(picture, THRESHOLD) == 1                   # brighter than all 16 neighbours


def test_a_straight_edge_is_not_a_corner():
    picture = np.full((40, 40), 50, dtype=np.uint8)
    picture[:, 20:] = 200
    assert count_fast_corners(picture, THRESHOLD) == 0                   # only 7 of 16 on either side


def test_a_textured_picture_has_hundreds_of_corners_and_a_dim_one_none():
    textured = scattered_squares()
    assert count_fast_corners(textured, THRESHOLD) > 200
    dim = (textured // 20).astype(np.uint8)                              # contrast below the threshold
    assert count_fast_corners(dim, THRESHOLD) == 0


def test_a_checkerboard_crossing_is_not_a_fast_corner():
    assert count_fast_corners(checkerboard(), THRESHOLD) == 0


def image_message(pixels, encoding, bigendian=False):
    message = fake_ros.Image()
    message.height, message.width = pixels.shape[:2]
    message.encoding = encoding
    message.is_bigendian = bigendian
    data = pixels.tobytes()
    message.step = len(data) // message.height + 4                      # rows padded by 4 bytes
    rows = [data[row * (len(data) // message.height):(row + 1) * (len(data) // message.height)] + b"\0" * 4
            for row in range(message.height)]
    message.data = b"".join(rows)
    return message


@pytest.mark.parametrize("encoding", ["mono8", "8UC1", "bgr8", "rgb8", "bgra8", "rgba8", "mono16", "16UC1"])
def test_every_accepted_format_reads_as_the_same_grey_picture(encoding):
    grey = checkerboard(8, 12, 2)
    if encoding in ("mono8", "8UC1"):
        pixels = grey
    elif encoding in ("mono16", "16UC1"):
        pixels = grey.astype("<u2") * 256
    else:
        pixels = np.stack([grey] * (4 if encoding.endswith("a8") else 3), axis=2)
    assert np.array_equal(grey_from_image_message(image_message(pixels, encoding)), grey)


def test_red_and_blue_are_told_apart():
    pixels = np.zeros((2, 2, 3), dtype=np.uint8)
    pixels[:, :, 0] = 255                                                # the first channel only
    assert grey_from_image_message(image_message(pixels, "rgb8"))[0, 0] == 76     # red looks brighter...
    assert grey_from_image_message(image_message(pixels, "bgr8"))[0, 0] == 29     # ...than blue


def test_an_unknown_or_damaged_picture_is_a_clear_error():
    message = image_message(np.zeros((4, 4), dtype=np.uint8), "yuv422")
    with pytest.raises(ValueError, match="only mono8"):
        grey_from_image_message(message)
    message = image_message(np.zeros((4, 4), dtype=np.uint8), "mono8")
    message.data = message.data[:10]
    with pytest.raises(ValueError, match="It is damaged"):
        grey_from_image_message(message)


def test_shrinking_by_two_averages_each_block():
    grey = np.array([[0, 2, 10, 10], [4, 6, 10, 10]], dtype=np.uint8)
    assert shrink_by_whole_factor(grey, 2).tolist() == [[3, 10]]


@pytest.fixture
def corner_node():
    fake_ros.install()
    sys.modules.pop("team_nav.corner_count_node", None)
    module = importlib.import_module("team_nav.corner_count_node")
    fake_ros.Node.parameter_overrides = {}
    yield module.CornerCountNode()
    sys.modules.pop("team_nav.corner_count_node", None)
    fake_ros.uninstall()


def test_the_node_publishes_the_count_of_the_newest_picture_once(corner_node):
    assert set(corner_node.subscriptions) == {"/oak/left/image_raw"}
    count_now = corner_node.timers[0][1]
    corner_node.subscriptions["/oak/left/image_raw"][1](image_message(np.kron(scattered_squares(), np.ones((2, 2), np.uint8)),
                                                                      "mono8"))
    count_now()
    count_now()                                                          # the same picture is not counted twice
    sent = corner_node.publishers["/team/left_corner_count"].sent
    assert len(sent) == 1 and sent[0].data > 200
    corner_node.subscriptions["/oak/left/image_raw"][1](image_message(np.zeros((640, 1024), np.uint8), "mono8"))
    count_now()
    assert sent[1].data == 0


def test_the_node_skips_a_picture_it_cannot_read_and_warns_once(corner_node):
    for _ in range(2):
        corner_node.subscriptions["/oak/left/image_raw"][1](image_message(np.zeros((4, 4), np.uint8), "yuv422"))
        corner_node.timers[0][1]()
    assert corner_node.publishers["/team/left_corner_count"].sent == []
    assert sum(level == "warn" for level, _ in corner_node.log_lines) == 1
