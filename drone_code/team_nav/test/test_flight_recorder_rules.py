"""Tests the parts of the flight recorder that work without ROS: which topics it records and how
often (recorded_topics.py), how a message becomes a CSV row (message_flattening.py), the PNG
pictures (camera_pictures.py), the session fingerprint (session_description.py), and that the
recorder has no way to send anything.

The messages here are small pretend classes that look like ROS messages to the code under test.
test_flight_recorder_with_ros.py repeats some checks with real ROS messages when ROS is
installed. How to run these tests is in the package's README.
"""
import json
import os
import struct
import zlib

import pytest

from team_nav.camera_pictures import picture_message_to_png
from team_nav.message_flattening import flatten_message, message_to_plain_data
from team_nav.recorded_topics import (
    COMPETITION_ROWS_PER_SECOND, DO_NOT_RECORD, DRONE_16_CAMERA_TOPICS, PICTURES_PER_SECOND_PER_CAMERA,
    RECORD_FIRST_MESSAGE, RECORD_ROWS, SAVE_PICTURES, RowRateLimiter, most_pictures_per_second,
    most_rows_per_second, picture_topics, plan_for_topic, table_name_for_topic, use_reliable_saved_messages,
)
from team_nav.records_space import RECORD_A_TRICKLE, RECORD_EVERYTHING, RECORD_LESS
from team_nav.run_mode import COMPETITION, PRACTICE, RunMode
from team_nav.session_description import fingerprint_files, overall_fingerprint

COMPETITION_MODE = RunMode(COMPETITION, "")
PRACTICE_MODE = RunMode(PRACTICE, "")
PACKAGE_FOLDER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---- Pretend ROS messages -----------------------------------------------------------------------

class PretendMessage:
    """Looks like a ROS message: it lists its fields and their types."""
    FIELDS = {}

    def __init__(self, **values):
        for name, value in values.items():
            setattr(self, name, value)

    @classmethod
    def get_fields_and_field_types(cls):
        return cls.FIELDS


class PretendTime(PretendMessage):
    FIELDS = {"sec": "int32", "nanosec": "uint32"}


class PretendHeader(PretendMessage):
    FIELDS = {"stamp": "builtin_interfaces/Time", "frame_id": "string"}


class PretendPoint(PretendMessage):
    FIELDS = {"x": "double", "y": "double", "z": "double"}


class PretendOdometry(PretendMessage):
    FIELDS = {
        "header": "std_msgs/Header",
        "position": "geometry_msgs/Point",
        "q": "float[4]",
        "covariance": "double[36]",
        "names": "sequence<string>",
        "points": "sequence<geometry_msgs/Point>",
        "armed": "boolean",
    }


def pretend_odometry():
    return PretendOdometry(
        header=PretendHeader(stamp=PretendTime(sec=12, nanosec=500), frame_id="odom"),
        position=PretendPoint(x=1.5, y=-2.0, z=0.8),
        q=[1.0, 0.0, 0.0, 0.0],
        covariance=[0.0] * 36,
        names=["a", "b"],
        points=[PretendPoint(x=1.0, y=2.0, z=3.0)],
        armed=True,
    )


class PretendPicture:
    def __init__(self, width, height, encoding, data, step=None, is_bigendian=0):
        self.width, self.height, self.encoding, self.data = width, height, encoding, data
        self.step = step if step is not None else len(data) // height
        self.is_bigendian = is_bigendian


# ---- Which topics, and how often ------------------------------------------------------------------

def test_big_messages_are_never_recorded_as_rows():
    for type_name in ("sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/CompressedImage", "nav_msgs/msg/Path"):
        assert plan_for_topic("/oak/points", type_name, PRACTICE_MODE, DRONE_16_CAMERA_TOPICS).how == DO_NOT_RECORD
    competition_plan = plan_for_topic("/oak/left/image_raw", "sensor_msgs/msg/Image", COMPETITION_MODE, DRONE_16_CAMERA_TOPICS)
    assert competition_plan.how == DO_NOT_RECORD


def test_practice_mode_saves_no_camera_pictures_unless_asked():
    # Since 2026-11-08 the recorder reads no camera pictures by default: an extra picture reader
    # slowed camera tracking (recorded_topics.DRONE_16_CAMERA_TOPICS explains this).
    for camera_topic in ("/oak/left/image_raw", "/oak/right/image_raw", "/oak/rgb/image_raw"):
        assert plan_for_topic(camera_topic, "sensor_msgs/msg/Image", PRACTICE_MODE, picture_topics({})).how == DO_NOT_RECORD
    asked_for = picture_topics({"TEAM_NAV_PICTURE_TOPICS": "/oak/left/image_raw"})
    assert plan_for_topic("/oak/left/image_raw", "sensor_msgs/msg/Image", PRACTICE_MODE, asked_for).how == SAVE_PICTURES


def test_the_picture_topics_can_be_replaced_by_a_setting():
    assert picture_topics({"TEAM_NAV_PICTURE_TOPICS": " /a/image , /b/image "}) == ("/a/image", "/b/image")


def test_camera_calibration_is_recorded_once():
    assert plan_for_topic("/oak/left/camera_info", "sensor_msgs/msg/CameraInfo", COMPETITION_MODE, ()).how == RECORD_FIRST_MESSAGE


def test_important_topics_are_never_limited_unless_the_disk_is_almost_full():
    for topic_name in ("/rosout", "/fmu/out/vehicle_status_v1", "/fmu/out/vehicle_command_ack",
                       "/fmu/in/vehicle_command", "/skybeta/setpoint/pose"):
        plan = plan_for_topic(topic_name, "some/msg/Type", COMPETITION_MODE, ())
        assert plan.is_important, topic_name
        assert most_rows_per_second(plan, COMPETITION_MODE, RECORD_EVERYTHING) is None
        assert most_rows_per_second(plan, COMPETITION_MODE, RECORD_LESS) is None
        assert most_rows_per_second(plan, COMPETITION_MODE, RECORD_A_TRICKLE) > 0


def test_ordinary_topics_are_limited_in_competition_mode_only():
    competition_plan = plan_for_topic("/fmu/out/sensor_combined", "px4_msgs/msg/SensorCombined", COMPETITION_MODE, ())
    practice_plan = plan_for_topic("/fmu/out/sensor_combined", "px4_msgs/msg/SensorCombined", PRACTICE_MODE, ())

    assert competition_plan.how == RECORD_ROWS and not competition_plan.is_important
    assert most_rows_per_second(competition_plan, COMPETITION_MODE, RECORD_EVERYTHING) == COMPETITION_ROWS_PER_SECOND
    assert most_rows_per_second(practice_plan, PRACTICE_MODE, RECORD_EVERYTHING) is None


def test_a_nearly_full_disk_lowers_every_limit_and_stops_pictures():
    plan = plan_for_topic("/fmu/out/sensor_combined", "px4_msgs/msg/SensorCombined", PRACTICE_MODE, ())
    nearly_full_limit = most_rows_per_second(plan, PRACTICE_MODE, RECORD_LESS)
    almost_full_limit = most_rows_per_second(plan, PRACTICE_MODE, RECORD_A_TRICKLE)

    assert 0 < almost_full_limit < nearly_full_limit < COMPETITION_ROWS_PER_SECOND
    assert most_pictures_per_second(RECORD_EVERYTHING) == PICTURES_PER_SECOND_PER_CAMERA
    assert most_pictures_per_second(RECORD_LESS) == 0


def test_the_rate_limiter_keeps_at_most_the_limit():
    limiter = RowRateLimiter()
    kept = 0
    for message_number in range(1000):  # 100 messages a second for 10 seconds.
        if limiter.should_keep(message_number * 0.01, 30.0):
            kept += 1
    assert 295 <= kept <= 305
    assert RowRateLimiter().should_keep(0.0, None)
    assert not RowRateLimiter().should_keep(0.0, 0.0)


def test_an_uneven_topic_slower_than_the_limit_is_kept_in_full():
    limiter = RowRateLimiter()
    kept = 0
    for message_number in range(270):  # 27 a second, each up to 8 ms early or late.
        early_or_late_seconds = 0.008 if message_number % 2 else -0.008
        if limiter.should_keep(message_number / 27.0 + early_or_late_seconds, 30.0):
            kept += 1
    assert kept == 270


def test_the_rate_limiter_survives_the_clock_being_set_back():
    limiter = RowRateLimiter()
    assert limiter.should_keep(1_794_000_000.0, 1.0)
    assert limiter.should_keep(1_794_000_000.1, 1.0), "the bucket holds two"
    assert not limiter.should_keep(1_794_000_000.2, 1.0)
    assert not limiter.should_keep(1_000.0, 1.0)
    assert limiter.should_keep(1_001.0, 1.0), "after the clock jumps back, recording carries on"


def test_each_topic_gets_its_own_file_name():
    assert table_name_for_topic("/fmu/out/vehicle_status_v1") == "topics/fmu__out__vehicle_status_v1"
    assert table_name_for_topic("/a_b/c") != table_name_for_topic("/a/b_c")


def test_saved_messages_are_asked_for_only_when_every_publisher_offers_them():
    assert use_reliable_saved_messages([(True, True), (True, True)])
    assert not use_reliable_saved_messages([(True, True), (False, True)])
    assert not use_reliable_saved_messages([(True, False)])
    assert not use_reliable_saved_messages([])


# ---- Messages into rows -------------------------------------------------------------------------

def test_a_message_becomes_one_flat_row():
    row = flatten_message(pretend_odometry())

    assert row["header.stamp.sec"] == 12 and row["header.stamp.nanosec"] == 500
    assert row["header.frame_id"] == "odom"
    assert row["position.x"] == 1.5 and row["position.z"] == 0.8
    assert [row[f"q[{position}]"] for position in range(4)] == [1.0, 0.0, 0.0, 0.0]
    assert json.loads(row["covariance"]) == [0.0] * 36, "a long list goes into one column"
    assert json.loads(row["names"]) == ["a", "b"], "a variable-length list goes into one column"
    assert json.loads(row["points"]) == [{"x": 1.0, "y": 2.0, "z": 3.0}]
    assert row["armed"] is True


def test_every_message_of_a_type_gives_the_same_columns():
    longer = pretend_odometry()
    longer.names = ["a", "b", "c", "d"]
    longer.points = []
    assert list(flatten_message(pretend_odometry())) == list(flatten_message(longer))


def test_numpy_lists_and_numbers_become_plain_python():
    numpy = pytest.importorskip("numpy")
    message = pretend_odometry()
    message.q = numpy.array([1.0, 0.5, 0.25, 0.0], dtype=numpy.float32)
    message.covariance = numpy.zeros(36)

    row = flatten_message(message)

    assert row["q[1]"] == 0.5 and type(row["q[1]"]) is float
    assert json.loads(row["covariance"]) == [0.0] * 36
    assert message_to_plain_data(numpy.float32(2.5)) == 2.5


# ---- Pictures -------------------------------------------------------------------------------------

def read_png(png_bytes):
    """A tiny PNG reader for these tests: (width, height, colour type, pixel bytes)."""
    assert png_bytes[:8] == b"\x89PNG\r\n\x1a\n"
    position, blocks = 8, {}
    while position < len(png_bytes):
        length = struct.unpack(">I", png_bytes[position:position + 4])[0]
        kind = png_bytes[position + 4:position + 8]
        content = png_bytes[position + 8:position + 8 + length]
        checksum = struct.unpack(">I", png_bytes[position + 8 + length:position + 12 + length])[0]
        assert checksum == zlib.crc32(kind + content) & 0xFFFFFFFF
        blocks[kind] = content
        position += 12 + length
    width, height, _, colour_type = struct.unpack(">IIBB", blocks[b"IHDR"][:10])
    filtered = zlib.decompress(blocks[b"IDAT"])
    row_bytes = len(filtered) // height
    pixels = b""
    for row_number in range(height):
        assert filtered[row_number * row_bytes] == 0
        pixels += filtered[row_number * row_bytes + 1:(row_number + 1) * row_bytes]
    return width, height, colour_type, pixels


def test_a_blue_first_picture_is_saved_red_first():
    # 2 x 2 pixels, blue first: pure blue, pure green / pure red, white.
    blue_first = bytes([255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 255])
    width, height, colour_type, pixels = read_png(picture_message_to_png(PretendPicture(2, 2, "bgr8", blue_first)))

    assert (width, height, colour_type) == (2, 2, 2)
    assert pixels == bytes([0, 0, 255, 0, 255, 0, 255, 0, 0, 255, 255, 255])


def test_row_padding_is_removed_from_a_grey_picture():
    # 3 pixels wide, with one padding byte at the end of each row.
    padded = bytes([1, 2, 3, 99, 4, 5, 6, 99])
    _, _, colour_type, pixels = read_png(picture_message_to_png(PretendPicture(3, 2, "mono8", padded, step=4)))

    assert colour_type == 0
    assert pixels == bytes([1, 2, 3, 4, 5, 6])


def test_an_unknown_picture_encoding_says_what_to_do():
    with pytest.raises(ValueError, match="KNOWN_ENCODINGS"):
        picture_message_to_png(PretendPicture(1, 1, "yuv422", bytes(2)))


# ---- Session and safety -------------------------------------------------------------------------

def test_the_code_fingerprint_changes_when_a_file_changes(tmp_path):
    (tmp_path / "a.py").write_text("x = 1\n")
    (tmp_path / "notes.txt").write_text("not fingerprinted\n")
    before = fingerprint_files(str(tmp_path))
    (tmp_path / "a.py").write_text("x = 2\n")
    after = fingerprint_files(str(tmp_path))

    assert list(before) == ["a.py"]
    assert overall_fingerprint(before) != overall_fingerprint(after)


def test_the_recorder_has_no_way_to_send_anything():
    """The recorder must never be able to move the drone. Sending needs a publisher, a service
    client or an action client, so none of those may appear in its code."""
    recorder_files = ["flight_recorder.py", "recorded_topics.py", "message_flattening.py", "computer_health.py",
                      "records_space.py", "camera_pictures.py", "session_description.py"]
    for file_name in recorder_files:
        source = open(os.path.join(PACKAGE_FOLDER, "team_nav", file_name), encoding="utf-8").read()
        for forbidden in ("create_publisher", "create_client", "ActionClient", ".publish(", "call_async"):
            assert forbidden not in source, f"{file_name} contains {forbidden}"
