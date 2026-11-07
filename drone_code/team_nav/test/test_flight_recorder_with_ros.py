"""Tests the flight recorder with real ROS messages and a real ROS node. Skipped where ROS is not
installed.

The usual test command sets PYTHONPATH to our package only, which hides ROS, so these tests are
skipped there. To run them, use the simulator's box with ROS loaded:

    docker run --rm --network none --volume "$PWD:/repository" --workdir /repository drone-simulator \
        bash -c "source simulator/inside_the_box/load_ros_environment.sh && \
                 python3 -m pytest drone_code/team_nav/test/test_flight_recorder_with_ros.py -q -p no:cacheprovider"

They check that real message types flatten into rows, that the recorder node has no publishers
at all, and that a message sent on a topic arrives in that topic's CSV file.
"""
import csv
import json
import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("rosidl_runtime_py")

from nav_msgs.msg import Odometry  # noqa: E402  (only importable when ROS is installed)
from rclpy.serialization import deserialize_message, serialize_message  # noqa: E402
from sensor_msgs.msg import CameraInfo, Image  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from team_nav.camera_pictures import picture_message_to_png  # noqa: E402
from team_nav.flight_recorder import FlightRecorder  # noqa: E402
from team_nav.message_flattening import flatten_message  # noqa: E402
from team_nav.run_log import RunLog  # noqa: E402
from team_nav.run_mode import COMPETITION, RunMode  # noqa: E402

# Our own choice: long enough for ROS to find the topic and deliver a message on one computer.
LONGEST_WAIT_SECONDS = 15.0


def test_real_messages_flatten_after_a_trip_through_ros_packing():
    odometry = Odometry()
    odometry.header.stamp.sec = 7
    odometry.pose.pose.position.x = 1.25
    odometry.pose.pose.orientation.w = 1.0
    unpacked = deserialize_message(serialize_message(odometry), Odometry)

    row = flatten_message(unpacked)

    assert row["header.stamp.sec"] == 7
    assert row["pose.pose.position.x"] == 1.25
    assert len(json.loads(row["pose.covariance"])) == 36
    assert json.dumps(row)  # Every value can be written as text.


def test_camera_calibration_flattens():
    calibration = CameraInfo()
    calibration.k = [634.5, 0.0, 512.0, 0.0, 634.5, 320.0, 0.0, 0.0, 1.0]
    row = flatten_message(deserialize_message(serialize_message(calibration), CameraInfo))
    assert row["k[0]"] == 634.5 and row["k[8]"] == 1.0
    assert json.loads(row["d"]) == []


def test_px4_messages_flatten():
    px4_messages = pytest.importorskip("px4_msgs.msg")
    odometry = px4_messages.VehicleOdometry()
    odometry.q = [1.0, 0.0, 0.0, 0.0]
    row = flatten_message(deserialize_message(serialize_message(odometry), px4_messages.VehicleOdometry))
    assert row["q[0]"] == 1.0 and "timestamp" in row


def test_a_real_camera_picture_becomes_a_png():
    picture = Image(width=4, height=2, encoding="bgr8", step=12, data=bytes(range(24)))
    png_bytes = picture_message_to_png(deserialize_message(serialize_message(picture), Image))
    assert png_bytes.startswith(b"\x89PNG")


def test_the_recorder_node_has_no_publishers_and_records_what_it_hears(tmp_path):
    rclpy.init()
    run_log = RunLog("flight_recorder", RunMode(COMPETITION, ""), str(tmp_path))
    recorder = FlightRecorder(run_log, RunMode(COMPETITION, ""))
    sender = rclpy.create_node("test_sender")
    try:
        assert list(recorder.publishers) == []
        publisher = sender.create_publisher(String, "/recorder_test_topic", 10)
        csv_path = os.path.join(run_log.folder, "topics", "recorder_test_topic.csv")
        deadline = time.monotonic() + LONGEST_WAIT_SECONDS
        while time.monotonic() < deadline and not os.path.exists(csv_path):
            publisher.publish(String(data="hello"))
            rclpy.spin_once(recorder, timeout_sec=0.1)
        # ROS's own list of who publishes what must not include either of the recorder's nodes.
        assert list(recorder.listening_node.publishers) == []
        assert recorder.get_publisher_names_and_types_by_node("flight_recorder", "/") == []
        assert recorder.get_publisher_names_and_types_by_node("flight_recorder_listener", "/") == []
    finally:
        run_log.close()
        sender.destroy_node()
        recorder.destroy_node()
        rclpy.try_shutdown()

    rows = list(csv.DictReader(open(csv_path)))
    assert rows and rows[0]["data"] == "hello"
    assert float(rows[0]["received_unix_time_seconds"]) > 0
