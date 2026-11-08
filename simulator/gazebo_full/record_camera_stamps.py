"""Writes down the time stamp of every camera picture, for measuring the camera rate in flight.
SIMULATOR ONLY.

check_cameras_then_fly.sh runs it in the background for the whole run. For each picture on
/oak/left/image_raw and /oak/right/image_raw it writes one row to OUTPUT_FILE:

    eye,stamp_seconds,arrival_seconds

draw_run.py then works out the picture rate of each eye during the flight, and how far apart in
time the left and right pictures of a pair are stamped.

It asks ROS for the raw message bytes and reads only the time stamp from the front, so it doesn't
unpack the 0.5 MB picture: a normal Python subscriber to two picture streams would itself take a
noticeable share of the processor, and change what it measures.

Run it inside the box:  python3 record_camera_stamps.py OUTPUT_FILE
"""
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# ROS 2 messages start with a 4-byte header that says how the rest is packed ("CDR"), then the
# Image's own header, which starts with the stamp: seconds as a 4-byte signed number and
# nanoseconds as a 4-byte unsigned number, little-endian on this computer.
CDR_HEADER_BYTES = 4
STAMP_FORMAT = "<iI"

# Our own choice: write rows to disk at least this often, so a stopped box loses little.
SECONDS_BETWEEN_FLUSHES = 2.0


class CameraStampRecorder(Node):

    def __init__(self, output_path):
        super().__init__("simulator_only_record_camera_stamps")
        self.output_file = open(output_path, "w", encoding="utf-8")
        self.output_file.write("eye,stamp_seconds,arrival_seconds\n")
        self.last_flush_seconds = time.monotonic()
        self.create_subscription(Image, "/oak/left/image_raw", self.make_receiver("left"), 10, raw=True)
        self.create_subscription(Image, "/oak/right/image_raw", self.make_receiver("right"), 10, raw=True)

    def make_receiver(self, eye):
        def on_picture(raw_bytes):
            seconds, nanoseconds = struct.unpack_from(STAMP_FORMAT, raw_bytes, CDR_HEADER_BYTES)
            self.output_file.write(f"{eye},{seconds + nanoseconds * 1e-9:.4f},{time.time():.4f}\n")
            if time.monotonic() - self.last_flush_seconds > SECONDS_BETWEEN_FLUSHES:
                self.output_file.flush()
                self.last_flush_seconds = time.monotonic()
        return on_picture


def main():
    if len(sys.argv) != 2:
        print("Give one output file, for example: record_camera_stamps.py /repository/simulator/generated_files/x.csv")
        sys.exit(2)
    rclpy.init()
    recorder = CameraStampRecorder(sys.argv[1])
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.output_file.close()


if __name__ == "__main__":
    main()
