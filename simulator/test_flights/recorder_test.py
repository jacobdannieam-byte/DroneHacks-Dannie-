"""Runs the flight recorder during the landing test, measures what the recorder costs, and checks
that its files are intact, even when it is killed in the middle of the flight.

SIMULATOR TEST. Do not run this on the real drone: it starts pretend camera streams on topics
named /simulator_only/..., and the flight it runs asks for the simulator's start service, so it
stops before sending anything if that service is missing.

    simulator/run_test_flight.sh recorder_test.py --mode competition
    simulator/run_test_flight.sh recorder_test.py --mode practice --kill-recorder-after-seconds 25

What it does
------------
  1. Starts pretend camera streams as a separate program, so that the recorder sees roughly the
     load it will see on drone 16: two cameras sending 1024 x 640 blue-first colour pictures 22
     times a second, their calibration 30 times a second, and the camera's motion sensor 200
     times a second (rates measured on drone 16 with drone_code/hardware_checks/check_drone_16.sh
     on 2026-11-07). Our simulator has no real camera pictures.
  2. Starts the recorder exactly as the drone would, from the program that "ros2 run team_nav
     flight_recorder" starts, with its records in simulator/generated_files/team_runs.
  3. Flies landing_test.py's flight: climb to 0.8 m, 1 m forward, land.
  4. Measures the recorder's processor use from Linux's /proc once a second.
  5. Optionally kills the recorder with SIGKILL partway through, which is like a crash or a
     power cut for the recorder: it gets no chance to tidy up. Afterwards it checks that every
     file the recorder wrote can still be read, and how much was lost before the kill.
  6. Prints a summary: processor use, disk use per minute, and rows per topic.
"""
import argparse
import csv
import glob
import json
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy

from landing_test import LandingTest

# Where the recorder writes in the simulator. simulator/generated_files is ignored by git.
RECORDS_FOLDER = "/repository/simulator/generated_files/team_runs"

# The recorder as built by colcon from drone_code/team_nav/setup.py's entry point. setup.cfg puts
# it in lib/team_nav, where "ros2 run" looks. Older builds left copies in bin/; those are stale.
RECORDER_PROGRAM = "/repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/flight_recorder"

# Measured on drone 16 on 2026-11-07 (drone_code/hardware_checks/results/).
PRETEND_PICTURES_PER_SECOND = 22.0
PRETEND_CALIBRATIONS_PER_SECOND = 30.0
PRETEND_MOTION_SENSOR_READINGS_PER_SECOND = 200.0
PICTURE_WIDTH_PIXELS = 1024
PICTURE_HEIGHT_PIXELS = 640
PRETEND_PICTURE_TOPICS = ("/simulator_only/pretend_oak/left/image_raw", "/simulator_only/pretend_oak/right/image_raw")

# Our own choices for this test.
SECONDS_BETWEEN_PROCESSOR_MEASUREMENTS = 1.0
SECONDS_TO_LET_THE_RECORDER_FIND_TOPICS = 8.0
SECONDS_TO_WAIT_FOR_THE_RECORDER_TO_STOP = 10.0
CLOCK_TICKS_PER_SECOND = os.sysconf("SC_CLK_TCK")


# ---- Pretend camera (a separate program) -----------------------------------------------------------

def run_pretend_camera():
    """Publishes pretend camera pictures, calibration and motion sensor readings until stopped.
    SIMULATOR ONLY: every topic name starts with /simulator_only/."""
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, Imu

    rclpy.init()
    node = Node("simulator_only_pretend_camera")
    picture = Image(width=PICTURE_WIDTH_PIXELS, height=PICTURE_HEIGHT_PIXELS, encoding="bgr8",
                    step=PICTURE_WIDTH_PIXELS * 3)
    # Random pixels cannot be squeezed at all, so the PNG files are as large as they can ever
    # be. A real picture gives smaller files, so this is the worst case for the disk.
    picture.data = os.urandom(PICTURE_WIDTH_PIXELS * 3 * PICTURE_HEIGHT_PIXELS)
    for side, picture_topic in zip(("left", "right"), PRETEND_PICTURE_TOPICS):
        picture_publisher = node.create_publisher(Image, picture_topic, 5)
        calibration_publisher = node.create_publisher(CameraInfo, f"/simulator_only/pretend_oak/{side}/camera_info", 5)
        node.create_timer(1.0 / PRETEND_PICTURES_PER_SECOND, _make_sender(node, picture_publisher, picture))
        node.create_timer(1.0 / PRETEND_CALIBRATIONS_PER_SECOND, _make_sender(node, calibration_publisher, CameraInfo()))
    motion_publisher = node.create_publisher(Imu, "/simulator_only/pretend_oak/imu/data", 20)
    node.create_timer(1.0 / PRETEND_MOTION_SENSOR_READINGS_PER_SECOND, _make_sender(node, motion_publisher, Imu()))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


def _make_sender(node, publisher, message):
    def send():
        if hasattr(message, "header"):
            message.header.stamp = node.get_clock().now().to_msg()
        publisher.publish(message)
    return send


# ---- Measuring the recorder -----------------------------------------------------------------------

class ProcessorMeter:
    """Reads one program's processor time from /proc once a second, in a background thread."""

    def __init__(self, process_number):
        self.process_number = process_number
        self.measurements = []    # (time.monotonic(), processor seconds used so far)
        self.peak_memory_megabytes = 0.0
        self.keep_measuring = True
        self.thread = threading.Thread(target=self._measure_until_stopped, daemon=True)
        self.thread.start()

    def _measure_until_stopped(self):
        while self.keep_measuring:
            try:
                with open(f"/proc/{self.process_number}/stat") as stat_file:
                    fields_after_name = stat_file.read().rsplit(")", 1)[1].split()
            except OSError:
                return  # The recorder has stopped.
            processor_seconds = (int(fields_after_name[11]) + int(fields_after_name[12])) / CLOCK_TICKS_PER_SECOND
            memory_megabytes = int(fields_after_name[21]) * os.sysconf("SC_PAGE_SIZE") / 1024 / 1024
            self.peak_memory_megabytes = max(self.peak_memory_megabytes, memory_megabytes)
            self.measurements.append((time.monotonic(), processor_seconds))
            time.sleep(SECONDS_BETWEEN_PROCESSOR_MEASUREMENTS)

    def stop(self):
        self.keep_measuring = False

    def summary(self):
        """(average percent of one core, highest percent over any 5 seconds)."""
        if len(self.measurements) < 2:
            return None, None
        (first_time, first_seconds), (last_time, last_seconds) = self.measurements[0], self.measurements[-1]
        average_percent = 100.0 * (last_seconds - first_seconds) / (last_time - first_time)
        highest_percent = 0.0
        for start in range(len(self.measurements) - 5):
            (start_time, start_seconds), (end_time, end_seconds) = self.measurements[start], self.measurements[start + 5]
            highest_percent = max(highest_percent, 100.0 * (end_seconds - start_seconds) / (end_time - start_time))
        return average_percent, highest_percent


# ---- Checking the records --------------------------------------------------------------------------

def newest_recorder_folder(records_existing_before):
    folders = sorted(set(glob.glob(os.path.join(RECORDS_FOLDER, "*_flight_recorder*"))) - records_existing_before)
    if not folders:
        raise RuntimeError(f"The recorder made no folder in {RECORDS_FOLDER}. Its output above should say why.")
    return folders[-1]


def check_every_file_can_be_read(folder):
    """Reads every CSV and the events file. Returns (rows per CSV file, events, problems)."""
    rows_per_file = {}
    problems = []
    for csv_path in sorted(glob.glob(os.path.join(folder, "**", "*.csv"), recursive=True)):
        with open(csv_path, newline="") as csv_file:
            rows = list(csv.reader(csv_file))
        column_count = len(rows[0])
        # A kill can cut the last line short; anything before it must be complete.
        for row in rows[1:-1]:
            if len(row) != column_count:
                problems.append(f"{os.path.relpath(csv_path, folder)} has a row with {len(row)} of {column_count} columns")
                break
        rows_per_file[os.path.relpath(csv_path, folder)] = len(rows) - 1
    events = []
    with open(os.path.join(folder, "events.jsonl")) as events_file:
        for line in events_file:
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                problems.append(f"events.jsonl has a line that is not complete JSON: {line[:80]!r}")
    return rows_per_file, events, problems


def last_row_seconds(folder):
    """The latest seconds_since_start in any topic table: how far the record reaches."""
    latest = 0.0
    for csv_path in glob.glob(os.path.join(folder, "topics", "*.csv")) + [os.path.join(folder, "computer_health.csv")]:
        with open(csv_path, newline="") as csv_file:
            for row in csv.DictReader(csv_file):
                try:
                    latest = max(latest, float(row["seconds_since_start"]))
                except (TypeError, ValueError):
                    pass  # A line cut short by the kill.
    return latest


def folder_size_megabytes(folder):
    total_bytes = 0
    for current_folder, _, file_names in os.walk(folder):
        for file_name in file_names:
            total_bytes += os.path.getsize(os.path.join(current_folder, file_name))
    return total_bytes / 1024 / 1024


# ---- The test ----------------------------------------------------------------------------------------

def start_recorder(mode):
    # The pretend cameras' names, so that practice mode saves their pictures (recorded_topics.py).
    environment = dict(os.environ, TEAM_NAV_RUN_MODE=mode, TEAM_NAV_RECORDS_FOLDER=RECORDS_FOLDER,
                       TEAM_NAV_PICTURE_TOPICS=",".join(PRETEND_PICTURE_TOPICS))
    if not os.path.exists(RECORDER_PROGRAM):
        raise RuntimeError(f"The recorder program is not built at {RECORDER_PROGRAM}. "
                           f"Check simulator/generated_files/last_build_log.txt.")
    print(f"[recorder test] Starting {RECORDER_PROGRAM} in {mode} mode.", flush=True)
    return subprocess.Popen([RECORDER_PROGRAM], env=environment)


def fly_landing_test():
    rclpy.init()
    flight = LandingTest(use_push_down=True)
    try:
        flight.run()
    finally:
        flight.destroy_node()
        rclpy.try_shutdown()


def report(folder, meter, recorder_seconds, killed_at_seconds):
    rows_per_file, events, problems = check_every_file_can_be_read(folder)
    average_percent, highest_percent = meter.summary()
    megabytes = folder_size_megabytes(folder)
    print("\n===== Flight recorder test summary =====", flush=True)
    print(f"Records: {folder}")
    print(f"Recorder processor use: {average_percent:.1f}% of one core on average, "
          f"{highest_percent:.1f}% at most over 5 seconds. Peak memory {meter.peak_memory_megabytes:.0f} MB.")
    print(f"Disk: {megabytes:.1f} MB in {recorder_seconds:.0f} s, about {megabytes / recorder_seconds * 60:.1f} MB per minute.")
    print(f"{len(rows_per_file)} tables, {sum(rows_per_file.values())} rows; {len(events)} events.")
    for name in sorted(rows_per_file):
        print(f"  {rows_per_file[name]:7d}  {name}")
    status_events = [event for event in events if event["kind"] == "status"]
    for event in status_events:
        print(f"  Recorder's own status at {event['seconds_since_start']:.0f} s: {event['text']}")
    if killed_at_seconds is not None:
        reach_seconds = last_row_seconds(folder)
        print(f"Killed with SIGKILL {killed_at_seconds:.1f} s after the recorder started. The record "
              f"reaches {reach_seconds:.2f} s, so {killed_at_seconds - reach_seconds:.2f} s were lost.")
        if any(event["kind"] == "stop" for event in events):
            problems.append("a 'stop' event exists although the recorder was killed")
    elif not any(event["kind"] == "stop" for event in events):
        problems.append("the recorder was stopped normally but wrote no 'stop' event")
    for problem in problems:
        print(f"PROBLEM: {problem}")
    return not problems


def main():
    parser = argparse.ArgumentParser(description="Fly the landing test with the flight recorder running. SIMULATOR ONLY.")
    parser.add_argument("--mode", choices=["competition", "practice"], default="competition")
    parser.add_argument("--kill-recorder-after-seconds", type=float, default=None)
    parser.add_argument("--no-pretend-camera", action="store_true")
    parser.add_argument("--pretend-camera-only", action="store_true", help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    if arguments.pretend_camera_only:
        run_pretend_camera()
        return

    pretend_camera = None
    if not arguments.no_pretend_camera:
        pretend_camera = subprocess.Popen([sys.executable, os.path.abspath(__file__), "--pretend-camera-only"])
    records_existing_before = set(glob.glob(os.path.join(RECORDS_FOLDER, "*")))
    recorder = start_recorder(arguments.mode)
    recorder_start_seconds = time.monotonic()
    meter = ProcessorMeter(recorder.pid)
    killed_at_seconds = None
    kill_timer = None
    if arguments.kill_recorder_after_seconds is not None:
        def kill_the_recorder():
            nonlocal killed_at_seconds
            killed_at_seconds = time.monotonic() - recorder_start_seconds
            print(f"[recorder test] Killing the recorder with SIGKILL at {killed_at_seconds:.1f} s.", flush=True)
            os.kill(recorder.pid, signal.SIGKILL)
        kill_timer = threading.Timer(arguments.kill_recorder_after_seconds, kill_the_recorder)
        kill_timer.start()

    flight_failed = False
    try:
        time.sleep(SECONDS_TO_LET_THE_RECORDER_FIND_TOPICS)
        fly_landing_test()
    except TimeoutError as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        flight_failed = True
    finally:
        if kill_timer is not None:
            kill_timer.cancel()
        recorder_seconds = time.monotonic() - recorder_start_seconds
        meter.stop()
        if recorder.poll() is None:
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=SECONDS_TO_WAIT_FOR_THE_RECORDER_TO_STOP)
        if pretend_camera is not None:
            pretend_camera.send_signal(signal.SIGINT)
            pretend_camera.wait(timeout=SECONDS_TO_WAIT_FOR_THE_RECORDER_TO_STOP)

    records_are_intact = report(newest_recorder_folder(records_existing_before), meter, recorder_seconds, killed_at_seconds)
    if flight_failed or not records_are_intact:
        print("TEST FAILED: see the PROBLEM lines above.", flush=True)
        sys.exit(1)
    print("TEST PASSED: the recorder's files are intact.", flush=True)


if __name__ == "__main__":
    main()
