"""Checks, in about 8 seconds, everything that must be right before a test flight, and prints one
PASS or FAIL line for each, with what to do. REAL DRONE. "./drone.sh fly <test>" runs it after it
has made sure that the drone's software and our test_flights launch are running.

It only listens. It never arms the drone, never sends a target and never changes a setting.
Use --measure-tracking SECONDS for a longer measurement of camera and bridge timing.

The checks come from the problems we met on 2026-11-08:
  - PX4 not connected to the Jetson (the flight controller had no power);
  - no remote control signal (the remote's battery was empty, the receiver's light was red);
  - no main battery (the flight controller had only USB power);
  - camera tracking not running, slow, or late (late reports made PX4 drop the camera and turn
    to the compass, which is wrong indoors);
  - PX4 not using camera tracking for position and heading;
  - any of PX4's own failsafe checks failing ("system health" errors when arming).
"""
import argparse
import json
import statistics
import sys
import time

import rclpy
from px4_msgs.msg import FailsafeFlags
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args

from team_nav.drone_reports import PX4_REPORT_SETTINGS, DroneReports, px4_topic_name

# Our own choices. The checks are first judged after SHORTEST_LISTEN_SECONDS (about 60 tracking
# reports), then again every second, and the program stops as soon as all pass. It gives up after
# LONGEST_LISTEN_SECONDS, which covers ROS and camera tracking starting up.
SHORTEST_LISTEN_SECONDS = 2.0
LONGEST_LISTEN_SECONDS = 40.0

# Measured on drone 16 on 2026-11-08: camera tracking reports about 29 times a second, normally
# 55 to 70 ms after the picture is taken. PX4 drops the camera after 0.4 s without a usable report.
LOWEST_TRACKING_REPORTS_PER_SECOND = 20.0
LATEST_TRACKING_DELAY_MILLISECONDS = 180.0
# Our own choice: a few late reports are normal; more than this means the delay problem is back.
MOST_LATE_TRACKING_FRACTION = 0.02

# Our own choice: below this the battery may not last a test flight. PX4 reports 0 to 1.
LOWEST_BATTERY_FRACTION = 0.30

TRACKING_STATUS_TOPIC = "/visual_slam/status"

# PX4 failsafe flags that are always set indoors on drone 16 and do not stop arming in Position
# mode (measured 2026-11-08): no GPS, so no global position and no home point; no PX4 mission,
# because our programs send targets instead; and no ground station program connected.
NORMAL_INDOORS_FAILSAFES = {"global_position_invalid", "home_position_invalid", "auto_mission_missing",
                            "gcs_connection_lost",
                            # At the team's request: the remote is not always available at the venue.
                            "manual_control_signal_lost"}


class PreflightListener:
    """Collects the reports while the checks wait."""

    def __init__(self, node):
        self.reports = DroneReports(node)
        self.failsafe_flags = None
        self.tracking_delays_milliseconds = []
        node.create_subscription(FailsafeFlags, px4_topic_name("failsafe_flags", FailsafeFlags),
                                 self._on_failsafe_flags, PX4_REPORT_SETTINGS)
        self._listen_to_tracking(node)

    def _listen_to_tracking(self, node):
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus  # Installed with the organizers' software.
        node.create_subscription(VisualSlamStatus, TRACKING_STATUS_TOPIC, self._on_tracking_status,
                                 qos_profile_sensor_data)

    def _on_failsafe_flags(self, message):
        self.failsafe_flags = message

    def _on_tracking_status(self, message):
        picture_seconds = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.tracking_delays_milliseconds.append((time.time() - picture_seconds) * 1000.0)

    def start_new_window(self):
        """Forgets the tracking reports so far, so the rate is measured over the next window only."""
        self.tracking_delays_milliseconds = []
        self.window_start_seconds = time.monotonic()

    def window_seconds(self):
        return time.monotonic() - self.window_start_seconds


# =================================================================================================
# The checks. Each returns (passed, sentence).
# =================================================================================================

def check_px4_connected(listener):
    state = listener.reports.latest.get("state")
    if state is None:
        return False, ("No summary from the organizers' flight_interface. Our test_flights launch is not running, "
                       "or ROS has not finished starting. Wait 30 s and try again.")
    if not state.get("px4_link"):
        return False, ("PX4 is not connected to the Jetson. Check that the main battery is plugged in and on, "
                       "and that the flight controller's lights are on.")
    return True, "PX4 is connected to the Jetson."


def check_disarmed_and_landed(listener):
    state = listener.reports.latest.get("state") or {}
    if state.get("armed"):
        return False, "The drone is ARMED. Disarm it before a test starts."
    if state.get("landed") is not True:
        return False, f"PX4 does not say the drone is landed (landed: {state.get('landed')}). Put it on the floor."
    return True, "Disarmed and landed."


def check_battery(listener):
    battery = listener.reports.latest.get("battery")
    if battery is None:
        return False, "No battery report from PX4. Check that the main battery is plugged in."
    if battery.voltage_v <= 1.0 or battery.remaining < 0.0:
        return False, ("PX4 measures no main battery (it may have USB power only). Plug in the main battery and "
                       "switch it on.")
    if battery.remaining < LOWEST_BATTERY_FRACTION:
        return False, (f"Battery at {battery.remaining * 100:.0f}% ({battery.voltage_v:.1f} V), below "
                       f"{LOWEST_BATTERY_FRACTION * 100:.0f}%. Change to a charged battery.")
    return True, f"Battery at {battery.remaining * 100:.0f}% ({battery.voltage_v:.1f} V)."


def check_failsafes(listener):
    flags = listener.failsafe_flags
    if flags is None:
        return False, "No failsafe report from PX4, so its health checks can't be read. Is PX4 connected?"
    failing = []
    for field_name, field_type in flags.get_fields_and_field_types().items():
        if field_type == "boolean" and getattr(flags, field_name) and field_name not in NORMAL_INDOORS_FAILSAFES:
            failing.append(field_name)
    if not failing:
        return True, "None of PX4's failsafe checks fail."
    advice = []
    if "local_position_invalid" in failing or "local_altitude_invalid" in failing:
        advice.append("PX4 has no position: camera tracking must be running and used by PX4 (see the checks below).")
    if "battery_unhealthy" in failing or "battery_low_remaining_time" in failing or "battery_warning" in failing:
        advice.append("Battery problem: change to a charged battery.")
    return False, f"PX4's failsafe checks failing: {', '.join(failing)}. " + " ".join(advice)


def check_tracking(listener):
    delays = listener.tracking_delays_milliseconds
    reports_per_second = len(delays) / listener.window_seconds()
    if reports_per_second < LOWEST_TRACKING_REPORTS_PER_SECOND:
        return False, (f"Camera tracking sent {reports_per_second:.1f} reports a second, fewer than "
                       f"{LOWEST_TRACKING_REPORTS_PER_SECOND:.0f}. It may still be starting: wait 30 s and try again. "
                       f"If it stays low, restart the drone's software.")
    late_fraction = sum(1 for delay in delays if delay > LATEST_TRACKING_DELAY_MILLISECONDS) / len(delays)
    typical_milliseconds = sorted(delays)[len(delays) // 2]
    sentence = (f"Camera tracking: {reports_per_second:.0f} reports a second, typically {typical_milliseconds:.0f} ms "
                f"late, {late_fraction * 100:.1f}% over {LATEST_TRACKING_DELAY_MILLISECONDS:.0f} ms.")
    if late_fraction > MOST_LATE_TRACKING_FRACTION:
        # Only a warning, at the team's request: it does not stop the flight. The safety checks in
        # flight land the drone if PX4 stops following camera tracking.
        return True, sentence + " WARNING: many late reports. PX4 may drop the camera in flight."
    return True, sentence


def check_px4_uses_tracking(listener):
    flags = listener.reports.latest.get("estimator_flags")
    if flags is None:
        return False, "No estimator report from PX4, so it is not known what PX4 uses for its position."
    if flags.cs_ev_pos and flags.cs_ev_yaw:
        return True, "PX4 uses camera tracking for its position and its heading."
    return False, (f"PX4 does not use camera tracking for everything (position: {bool(flags.cs_ev_pos)}, heading: "
                   f"{bool(flags.cs_ev_yaw)}, compass heading: {bool(flags.cs_mag_hdg)}). Keep the drone still for "
                   f"20 s. If it stays like this, restart the drone's software.")


CHECKS = [check_px4_connected, check_disarmed_and_landed, check_battery, check_failsafes,
          check_tracking, check_px4_uses_tracking]


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--measure-tracking", type=int, metavar="SECONDS",
                        help="Measure camera and bridge timing for this many seconds, with the motors off.")
    arguments = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if arguments.measure_tracking is not None:
        if arguments.measure_tracking <= 0:
            parser.error("--measure-tracking needs a positive number of seconds")
        return measure_tracking(arguments.measure_tracking)

    rclpy.init()
    node = rclpy.create_node("team_nav_preflight_check")
    listener = PreflightListener(node)
    print("Checking. Nothing is sent to the drone.", flush=True)
    give_up_seconds = time.monotonic() + LONGEST_LISTEN_SECONDS
    results = []
    while time.monotonic() < give_up_seconds:
        listener.start_new_window()
        listen_for(node, SHORTEST_LISTEN_SECONDS)
        results = [check(listener) for check in CHECKS]
        if all(passed for passed, _ in results):
            break
        waiting_for = [sentence.split(".")[0] for passed, sentence in results if not passed]
        print("Waiting: " + "; ".join(waiting_for), flush=True)
    for passed, sentence in results:
        print(("PASS  " if passed else "FAIL  ") + sentence, flush=True)
    print("\nSTATUS:  " + status_line(listener), flush=True)
    all_passed = all(passed for passed, _ in results)
    print("\nREADY FOR A TEST FLIGHT." if all_passed else "\nNOT READY: fix each FAIL above, then run the same ./drone.sh fly command again.")
    node.destroy_node()
    rclpy.try_shutdown()
    return 0 if all_passed else 1


def status_line(listener):
    """One line with what the team must know before arming: battery, remote and camera."""
    battery = listener.reports.latest.get("battery")
    battery_text = "unknown" if battery is None else f"{battery.remaining * 100:.0f}%"
    flags = listener.failsafe_flags
    if flags is None:
        remote_text = "unknown"
    else:
        remote_text = "NOT connected" if flags.manual_control_signal_lost else "connected"
    estimator = listener.reports.latest.get("estimator_flags")
    camera_text = "used by PX4" if estimator is not None and estimator.cs_ev_pos and estimator.cs_ev_yaw else "NOT used by PX4"
    return f"Battery {battery_text}  |  Remote {remote_text}  |  Camera {camera_text}"


def listen_for(node, seconds):
    end_seconds = time.monotonic() + seconds
    while time.monotonic() < end_seconds:
        rclpy.spin_once(node, timeout_sec=0.1)


def measure_tracking(seconds):
    """Compare camera metadata, tracker output and the vision messages handed to PX4.

    Subscribing to images would change the load we are trying to measure.
    """
    from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
    from px4_msgs.msg import VehicleOdometry
    from sensor_msgs.msg import CameraInfo

    rclpy.init()
    node = rclpy.create_node("team_nav_tracking_timing")
    try:
        timing = {}
        for topic, message_type in (
            ("/oak/left/camera_info", CameraInfo),
            ("/oak/right/camera_info", CameraInfo),
            (TRACKING_STATUS_TOPIC, VisualSlamStatus),
            ("/fmu/in/vehicle_visual_odometry", VehicleOdometry),
        ):
            samples = []
            timing[topic] = samples
            node.create_subscription(message_type, topic, timing_callback(samples), qos_profile_sensor_data)
        print(f"Measuring tracking for {seconds} s. Nothing is sent to the drone.", flush=True)
        listen_for(node, seconds)
        for topic, samples in timing.items():
            print(f"{topic}  {describe_timing(samples)}", flush=True)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def timing_callback(samples):
    def record(message):
        arrival_seconds = time.monotonic()
        if hasattr(message, "header"):
            stamp = message.header.stamp
            age_milliseconds = (time.time() - stamp.sec - stamp.nanosec * 1e-9) * 1000.0
        else:
            age_milliseconds = (message.timestamp - message.timestamp_sample) / 1000.0
        samples.append((arrival_seconds, age_milliseconds))

    return record


def describe_timing(samples):
    if len(samples) < 2:
        return f"{len(samples)} reports. Need at least two to compare timing."
    ages_milliseconds = [age for _, age in samples]
    longest_gap_seconds = max(later[0] - earlier[0] for earlier, later in zip(samples, samples[1:]))
    late_count = sum(age > LATEST_TRACKING_DELAY_MILLISECONDS for age in ages_milliseconds)
    return (f"{len(samples)} reports, median/max age {statistics.median(ages_milliseconds):.0f}/"
            f"{max(ages_milliseconds):.0f} ms, {late_count / len(samples):.1%} over "
            f"{LATEST_TRACKING_DELAY_MILLISECONDS:.0f} ms, longest receive gap {longest_gap_seconds * 1000:.0f} ms.")


if __name__ == "__main__":
    sys.exit(main())
