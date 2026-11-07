"""Listens to every sensor on drone 16 for a few seconds and says which ones are alive. READ ONLY.

check_drone_16.sh sends this file to the drone and runs it inside the organizers' software
(the skybeta-stack container), where ROS and the drone's message types are installed.

It only listens. It never sends a message, so it cannot arm the drone, move a motor or change
a setting.

For each sensor it prints PASS if messages arrived and FAIL if none did, with the number of
messages per second. For some sensors it also prints the reading that matters, such as the
bottom laser's distance or the battery voltage, so a person can judge whether it looks right.
"""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

# Our own choice: time for this program to discover the drone's sensors before listening.
DISCOVERY_SECONDS = 3.0
# Our own choice: long enough for several laser readings and camera pictures.
LISTEN_SECONDS = 8.0

# From PX4 1.16 source: VehicleStatus.arming_state is 2 when the motors are allowed to spin.
ARMING_STATE_ARMED = 2

# Our own choice. On a table, the bottom laser should read a few centimetres. Outside this range
# a person should look at it.
LASER_SENSIBLE_RANGE_METRES = (0.0, 4.0)

# The sensors to check, with the names from the organizers' documentation. PX4 adds a version
# ending such as "_v1" to some names, so each name is matched by how it starts. "Required" ones
# are needed to fly; the others are useful but may not exist on this drone.
SENSORS = [
    # (topic name, what it is, required)
    ("/fmu/out/vehicle_status", "Flight controller status", True),
    ("/fmu/out/vehicle_attitude", "Flight controller tilt and heading", True),
    ("/fmu/out/sensor_combined", "Flight controller motion sensors", True),
    ("/fmu/out/vehicle_local_position", "Flight controller position and height estimate", True),
    ("/fmu/out/distance_sensor", "Bottom laser, raw reading", False),
    ("/fmu/out/battery_status", "Battery", False),
    ("/fmu/out/estimator_status_flags", "Which sensors the flight controller is using", False),
    ("/oak/left/image_raw", "Left camera pictures", True),
    ("/oak/right/image_raw", "Right camera pictures", True),
    ("/oak/left/camera_info", "Left camera lens settings", True),
    ("/oak/right/camera_info", "Right camera lens settings", False),
    ("/oak/imu/data", "Camera's own motion sensor", True),
    ("/visual_slam/tracking/odometry", "Camera tracking position", True),
    ("/visual_slam/status", "Camera tracking status", False),
    ("/skybeta/state", "Organizers' flight interface state", False),
]


class SensorListener(Node):
    """Subscribes to one topic per sensor and remembers the message count and the last message."""

    def __init__(self):
        super().__init__("team_hardware_check_listener")
        self.message_counts = {}
        self.last_messages = {}

    def listen_to(self, topic_name, type_name):
        self.message_counts[topic_name] = 0
        message_class = get_message(type_name)

        def on_message(message, topic_name=topic_name):
            self.message_counts[topic_name] += 1
            self.last_messages[topic_name] = message

        # "Sensor data" quality settings receive from both reliable and best-effort senders.
        self.create_subscription(message_class, topic_name, on_message, qos_profile_sensor_data)


def find_live_topic(wanted_name, live_topics):
    """The live topic for a sensor name, preferring the highest version ending. None if absent."""
    if wanted_name in live_topics:
        best_name = wanted_name
    else:
        best_name = None
    best_version = -1
    for name in live_topics:
        ending = name[len(wanted_name):]
        if name.startswith(wanted_name + "_v") and ending[2:].isdigit():
            if int(ending[2:]) > best_version:
                best_version = int(ending[2:])
                best_name = name
    return best_name


def describe_reading(message):
    """A short, plain description of the reading that matters for this kind of message."""
    kind = type(message).__name__
    if kind == "DistanceSensor":
        return f"laser reads {message.current_distance:.2f} m"
    if kind == "VehicleLocalPosition":
        height_above_floor = message.dist_bottom if message.dist_bottom_valid else None
        laser_text = f"{height_above_floor:.2f} m" if height_above_floor is not None else "not valid"
        return f"height from laser: {laser_text}; position valid: {bool(message.xy_valid)}"
    if kind == "VehicleStatus":
        armed = message.arming_state == ARMING_STATE_ARMED
        return f"armed: {'YES - motors could spin' if armed else 'no'}; mode number {message.nav_state}"
    if kind == "BatteryStatus":
        return f"{message.voltage_v:.2f} V, {message.remaining * 100:.0f}% left"
    if kind == "Image":
        return f"{message.width} x {message.height} pixels, {message.encoding}"
    if kind == "CameraInfo":
        lens_correction = list(message.d)
        if all(value == 0.0 for value in lens_correction):
            return "lens correction values are all zero: either already corrected, or calibration missing"
        return f"lens correction values: {[round(value, 4) for value in lens_correction]}"
    if kind == "Odometry":
        position = message.pose.pose.position
        return f"position x {position.x:.2f}, y {position.y:.2f}, z {position.z:.2f} m"
    return ""


def warnings_about(message):
    """Things a person must look at before going further."""
    kind = type(message).__name__
    if kind == "VehicleStatus" and message.arming_state == ARMING_STATE_ARMED:
        return ["The drone reports it is ARMED. Disarm it with the remote control before handling it."]
    if kind == "DistanceSensor":
        low, high = LASER_SENSIBLE_RANGE_METRES
        if not low <= message.current_distance <= high:
            return [f"The laser reading {message.current_distance:.2f} m is outside {low}-{high} m."]
    return []


def main():
    rclpy.init()
    listener = SensorListener()

    # Let this program discover what the drone is sending.
    discovery_end = time.monotonic() + DISCOVERY_SECONDS
    while time.monotonic() < discovery_end:
        rclpy.spin_once(listener, timeout_sec=0.1)
    live_topics = dict(listener.get_topic_names_and_types())

    found_topics = {}
    for wanted_name, _, _ in SENSORS:
        live_name = find_live_topic(wanted_name, live_topics)
        if live_name is not None:
            found_topics[wanted_name] = live_name
            listener.listen_to(live_name, live_topics[live_name][0])

    listen_end = time.monotonic() + LISTEN_SECONDS
    while time.monotonic() < listen_end:
        rclpy.spin_once(listener, timeout_sec=0.05)

    required_failures = 0
    all_warnings = []
    print(f"Listened for {LISTEN_SECONDS:.0f} seconds. ROS knows about {len(live_topics)} topics.\n")
    for wanted_name, description, required in SENSORS:
        live_name = found_topics.get(wanted_name)
        if live_name is None:
            verdict = "FAIL" if required else "ABSENT"
            print(f"{verdict:6} {description}: no such topic ({wanted_name})")
            required_failures += 1 if required else 0
            continue
        count = listener.message_counts[live_name]
        if count == 0:
            verdict = "FAIL" if required else "SILENT"
            print(f"{verdict:6} {description}: the topic exists but sent nothing ({live_name})")
            required_failures += 1 if required else 0
            continue
        message = listener.last_messages[live_name]
        rate_per_second = count / LISTEN_SECONDS
        print(f"PASS   {description}: {rate_per_second:.1f} per second; {describe_reading(message)} ({live_name})")
        all_warnings += warnings_about(message)

    print()
    for warning in all_warnings:
        print(f"LOOK:  {warning}")
    if required_failures == 0:
        print("RESULT: every sensor needed for flight is sending data.")
    else:
        print(f"RESULT: {required_failures} sensor(s) needed for flight sent nothing. See the FAIL lines above.")

    listener.destroy_node()
    rclpy.shutdown()
    return 0 if required_failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
