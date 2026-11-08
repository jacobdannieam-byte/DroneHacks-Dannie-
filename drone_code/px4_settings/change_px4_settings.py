"""Reads, and if asked changes, a few of PX4's settings ("parameters") on drone 16's flight
controller, over its USB cable. It does the same job as QGroundControl's Parameters page, for
the settings listed below only.

REAL DRONE TOOL. It runs on a laptop, not on the drone, and talks MAVLink (the flight
controller's own message language) over the USB cable plugged into the flight controller.

How to use it
-------------
1. Disarm the drone. Plug the flight controller's USB port into the laptop.
2. Read only (changes nothing):
       python3 change_px4_settings.py
3. Change them, after the team has agreed:
       python3 change_px4_settings.py --change
   It sets each value, reads it back to prove it was saved, and prints old and new values.
4. Restart the flight controller (unplug its power and plug it in again). Some estimator settings
   only take effect after a restart.

What it does not do: arm, change modes, or touch any setting not listed in SETTINGS_TO_CHANGE.
It refuses to change anything if the drone reports that it is armed.
"""
import argparse
import glob
import sys
import time

from pymavlink import mavutil

# The settings we change, with the reason for each. To undo a change, put the old value
# (printed by this tool and written down in px4_settings/README.md) back here and run it again.
SETTINGS_TO_CHANGE = {
    # From PX4's documentation: 5 means "never use the compass". Indoors, drone 16's compass points
    # 135 to 180 degrees away from the true heading (measured 2026-11-08). With the old value 0
    # ("automatic"), PX4 switched to the compass whenever camera tracking's reports came late, and
    # its heading swung round. Without the compass, PX4 keeps its heading from the gyro while it
    # waits for camera tracking: a slow drift of a few degrees instead of a swing.
    "EKF2_MAG_TYPE": 5,
    # From PX4's source code (EstimatorInterface::setExtVisionData): PX4 subtracts this delay from
    # each report's own time stamp. The organizers' bridge already stamps each report with the
    # time its picture was taken, so the old 70 ms (read on drone 16) was counted twice, and PX4 threw away reports
    # 70 ms sooner than it needed to.
    "EKF2_EV_DELAY": 0.0,
    # From PX4's source code (RingBuffer::pop_first_older_than): PX4 uses a camera report only if
    # it is at most about EKF2_DELAY_MAX + 100 ms late. With the old 200 that was about 235 ms
    # (after the double delay above), but on drone 16 reports came up to 412 ms late in bursts, and
    # after 400 ms with no usable report PX4 dropped the camera. 350 accepts up to about 445 ms.
    # The cost: PX4 predicts further ahead from its motion sensors, so corrections come later.
    "EKF2_DELAY_MAX": 350.0,
}

# Our own choice. PX4 normally answers a settings request within a fraction of a second.
ANSWER_WAIT_SECONDS = 3.0
HEARTBEAT_WAIT_SECONDS = 8.0

# From MAVLink's documentation: autopilot type 12 is PX4.
PX4_AUTOPILOT_TYPE = 12


def main():
    arguments = _read_arguments()
    link = _connect(arguments.port)
    if arguments.change and _drone_is_armed(link):
        sys.exit("The drone is armed. Disarm it before changing settings.")
    for name, wanted in SETTINGS_TO_CHANGE.items():
        old = read_setting(link, name)
        if not arguments.change:
            print(f"{name}: now {old}, we want {wanted}")
            continue
        new = change_setting(link, name, wanted)
        print(f"{name}: was {old}, now {new}" + ("" if new == wanted else "   <-- NOT CHANGED, check it in QGroundControl"))
    if arguments.change:
        print("Done. Restart the flight controller so the estimator uses the new settings.")


def _read_arguments():
    parser = argparse.ArgumentParser(description="Read or change drone 16's PX4 settings over USB.")
    parser.add_argument("--change", action="store_true", help="change the settings (default: only read them)")
    parser.add_argument("--port", default=None, help="the USB serial port; found by itself if left out")
    return parser.parse_args()


def _connect(port):
    ports = [port] if port else sorted(glob.glob("/dev/cu.usbmodem*") + glob.glob("/dev/ttyACM*"))
    for candidate in ports:
        link = mavutil.mavlink_connection(candidate, baud=115200)
        heartbeat = link.wait_heartbeat(timeout=HEARTBEAT_WAIT_SECONDS)
        if heartbeat is not None and heartbeat.autopilot == PX4_AUTOPILOT_TYPE:
            print(f"Connected to PX4 on {candidate}.")
            return link
        link.close()
    sys.exit(f"No PX4 flight controller answered on {ports or 'any USB port'}. Check that the flight controller "
             f"has power and its USB cable is plugged into this laptop (not the Jetson's).")


def _drone_is_armed(link):
    heartbeat = link.wait_heartbeat(timeout=HEARTBEAT_WAIT_SECONDS)
    return bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)


def read_setting(link, name):
    """The setting's value, or None if PX4 does not answer."""
    link.mav.param_request_read_send(link.target_system, link.target_component, name.encode(), -1)
    return _wait_for_value(link, name)


def change_setting(link, name, value):
    """Sets the value and returns what PX4 reports back afterwards."""
    if isinstance(value, int):
        # PX4 reads a whole-number setting from the raw bytes of MAVLink's float field. Sending
        # the plain number 5.0 made PX4 read a huge whole number and refuse it (seen on drone 16
        # on 2026-11-08: EKF2_MAG_TYPE stayed 0). So we pack the whole number's bytes ourselves.
        import struct
        packed_value = struct.unpack("<f", struct.pack("<i", value))[0]
        link.param_set_send(name, packed_value, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    else:
        link.param_set_send(name, value, parm_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    _wait_for_value(link, name)
    return read_setting(link, name)


def _wait_for_value(link, name):
    end_seconds = time.monotonic() + ANSWER_WAIT_SECONDS
    while time.monotonic() < end_seconds:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if message is not None and message.param_id == name:
            return _as_number(message)
    return None


def _as_number(message):
    # PX4 sends whole-number settings packed into the float field's bytes; pymavlink's
    # param_set_send packs them the same way, so we unpack them here to show the real value.
    if message.param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        import struct
        return struct.unpack("<i", struct.pack("<f", message.param_value))[0]
    return round(message.param_value, 4)


if __name__ == "__main__":
    main()
