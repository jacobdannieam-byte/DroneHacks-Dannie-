"""The two programs that every one of our launch files starts on the real drone: the organizers'
flight_interface and our flight_recorder. REAL DRONE.

Our launch files (in ../launch/) say what starts when the drone's software comes up. There are
three, and they differ only in whether the hand-over is on and whether a flight program starts.
A fourth, team_nav.launch.py, only starts watch_only:

    watch_only.launch.py     hand-over OFF, no flight program. Nothing can fly. For the checks
                             and hand tests.
    test_flights.launch.py   hand-over ON, no flight program. Each test flight is started by hand;
                             with no test running, nothing flies even if someone arms the drone.
    course_run.launch.py     the competition: hand-over (OFF until the team approves it), and the
                             course program starts by itself.

This file builds the two programs they share, so the three can't drift apart.

The hand-over
-------------
"Arming" gives the motors permission to spin; a teammate does it on the remote control. The
"hand-over" (PX4's "Offboard" mode) is when the organizers' flight_interface lets our program steer.
With auto_offboard_on_arm on, it hands over when a teammate arms the drone and then flips the mode
switch to Position, but only if one of our programs is sending targets at that moment
(auto_offboard.py in the organizers' code). It reads the setting once, at start-up, so changing it
means restarting the drone's software with another launch file.

This file does not decide which launch file runs. After a restart, the drone starts
team_nav.launch.py, which starts watch_only. To start a different one, type
"./drone.sh start <name>" on the drone (team_nav/drone.sh).
"""
from launch_ros.actions import Node

# From the organizers' fleet settings file for drone 16 (px4/params/fleet/drone16-vio-2026-11-03.params,
# read on the drone by a teammate's session; it is not in our repository): drone 16's PX4 has
# MAV_SYS_ID = 16, and PX4 only obeys commands addressed to its own number. flight_interface's
# default is 1, which would make the hand-over silently fail on drone 16. The simulator's PX4 uses
# 1 and starts its own flight_interface, so this value has never been exercised.
DRONE_16_PX4_SYSTEM_NUMBER = 16


def organizers_flight_interface(hand_over_on, px4_system_number=DRONE_16_PX4_SYSTEM_NUMBER):
    """The organizers' program that passes our targets to PX4. It is the only program that talks
    to PX4. hand_over_on decides whether arming and flipping the switch hands control to us."""
    return Node(
        package="skybeta_flight_interface",
        executable="flight_interface",
        name="flight_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "auto_offboard_on_arm": bool(hand_over_on),
            # Lets software ask for the hand-over. The organizers reserve it for their simulator.
            "allow_mode_request": False,
            "target_system": px4_system_number,
        }],
    )


def depth_camera(mode_settings):
    """Our depth camera program (team_nav/depth_camera.py). It only reads the stereo pictures and
    publishes obstacle points, so it is safe in every launch file. respawn restarts it if it
    crashes, so live navigation gets its depth back within seconds."""
    return Node(
        package="team_nav",
        executable="depth_camera",
        name="team_nav_depth_camera",
        output="log",
        additional_env=mode_settings,
        # Our own choice, as for the recorder: wait 2 seconds before restarting.
        respawn=True,
        respawn_delay=2.0,
    )


def flight_recorder(mode_settings):
    """Our recorder. It only listens, so it is safe in every launch file. respawn restarts it if it
    ever crashes, so a recorder problem never leaves a flight unrecorded for long."""
    return Node(
        package="team_nav",
        executable="flight_recorder",
        name="flight_recorder",
        output="log",
        additional_env=mode_settings,
        # Our own choice: wait 2 seconds before restarting, so a recorder that crashes at once
        # doesn't fill the log by restarting many times a second.
        respawn=True,
        respawn_delay=2.0,
    )
