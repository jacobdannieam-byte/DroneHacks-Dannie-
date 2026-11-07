"""WATCH ONLY: what the drone starts for the checks and the hand tests. Nothing can fly. REAL DRONE.

It starts the organizers' flight_interface with the hand-over OFF, and our flight_recorder. No
flight program starts. Even if someone arms the drone and flips the switch to Position, the
organizers' software never hands control to any program, so the drone stays under the remote
control, and PX4 switches the motors off again after 10 seconds on the ground (COM_DISARM_PRFLT).

Hand tests and checks are started by hand while this runs (drone_code/TEST_SEQUENCE.md). The
recorder records everything the whole time, in practice mode, because nothing here is scored.

The other two launch files, and what they share, are described in team_nav/launch_pieces.py.
"""
from launch import LaunchDescription

from team_nav.launch_pieces import flight_recorder, organizers_flight_interface

# Our own choice: nothing in watch-only mode is scored, so record the extra practice data.
RECORDER_MODE = {"TEAM_NAV_RUN_MODE": "practice"}


def generate_launch_description():
    return LaunchDescription([
        organizers_flight_interface(hand_over_on=False),
        flight_recorder(RECORDER_MODE),
    ])
