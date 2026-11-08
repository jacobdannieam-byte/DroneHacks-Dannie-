"""TEST FLIGHTS: what the drone starts for the practice flights. REAL DRONE.

!!! The hand-over is ON in this file. Use it only after the team has approved test flights, with a
teammate holding the kill switch. !!!

It starts the organizers' flight_interface with the hand-over ON, our flight_recorder and our
depth_camera (team_nav/depth_camera.py, which only reads pictures). No
flight program starts by itself. A test flight flies only when all three of these happen, in order:
  1. someone starts one test program by hand (drone_code/TEST_SEQUENCE.md), which then sends a
     target to hold still above the launch mark;
  2. a teammate arms the drone on the remote, with the mode switch NOT on Position;
  3. the teammate flips the switch to Position.
With no test program running there are no targets, so step 3 hands nothing over: the drone stays
under the remote control and PX4 switches the motors off after 10 seconds on the ground. Flipping
the switch away from Position, or the kill switch, takes control back at any time.

The other two launch files, and what they share, are described in team_nav/launch_pieces.py.
"""
from launch import LaunchDescription

from team_nav.launch_pieces import depth_camera, flight_recorder, organizers_flight_interface

# Our own choice: test flights are practice, so record the extra practice data.
RECORDER_MODE = {"TEAM_NAV_RUN_MODE": "practice"}


def generate_launch_description():
    return LaunchDescription([
        organizers_flight_interface(hand_over_on=True),
        flight_recorder(RECORDER_MODE),
        depth_camera(RECORDER_MODE),
    ])
