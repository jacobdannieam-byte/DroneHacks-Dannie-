"""Starts every part of the simulator together. SIMULATOR ONLY.

This list is the clearest picture of how the simulator fits together. Read it from top to
bottom. Parts marked REAL are the same software the drone runs. Parts marked PRETEND stand
in for hardware or people, because those do not exist on a laptop.
"""
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from course_simulator.world import prepare_world

REPOSITORY = "/repository"

# The organizers' bridge file is a single script, not an installed package. We run it from
# its folder without changing it, so it must be started by importing it.
RUN_THE_ORGANIZERS_BRIDGE = (
    f"import sys; sys.path.insert(0, '{REPOSITORY}/simulator/organizers_code'); "
    "import px4_odom_bridge; px4_odom_bridge.main()"
)


def run_program(package, program_name, settings=None):
    return Node(package=package, executable=program_name, name=program_name,
                output="screen", parameters=[settings or {}])


def generate_launch_description():
    # Before anything starts: make the course's 3D model and distance tables, if this world
    # doesn't have them yet. world.py explains why this can't be left to the programs below.
    prepare_world()

    return LaunchDescription([
        # REAL: the flight control software. It simulates the drone's physics inside itself.
        ExecuteProcess(cmd=["bash", f"{REPOSITORY}/simulator/px4_changes/start_px4.sh"], output="screen"),

        # REAL: the translator between ROS and PX4.
        ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="log"),

        # REAL: the organizers' program that passes camera tracking reports on to PX4.
        ExecuteProcess(cmd=["python3", "-c", RUN_THE_ORGANIZERS_BRIDGE], output="screen"),

        # REAL: the organizers' program that our flight code talks to. These are the same
        # settings as the organizers' launch file: it enters automatic flight when the switch
        # flips to Position after arming, and it refuses mode requests from software.
        run_program("skybeta_flight_interface", "flight_interface",
                    {"auto_offboard_on_arm": True, "allow_mode_request": False}),

        # PRETEND: stands in for the camera tracking program, which needs real pictures.
        run_program("course_simulator", "pretend_camera_tracking"),

        # PRETEND: stands in for the laser sensor under the drone.
        run_program("course_simulator", "pretend_bottom_laser"),

        # PRETEND: stands in for the distance measurements from the two front cameras.
        run_program("course_simulator", "pretend_depth_camera"),

        # PRETEND: stands in for the teammate with the remote control at the start of a run.
        run_program("course_simulator", "pretend_remote_control"),

        # PRETEND: stands in for the human referee. It also records each flight.
        run_program("course_simulator", "referee"),
    ])
