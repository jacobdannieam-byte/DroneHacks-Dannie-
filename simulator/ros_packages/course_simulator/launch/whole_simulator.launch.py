"""Starts every part of the simulator together. SIMULATOR ONLY.

This list is the clearest picture of how the simulator fits together. Read it from top to
bottom. Parts marked REAL are the same software the drone runs. Parts marked PRETEND stand
in for hardware or people, because those do not exist on a laptop.

start_flight_interface:=false leaves out the organizers' flight interface. Use it when the flight
code's own launch file starts that program, as drone_code/team_nav/launch/team_nav.launch.py does
(simulator/run_team_nav.sh). The organizers' guide warns that two flight interfaces fight over the
control flags.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPOSITORY = "/repository"

# The organizers' bridge file is a single script, not an installed package. We run it from
# its folder without changing it, so it must be started by importing it.
RUN_THE_ORGANIZERS_BRIDGE = (
    f"import sys; sys.path.insert(0, '{REPOSITORY}/simulator/organizers_code'); "
    "import px4_odom_bridge; px4_odom_bridge.main()"
)


def run_program(package, program_name, settings=None, condition=None):
    return Node(package=package, executable=program_name, name=program_name,
                output="screen", parameters=[settings or {}], condition=condition)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_flight_interface", default_value="true",
            description="false when the flight code's launch file starts the flight interface itself"),

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
                    {"auto_offboard_on_arm": True, "allow_mode_request": False},
                    condition=IfCondition(LaunchConfiguration("start_flight_interface"))),

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
