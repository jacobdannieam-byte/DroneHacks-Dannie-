"""Starts the whole simulator, with Gazebo in place of PX4's built-in physics. TRIAL ONLY,
SIMULATOR ONLY.

This is the normal simulator's list of programs
(simulator/ros_packages/course_simulator/launch/whole_simulator.launch.py) with one change: the
first program starts Gazebo and PX4 flying a Gazebo drone shaped like drone 16
(inside_the_box/start_px4_in_gazebo.sh), instead of PX4 with its own physics. Everything else
is the same: the organizers' programs, our pretend sensors, the pretend remote control and the
referee. So any test flight in simulator/test_flights can fly in Gazebo unchanged.

What Gazebo changes: walls, boulders and the net are solid, so a touch is a real collision that
can knock the drone off course or crash it. The referee still records touches the same way. The
Gazebo drone has drone 16's motor positions and estimated weight and thrust
(make_drone_16_model.py), but Gazebo's motor and air models, not PX4's.

run_full_route_in_gazebo.sh starts this. Keep it in step with whole_simulator.launch.py: if a
program is added there, add it here too.
"""
import math
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from course_simulator.world import prepare_world

REPOSITORY = "/repository"

def run_program(package, program_name, settings=None, remappings=()):
    return Node(package=package, executable=program_name, name=program_name,
                output="screen", parameters=[{"use_sim_time": True, **(settings or {})}],
                remappings=list(remappings))


def launch_classical(context):
    observe = LaunchConfiguration("observe_only").perform(context) == "true"
    command = ("/fmu/in/vehicle_command", "/classical_observe/vehicle_command")
    interface_remaps = (command,
                        ("/fmu/in/offboard_control_mode", "/classical_observe/offboard_control_mode"),
                        ("/fmu/in/trajectory_setpoint", "/classical_observe/trajectory_setpoint"))
    radio_remaps = (command, ("/simulator_only/start_run", "/classical_observe/start_run"))
    python_env = {"PYTHONPATH": os.pathsep.join((REPOSITORY, os.environ.get("PYTHONPATH", "")))}
    # The pretend sensors and the referee need the course's 3D model and distance tables.
    prepare_world()

    services = [
        # GAZEBO: the physics and the solid course. REAL: PX4, with drone 16's settings.
        ExecuteProcess(cmd=["bash", f"{REPOSITORY}/simulator/gazebo_trial/inside_the_box/start_px4_in_gazebo.sh"],
                       output="screen"),

        # The rest is the same as whole_simulator.launch.py.
        ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="log"),
        ExecuteProcess(cmd=["python3", "-m", "simulator.gazebo_trial.classical_clock"],
                       additional_env=python_env, cwd=REPOSITORY, output="screen"),
        ExecuteProcess(cmd=["python3", "-m", "simulator.gazebo_trial.hybrid_ev_bridge", "--classical"],
                       additional_env=python_env, cwd=REPOSITORY, output="screen"),
        run_program("skybeta_flight_interface", "flight_interface",
                    {"auto_offboard_on_arm": not observe, "allow_mode_request": False,
                     # The route expresses position in the north-facing startup frame.
                     "odom_to_ned_yaw_offset_rad": -math.pi / 2},
                    interface_remaps if observe else ()),
        run_program("course_simulator", "pretend_camera_tracking"),
        run_program("course_simulator", "pretend_bottom_laser"),
        run_program("course_simulator", "pretend_depth_camera"),
        run_program("course_simulator", "pretend_remote_control", remappings=radio_remaps if observe else ()),
        run_program("course_simulator", "referee"),
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=service, on_exit=[EmitEvent(event=Shutdown(reason="Classical Gazebo service exited"))]))
        for service in services]
    return [*handlers, *services]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("observe_only", default_value="false", choices=["true", "false"]),
        OpaqueFunction(function=launch_classical),
    ])
