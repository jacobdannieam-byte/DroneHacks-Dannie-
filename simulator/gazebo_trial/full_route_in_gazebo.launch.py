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
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

from course_simulator.world import prepare_world

REPOSITORY = "/repository"

RUN_THE_ORGANIZERS_BRIDGE = (
    f"import sys; sys.path.insert(0, '{REPOSITORY}/simulator/organizers_code'); "
    "import px4_odom_bridge; px4_odom_bridge.main()"
)


def run_program(package, program_name, settings=None):
    return Node(package=package, executable=program_name, name=program_name,
                output="screen", parameters=[settings or {}])


def generate_launch_description():
    # The pretend sensors and the referee need the course's 3D model and distance tables.
    prepare_world()

    return LaunchDescription([
        # GAZEBO: the physics and the solid course. REAL: PX4, with drone 16's settings.
        ExecuteProcess(cmd=["bash", f"{REPOSITORY}/simulator/gazebo_trial/inside_the_box/start_px4_in_gazebo.sh"],
                       output="screen"),

        # The rest is the same as whole_simulator.launch.py.
        ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="log"),
        ExecuteProcess(cmd=["python3", "-c", RUN_THE_ORGANIZERS_BRIDGE], output="screen"),
        run_program("skybeta_flight_interface", "flight_interface",
                    {"auto_offboard_on_arm": True, "allow_mode_request": False}),
        run_program("course_simulator", "pretend_camera_tracking"),
        run_program("course_simulator", "pretend_bottom_laser"),
        run_program("course_simulator", "pretend_depth_camera"),
        run_program("course_simulator", "pretend_remote_control"),
        run_program("course_simulator", "referee"),
    ])
