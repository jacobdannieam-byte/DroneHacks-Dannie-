"""Starts every program of the full Gazebo simulator, except the team's flight program.
SIMULATOR ONLY.

Read it from top to bottom: it is the clearest picture of what runs. Parts marked REAL are the
same software drone 16 runs. Parts marked PRETEND stand in for hardware or people.

fly_course_in_gazebo.sh starts this inside the box, and then starts the team's flight program
(ros2 run team_nav course_run) the same way the drone does, through
simulator/test_flights/real_program_test.py, which also plays the teammate who arms the drone.

How it differs from the normal simulator (simulator/ros_packages/course_simulator/launch/
whole_simulator.launch.py): Gazebo provides the physics, the solid course and two camera eyes,
instead of PX4's built-in physics; the ray-traced pretend depth camera is left out, and the
team's own depth program (team_nav depth_camera) runs instead, on the Gazebo camera pictures.
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

# PX4's system number. The Gazebo PX4 keeps PX4's built-in MAV_SYS_ID of 1, because
# drone_16_settings.sh doesn't set it, and the flight interface must address the same number.
PX4_SYSTEM_NUMBER = 1


def run_program(package, program_name, settings=None):
    return Node(package=package, executable=program_name, name=program_name,
                output="screen", parameters=[settings or {}])


def generate_launch_description():
    # The pretend sensors and the referee need the world map's 3D model and distance tables.
    # Making them before anything starts keeps the programs below from all making them at once.
    prepare_world()

    return LaunchDescription([
        # GAZEBO: the physics, the solid textured course and the two camera eyes (PRETEND OAK-D).
        # REAL: PX4 1.16.2 with drone 16's settings.
        ExecuteProcess(cmd=["bash", f"{REPOSITORY}/simulator/gazebo_full/inside_the_box/start_px4_and_gazebo.sh"],
                       output="screen"),

        # REAL: the translator between ROS and PX4.
        ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="log"),

        # REAL: the organizers' program that passes camera tracking reports on to PX4.
        ExecuteProcess(cmd=["python3", "-c", RUN_THE_ORGANIZERS_BRIDGE], output="screen"),

        # REAL: the organizers' program our flight code talks to, with the organizers' settings:
        # it enters automatic flight when the switch flips to Position after arming, and it
        # refuses mode requests from software.
        run_program("skybeta_flight_interface", "flight_interface",
                    {"auto_offboard_on_arm": True, "allow_mode_request": False,
                     "target_system": PX4_SYSTEM_NUMBER}),

        # REAL: the team's depth program. It reads the two camera eyes' pictures and lens numbers
        # (/oak/left|right/image_raw and camera_info) and publishes /team_nav/obstacle_points,
        # which course_run's live navigation uses. Started as "ros2 run team_nav depth_camera".
        run_program("team_nav", "depth_camera"),

        # PRETEND: the camera tracking (cuVSLAM), made from the drone's true position in Gazebo,
        # with a delay, jitter and slow sideways drift.
        run_program("course_simulator", "pretend_camera_tracking"),

        # PRETEND: the laser under the drone.
        run_program("course_simulator", "pretend_bottom_laser"),

        # PRETEND: the teammate with the remote control at the start of a run.
        run_program("course_simulator", "pretend_remote_control"),

        # PRETEND: the human referee. It counts touches and records the true path.
        run_program("course_simulator", "referee"),
    ])
