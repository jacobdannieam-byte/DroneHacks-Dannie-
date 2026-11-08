"""Starts the course run on the real drone: the organizers' flight_interface, our course_run
program, and our flight_recorder. REAL DRONE.

    ros2 launch team_nav course_run.launch.py run_mode:=practice stop_after_stage:=climb

This is the competition launch file. For the checks and hand tests the drone starts
watch_only.launch.py instead, and for practice flights test_flights.launch.py (team_nav/launch_pieces.py
compares the three). To use this one, the drone's settings must name it (drone_code/README.md).

!!! The hand-over is OFF in this file !!!
-----------------------------------------
HAND_OVER_ON is False below. With it False, flight_interface never hands control to our
program, even when a teammate arms the drone and flips the switch to Position, so course_run just
keeps waiting and nothing flies. Turning it to True is what lets the drone fly by itself. Only do
that after the team has approved a real flight of this program, with a teammate on the kill switch.

allow_mode_request stays False always: it lets software ask for the hand-over, which the
organizers reserve for their simulator.

The four programs
-----------------
  1. flight_interface (the organizers' program) is the only program that talks to PX4. It turns
     our targets into PX4's, and does the hand-over ("Offboard") when the switch flips.
  2. course_run (team_nav/course_run.py) plans the route and flies it.
  3. flight_recorder (team_nav/flight_recorder.py) records everything the drone reports, as a
     separate program, so the record survives a crash of course_run.
  4. depth_camera (team_nav/depth_camera.py) turns the stereo pictures into obstacle points for
     course_run's live map.
If course_run ends, the other two keep running. course_run.py's header says what the drone does
if course_run dies mid-flight.

Launch arguments
----------------
  run_mode              "competition" (the default) or "practice". Passed to course_run and
                        flight_recorder as TEAM_NAV_RUN_MODE.
  stop_after_stage      practice mode only: land after this stage (course_run.py lists them).
  map_file, mission_file, other_boulder_map_files
                        file names in team_nav's course_maps and missions folders, or full paths.
                        other_boulder_map_files is a space-separated list.
  start_heading_degrees which way the drone's nose points on the launch mark (course_run.py).
  navigation            "live" (the default): plan in flight on the live map. "waypoints" and
                        "map_route" are kept to compare (course_run.py).
The number PX4 answers to (16 on drone 16) is set in team_nav/launch_pieces.py.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from team_nav.launch_pieces import depth_camera, flight_recorder, organizers_flight_interface

# !!! OFF until the team approves a real flight of course_run. See the header. !!!
HAND_OVER_ON = False


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("run_mode", default_value="competition",
                              description="competition or practice."),
        DeclareLaunchArgument("stop_after_stage", default_value="",
                              description="Practice mode only: the stage after which the drone lands."),
        DeclareLaunchArgument("map_file", default_value="sketch_2026-11-08.json"),
        DeclareLaunchArgument("mission_file", default_value="dronehacks_2026_side_gap_first.json"),
        DeclareLaunchArgument("other_boulder_map_files", default_value="organizers_drawing.json"),
        # Live navigation: the "waypoints" route refuses to start on both of our maps (the sketch's
        # first checkpoint line is too close to its top wall; the drawing has a turn that is too tight).
        DeclareLaunchArgument("navigation", default_value="live",
                              description="live (default), waypoints or map_route (course_run.py)."),
        DeclareLaunchArgument("start_heading_degrees", default_value="90.0",
                              description="Which way the drone's nose points on the launch mark, in degrees "
                                          "from the course's east towards north. 90 is down the first corridor."),
    ]
    mode_settings = {
        "TEAM_NAV_RUN_MODE": LaunchConfiguration("run_mode"),
        "TEAM_NAV_STOP_AFTER_STAGE": LaunchConfiguration("stop_after_stage"),
    }
    course_run = Node(
        package="team_nav",
        executable="course_run",
        name="course_run",
        output="screen",
        emulate_tty=True,
        additional_env=mode_settings,
        arguments=[
            "--map", LaunchConfiguration("map_file"),
            "--mission", LaunchConfiguration("mission_file"),
            "--start-heading-degrees", LaunchConfiguration("start_heading_degrees"),
            "--other-boulder-maps", LaunchConfiguration("other_boulder_map_files"),
            "--navigation", LaunchConfiguration("navigation"),
        ],
        # Our own choice. On Ctrl+C, give course_run time to land before launch forces it to stop.
        # It only helps if flight_interface is still running, which launch stops at the same time.
        sigterm_timeout="30",
    )
    return LaunchDescription(arguments + [
        organizers_flight_interface(HAND_OVER_ON),
        course_run,
        flight_recorder(mode_settings),
        depth_camera(mode_settings),
    ])
