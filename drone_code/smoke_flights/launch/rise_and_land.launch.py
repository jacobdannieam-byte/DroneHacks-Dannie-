"""What starts on the real drone for the rise-and-land test flight. FOR THE REAL DRONE.

The organizers' software starts this file when the drone's settings name this package (see
README.md in this folder). It starts two programs:

1. The organizers' flight interface, the only program that talks to PX4. It is set so that when a
   teammate arms the drone and flips the switch to Position, it hands control to our program
   once ("auto_offboard_on_arm"). Its other limits are the organizers' own defaults: no higher
   than 3 m and no lower than 0.3 m above the starting point, and no faster than 1.5 m/s.
2. Our rise-and-land program (team_smoke_flights/rise_and_land.py).

To change the height or the hold time without editing code, for example:
    ros2 launch team_smoke_flights rise_and_land.launch.py rise_height_metres:=0.6 hold_seconds:=8.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# From the organizers' fleet settings file for drone 16 (px4/params/fleet/drone16-vio-2026-11-03.params):
# drone 16's PX4 has MAV_SYS_ID = 16, and PX4 only obeys commands addressed to its own number.
# The flight interface's default is 1, which would make the hand-over silently fail on drone 16.
DRONE_16_PX4_SYSTEM_NUMBER = 16


def generate_launch_description():
    rise_height = DeclareLaunchArgument("rise_height_metres", default_value="0.5",
                                        description="How high to rise above the starting point, in metres.")
    hold_time = DeclareLaunchArgument("hold_seconds", default_value="5.0",
                                      description="How long to hold at that height, in seconds.")

    flight_interface = Node(
        package="skybeta_flight_interface",
        executable="flight_interface",
        name="flight_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "auto_offboard_on_arm": True,
            # The service that lets a program request the hand-over is for the simulator only.
            "allow_mode_request": False,
            "target_system": DRONE_16_PX4_SYSTEM_NUMBER,
        }],
    )
    rise_and_land = Node(
        package="team_smoke_flights",
        executable="rise_and_land",
        name="team_rise_and_land",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "rise_height_metres": LaunchConfiguration("rise_height_metres"),
            "hold_seconds": LaunchConfiguration("hold_seconds"),
        }],
    )
    return LaunchDescription([rise_height, hold_time, flight_interface, rise_and_land])
