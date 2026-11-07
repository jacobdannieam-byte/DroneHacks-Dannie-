"""What starts on the real drone when its software comes up.

For now this starts only the organizers' flight_interface program, and no mission. The drone's
software requires this program to be running, because it keeps the connection with PX4 alive. It
is PX4 that decides what the motors do, and PX4 only moves the drone when told to.

Why this is safe
----------------
We set auto_offboard_on_arm to false. That means the drone never hands control over to software,
even if someone arms it and flips the switch to Position. If someone does arm it, the drone just
stays under the control of the remote, and PX4 stops the motors after 10 seconds if it has not
taken off (the setting COM_DISARM_PRFLT).

What changes later
------------------
When our mission program is ready and the team has approved it, we add it here and set
auto_offboard_on_arm to true. That is also the setting the organizers' demo uses.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    flight_interface = Node(
        package="skybeta_flight_interface",
        executable="flight_interface",
        name="flight_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "auto_offboard_on_arm": False,
            "allow_mode_request": False,
        }],
    )
    return LaunchDescription([flight_interface])
