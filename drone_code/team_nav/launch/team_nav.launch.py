"""What starts on the real drone when its software comes up: the organizers' flight interface and
our mission program.

This follows the organizers' example launch file (competitor_example/launch/competitor.launch.py,
skybeta 0.3.2): the flight_interface node is started exactly as the example starts it, and our
mission node takes the place of their square_mission demo. Their guide says a team's launch file
must start flight_interface with auto_offboard_on_arm set to true, or the drone stays under the
pilot's control.

What each program does
----------------------
flight_interface (the organizers') is the only program that talks to PX4, the flight controller's
software. It keeps the connection to PX4 alive, applies the safety limits (height band, speed cap)
to every target, and hands control to our code at the start of a run.

team_nav mission (ours, team_nav/mission_node.py) sends targets to flight_interface: a target just
above the drone while it waits, then the takeoff, the course and the landing. It also caps the
camera's automatic exposure (team_nav/camera_exposure.py).

Why this is safe
----------------
Nothing moves until a person acts. "Arming" means giving the motors permission to spin, and only
the pilot can arm, on the remote control. flight_interface hands control to our code ("Offboard")
only after the pilot arms on the ground and then flips the mode switch to Position, once per
arming. The pilot can take the drone back at any time with the mode switch or the sticks, and our
mission then stops sending anything. The team rehearses the start with the propellers off first.

How to use it
-------------
    ros2 launch team_nav team_nav.launch.py
    ros2 launch team_nav team_nav.launch.py cruise_height_metres:=0.6
    ros2 launch team_nav team_nav.launch.py mission:=false     # flight_interface only

With auto_offboard_on_arm:=false the drone never hands control to any program.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name -> (default, help). The flight interface values are the organizers' example defaults, so the
# safety limits are exactly theirs unless someone tightens them on the command line.
ARGUMENTS = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0',
     'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false',
     'enable /skybeta/request_offboard. SIM ONLY: on a real vehicle Offboard '
     'follows the pilot arming, then switching to Position'),
    ('auto_offboard_on_arm', 'true',
     'enter Offboard once when the pilot arms, then switches to Position. false: the vehicle '
     'stays in Position under the pilot'),
    ('odometry_topic', '/visual_slam/tracking/odometry',
     'source of the ENU pose the altitude band is measured against'),
    ('cruise_height_metres', '0.8', 'our mission: flight height above the launch floor, metres'),
    ('tunnel_enabled', 'false', 'our mission: false hovers at the tunnel entrance, then lands'),
    ('window_enabled', 'false', 'our mission: false goes through Gate B'),
    ('mission', 'true', 'also start our mission program'),
]


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(name, default_value=default, description=help_)
        for name, default, help_ in ARGUMENTS
    ]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    mission = Node(
        package='team_nav',
        executable='mission',
        name='team_nav_mission',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('mission')),
        parameters=[{
            'cruise_height_metres': LaunchConfiguration('cruise_height_metres'),
            'tunnel_enabled': LaunchConfiguration('tunnel_enabled'),
            'window_enabled': LaunchConfiguration('window_enabled'),
            'position_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    return LaunchDescription(declared + [flight_interface, mission])
