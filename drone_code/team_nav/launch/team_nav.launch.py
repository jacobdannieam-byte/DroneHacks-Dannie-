"""What starts on the real drone when its software comes up: the organizers' flight interface and
our mission program.

This follows the organizers' example launch file (competitor_example/launch/competitor.launch.py,
skybeta 0.3.2): the flight_interface node is started as the example starts it, and our mission node
takes the place of their square_mission demo. The one addition is target_system, explained below. Their guide says a team's launch file
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
    ros2 launch team_nav team_nav.launch.py                    # the course stages
    ros2 launch team_nav team_nav.launch.py mission:=route     # the route in missions/route_waypoints.json
    ros2 launch team_nav team_nav.launch.py cruise_height_metres:=0.6
    ros2 launch team_nav team_nav.launch.py mission:=none      # flight_interface only

With auto_offboard_on_arm:=false the drone never hands control to any program.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Measured on drone 16: its PX4 system ID (MAV_SYS_ID) is 16, in
# ~/skybeta/px4/params/fleet/drone16-vio-2026-11-03.params. The flight interface in skybeta 0.3.2
# addresses its Offboard request to target_system, which defaults to 1, so without this the request
# would go to a system that is not there and the hand-over would never happen.
DRONE_16_SYSTEM_ID = '16'

# The position watchdog's thresholds (team_nav/position_watchdog.py), one launch argument each. The
# defaults are the team's, and must match mission_settings.py (a test checks they do).
WATCHDOG_ARGUMENTS = [
    ('watchdog_odometry_timeout_seconds', '0.3', 'rule (a): land if no camera tracking report for this long'),
    ('watchdog_jump_speed_metres_per_second', '1.5', 'rule (b): believable speed between two reports'),
    ('watchdog_jump_margin_metres', '0.05', 'rule (b): plus this much, in metres'),
    ('watchdog_frozen_seconds', '0.5', 'rule (c): land if the pose is bit-identical this long while moving'),
    ('watchdog_motion_request_metres', '0.05', 'rule (c): a position target this far away asks for motion'),
    ('watchdog_motion_request_degrees', '2.0', 'rule (c): a heading this far round asks for motion'),
    ('watchdog_motion_request_metres_per_second', '0.02', 'rule (c): a velocity this fast asks for motion'),
]

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
    ('target_system', DRONE_16_SYSTEM_ID,
     'PX4 system ID (MAV_SYS_ID) that the Offboard request is addressed to; 16 on drone 16'),
    ('cruise_height_metres', '0.8', 'our mission: flight height above the launch floor, metres'),
    ('tunnel_enabled', 'false', 'our mission: false hovers at the tunnel entrance, then lands'),
    ('window_enabled', 'false', 'our mission: false goes through Gate B'),
    ('vision_check', 'true',
     'SIMULATOR ONLY: false treats camera tracking health as OK, because the simulator has no camera '
     'images. The mission refuses false unless simulator/run_team_nav.sh started it'),
    ('mission', 'stages', 'our mission: stages (the course stages), route (missions/route_waypoints.json), '
                          'or none (start only the flight interface)'),
    ('route_speed_metres_per_second', '0.5', 'route mission: speed along each leg'),
    ('route_turn_rate_degrees_per_second', '25.0', 'route mission: fastest turn, in place or while flying through'),
    ('route_arrive_metres', '0.10', 'route mission: a waypoint where the drone stops counts as reached this close'),
    ('route_fly_through_degrees', '25.0', 'route mission: fly through a waypoint whose next leg turns by less than this'),
] + WATCHDOG_ARGUMENTS



def watchdog_parameters():
    """Each watchdog threshold, passed from its launch argument to the mission node."""
    parameters = {}
    for name, _, _ in WATCHDOG_ARGUMENTS:
        parameters[name] = LaunchConfiguration(name)
    return parameters


def generate_launch_description(mission_default='stages'):
    """mission_default lets team_nav_route.launch.py reuse this file with mission:=route as the
    default; nothing else differs between the two."""
    declared = []
    for name, default, help_ in ARGUMENTS:
        if name == 'mission':
            default = mission_default
        declared.append(DeclareLaunchArgument(name, default_value=default, description=help_))

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
            'target_system': LaunchConfiguration('target_system'),
        }],
    )

    mission = Node(
        package='team_nav',
        executable='mission',
        name='team_nav_mission',
        output='screen',
        emulate_tty=True,
        condition=LaunchConfigurationNotEquals('mission', 'none'),
        parameters=[{
            'mission_kind': LaunchConfiguration('mission'),
            'cruise_height_metres': LaunchConfiguration('cruise_height_metres'),
            'route_speed_metres_per_second': LaunchConfiguration('route_speed_metres_per_second'),
            'route_turn_rate_degrees_per_second': LaunchConfiguration('route_turn_rate_degrees_per_second'),
            'route_arrive_metres': LaunchConfiguration('route_arrive_metres'),
            'route_fly_through_degrees': LaunchConfiguration('route_fly_through_degrees'),
            'tunnel_enabled': LaunchConfiguration('tunnel_enabled'),
            'window_enabled': LaunchConfiguration('window_enabled'),
            'vision_check': LaunchConfiguration('vision_check'),
            'position_topic': LaunchConfiguration('odometry_topic'),
        }, watchdog_parameters()],
    )

    return LaunchDescription(declared + [flight_interface, mission])
