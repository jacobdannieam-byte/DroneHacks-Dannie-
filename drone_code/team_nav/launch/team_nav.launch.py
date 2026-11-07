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

team_nav mission (ours, team_nav/mission_node.py) sends targets to flight_interface: a hold target
at the drone 10 times a second while it waits, then the takeoff, the course and the landing. It also
caps the camera's automatic exposure (team_nav/camera_exposure.py).

team_nav corner_count (ours, team_nav/corner_count_node.py, launch argument corner_count) counts
FAST corners in the left camera's picture for the mission's pre-start check.

team_nav depth_points (ours, team_nav/depth_points_node.py, launch argument depth_points) makes depth
points from the two front cameras on /team/depth_points, for the drift correction.

Both are separate programs, so a crash in either cannot stop the mission or the flight.

This file starts exactly one flight_interface. The organizers' guide warns that two fight over
PX4's control flags, so never run another one (for example competitor_example's) at the same time.

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
    ros2 launch team_nav team_nav.launch.py mission:=route drift_correction:=true depth_topic:=<depth points>
    ros2 launch team_nav team_nav.launch.py cruise_height_metres:=0.6
    ros2 launch team_nav team_nav.launch.py mission:=none      # flight_interface only

With auto_offboard_on_arm:=false the drone never hands control to any program.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, LaunchConfigurationNotEquals
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
    ('watchdog_odometry_timeout_seconds', '0.5', 'rule (a): land if no camera tracking report for this long'),
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
    ('route_file', 'route_waypoints.json',
     'route mission: which waypoints to fly; a file name in the package\'s missions folder '
     '(route_waypoints.json, route_waypoints_drawing.json, route_waypoints_scan.json) or a full path'),
    ('route_speed_metres_per_second', '0.5', 'route mission: speed along each leg'),
    ('route_turn_rate_degrees_per_second', '25.0', 'route mission: fastest turn, in place or while flying through'),
    ('route_arrive_metres', '0.10', 'route mission: a waypoint where the drone stops counts as reached this close'),
    ('route_fly_through_degrees', '25.0', 'route mission: fly through a waypoint whose next leg turns by less than this'),
    ('origin_check_horizontal_metres', '1.0',
     'before the start: warn that the drone was moved after the stack started if camera tracking puts it '
     'further than this sideways from where the stack started'),
    ('origin_check_vertical_metres', '0.3', 'before the start: the same, up or down'),
    ('corner_count', 'true', 'start team_nav corner_count, the left camera FAST corner count for the pre-start check'),
    ('left_image_topic', '/oak/left/image_raw', 'the left camera picture (from the organizers\' guide)'),
    ('right_image_topic', '/oak/right/image_raw', 'the right camera picture (from the organizers\' guide)'),
    ('left_camera_info_topic', '/oak/left/camera_info', 'the left camera\'s lens numbers'),
    ('right_camera_info_topic', '/oak/right/camera_info', 'the right camera\'s lens numbers'),
    ('depth_points', 'true', 'start team_nav depth_points, stereo depth points on /team/depth_points'),
    ('camera_forward_metres', '0.116', 'depth points: the left camera, forward of the drone centre'),
    ('camera_left_metres', '0.037', 'depth points: the left camera, left of the drone centre'),
    ('camera_up_metres', '0.029', 'depth points: the left camera, above the drone centre'),
    ('drift_correction', 'false',
     'route mission: at each corner, correct camera tracking drift from the depth straight ahead and the '
     'course map (team_nav/drift_correction.py). Tried only in the simulator so far'),
    ('depth_topic', '/team/depth_points',
     'drift correction: topic of the depth points (sensor_msgs/PointCloud2, forward/left/up from the '
     'drone centre); team_nav depth_points publishes /team/depth_points'),
    ('course_map_file', 'organizers_drawing.json',
     'drift correction: the course map; a file name in the package\'s course_maps folder '
     '(organizers_drawing.json, phone_scan_2026-11-06.json) or a full path'),
    ('start_heading_degrees', '90.0',
     'drift correction: the way the nose points at takeoff, anticlockwise from the map\'s east; 90 is north, '
     'up the first corridor'),
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
            'route_file': LaunchConfiguration('route_file'),
            'cruise_height_metres': LaunchConfiguration('cruise_height_metres'),
            'route_speed_metres_per_second': LaunchConfiguration('route_speed_metres_per_second'),
            'route_turn_rate_degrees_per_second': LaunchConfiguration('route_turn_rate_degrees_per_second'),
            'route_arrive_metres': LaunchConfiguration('route_arrive_metres'),
            'route_fly_through_degrees': LaunchConfiguration('route_fly_through_degrees'),
            'origin_check_horizontal_metres': LaunchConfiguration('origin_check_horizontal_metres'),
            'origin_check_vertical_metres': LaunchConfiguration('origin_check_vertical_metres'),
            'drift_correction': LaunchConfiguration('drift_correction'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'course_map_file': LaunchConfiguration('course_map_file'),
            'start_heading_degrees': LaunchConfiguration('start_heading_degrees'),
            'tunnel_enabled': LaunchConfiguration('tunnel_enabled'),
            'window_enabled': LaunchConfiguration('window_enabled'),
            'vision_check': LaunchConfiguration('vision_check'),
            'position_topic': LaunchConfiguration('odometry_topic'),
        }, watchdog_parameters()],
    )

    corner_count = Node(
        package='team_nav',
        executable='corner_count',
        name='team_corner_count',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('corner_count')),
        parameters=[{'image_topic': LaunchConfiguration('left_image_topic')}],
    )

    depth_points = Node(
        package='team_nav',
        executable='depth_points',
        name='team_depth_points',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('depth_points')),
        parameters=[{name: LaunchConfiguration(name) for name in (
            'left_image_topic', 'right_image_topic', 'left_camera_info_topic', 'right_camera_info_topic',
            'camera_forward_metres', 'camera_left_metres', 'camera_up_metres')}],
    )

    return LaunchDescription(declared + [flight_interface, mission, corner_count, depth_points])
