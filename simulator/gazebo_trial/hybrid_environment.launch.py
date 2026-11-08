"""Launch the imported course and simulated sensors, with no command owner."""

from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

from course_simulator.world import prepare_world
from examples.gazebo_sensor_contract import SIM_ODOM, SIM_STATUS
from simulator.gazebo_trial.calibration_referee import calibration_observer_enabled

ROOT = "/repository"
SIM = {"use_sim_time": True}


def generate_launch_description():
    calibration = calibration_observer_enabled()
    prepare_world()
    observer = (ExecuteProcess(cmd=['python3', '-m', 'simulator.gazebo_trial.calibration_referee',
                                    '--ros-args', '-p', 'use_sim_time:=true'], cwd=ROOT, output='screen')
                if calibration else Node(package='course_simulator', executable='referee', parameters=[SIM]))
    stack = [
        ExecuteProcess(cmd=["bash", f"{ROOT}/simulator/gazebo_trial/inside_the_box/start_px4_in_gazebo.sh"],
                       output="screen"),
        ExecuteProcess(cmd=["MicroXRCEAgent", "udp4", "-p", "8888"], output="screen"),
        ExecuteProcess(cmd=["python3", "-m", "simulator.gazebo_trial.hybrid_sensor_bridge",
                            "--ros-args", "-p", "use_sim_time:=true"], cwd=ROOT, output="screen"),
        ExecuteProcess(cmd=["python3", "-m", "simulator.gazebo_trial.hybrid_ev_bridge"],
                       output="screen"),
        Node(package="course_simulator", executable="pretend_camera_tracking", parameters=[SIM],
             remappings=[("/visual_slam/tracking/odometry", SIM_ODOM), ("/visual_slam/status", SIM_STATUS)]),
        Node(package="course_simulator", executable="pretend_bottom_laser", parameters=[SIM]),
        observer,
    ]
    handlers = [RegisterEventHandler(OnProcessExit(
        target_action=action, on_exit=[EmitEvent(event=Shutdown(reason="Gazebo service exited"))]))
        for action in stack]
    return LaunchDescription([*handlers, *stack])
