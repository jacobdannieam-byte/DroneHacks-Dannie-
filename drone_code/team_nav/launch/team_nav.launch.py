"""The file that drone 16 starts by itself when its software starts. REAL DRONE.

The drone's settings file (/etc/skybeta/drone.env) names this file. This file starts
watch_only.launch.py and nothing else. Thus, after each restart, no program can fly the drone.

To fly, type "./ours.sh start <name>" on the drone. drone_code/TEST_SEQUENCE.md gives the steps.
team_nav/launch_pieces.py compares the launch files.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_folder = os.path.join(get_package_share_directory("team_nav"), "launch")
    watch_only = PythonLaunchDescriptionSource(os.path.join(launch_folder, "watch_only.launch.py"))
    return LaunchDescription([IncludeLaunchDescription(watch_only)])
