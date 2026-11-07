"""Stand-ins for the parts of the ROS 2 launch system that launch/team_nav.launch.py uses, so the
launch file can be read and checked on a laptop without ROS. Test code only: it never runs on the
drone. Each stand-in just records what it was given."""
import sys
import types


class LaunchDescription:
    def __init__(self, entities):
        self.entities = entities


class DeclareLaunchArgument:
    def __init__(self, name, default_value=None, description=""):
        self.name = name
        self.default_value = default_value
        self.description = description


class IfCondition:
    def __init__(self, predicate):
        self.predicate = predicate


class LaunchConfigurationNotEquals:
    def __init__(self, launch_configuration_name, expected_value):
        self.launch_configuration_name = launch_configuration_name
        self.expected_value = expected_value


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name


class Node:
    def __init__(self, **settings):
        self.settings = settings


STAND_IN_MODULES = ("launch", "launch.actions", "launch.conditions", "launch.substitutions", "launch_ros",
                    "launch_ros.actions")


def module(name, **contents):
    made = types.ModuleType(name)
    for key, value in contents.items():
        setattr(made, key, value)
    return made


def install():
    sys.modules["launch"] = module("launch", LaunchDescription=LaunchDescription)
    sys.modules["launch.actions"] = module("launch.actions", DeclareLaunchArgument=DeclareLaunchArgument)
    sys.modules["launch.conditions"] = module("launch.conditions", IfCondition=IfCondition,
                                              LaunchConfigurationNotEquals=LaunchConfigurationNotEquals)
    sys.modules["launch.substitutions"] = module("launch.substitutions", LaunchConfiguration=LaunchConfiguration)
    sys.modules["launch_ros"] = module("launch_ros")
    sys.modules["launch_ros.actions"] = module("launch_ros.actions", Node=Node)


def uninstall():
    for name in STAND_IN_MODULES:
        sys.modules.pop(name, None)
