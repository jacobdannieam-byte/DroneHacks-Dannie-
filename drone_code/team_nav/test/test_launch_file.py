"""launch/team_nav.launch.py, read with stand-in launch modules (fake_launch.py): it starts the
organizers' flight interface exactly as their example does, with the hand-over to software on,
and our mission program next to it."""
import importlib.util
import re
from dataclasses import fields
from pathlib import Path

import pytest

import fake_launch
from team_nav.mission_settings import MissionSettings

PACKAGE_FOLDER = Path(__file__).resolve().parents[1]
LAUNCH_FILE = PACKAGE_FOLDER / "launch" / "team_nav.launch.py"
# From the organizers' example launch file (competitor.launch.py, skybeta 0.3.2): the parameters
# it gives flight_interface, and their defaults. We add only target_system (see below).
EXAMPLE_FLIGHT_INTERFACE_DEFAULTS = {
    "max_altitude_m": "3.0", "min_altitude_m": "0.3", "max_speed_mps": "1.5", "setpoint_rate_hz": "10.0",
    "allow_mode_request": "false", "auto_offboard_on_arm": "true", "odometry_topic": "/visual_slam/tracking/odometry",
}
# Measured on drone 16: MAV_SYS_ID is 16; flight_interface would otherwise address PX4 as system 1.
DRONE_16_SYSTEM_ID = "16"
MISSION_NODE_PARAMETERS_NOT_IN_SETTINGS = {"position_topic", "flight_state_topic", "px4_local_position_topic",
                                           "px4_status_topic", "visual_slam_status_topic", "vision_check",
                                           "mission_kind", "route_file"}
WATCHDOG_THRESHOLDS = ["watchdog_odometry_timeout_seconds", "watchdog_jump_speed_metres_per_second",
                       "watchdog_jump_margin_metres", "watchdog_frozen_seconds", "watchdog_motion_request_metres",
                       "watchdog_motion_request_degrees", "watchdog_motion_request_metres_per_second"]


@pytest.fixture
def launched():
    fake_launch.install()
    specification = importlib.util.spec_from_file_location("team_nav_launch", LAUNCH_FILE)
    launch_module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(launch_module)
    description = launch_module.generate_launch_description()
    yield description
    fake_launch.uninstall()


def arguments(description):
    return {entity.name: entity.default_value for entity in description.entities
            if isinstance(entity, fake_launch.DeclareLaunchArgument)}


def node(description, executable):
    return next(entity.settings for entity in description.entities
                if isinstance(entity, fake_launch.Node) and entity.settings["executable"] == executable)


def test_flight_interface_starts_as_in_the_organizers_example_plus_target_system(launched):
    flight_interface = node(launched, "flight_interface")
    assert flight_interface["package"] == "skybeta_flight_interface"
    passed = {name: value.name for name, value in flight_interface["parameters"][0].items()}
    assert passed == {name: name for name in list(EXAMPLE_FLIGHT_INTERFACE_DEFAULTS) + ["target_system"]}
    declared = arguments(launched)
    for name, default in EXAMPLE_FLIGHT_INTERFACE_DEFAULTS.items():
        assert declared[name] == default, name


def test_the_offboard_request_is_addressed_to_drone_16(launched):
    assert arguments(launched)["target_system"] == DRONE_16_SYSTEM_ID
    assert node(launched, "flight_interface")["parameters"][0]["target_system"].name == "target_system"


def test_the_hand_over_to_software_is_on_and_mode_requests_are_off(launched):
    assert arguments(launched)["auto_offboard_on_arm"] == "true"
    assert arguments(launched)["allow_mode_request"] == "false"


def mission_parameters(launched):
    """Every parameter the launch file gives our mission node, from all its parameter groups."""
    merged = {}
    for group in node(launched, "mission")["parameters"]:
        merged.update(group)
    return merged


def test_our_mission_starts_unless_mission_is_none(launched):
    mission = node(launched, "mission")
    assert mission["package"] == "team_nav"
    condition = mission["condition"]
    assert (condition.launch_configuration_name, condition.expected_value) == ("mission", "none")
    assert arguments(launched)["mission"] == "stages"                     # the course stages by default
    assert mission_parameters(launched)["mission_kind"].name == "mission"
    assert arguments(launched)["cruise_height_metres"] == "0.8"
    known = {setting.name for setting in fields(MissionSettings)} | MISSION_NODE_PARAMETERS_NOT_IN_SETTINGS
    assert set(mission_parameters(launched)) <= known                    # no misspelt parameter names
    assert mission_parameters(launched)["position_topic"].name == "odometry_topic"   # same odometry as the interface


def test_every_watchdog_threshold_is_a_launch_argument_with_the_teams_default(launched):
    defaults = MissionSettings()
    for name in WATCHDOG_THRESHOLDS:
        assert float(arguments(launched)[name]) == getattr(defaults, name), name
        assert mission_parameters(launched)[name].name == name, name
    team_defaults = {"watchdog_odometry_timeout_seconds": 0.2, "watchdog_jump_speed_metres_per_second": 1.5,
                     "watchdog_jump_margin_metres": 0.05, "watchdog_frozen_seconds": 0.5}
    for name, value in team_defaults.items():
        assert getattr(defaults, name) == value, name


def test_the_route_speeds_are_launch_arguments(launched):
    assert float(arguments(launched)["route_speed_metres_per_second"]) == MissionSettings().route_speed_metres_per_second == 0.4
    assert float(arguments(launched)["route_turn_rate_degrees_per_second"]) == MissionSettings().route_turn_rate_degrees_per_second == 15.0
    assert mission_parameters(launched)["route_speed_metres_per_second"].name == "route_speed_metres_per_second"


def test_the_vision_check_is_on_unless_the_simulator_script_switches_it_off(launched):
    assert arguments(launched)["vision_check"] == "true"
    assert mission_parameters(launched)["vision_check"].name == "vision_check"


def test_the_mission_program_the_launch_file_names_is_installed():
    setup_text = (PACKAGE_FOLDER / "setup.py").read_text()
    assert re.search(r'"mission = team_nav\.mission_node:main"', setup_text)
    assert '"launch/team_nav.launch.py"' in setup_text
    assert '"missions/route_waypoints.json"' in setup_text
    assert "install_scripts=$base/lib/team_nav" in (PACKAGE_FOLDER / "setup.cfg").read_text()
