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
ROUTE_LAUNCH_FILE = PACKAGE_FOLDER / "launch" / "team_nav_route.launch.py"
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
                                           "mission_kind", "route_file", "depth_topic", "course_map_file"}
WATCHDOG_THRESHOLDS = ["watchdog_odometry_timeout_seconds", "watchdog_jump_speed_metres_per_second",
                       "watchdog_jump_margin_metres", "watchdog_frozen_seconds", "watchdog_motion_request_metres",
                       "watchdog_motion_request_degrees", "watchdog_motion_request_metres_per_second"]


def load(launch_file):
    specification = importlib.util.spec_from_file_location(launch_file.stem.replace(".", "_"), launch_file)
    launch_module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(launch_module)
    return launch_module.generate_launch_description()


@pytest.fixture
def launched():
    fake_launch.install()
    yield load(LAUNCH_FILE)
    fake_launch.uninstall()


@pytest.fixture
def launched_route():
    fake_launch.install()
    yield load(ROUTE_LAUNCH_FILE)
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
    team_defaults = {"watchdog_odometry_timeout_seconds": 0.5, "watchdog_jump_speed_metres_per_second": 1.5,
                     "watchdog_jump_margin_metres": 0.05, "watchdog_frozen_seconds": 0.5}
    for name, value in team_defaults.items():
        assert getattr(defaults, name) == value, name


def test_the_route_file_is_a_launch_argument_defaulting_to_the_current_route(launched):
    assert arguments(launched)["route_file"] == "route_waypoints.json"
    assert mission_parameters(launched)["route_file"].name == "route_file"


def test_the_route_settings_are_launch_arguments_with_the_teams_defaults(launched):
    team_defaults = {"route_speed_metres_per_second": 0.5, "route_turn_rate_degrees_per_second": 25.0,
                     "route_arrive_metres": 0.10, "route_fly_through_degrees": 25.0}
    for name, value in team_defaults.items():
        assert float(arguments(launched)[name]) == getattr(MissionSettings(), name) == value, name
        assert mission_parameters(launched)[name].name == name, name


def test_the_vision_check_is_on_unless_the_simulator_script_switches_it_off(launched):
    assert arguments(launched)["vision_check"] == "true"
    assert mission_parameters(launched)["vision_check"].name == "vision_check"


def test_the_route_launch_file_loads_with_mission_route_and_is_otherwise_identical(launched, launched_route):
    assert arguments(launched_route)["mission"] == "route"
    route_arguments = dict(arguments(launched_route))
    main_arguments = dict(arguments(launched))
    route_arguments.pop("mission")
    main_arguments.pop("mission")
    assert route_arguments == main_arguments
    for executable in ("flight_interface", "mission"):
        route_node, main_node = node(launched_route, executable), node(launched, executable)
        assert route_node.keys() == main_node.keys()
        assert route_node["package"] == main_node["package"]
        assert [{name: value.name for name, value in group.items()} for group in route_node["parameters"]] == \
               [{name: value.name for name, value in group.items()} for group in main_node["parameters"]]


def test_the_mission_program_the_launch_file_names_is_installed():
    setup_text = (PACKAGE_FOLDER / "setup.py").read_text()
    assert re.search(r'"mission = team_nav\.mission_node:main"', setup_text)
    assert '"launch/team_nav.launch.py"' in setup_text
    assert '"launch/team_nav_route.launch.py"' in setup_text
    for route_file in ("route_waypoints.json", "route_waypoints_drawing.json", "route_waypoints_scan.json"):
        assert f'"missions/{route_file}"' in setup_text, route_file
        assert (PACKAGE_FOLDER / "missions" / route_file).is_file(), route_file
    assert "install_scripts=$base/lib/team_nav" in (PACKAGE_FOLDER / "setup.cfg").read_text()
    for map_file in ("organizers_drawing.json", "phone_scan_2026-11-06.json"):
        assert f'"course_maps/{map_file}"' in setup_text, map_file
        assert (PACKAGE_FOLDER / "course_maps" / map_file).is_file(), map_file


def test_the_drift_correction_is_off_by_default_and_its_inputs_are_launch_arguments(launched):
    declared = arguments(launched)
    assert declared["drift_correction"] == "false"
    assert MissionSettings().drift_correction is False
    assert declared["depth_topic"] == ""                                   # no depth program on the drone yet
    assert declared["course_map_file"] == "organizers_drawing.json"
    assert float(declared["start_heading_degrees"]) == MissionSettings().start_heading_degrees == 90.0
    for name in ("drift_correction", "depth_topic", "course_map_file", "start_heading_degrees"):
        assert mission_parameters(launched)[name].name == name, name


def test_exactly_one_flight_interface_starts_with_the_hand_over_on_and_drone_16s_system_id(launched):
    """The organizers' guide: exactly one flight_interface, started from our launch file with
    auto_offboard_on_arm true; two fight over PX4's control flags."""
    started = [entity.settings for entity in launched.entities if isinstance(entity, fake_launch.Node)]
    interfaces = [settings for settings in started if settings["executable"] == "flight_interface"]
    assert len(interfaces) == 1
    assert interfaces[0]["package"] == "skybeta_flight_interface"
    assert "condition" not in interfaces[0]                              # nothing can switch it off
    parameters = interfaces[0]["parameters"][0]
    assert parameters["auto_offboard_on_arm"].name == "auto_offboard_on_arm"
    assert parameters["target_system"].name == "target_system"
    declared = arguments(launched)
    assert declared["auto_offboard_on_arm"] == "true" and declared["target_system"] == "16"
    assert not any(settings["package"] == "competitor_example" for settings in started)   # no square_mission


def test_the_corner_count_starts_as_its_own_program_and_the_origin_limits_are_arguments(launched):
    corner_count = node(launched, "corner_count")
    assert corner_count["package"] == "team_nav"
    assert corner_count["condition"].predicate.name == "corner_count"
    assert corner_count["parameters"][0]["image_topic"].name == "left_image_topic"
    declared = arguments(launched)
    assert declared["corner_count"] == "true"
    assert declared["left_image_topic"] == "/oak/left/image_raw"
    for name, value in (("origin_check_horizontal_metres", 1.0), ("origin_check_vertical_metres", 0.3)):
        assert float(declared[name]) == getattr(MissionSettings(), name) == value
        assert mission_parameters(launched)[name].name == name
    setup_text = (PACKAGE_FOLDER / "setup.py").read_text()
    assert '"corner_count = team_nav.corner_count_node:main"' in setup_text
