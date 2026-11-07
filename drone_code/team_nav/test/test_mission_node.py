"""mission_node.py, the ROS 2 program, run on stand-in ROS (fake_ros.py) against the pretend drone
(fake_drone.py): which topics it hears and sends on, and a whole flight through its callbacks."""
import importlib
import json
import math
import struct
import sys
import types
from dataclasses import fields
from pathlib import Path

import pytest

import fake_ros
from fake_drone import FakeDrone
from team_nav.mission_settings import MissionSettings

POSITION_TOPIC = "/visual_slam/tracking/odometry"
FLIGHT_STATE_TOPIC = "/skybeta/state"
PX4_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
PX4_STATUS_TOPIC = "/fmu/out/vehicle_status_v1"
VISUAL_SLAM_STATUS_TOPIC = "/visual_slam/status"
CORNER_COUNT_TOPIC = "/team/left_corner_count"
POSE_TOPIC = "/skybeta/setpoint/pose"
TWIST_TOPIC = "/skybeta/setpoint/twist"
STEP_SECONDS = 1.0 / MissionSettings().steps_per_second           # the node's timer period: 20 Hz


@pytest.fixture
def build_node():
    fake_ros.install()
    # Where the package is installed: here, the source folder, which has missions/ and course_maps/.
    installed = types.ModuleType("ament_index_python.packages")
    installed.get_package_share_directory = lambda package: str(Path(__file__).resolve().parents[1])
    sys.modules["ament_index_python"] = types.ModuleType("ament_index_python")
    sys.modules["ament_index_python.packages"] = installed
    sys.modules.pop("team_nav.mission_node", None)
    node_module = importlib.import_module("team_nav.mission_node")

    def build(**parameters):
        fake_ros.Node.parameter_overrides = parameters
        fake_ros.Clock.seconds = 0.0
        return node_module.MissionNode()
    yield build
    fake_ros.Node.parameter_overrides = {}
    sys.modules.pop("team_nav.mission_node", None)
    sys.modules.pop("ament_index_python", None)
    sys.modules.pop("ament_index_python.packages", None)
    fake_ros.uninstall()


def deliver(node, topic, message):
    node.subscriptions[topic][1](message)


def deliver_reports(node, drone):
    odometry = fake_ros.Odometry()
    odometry.header.stamp = fake_ros.Clock().now().to_msg()
    position = odometry.pose.pose.position
    position.x, position.y, position.z = drone.reported_position()
    odometry.pose.pose.orientation = fake_ros.Quaternion(z=math.sin(drone.heading_radians / 2),
                                                         w=math.cos(drone.heading_radians / 2))
    deliver(node, POSITION_TOPIC, odometry)
    offboard, armed, landed, px4_link = drone.flight_state()
    state = {"offboard_active": offboard, "armed": armed, "landed": landed, "px4_link": px4_link,
             "nav_state_name": "whatever"}
    deliver(node, FLIGHT_STATE_TOPIC, fake_ros.String(json.dumps(state)))
    local_position = fake_ros.VehicleLocalPosition()
    local_position.dist_bottom = drone.laser_reading()
    deliver(node, PX4_LOCAL_POSITION_TOPIC, local_position)
    deliver(node, PX4_STATUS_TOPIC, fake_ros.VehicleStatus(drone.px4_nav_state()))


def as_command(topic, message):
    """Turn a published message back into the command the pretend drone understands."""
    if topic == POSE_TOPIC:
        orientation = message.pose.orientation
        heading = math.atan2(2 * orientation.w * orientation.z, 1 - 2 * orientation.z ** 2)
        position = message.pose.position
        return types.SimpleNamespace(kind="position", position=(position.x, position.y, position.z),
                                     heading_radians=heading, velocity=None)
    linear = message.twist.linear
    return types.SimpleNamespace(kind="velocity", position=None, heading_radians=None,
                                 velocity=(linear.x, linear.y, linear.z))


def fly_node(node, drone, seconds, every_step=None, arm_at_seconds=0.5, switch_at_seconds=1.0):
    """Returns [(seconds, topic, message)] for everything the node published."""
    sent = []
    seen = {POSE_TOPIC: 0, TWIST_TOPIC: 0}
    tick = node.timers[0][1]
    now = 0.0
    while now < seconds - 1e-9:
        fake_ros.Clock.seconds = now
        if every_step is not None:
            every_step(now, node, drone)
        if now >= arm_at_seconds and not drone.armed and not drone.has_flown:
            drone.arm()
        if now >= switch_at_seconds and not drone.offboard and not drone.has_flown:
            drone.switch_to_position_mode(now)
        deliver_reports(node, drone)
        tick()
        for topic in (POSE_TOPIC, TWIST_TOPIC):
            for message in node.publishers[topic].sent[seen[topic]:]:
                sent.append((now, topic, message))
                drone.receive(now, as_command(topic, message))
            seen[topic] = len(node.publishers[topic].sent)
        drone.advance(now, STEP_SECONDS)
        now = round(now + STEP_SECONDS, 9)
    return sent


def logged(node):
    return [text for _, text in node.log_lines]


# ---------------------------------------------------------------- wiring
def test_hears_six_topics_and_sends_only_on_the_two_setpoint_topics(build_node):
    node = build_node()
    assert set(node.subscriptions) == {POSITION_TOPIC, FLIGHT_STATE_TOPIC, PX4_LOCAL_POSITION_TOPIC,
                                       PX4_STATUS_TOPIC, VISUAL_SLAM_STATUS_TOPIC, CORNER_COUNT_TOPIC}
    assert set(node.publishers) == {POSE_TOPIC, TWIST_TOPIC}
    assert not any(topic.startswith("/fmu/in") for topic in node.publishers)
    assert node.timers[0][0] == pytest.approx(1.0 / MissionSettings().steps_per_second)


def test_px4_reports_are_heard_the_way_px4_sends_them(build_node):
    quality = build_node().subscriptions[PX4_LOCAL_POSITION_TOPIC][2]
    assert quality.settings["reliability"] == "best_effort" and quality.settings["durability"] == "transient_local"


def test_every_setting_is_a_ros_parameter_and_cruise_height_can_be_changed(build_node):
    node = build_node(cruise_height_metres=0.6, px4_local_position_topic="/fmu/out/vehicle_local_position_v1")
    assert {setting.name for setting in fields(MissionSettings)} <= set(node.parameters)
    assert node.mission.settings.cruise_height_metres == 0.6
    assert "/fmu/out/vehicle_local_position_v1" in node.subscriptions


def test_the_px4_topic_defaults_to_the_name_measured_on_drone_16(build_node):
    assert build_node().parameters["px4_local_position_topic"] == "/fmu/out/vehicle_local_position"


def test_an_empty_px4_topic_asks_the_organizers_code_for_the_name(build_node):
    organizers_topics = types.ModuleType("skybeta_flight_interface.topics")
    organizers_topics.fmu_topic = lambda direction, name: f"/fmu/{direction}/{name}_v7"
    sys.modules["skybeta_flight_interface"] = types.ModuleType("skybeta_flight_interface")
    sys.modules["skybeta_flight_interface.topics"] = organizers_topics
    try:
        assert "/fmu/out/vehicle_local_position_v7" in build_node(px4_local_position_topic="").subscriptions
    finally:
        sys.modules.pop("skybeta_flight_interface", None)
        sys.modules.pop("skybeta_flight_interface.topics", None)


def test_an_empty_px4_topic_without_the_organizers_code_uses_the_plain_name(build_node):
    assert PX4_LOCAL_POSITION_TOPIC in build_node(px4_local_position_topic="").subscriptions


def test_launch_values_are_converted_to_each_settings_type(build_node):
    node = build_node(cruise_height_metres=1, tunnel_enabled="true", sightings_to_confirm=4.0)
    assert node.mission.settings.cruise_height_metres == 1.0
    assert isinstance(node.mission.settings.cruise_height_metres, float)
    assert node.mission.settings.tunnel_enabled is True and node.mission.settings.sightings_to_confirm == 4
    with pytest.raises(ValueError, match="cruise_height_metres is 'high'"):
        build_node(cruise_height_metres="high")


def test_the_node_caps_the_camera_exposure_on_the_camera_named_by_a_parameter(build_node):
    node = build_node(camera_node_name="/front_camera")
    assert set(node.service_clients) == {"/front_camera/set_parameters", "/front_camera/get_parameters"}
    assert (MissionSettings().camera_check_seconds, node.camera_exposure.check) in node.timers
    assert build_node(cap_camera_exposure=False).camera_exposure is None


def test_the_vision_check_is_on_by_default(build_node):
    assert build_node().mission.vision_check is True


def test_vision_check_false_is_refused_outside_a_simulator_run(build_node, monkeypatch, tmp_path):
    monkeypatch.delenv("TEAM_NAV_STARTED_BY", raising=False)
    node = build_node(vision_check=False)
    assert node.mission.vision_check is True
    assert any(level == "error" and "only for the simulator" in text for level, text in node.log_lines)
    monkeypatch.setenv("TEAM_NAV_STARTED_BY", "simulator/run_team_nav.sh")   # the variable alone is not enough
    monkeypatch.setattr(sys.modules["team_nav.mission_node"], "SIMULATOR_FOLDER", str(tmp_path / "not_here"))
    assert build_node(vision_check=False).mission.vision_check is True


def test_vision_check_false_is_accepted_in_a_simulator_run(build_node, monkeypatch, tmp_path):
    monkeypatch.setenv("TEAM_NAV_STARTED_BY", "simulator/run_team_nav.sh")
    monkeypatch.setattr(sys.modules["team_nav.mission_node"], "SIMULATOR_FOLDER", str(tmp_path))
    node = build_node(vision_check="false")
    assert node.mission.vision_check is False
    assert any(level == "warn" and "SIMULATOR ONLY" in text for level, text in node.log_lines)


def test_px4s_status_is_heard_on_its_versioned_topic(build_node):
    node = build_node()
    assert node.parameters["px4_status_topic"] == "/fmu/out/vehicle_status_v1"
    deliver(node, PX4_STATUS_TOPIC, fake_ros.VehicleStatus(14))
    assert node.mission.nav_state == 14


def test_camera_trackings_own_status_is_logged_on_change_and_never_lands_the_drone(build_node):
    node = build_node()
    for vo_state in (1, 1, 2, 2, 1):
        deliver(node, VISUAL_SLAM_STATUS_TOPIC, fake_ros.VisualSlamStatus(vo_state))
    status_lines = [text for _, text in node.log_lines if text.startswith("camera tracking status")]
    assert [line.split(" (")[0] for line in status_lines] == [
        "camera tracking status: vo_state 1", "camera tracking status: vo_state 2", "camera tracking status: vo_state 1"]
    assert all("never makes the drone land" in line for line in status_lines)
    assert not any("vo_state" in reason for _, _, reason in node.mission.history)   # the mission never sees it


def test_the_node_starts_without_the_visual_slam_package(build_node):
    sys.modules.pop("isaac_ros_visual_slam_interfaces", None)
    sys.modules.pop("isaac_ros_visual_slam_interfaces.msg", None)
    sys.modules.pop("team_nav.mission_node", None)
    node_module = importlib.import_module("team_nav.mission_node")
    fake_ros.Node.parameter_overrides = {}
    node = node_module.MissionNode()
    assert VISUAL_SLAM_STATUS_TOPIC not in node.subscriptions
    assert any(level == "warn" and "isaac_ros_visual_slam_interfaces is not installed" in text
               for level, text in node.log_lines)


def test_the_route_mission_reads_its_waypoints_from_the_route_file(build_node):
    route_file = Path(__file__).resolve().parents[1] / "missions" / "route_waypoints.json"
    node = build_node(mission_kind="route", route_file=str(route_file))
    assert node.mission.mission_kind == "route" and len(node.mission.route_waypoints) == 20
    assert node.mission.route_waypoints[0] == (1.513, -0.141)


@pytest.mark.parametrize("route_file, count", [("route_waypoints.json", 20), ("route_waypoints_drawing.json", 22),
                                               ("route_waypoints_scan.json", 25)])
def test_a_bare_route_file_name_means_that_file_in_the_packages_missions_folder(build_node, route_file, count):
    node = build_node(mission_kind="route", route_file=route_file)
    assert len(node.mission.route_waypoints) == count
    assert node.parameters["route_file"] == route_file
    with_height = sum(len(waypoint) == 3 for waypoint in node.mission.route_waypoints)
    assert with_height == (0 if route_file == "route_waypoints.json" else 2)   # the window legs' heights


def test_a_waypoint_with_the_wrong_count_of_numbers_in_the_route_file_is_a_clear_error(build_node, tmp_path):
    route_file = tmp_path / "route.json"
    route_file.write_text(json.dumps({"waypoints_forward_left_metres": [[1.0, 0.0], [1.0]]}))
    with pytest.raises(ValueError, match=r"Waypoint 2 in .*route.json is \[1.0\]"):
        build_node(mission_kind="route", route_file=str(route_file))


DEPTH_TOPIC = "/some/depth_points"


def depth_cloud(points):
    """A PointCloud2 of float32 x, y, z points, packed as the pretend depth camera packs them."""
    message = fake_ros.PointCloud2()
    message.fields = [fake_ros.PointField(name, offset) for name, offset in (("x", 0), ("y", 4), ("z", 8))]
    message.point_step = 12
    message.data = b"".join(struct.pack("<fff", *point) for point in points)
    return message


def build_drift_node(build_node, **parameters):
    maps = str(Path(__file__).resolve().parents[1] / "course_maps" / "organizers_drawing.json")
    return build_node(mission_kind="route", route_file=str(Path(__file__).resolve().parents[1] / "missions" /
                                                           "route_waypoints.json"),
                      drift_correction=True, course_map_file=maps, **parameters)


def test_without_drift_correction_no_depth_is_heard(build_node):
    node = build_node(depth_topic=DEPTH_TOPIC)
    assert DEPTH_TOPIC not in node.subscriptions
    assert node.mission.map_ranging is None


def test_with_drift_correction_the_depth_straight_ahead_reaches_the_mission(build_node):
    node = build_drift_node(build_node, depth_topic=DEPTH_TOPIC)
    assert node.subscriptions[DEPTH_TOPIC][0] is fake_ros.PointCloud2
    assert node.mission.map_ranging.course_map.launch_mark_centre_metres is not None
    fake_ros.Clock.seconds = 4.0
    deliver(node, DEPTH_TOPIC, depth_cloud([(1.2, 0.0, 0.0)] * 6 + [(0.5, 0.5, 0.0)] * 20))
    assert node.mission.depth_ahead_metres == pytest.approx(1.2)
    assert node.mission.depth_ahead_seconds == 4.0
    deliver(node, DEPTH_TOPIC, depth_cloud([(1.2, 0.0, 0.0)] * 2))           # too few straight ahead
    assert node.mission.depth_ahead_metres is None


def test_drift_correction_listens_to_our_depth_points_by_default(build_node):
    node = build_drift_node(build_node)
    assert node.subscriptions["/team/depth_points"][0] is fake_ros.PointCloud2


def test_drift_correction_with_an_empty_depth_topic_warns(build_node):
    node = build_drift_node(build_node, depth_topic="")
    assert any(level == "warn" and "depth_topic is empty" in text for level, text in node.log_lines)


def test_depth_points_without_float_xyz_are_ignored_with_one_warning(build_node):
    node = build_drift_node(build_node, depth_topic=DEPTH_TOPIC)
    message = depth_cloud([(1.2, 0.0, 0.0)] * 6)
    message.fields = message.fields[:2]
    deliver(node, DEPTH_TOPIC, message)
    deliver(node, DEPTH_TOPIC, message)
    assert node.mission.depth_ahead_seconds is None
    assert sum("no float32 x, y and z" in text for _, text in node.log_lines) == 1


def test_a_course_map_that_cannot_be_read_is_a_clear_error(build_node):
    with pytest.raises(ValueError, match="Could not read the course map /no/such/map.json"):
        build_node(mission_kind="route", drift_correction=True, course_map_file="/no/such/map.json",
                   route_file=str(Path(__file__).resolve().parents[1] / "missions" / "route_waypoints.json"))


def test_a_route_file_that_cannot_be_read_is_a_clear_error(build_node):
    with pytest.raises(ValueError, match="Could not read the route's waypoints from /no/such/route.json"):
        build_node(mission_kind="route", route_file="/no/such/route.json")


def test_the_laser_distance_is_read_even_though_px4_marks_it_invalid(build_node):
    node = build_node()
    report = fake_ros.VehicleLocalPosition()
    report.dist_bottom, report.dist_bottom_valid = 0.12, False
    deliver(node, PX4_LOCAL_POSITION_TOPIC, report)
    assert node.mission.laser_metres == 0.12


def test_a_flight_state_that_is_not_json_is_ignored_with_a_warning(build_node):
    node = build_node()
    deliver(node, FLIGHT_STATE_TOPIC, fake_ros.String("not json"))
    assert node.mission.flight_state_seconds is None
    assert ("warn", "The flight state on /skybeta/state was not valid JSON, so it was ignored.") in node.log_lines


# ---------------------------------------------------------------- flights through the topics
def test_a_whole_flight_through_the_topics(build_node):
    node, drone = build_node(), FakeDrone()
    sent = fly_node(node, drone, 40.0)
    before_start = [message for seconds, topic, message in sent if seconds < 1.0]
    assert before_start and all(message.pose.position.z == pytest.approx(0.3) for message in before_start)
    cruise = [message.pose.position.z for _, topic, message in sent if topic == POSE_TOPIC]
    assert max(cruise) == pytest.approx(0.8)
    assert all(message.header.frame_id == "odom" for _, _, message in sent)
    twists = [message.twist for _, topic, message in sent if topic == TWIST_TOPIC]
    assert {round(-twist.linear.z, 2) for twist in twists} == {0.5, 0.35, 1.0}
    assert all(twist.angular.z == 0.0 for twist in twists)              # never turning while landing
    assert not drone.armed and drone.landed
    motors_off_at = next(seconds for seconds, phase, _ in node.mission.history if phase == "finished")
    assert all(seconds < motors_off_at for seconds, _, _ in sent)       # nothing published once the motors are off
    assert drone.offboard                                               # PX4 still shows Offboard while disarmed
    for line in ("waiting_for_start: ready: waiting for Offboard (the pilot's switch to Position after arming)",
                 "flying_course: follow_line: camera tracking missing, holding",
                 "landing: touchdown confirmed: pushing down"):
        assert line in logged(node), line
    assert any(line.startswith("finished: landed: PX4 switched the motors off (peak horizontal speed")
               for line in logged(node))


def test_the_landing_sends_a_pose_at_the_drone_then_twists_at_20_hz_until_the_motors_are_off(build_node):
    node, drone = build_node(), FakeDrone()
    positions_when_sent = []

    def remember_position(now_seconds, node, drone):
        positions_when_sent.append((now_seconds, tuple(drone.position)))
    sent = fly_node(node, drone, 40.0, every_step=remember_position)
    first_twist = next(index for index, (_, topic, _) in enumerate(sent) if topic == TWIST_TOPIC)
    seconds, topic, pose = sent[first_twist - 1]
    assert topic == POSE_TOPIC and seconds == sent[first_twist][0]        # same step, just before
    drone_then = dict(positions_when_sent)[seconds]
    position = pose.pose.position
    assert math.dist((position.x, position.y, position.z), drone_then) < 1e-5   # tracking wobbles by 1e-6 m
    twist_times = [seconds for seconds, topic, _ in sent[first_twist:] if topic == TWIST_TOPIC]
    assert all(topic == TWIST_TOPIC for _, topic, _ in sent[first_twist:])
    gaps = [later - earlier for earlier, later in zip(twist_times, twist_times[1:])]
    assert max(gaps) == pytest.approx(STEP_SECONDS)                     # one every 0.05 s, no gaps
    motors_off_at = next(seconds for seconds, phase, _ in node.mission.history if phase == "finished")
    assert twist_times[-1] == pytest.approx(motors_off_at - STEP_SECONDS)  # right up to the motors stopping


def test_the_flight_states_px4_link_reaches_the_mission(build_node):
    node = build_node()
    deliver(node, FLIGHT_STATE_TOPIC, fake_ros.String(json.dumps({"px4_link": False, "armed": False})))
    assert node.mission.px4_link is False


def test_publishing_stops_when_offboard_ends(build_node):
    def pilot_takes_over(now_seconds, node, drone):
        if now_seconds >= 5.0:
            drone.offboard = False
    node, drone = build_node(), FakeDrone()
    sent = fly_node(node, drone, 10.0, every_step=pilot_takes_over)
    assert sent and max(seconds for seconds, _, _ in sent) < 5.0
    assert any("PX4 left Offboard" in line for line in logged(node))


# ---------------------------------------------------------------- before the start, on drone 16
def test_a_still_drone_gets_a_hold_target_10_times_a_second_from_the_20_hz_node(build_node):
    """Drone 16 on 2026-11-07: standing still, the node flapped to "not ready" and sent nothing."""
    node = build_node()
    drone = FakeDrone()
    drone.still_reports_identical = True
    sent = fly_node(node, drone, 20.0, arm_at_seconds=1e9, switch_at_seconds=1e9)
    poses = [seconds for seconds, topic, _ in sent if topic == POSE_TOPIC]
    assert len(poses) == pytest.approx(20.0 * 10, abs=2)
    assert max(later - earlier for earlier, later in zip(poses, poses[1:])) <= 0.1 + 1e-6
    assert not any(text.startswith("not ready") for text in logged(node))


def test_the_corner_count_reaches_the_pre_start_check(build_node):
    node = build_node()
    fake_ros.Clock.seconds = 2.0
    deliver(node, CORNER_COUNT_TOPIC, fake_ros.Int32(123))
    assert node.mission.prestart.corner_count == 123
    assert node.mission.prestart.corner_count_seconds == 2.0


# ---------------------------------------------------------------- the route's course map
@pytest.mark.parametrize("route_file, map_file", [("route_waypoints.json", "organizers_drawing.json"),
                                                  ("route_waypoints_drawing.json", "organizers_drawing.json"),
                                                  ("route_waypoints_scan.json", "phone_scan_2026-11-06.json")])
def test_a_route_mission_loads_the_map_its_route_was_planned_on(build_node, route_file, map_file):
    node = build_node(mission_kind="route", route_file=route_file)
    assert node.mission.map_ranging is not None
    assert any(text.endswith(f"on the course map {Path(__file__).resolve().parents[1] / 'course_maps' / map_file}.")
               for text in logged(node))


def test_the_course_map_can_be_chosen_and_the_laser_rule_uses_it(build_node):
    node = build_node(mission_kind="route", route_file="route_waypoints_drawing.json")
    assert sorted(node.mission.route_must_fly_through) == [17, 18, 19, 20]   # waypoints 18 to 21
    node = build_node(mission_kind="route", route_file="route_waypoints.json")
    assert node.mission.route_must_fly_through == {}                       # Gate B: no window, no boulder


def test_a_stages_mission_loads_no_map(build_node):
    assert build_node().mission.map_ranging is None
