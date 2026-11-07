"""mission_node.py, the ROS 2 program, run on stand-in ROS (fake_ros.py) against the pretend drone
(fake_drone.py): which topics it hears and sends on, and a whole flight through its callbacks."""
import importlib
import json
import math
import sys
import types
from dataclasses import fields

import pytest

import fake_ros
from fake_drone import FakeDrone
from team_nav.mission_settings import MissionSettings

POSITION_TOPIC = "/visual_slam/tracking/odometry"
FLIGHT_STATE_TOPIC = "/skybeta/state"
PX4_LOCAL_POSITION_TOPIC = "/fmu/out/vehicle_local_position"
POSE_TOPIC = "/skybeta/setpoint/pose"
TWIST_TOPIC = "/skybeta/setpoint/twist"
STEP_SECONDS = 0.1


@pytest.fixture
def build_node():
    fake_ros.install()
    sys.modules.pop("team_nav.mission_node", None)
    node_module = importlib.import_module("team_nav.mission_node")

    def build(**parameters):
        fake_ros.Node.parameter_overrides = parameters
        fake_ros.Clock.seconds = 0.0
        return node_module.MissionNode()
    yield build
    fake_ros.Node.parameter_overrides = {}
    sys.modules.pop("team_nav.mission_node", None)
    fake_ros.uninstall()


def deliver(node, topic, message):
    node.subscriptions[topic][1](message)


def deliver_reports(node, drone):
    odometry = fake_ros.Odometry()
    position = odometry.pose.pose.position
    position.x, position.y, position.z = drone.position
    odometry.pose.pose.orientation = fake_ros.Quaternion(z=math.sin(drone.heading_radians / 2),
                                                         w=math.cos(drone.heading_radians / 2))
    deliver(node, POSITION_TOPIC, odometry)
    offboard, armed, landed = drone.flight_state()
    state = {"offboard_active": offboard, "armed": armed, "landed": landed, "nav_state_name": "whatever"}
    deliver(node, FLIGHT_STATE_TOPIC, fake_ros.String(json.dumps(state)))
    local_position = fake_ros.VehicleLocalPosition()
    local_position.dist_bottom = drone.laser_reading()
    deliver(node, PX4_LOCAL_POSITION_TOPIC, local_position)


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
def test_hears_three_topics_and_sends_only_on_the_two_setpoint_topics(build_node):
    node = build_node()
    assert set(node.subscriptions) == {POSITION_TOPIC, FLIGHT_STATE_TOPIC, PX4_LOCAL_POSITION_TOPIC}
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
    for line in ("waiting_for_start: ready: waiting for Offboard (the pilot's switch to Position after arming)",
                 "flying_course: follow_line: camera tracking missing, holding",
                 "landing: touchdown confirmed: pushing down", "finished: landed: PX4 switched the motors off"):
        assert line in logged(node), line


def test_publishing_stops_when_offboard_ends(build_node):
    def pilot_takes_over(now_seconds, node, drone):
        if now_seconds >= 5.0:
            drone.offboard = False
    node, drone = build_node(), FakeDrone()
    sent = fly_node(node, drone, 10.0, every_step=pilot_takes_over)
    assert sent and max(seconds for seconds, _, _ in sent) < 5.0
    assert any("PX4 left Offboard" in line for line in logged(node))
