"""The route mission (route_flight.py, through mission.py) against the pretend drone: it flies every
waypoint in order at cruise height, stops at each corner, turns in place no faster than 15 deg/s,
flies each leg no faster than 0.4 m/s, lands at the end, and the watchdog stays active throughout."""
import json
import math
from pathlib import Path

import pytest

from fake_drone import FakeDrone, fly
from team_nav.course_stages import body_to_world, wrap_angle
from team_nav.mission import Mission
from team_nav.mission_settings import MissionSettings

ROUTE_FILE = Path(__file__).resolve().parents[1] / "missions" / "route_waypoints.json"
# The team's route, as given on 2026-11-07: (forward, left) in metres from the takeoff spot.
TEAM_ROUTE = [(1.613, -0.241), (1.513, -1.042), (0.113, -1.341), (0.213, -2.142), (1.613, -2.542),
              (1.513, -3.392), (0.263, -4.792), (-1.837, -4.442), (-2.537, -3.442), (-2.087, -2.992),
              (-1.187, -3.142), (-0.987, -2.341), (-2.137, -1.642), (-1.687, -0.492)]
SETTINGS = MissionSettings()
STEP_SECONDS = 0.1


def route_from_file():
    with open(ROUTE_FILE, encoding="utf-8") as opened:
        return [tuple(waypoint) for waypoint in json.load(opened)["waypoints_forward_left_metres"]]


def fly_route(route=None, heading_radians=0.0, seconds=240.0, every_step=None):
    mission = Mission(mission_kind="route", route_waypoints=route or route_from_file())
    drone = FakeDrone(heading_radians=heading_radians)
    record = fly(mission, drone, seconds, every_step=every_step)
    return mission, drone, record


def reasons(mission):
    return [reason for _, _, reason in mission.history]


def route_steps(record, text):
    return [step for step in record if step.phase == "flying_route" and text in step.command.reason]


def test_the_route_file_holds_the_teams_waypoints():
    assert route_from_file() == TEAM_ROUTE


def test_it_flies_every_waypoint_in_order_at_cruise_height_and_then_lands():
    mission, drone, record = fly_route()
    reached = [reason for reason in reasons(mission) if reason.startswith("route: reached waypoint")]
    assert reached == [f"route: reached waypoint {index} of 14" for index in range(1, 15)]
    assert "route: route complete: reached waypoint 14 of 14" in reasons(mission)
    assert any(reason.startswith("landed: PX4 switched the motors off") for reason in reasons(mission))
    for forward, left in TEAM_ROUTE:                                       # it really went to each one
        target = body_to_world((0.0, 0.0), 0.0, forward, left)
        closest = min(math.hypot(step.drone_position[0] - target[0], step.drone_position[1] - target[1])
                      for step in record)
        assert closest < SETTINGS.route_arrive_metres + 0.01
    flying = [step.drone_position[2] for step in route_steps(record, "flying to")]
    assert min(flying) == pytest.approx(SETTINGS.cruise_height_metres, abs=0.05)


def test_it_turns_in_place_no_faster_than_15_degrees_per_second():
    mission, drone, record = fly_route()
    turning = route_steps(record, "turning to face")
    assert turning
    for earlier, later in zip(turning, turning[1:]):
        if abs(later.seconds - earlier.seconds - STEP_SECONDS) > 1e-6:
            continue                                                       # a different corner
        turned = abs(wrap_angle(later.command.heading_radians - earlier.command.heading_radians))
        assert math.degrees(turned) / STEP_SECONDS <= SETTINGS.route_turn_rate_degrees_per_second + 1e-6
        assert later.command.position[:2] == earlier.command.position[:2]  # in place: the target stays put


def test_it_flies_each_leg_no_faster_than_0_4_metres_per_second():
    mission, drone, record = fly_route()
    flying = route_steps(record, "flying to")
    for earlier, later in zip(flying, flying[1:]):
        carrot_moved = math.dist(later.command.position[:2], earlier.command.position[:2])
        assert carrot_moved <= SETTINGS.route_speed_metres_per_second * STEP_SECONDS + 1e-6
    assert mission.position_watch.peak_horizontal_speed_metres_per_second < SETTINGS.route_speed_metres_per_second + 0.05


def test_it_stops_at_each_corner_before_turning():
    mission, drone, record = fly_route()
    first_turning_steps = []
    for earlier, later in zip(record, record[1:]):
        if "turning to face" in later.command.reason and "turning to face" not in earlier.command.reason:
            first_turning_steps.append((earlier, later))
    assert len(first_turning_steps) == 14
    for earlier, later in first_turning_steps:
        speed = math.dist(later.drone_position[:2], earlier.drone_position[:2]) / STEP_SECONDS
        assert speed < SETTINGS.settled_speed_metres_per_second + 0.02


def test_the_waypoints_are_measured_from_the_nose_direction_at_takeoff():
    mission, drone, record = fly_route(route=[(1.0, -0.5)], heading_radians=math.pi / 2)   # nose North
    assert "route: route complete: reached waypoint 1 of 1" in reasons(mission)
    target_east, target_north = 0.5, 1.0                                    # 1 m forward = North, 0.5 m right = East
    closest = min(math.hypot(step.drone_position[0] - target_east, step.drone_position[1] - target_north)
                  for step in record)
    assert closest < SETTINGS.route_arrive_metres + 0.01


def test_the_watchdog_stays_active_during_the_route():
    def jump_during_the_third_leg(now_seconds, mission, drone):
        if mission.phase == "flying_route" and "flying to waypoint 3" in mission.reason and not jump.done:
            drone.position[0] += 0.6
            jump.done = True

    class Jump:
        done = False
    jump = Jump()
    mission, drone, record = fly_route(every_step=jump_during_the_third_leg)
    tripped = next(reason for reason in reasons(mission) if reason.startswith("watchdog rule (b) tripped"))
    assert tripped.endswith("landing now")
    assert "route: reached waypoint 3 of 14" not in reasons(mission)
    assert any(reason.startswith("landed: PX4 switched the motors off") for reason in reasons(mission))


def test_a_route_mission_without_waypoints_is_refused():
    with pytest.raises(ValueError, match="needs waypoints"):
        Mission(mission_kind="route", route_waypoints=[])
    with pytest.raises(ValueError, match="must be one of stages, route"):
        Mission(mission_kind="square")
