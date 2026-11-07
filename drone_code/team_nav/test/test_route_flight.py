"""The route mission (route_flight.py, through mission.py) against the pretend drone: it flies every
waypoint in order at cruise height, stops and turns in place at corners that turn by 25 deg or more,
flies through the others with the heading blending round, never turns faster than 25 deg/s nor flies
a leg faster than 0.5 m/s, lands at the end, and the watchdog stays active throughout. A waypoint
with a height has the leg into it flown at that height, changed while stopped before the leg."""
import json
import math
from pathlib import Path

import pytest

from fake_drone import FakeDrone, fly
from team_nav.course_stages import body_to_world, wrap_angle
from team_nav.mission import Mission
from team_nav.mission_settings import MissionSettings

ROUTE_FILE = Path(__file__).resolve().parents[1] / "missions" / "route_waypoints.json"
SETTINGS = MissionSettings()
# The team's route, as given on 2026-11-07 (it keeps 0.40 m from walls except at Gate B):
# (forward, left) in metres from the takeoff spot.
TEAM_ROUTE = [(1.513, -0.141), (1.663, -0.591), (1.513, -1.042), (0.163, -1.292), (0.063, -1.942),
              (0.463, -2.242), (1.563, -2.442), (1.663, -3.091), (0.413, -4.742), (-1.737, -4.492),
              (-2.537, -3.641), (-2.337, -2.992), (-1.837, -3.042), (-1.487, -3.142), (-1.137, -3.091),
              (-0.987, -2.642), (-1.187, -2.142), (-1.937, -1.892), (-1.937, -0.992), (-1.687, -0.492)]
LAST = len(TEAM_ROUTE)


def turn_at_each_waypoint_degrees():
    """How much the route turns at each waypoint but the last, worked out here from the list."""
    points = [(0.0, 0.0)] + TEAM_ROUTE
    headings = [math.atan2(b[1] - a[1], b[0] - a[0]) for a, b in zip(points, points[1:])]
    return [abs(math.degrees(wrap_angle(after - before))) for before, after in zip(headings, headings[1:])]


FLY_THROUGH = [index + 1 for index, turn in enumerate(turn_at_each_waypoint_degrees())
               if turn < SETTINGS.route_fly_through_degrees]
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
    reached = [reason.removesuffix(" (flying through)") for reason in reasons(mission)
               if reason.startswith("route: reached waypoint")]
    assert reached == [f"route: reached waypoint {index} of {LAST}" for index in range(1, LAST + 1)]
    assert f"route: route complete: reached waypoint {LAST} of {LAST}" in reasons(mission)
    assert any(reason.startswith("landed: PX4 switched the motors off") for reason in reasons(mission))
    for number, (forward, left) in enumerate(TEAM_ROUTE, start=1):           # it really went to each one
        target = body_to_world((0.0, 0.0), 0.0, forward, left)
        closest = min(math.hypot(step.drone_position[0] - target[0], step.drone_position[1] - target[1])
                      for step in record)
        allowed = 0.15 if number in FLY_THROUGH else SETTINGS.route_arrive_metres + 0.01   # cuts a shallow corner
        assert closest < allowed, (number, closest)
    first_reached = next(step.seconds for step in record if "reached waypoint 1 of" in step.command.reason)
    flying = [step.drone_position[2] for step in route_steps(record, "flying to") if step.seconds > first_reached]
    assert min(flying) == pytest.approx(SETTINGS.cruise_height_metres, abs=0.05)   # (the first leg ends the climb)


def test_it_turns_in_place_no_faster_than_25_degrees_per_second():
    mission, drone, record = fly_route()
    turning = route_steps(record, "turning to face")
    assert turning
    for earlier, later in zip(turning, turning[1:]):
        if abs(later.seconds - earlier.seconds - STEP_SECONDS) > 1e-6:
            continue                                                       # a different corner
        turned = abs(wrap_angle(later.command.heading_radians - earlier.command.heading_radians))
        assert math.degrees(turned) / STEP_SECONDS <= SETTINGS.route_turn_rate_degrees_per_second + 1e-6
        assert later.command.position[:2] == earlier.command.position[:2]  # in place: the target stays put


def test_it_flies_each_leg_no_faster_than_0_5_metres_per_second_and_blends_the_heading_no_faster_than_25():
    mission, drone, record = fly_route()
    flying = route_steps(record, "flying to")
    for earlier, later in zip(flying, flying[1:]):
        if abs(later.seconds - earlier.seconds - STEP_SECONDS) > 1e-6:
            continue                                                       # a stop and turn in between
        carrot_moved = math.dist(later.command.position[:2], earlier.command.position[:2])
        assert carrot_moved <= SETTINGS.route_speed_metres_per_second * STEP_SECONDS + 1e-6
        turned = abs(wrap_angle(later.command.heading_radians - earlier.command.heading_radians))
        assert math.degrees(turned) / STEP_SECONDS <= SETTINGS.route_turn_rate_degrees_per_second + 1e-6
    assert mission.position_watch.peak_horizontal_speed_metres_per_second < SETTINGS.route_speed_metres_per_second + 0.05


def test_it_stops_before_turning_at_the_takeoff_spot_and_every_corner_of_25_degrees_or_more():
    mission, drone, record = fly_route()
    first_turning_steps = []
    for earlier, later in zip(record, record[1:]):
        if "turning to face" in later.command.reason and "turning to face" not in earlier.command.reason:
            first_turning_steps.append((earlier, later))
    big_corners = LAST - 1 - len(FLY_THROUGH)
    assert FLY_THROUGH == [13, 14]                                         # turns of 10 and 24 degrees
    assert len(first_turning_steps) == 1 + big_corners                     # plus the takeoff spot
    for earlier, later in first_turning_steps:
        speed = math.dist(later.drone_position[:2], earlier.drone_position[:2]) / STEP_SECONDS
        assert speed < SETTINGS.settled_speed_metres_per_second + 0.02


def test_it_flies_through_waypoints_where_the_route_turns_less_than_25_degrees():
    mission, drone, record = fly_route()
    for number in FLY_THROUGH:
        name = f"waypoint {number} of {LAST}"
        assert f"route: reached {name} (flying through)" in reasons(mission)
        assert f"route: stopping at {name}" not in reasons(mission)
        assert f"route: turning to face waypoint {number + 1} of {LAST}" not in reasons(mission)
        passing = next(index for index, step in enumerate(record) if step.command.reason == f"route: reached {name} (flying through)")
        speed = math.dist(record[passing + 1].drone_position[:2], record[passing].drone_position[:2]) / STEP_SECONDS
        assert speed > 0.15, (number, speed)                               # still moving (stopped is < 0.1 m/s)


def test_the_waypoints_are_measured_from_the_nose_direction_at_takeoff():
    mission, drone, record = fly_route(route=[(1.0, -0.5)], heading_radians=math.pi / 2)   # nose North
    assert "route: route complete: reached waypoint 1 of 1" in reasons(mission)
    target_east, target_north = 0.5, 1.0                                    # 1 m forward = North, 0.5 m right = East
    closest = min(math.hypot(step.drone_position[0] - target_east, step.drone_position[1] - target_north)
                  for step in record)
    assert closest < SETTINGS.route_arrive_metres + 0.01


def test_the_watchdog_stays_active_during_the_route():
    def jump_during_the_third_leg(now_seconds, mission, drone):
        if mission.phase == "flying_route" and "flying to waypoint 3 of" in mission.reason and not jump.done:
            drone.position[0] += 0.6
            jump.done = True

    class Jump:
        done = False
    jump = Jump()
    mission, drone, record = fly_route(every_step=jump_during_the_third_leg)
    tripped = next(reason for reason in reasons(mission) if reason.startswith("watchdog rule (b) tripped"))
    assert tripped.endswith("landing now")
    assert f"route: reached waypoint 3 of {LAST}" not in reasons(mission)
    assert any(reason.startswith("landed: PX4 switched the motors off") for reason in reasons(mission))


@pytest.mark.parametrize("name, count", [("route_waypoints_drawing.json", 22), ("route_waypoints_scan.json", 25)])
def test_the_planned_routes_fly_to_the_end_and_land(name, count):
    with open(ROUTE_FILE.parent / name, encoding="utf-8") as opened:
        document = json.load(opened)
    route = [tuple(waypoint) for waypoint in document["waypoints_forward_left_metres"]]
    assert len(route) == count == len(document["planned_heights_metres"])
    assert route[-1][:2] == (-1.707, -0.472)                               # both end over the landing pad
    mission, drone, record = fly_route(route=route)
    assert f"route: route complete: reached waypoint {count} of {count}" in reasons(mission)
    assert any(reason.startswith("landed: PX4 switched the motors off") for reason in reasons(mission))


def test_a_route_mission_without_waypoints_is_refused():
    with pytest.raises(ValueError, match="needs waypoints"):
        Mission(mission_kind="route", route_waypoints=[])
    with pytest.raises(ValueError, match="must be one of stages, route"):
        Mission(mission_kind="square")


# ---------------------------------------------------------------------- heights on waypoints
HEIGHT_ROUTE = [(1.0, 0.0), (1.5, 0.0, 0.5), (2.0, 0.0, 0.5), (2.5, 0.0), (2.5, -1.0)]


def height_above_floor(step):
    return step.drone_position[2]                                          # the pretend floor is at 0


def test_each_leg_is_flown_at_its_waypoints_height_or_cruise():
    mission, drone, record = fly_route(route=HEIGHT_ROUTE)
    assert "route: route complete: reached waypoint 5 of 5" in reasons(mission)
    wanted = {1: 0.8, 2: 0.5, 3: 0.5, 4: 0.8, 5: 0.8}
    for number, height in wanted.items():
        flying = route_steps(record, f"flying to waypoint {number} of 5")
        assert flying, number
        for step in flying:
            assert step.command.position[2] == pytest.approx(height), number      # the target never changes
        for step in flying[5:]:
            assert height_above_floor(step) == pytest.approx(height, abs=0.06), number


def test_height_changes_only_while_stopped_before_the_leg_never_faster_than_the_climb_speed():
    mission, drone, record = fly_route(route=HEIGHT_ROUTE)
    changing = route_steps(record, "changing height")
    assert [reason for reason in reasons(mission) if "changing height" in reason] == [
        "route: changing height to 0.50 m before waypoint 2 of 5",
        "route: changing height to 0.80 m before waypoint 4 of 5"]
    for step in changing:
        assert step.command.position[:2] == changing[0].command.position[:2] or step.command.position[:2] == \
            changing[-1].command.position[:2]                              # in place, at one of the two corners
    for earlier, later in zip(changing, changing[1:]):
        if abs(later.seconds - earlier.seconds - STEP_SECONDS) > 1e-6:
            continue
        moved = abs(later.command.position[2] - earlier.command.position[2])
        assert moved <= SETTINGS.climb_speed_metres_per_second * STEP_SECONDS + 1e-9
        speed = math.dist(later.drone_position[:2], earlier.drone_position[:2]) / STEP_SECONDS
        assert speed < SETTINGS.settled_speed_metres_per_second + 0.02


def test_a_waypoint_where_the_height_changes_is_never_flown_through():
    mission, drone, record = fly_route(route=HEIGHT_ROUTE)                 # straight on: would fly through
    assert "route: reached waypoint 1 of 5" in reasons(mission)            # stops: 0.8 m -> 0.5 m
    assert "route: reached waypoint 2 of 5 (flying through)" in reasons(mission)   # same height: flies through
    assert "route: reached waypoint 3 of 5" in reasons(mission)            # stops: 0.5 m -> 0.8 m


def test_the_planned_routes_carry_the_planners_window_crossing_heights():
    for name, numbers, height in [("route_waypoints_drawing.json", (19, 20), 0.76),
                                  ("route_waypoints_scan.json", (22, 23), 1.19)]:
        with open(ROUTE_FILE.parent / name, encoding="utf-8") as opened:
            route = json.load(opened)["waypoints_forward_left_metres"]
        with_height = [number for number, waypoint in enumerate(route, start=1) if len(waypoint) == 3]
        assert with_height == list(numbers), name
        assert all(route[number - 1][2] == height for number in numbers), name


def test_the_scan_route_crosses_the_window_at_1_19_metres():
    with open(ROUTE_FILE.parent / "route_waypoints_scan.json", encoding="utf-8") as opened:
        route = [tuple(waypoint) for waypoint in json.load(opened)["waypoints_forward_left_metres"]]
    mission, drone, record = fly_route(route=route, seconds=300.0)
    crossing = route_steps(record, "flying to waypoint 23 of 25")
    assert crossing and all(step.command.position[2] == pytest.approx(1.19) for step in crossing)
    assert all(step.command.position[2] == pytest.approx(SETTINGS.cruise_height_metres)
               for step in route_steps(record, "flying to waypoint 24 of 25"))


def test_a_waypoint_with_the_wrong_count_of_numbers_is_refused():
    with pytest.raises(ValueError, match="Waypoint 2 of the route has 4 numbers"):
        Mission(mission_kind="route", route_waypoints=[(1.0, 0.0), (1.0, 1.0, 0.5, 9.0)])
