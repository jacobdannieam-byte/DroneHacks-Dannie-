"""Tests the fallback missions without flying: fallback_routes.py's two routes on our three maps and
on changed courses, high_path_rules.py's laser rules on made-up paths, the follower and safety
settings they use, and how fallback_run.py plugs into course_run.

How to run these tests is in the package's README.
"""
import types

import numpy as np
import pytest

from our_files import ALL_OUR_MAPS, DRAWING_MAP, SCAN_MAP, load_our_map, load_side_gap_first_mission
from course_changes import with_wall_moved, without_wall
from team_nav.fallback_routes import (
    ARC_PIECE_METRES, ARC_STEP_DEGREES, CRUISE_HEIGHT_AT_THE_START_METRES, FALLBACK_PAD, FALLBACK_TUNNEL, HIGH_CLIMB_STAGE,
    HIGH_HEIGHT_METRES, HIGHEST_TARGET_HEIGHT_METRES, OVER_THE_MAZE_STAGE, TUNNEL_APPROACH_METRES, TUNNEL_STAGE,
    FallbackRouteError, plan_fallback_route, rounded_corners,
)
from team_nav.fallback_run import FallbackPlan, FallbackRun, fallback_route_from_environment
from team_nav.course_run import CLIMB_STAGE
from team_nav.high_path_rules import check_high_path
from team_nav.route_follower import RouteFollower
from team_nav.run_safety import RunSafety, TargetRefused
from team_nav.stopping_spots import find_stopping_spots

# The same pretend PX4 as test_route_follower.py: measured in the simulator on 2026-11-07, and
# drone 16's MPC_Z_P.
SIDEWAYS_SPEED_PER_METRE_OF_DISTANCE = 0.77
UPWARD_SPEED_PER_METRE_OF_DISTANCE = 1.0
SECONDS_PER_STEP = 0.1
LONGEST_FLIGHT_SECONDS = 200.0

MAZE_WALLS = ["M1 maze divider", "M2 maze divider", "M3 maze divider"]


@pytest.fixture(scope="module")
def routes():
    found = {}
    for map_name in ALL_OUR_MAPS:
        for route_name in (FALLBACK_PAD, FALLBACK_TUNNEL):
            found[(map_name, route_name)] = plan_fallback_route(load_our_map(map_name), route_name)
    return found


# ---- The routes on our maps ----------------------------------------------------------------------

@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
@pytest.mark.parametrize("route_name", [FALLBACK_PAD, FALLBACK_TUNNEL])
def test_no_route_breaks_a_laser_rule(routes, map_name, route_name):
    assert routes[(map_name, route_name)].rule_breaks() == []


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
@pytest.mark.parametrize("route_name", [FALLBACK_PAD, FALLBACK_TUNNEL])
def test_every_route_climbs_on_the_launch_mark_and_ends_high_above_the_pad(routes, map_name, route_name):
    course_map = load_our_map(map_name)
    waypoints = routes[(map_name, route_name)].waypoints_metres
    launch, pad = course_map.launch_mark_centre_metres, course_map.landing_pad_centre_metres
    assert np.allclose(waypoints[0], [launch[0], launch[1], CRUISE_HEIGHT_AT_THE_START_METRES])
    assert np.allclose(waypoints[1], [launch[0], launch[1], HIGH_HEIGHT_METRES])
    assert np.allclose(waypoints[-1], [pad[0], pad[1], HIGH_HEIGHT_METRES])
    assert waypoints[:, 2].max() <= HIGH_HEIGHT_METRES < HIGHEST_TARGET_HEIGHT_METRES


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
@pytest.mark.parametrize("route_name", [FALLBACK_PAD, FALLBACK_TUNNEL])
def test_the_drone_stops_only_where_the_height_changes_and_at_the_pad(routes, map_name, route_name):
    """Every arc is flown through, so the only stops are the climbs, the descent and the pad."""
    waypoints = routes[(map_name, route_name)].waypoints_metres
    stops = [number for number, is_stop in enumerate(find_stopping_spots(waypoints)) if is_stop]
    height_changes = [number for number in range(1, len(waypoints))
                      if abs(waypoints[number][2] - waypoints[number - 1][2]) > 1e-6]
    expected = sorted({0, len(waypoints) - 1} | set(height_changes) | {number - 1 for number in height_changes})
    assert stops == expected


def test_the_tunnel_route_descends_before_the_entrance_and_goes_straight_through(routes):
    course_map = load_our_map(SCAN_MAP)
    tube = course_map.tubes[0]
    route = routes[(SCAN_MAP, FALLBACK_TUNNEL)]
    low = route.waypoints_metres[route.waypoints_metres[:, 2] < HIGH_HEIGHT_METRES - 1e-6][1:]
    assert len(low) == 2
    assert np.allclose(low[:, 2], tube.outside_height_metres / 2.0)
    along = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
    assert np.allclose(low[0][:2], tube.entrance_centre_metres - along * TUNNEL_APPROACH_METRES)
    stage_ends = dict(route.stages)
    assert route.leg_speeds_metres_per_second[stage_ends[TUNNEL_STAGE]] == 0.5
    assert list(stage_ends) == [HIGH_CLIMB_STAGE, OVER_THE_MAZE_STAGE, TUNNEL_STAGE]


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
def test_the_tunnel_route_crosses_the_maze_walls_square_on(routes, map_name):
    maze_check = routes[(map_name, FALLBACK_TUNNEL)].path_checks[0][1]
    assert maze_check.crossings, "the route should cross at least one maze wall"
    for crossing in maze_check.crossings:
        assert crossing.degrees_off_perpendicular <= 10.0


# ---- Changed courses: the maze part must not depend on the maze walls ------------------------------

@pytest.mark.parametrize("change", [
    lambda course_map: without_wall(course_map, "M2 maze divider"),
    lambda course_map: without_wall(course_map, "M1 maze divider"),
    lambda course_map: with_wall_moved(course_map, "M2 maze divider", np.array([0.5, 0.0])),
    lambda course_map: with_wall_moved(course_map, "M3 maze divider", np.array([-0.4, 0.0])),
])
@pytest.mark.parametrize("map_name", [SCAN_MAP, DRAWING_MAP])
def test_the_maze_part_is_the_same_whatever_the_maze_walls_do(routes, map_name, change):
    changed = plan_fallback_route(change(load_our_map(map_name)), FALLBACK_TUNNEL)
    original = routes[(map_name, FALLBACK_TUNNEL)]
    maze_end = dict(original.stages)[OVER_THE_MAZE_STAGE]
    assert np.allclose(changed.waypoints_metres[:maze_end + 1], original.waypoints_metres[:maze_end + 1])
    assert changed.rule_breaks() == []


def test_a_slanted_maze_wall_is_still_crossed_without_breaking_a_rule():
    course_map = load_our_map(SCAN_MAP)
    wall = course_map.wall_named("M2 maze divider")
    middle = (wall.start_metres + wall.end_metres) / 2.0
    turn = np.radians(8.0)
    rotation = np.array([[np.cos(turn), -np.sin(turn)], [np.sin(turn), np.cos(turn)]])
    wall.start_metres = middle + rotation @ (wall.start_metres - middle)
    wall.end_metres = middle + rotation @ (wall.end_metres - middle)
    route = plan_fallback_route(course_map, FALLBACK_TUNNEL)
    assert route.rule_breaks() == []


# ---- The laser rules on made-up paths ------------------------------------------------------------

def made_up_course(wall_start, wall_end):
    wall = types.SimpleNamespace(
        name="the wall", start_metres=np.array(wall_start, float), end_metres=np.array(wall_end, float),
        thickness_metres=0.1, top_height_metres=1.75)
    wall.direction = lambda: (wall.end_metres - wall.start_metres) / np.linalg.norm(wall.end_metres - wall.start_metres)
    wall.length_metres = lambda: float(np.linalg.norm(wall.end_metres - wall.start_metres))
    return types.SimpleNamespace(
        walls=[wall], tubes=[], low_obstacles=[],
        lowest_corner_metres=np.array([-3.0, -3.0]), highest_corner_metres=np.array([3.0, 3.0]))


def test_a_square_crossing_with_a_run_up_keeps_every_rule():
    check = check_high_path(made_up_course([0.0, -1.0], [0.0, 1.0]), [[-1.5, 0.0], [1.5, 0.0]], HIGH_HEIGHT_METRES)
    assert check.rule_breaks == [] and check.cautions == []
    assert len(check.crossings) == 1 and check.crossings[0].degrees_off_perpendicular < 0.1


def test_an_oblique_crossing_is_a_rule_break():
    check = check_high_path(made_up_course([0.0, -1.0], [0.0, 1.0]), [[-1.5, -0.5], [1.5, 0.5]], HIGH_HEIGHT_METRES)
    assert any("off square" in text for text in check.rule_breaks)


def test_passing_over_a_walls_end_without_crossing_it_is_a_rule_break():
    check = check_high_path(made_up_course([0.0, 0.0], [0.0, 1.0]), [[-1.5, -0.05], [1.5, -0.05]], HIGH_HEIGHT_METRES)
    assert any("without crossing" in text for text in check.rule_breaks)


def test_turning_above_a_wall_is_a_rule_break():
    path = rounded_corners([[-1.5, 0.0], [0.0, 0.0], [0.0, -1.5]])
    check = check_high_path(made_up_course([-0.2, -2.0], [-0.2, 2.0]), path, HIGH_HEIGHT_METRES)
    assert any("turns" in text for text in check.rule_breaks)


def test_a_wall_right_after_a_stop_is_a_caution():
    check = check_high_path(made_up_course([0.0, -1.0], [0.0, 1.0]), [[-0.2, 0.0], [1.5, 0.0]], HIGH_HEIGHT_METRES)
    assert any("slower than 0.5 m/s" in text for text in check.cautions)


def test_arcs_turn_in_small_steps_and_join_the_legs_smoothly():
    path = np.array(rounded_corners([[0.0, 0.0], [2.0, 0.0], [2.0, 2.0]]))
    pieces = np.diff(path, axis=0)
    headings = pieces / np.linalg.norm(pieces, axis=1)[:, np.newaxis]
    turns = np.degrees(np.arccos(np.clip(np.sum(headings[:-1] * headings[1:], axis=1), -1, 1)))
    assert turns.max() <= ARC_STEP_DEGREES + 1e-6
    assert np.isclose(path[1][1], 0.0) and np.isclose(path[-2][0], 2.0)      # the arc starts and ends on the legs
    assert np.allclose(np.linalg.norm(pieces[1:-1], axis=1), ARC_PIECE_METRES)
    assert find_stopping_spots(np.column_stack([path, np.full(len(path), 2.3)]))[1:-1] == [False] * (len(path) - 2)


def test_a_leg_too_short_for_its_arcs_is_refused():
    with pytest.raises(FallbackRouteError):
        rounded_corners([[0.0, 0.0], [0.2, 0.0], [0.2, 2.0]])


# ---- The follower, the safety limit and the plumbing ---------------------------------------------

def fly_pretend_drone(follower, start_metres):
    drone = np.array(start_metres, dtype=float)
    positions, speeds = [drone.copy()], []
    rates = np.array([SIDEWAYS_SPEED_PER_METRE_OF_DISTANCE] * 2 + [UPWARD_SPEED_PER_METRE_OF_DISTANCE])
    for _ in range(int(LONGEST_FLIGHT_SECONDS / SECONDS_PER_STEP)):
        if follower.has_finished(drone):
            return np.array(positions), np.array(speeds)
        step = rates * (follower.next_target(drone, SECONDS_PER_STEP) - drone) * SECONDS_PER_STEP
        speeds.append(np.linalg.norm(step[:2]) / SECONDS_PER_STEP)
        drone = drone + step
        positions.append(drone.copy())
    raise AssertionError("The pretend drone did not finish the route.")


def test_the_pretend_drone_crosses_every_wall_faster_than_half_a_metre_per_second(routes):
    """With the fallback's speeds, the pretend PX4 is above 0.5 m/s wherever a wall is under the laser,
    except where high_path_rules.py already gave a caution about the run-up."""
    route = routes[(SCAN_MAP, FALLBACK_TUNNEL)]
    follower = RouteFollower(route.waypoints_metres, route.leg_speeds_metres_per_second, target_lead_metres=1.0)
    positions, speeds = fly_pretend_drone(follower, route.waypoints_metres[0])
    for part_name, check in route.path_checks:
        for crossing in check.crossings:
            if crossing.metres_from_last_stop < 0.35:
                continue
            nearest = np.argmin(np.linalg.norm(positions[1:, :2] - crossing.crossing_point_metres, axis=1))
            assert speeds[nearest] > 0.5, f"{part_name}: {crossing.wall_name} crossed at {speeds[nearest]:.2f} m/s"


def test_the_follower_without_speeds_behaves_as_before():
    waypoints = np.array([[0.0, 0.0, 0.8], [3.0, 0.0, 0.8]])
    default, explicit = RouteFollower(waypoints), RouteFollower(waypoints, [0.0, 0.5], target_lead_metres=0.65)
    for _ in range(20):
        assert np.allclose(default.next_target(waypoints[0], 0.1), explicit.next_target(waypoints[0], 0.1))


def test_the_safety_checks_allow_the_fallback_height_only_when_asked():
    course_map = load_our_map(SCAN_MAP)
    target = np.array([-2.5, 0.9, HIGH_HEIGHT_METRES])
    with pytest.raises(TargetRefused):
        RunSafety(course_map).check_target(target)
    RunSafety(course_map, highest_target_height_metres=HIGHEST_TARGET_HEIGHT_METRES).check_target(target)
    with pytest.raises(TargetRefused):
        RunSafety(course_map, highest_target_height_metres=HIGHEST_TARGET_HEIGHT_METRES).check_target(
            np.array([-2.5, 0.9, 2.6]))


def test_the_mission_setting_chooses_the_route(monkeypatch):
    monkeypatch.delenv("TEAM_NAV_MISSION", raising=False)
    assert fallback_route_from_environment() is None
    monkeypatch.setenv("TEAM_NAV_MISSION", "fallback_tunnel")
    assert fallback_route_from_environment() == FALLBACK_TUNNEL
    monkeypatch.setenv("TEAM_NAV_MISSION", "over_the_moon")
    with pytest.raises(ValueError, match="fallback_pad"):
        fallback_route_from_environment()


def test_a_practice_run_can_stop_after_each_fallback_stage(routes):
    course_map = load_our_map(SCAN_MAP)
    route = routes[(SCAN_MAP, FALLBACK_TUNNEL)]
    plan = FallbackPlan(course_map, load_side_gap_first_mission(), [], route, 90.0)
    assert plan.stage_names() == [CLIMB_STAGE, HIGH_CLIMB_STAGE, OVER_THE_MAZE_STAGE, TUNNEL_STAGE]
    cut = plan.waypoints_to_fly(HIGH_CLIMB_STAGE)
    assert len(cut) == 2 and len(plan.leg_speeds_to_fly(HIGH_CLIMB_STAGE)) == 2
    assert len(plan.waypoints_to_fly("")) == len(route.waypoints_metres)


def test_the_fallback_run_uses_its_own_speeds_and_height_limit(routes):
    course_map = load_our_map(SCAN_MAP)
    plan = FallbackPlan(course_map, load_side_gap_first_mission(), [], routes[(SCAN_MAP, FALLBACK_PAD)], 90.0)
    run = FallbackRun.__new__(FallbackRun)
    run.plan, run.stop_after_stage = plan, ""
    safety = run.make_run_safety()
    assert safety.highest_target_height_metres == HIGHEST_TARGET_HEIGHT_METRES
    assert safety.laser_use_check.stop_when_px4_stops_using_the_laser is False
    run.link = None
    follower = run.make_route_follower().follower
    assert follower.target_lead_metres == 1.0
    assert follower.leg_speeds_metres_per_second == plan.route.leg_speeds_metres_per_second


# ---- The organizers' changed course of 2026-11-08 ----------------------------------------------

def test_on_the_changed_course_nothing_can_start_from_the_launch_mark():
    """A new wall stands 0.08 m from the launch mark's centre, so the drone has no room there: the
    course route is refused, and both fallback routes would climb with the wall under the laser."""
    from our_files import CHANGED_COURSE_MAP
    from team_nav.route_planner import RoutePlanningError, plan_route
    course_map = load_our_map(CHANGED_COURSE_MAP)
    with pytest.raises(RoutePlanningError, match="S1 start diagonal wall"):
        plan_route(course_map, load_side_gap_first_mission())
    for route_name in (FALLBACK_PAD, FALLBACK_TUNNEL):
        assert any("S1 start diagonal wall" in text for text in plan_fallback_route(course_map, route_name).rule_breaks())


def test_the_cruise_height_setting_defaults_to_2_3_m_and_refuses_heights_outside_its_limits(monkeypatch):
    from team_nav.fallback_routes import CRUISE_HEIGHT_SETTING, cruise_height_from_environment
    monkeypatch.delenv(CRUISE_HEIGHT_SETTING, raising=False)
    assert cruise_height_from_environment() == 2.3
    monkeypatch.setenv(CRUISE_HEIGHT_SETTING, "2.4")
    assert cruise_height_from_environment() == 2.4
    for refused in ("2.6", "2.1", "high"):
        monkeypatch.setenv(CRUISE_HEIGHT_SETTING, refused)
        with pytest.raises(ValueError):
            cruise_height_from_environment()
