"""Tests the route planner on changed copies of the organizers' drawing and of our scan.

The course may change after our scan, and the drone's cameras may find things its map does not
have. These tests make such changes (course_changes.py) and check that the planner either finds
a safe new route (checked by route_checks.py) or says clearly why there is none.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from team_nav.route_planner import LASER_KEEP_OFF_BOULDER_METRES, RoutePlanningError, plan_route

import course_changes
from our_files import DRAWING_MAP, SCAN_MAP, load_our_map, load_rulebook_mission
from route_checks import check_route, closest_approach_to_boulder_metres

# The tunnel is the third of the mission's checkpoints: first corridor, maze, tunnel, chicane.
TUNNEL_CHECKPOINT_NUMBER = 3
# The size of the boulders in the organizers' drawing, rounded. Below the drone's body at its
# usual 0.8 m, so the drone could fly over one; the planner keeps the bottom laser off it if it can.
BOULDER_HEIGHT_METRES = 0.53
BOULDER_RADIUS_METRES = 0.23
# The boulder in front of the window, in the drawing and in both our scan maps.
BOULDER_IN_FRONT_OF_THE_WINDOW = "Boulder 3"
# A thin pole, tall enough to block the drone.
POLE_RADIUS_METRES = 0.10
POLE_HEIGHT_METRES = 1.7

BOTH_MAPS = (DRAWING_MAP, SCAN_MAP)


def lines_used(route):
    return [line_name for _, line_name, _ in route.checkpoints_cleared]


def boulders_passed_closely(route):
    """The names of the boulders the route has to pass closer than the planner would like."""
    names = set()
    for _, boulder_name, _ in route.legs_near_boulders:
        names.add(boulder_name)
    return names


def plan_and_check(course_map):
    """Plans a route from the launch mark and checks it. Returns the route."""
    mission = load_rulebook_mission()
    route = plan_route(course_map, mission)
    check_route(route, course_map, mission, course_map.launch_mark_centre_metres)
    return route


# ---- The window is blocked, so the drone must use Gate B --------------------------------------

@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_window_too_small_uses_gate_b(map_name):
    route = plan_and_check(course_changes.with_window_too_small(load_our_map(map_name)))

    assert lines_used(route)[-1] == "Gate B"


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_pillar_behind_window_uses_gate_b(map_name):
    route = plan_and_check(course_changes.with_pillar_behind_window(load_our_map(map_name)))

    assert lines_used(route)[-1] == "Gate B"


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_window_blocked_found_after_the_tunnel_replans_through_gate_b(map_name):
    # As if the drone only saw the window was blocked once it was through the tunnel.
    original_map = load_our_map(map_name)
    mission = load_rulebook_mission()
    first_route = plan_route(original_map, mission)
    _, _, tunnel_waypoint_number = first_route.checkpoints_cleared[TUNNEL_CHECKPOINT_NUMBER - 1]
    start_after_tunnel_metres = first_route.waypoints_metres[tunnel_waypoint_number][:2]
    changed_map = course_changes.with_window_too_small(original_map)

    route = plan_route(changed_map, mission, start_after_tunnel_metres, checkpoints_already_cleared=TUNNEL_CHECKPOINT_NUMBER)

    check_route(route, changed_map, mission, start_after_tunnel_metres, checkpoints_already_cleared=TUNNEL_CHECKPOINT_NUMBER)
    assert lines_used(route) == ["Gate B"]


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_window_and_gate_b_both_blocked_says_why_for_each(map_name):
    changed_map = course_changes.with_gate_b_closed(course_changes.with_window_too_small(load_our_map(map_name)))

    with pytest.raises(RoutePlanningError) as error:
        plan_route(changed_map, load_rulebook_mission())

    message = str(error.value)
    assert "The window is 0.30 m tall, but the drone needs 0.40 m" in message
    assert "the route can't get across 'Gate B'" in message


# ---- Something blocks the only way through -----------------------------------------------------

@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_blocked_tunnel_says_the_tunnel_exit_cannot_be_reached(map_name):
    changed_map = course_changes.with_tunnel_blocked(load_our_map(map_name))

    with pytest.raises(RoutePlanningError, match="the route can't get across 'tunnel exit'"):
        plan_route(changed_map, load_rulebook_mission())


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_closed_first_corridor_says_its_end_cannot_be_reached(map_name):
    changed_map = course_changes.with_first_corridor_closed(load_our_map(map_name))

    with pytest.raises(RoutePlanningError, match="the route can't get across 'end of first corridor'"):
        plan_route(changed_map, load_rulebook_mission())


def test_scan_with_the_drawings_m3_has_no_way_through_the_maze():
    # This is why our best scan map uses the scanned M3: the drawing's M3 with the scanned M2
    # leaves a gap too narrow for the drone (see make_map_from_phone_scan.py).
    changed_map = course_changes.with_wall_from_other_map(load_our_map(SCAN_MAP), load_our_map(DRAWING_MAP), "M3 maze divider")

    with pytest.raises(RoutePlanningError, match="the route can't get across 'Gate A'"):
        plan_route(changed_map, load_rulebook_mission())


# ---- Something new is in the way, but there is room to go round it -------------------------------

@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_narrowed_first_corridor_still_has_a_safe_route(map_name):
    # 0.8 m is a little more than the 0.7 m the drone needs (twice its radius plus margin).
    plan_and_check(course_changes.with_first_corridor_narrowed(load_our_map(map_name), gap_left_metres=0.8))


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_pole_in_the_way_is_flown_round(map_name):
    original_map = load_our_map(map_name)
    original_route = plan_route(original_map, load_rulebook_mission())
    pole_centre_metres = course_changes.point_halfway_from_tunnel_to_window(original_route)
    changed_map = course_changes.with_low_obstacle(original_map, "pole", pole_centre_metres, POLE_RADIUS_METRES, POLE_HEIGHT_METRES)

    route = plan_and_check(changed_map)

    # check_route has already checked that the route keeps clear of the pole.
    assert lines_used(route) == lines_used(original_route)


# ---- Boulders added, moved or removed -------------------------------------------------------------
# A moved boulder is, to the planner, a boulder removed from one place and added at another.

@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_added_boulder_with_room_round_it_is_flown_round(map_name):
    original_map = load_our_map(map_name)
    original_route = plan_route(original_map, load_rulebook_mission())
    boulder_centre_metres = course_changes.point_halfway_from_window_exit_to_landing_pad(original_route)
    changed_map = course_changes.with_low_obstacle(
        original_map, "added boulder", boulder_centre_metres, BOULDER_RADIUS_METRES, BOULDER_HEIGHT_METRES,
    )

    route = plan_and_check(changed_map)

    assert lines_used(route) == lines_used(original_route)
    assert "added boulder" not in boulders_passed_closely(route)


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_boulder_that_cannot_be_avoided_is_passed_over_its_edge_not_its_middle(map_name):
    # The first corridor leaves the drone's centre only about 30 cm of room to move sideways, so
    # it can't keep its laser off a 46 cm boulder in the middle. The best it can do is pass with
    # the laser over the boulder's edge.
    original_map = load_our_map(map_name)
    original_route = plan_route(original_map, load_rulebook_mission())
    boulder_centre_metres = course_changes.point_halfway_up_the_first_corridor(original_route)
    changed_map = course_changes.with_low_obstacle(
        original_map, "boulder in the corridor", boulder_centre_metres, BOULDER_RADIUS_METRES, BOULDER_HEIGHT_METRES,
    )

    route = plan_and_check(changed_map)

    boulder = changed_map.low_obstacles[-1]
    closest_metres = closest_approach_to_boulder_metres(route.waypoints_metres, boulder)
    assert "boulder in the corridor" in boulders_passed_closely(route)
    assert closest_metres > -BOULDER_RADIUS_METRES / 2.0


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_the_boulder_in_front_of_the_window_no_longer_bends_the_route(map_name):
    # Until 2026-11-07 the drone climbed on its way to the window, so it lined up 0.6 m further
    # out, where the safety net pinned the line-up point 13 cm from this boulder. Now it climbs
    # on the spot closer to the wall (opening_crossings.py), so the route keeps clear of the
    # boulder, and removing it changes almost nothing.
    original_map = load_our_map(map_name)
    original_route = plan_route(original_map, load_rulebook_mission())

    route = plan_and_check(course_changes.without_low_obstacle(original_map, BOULDER_IN_FRONT_OF_THE_WINDOW))

    assert BOULDER_IN_FRONT_OF_THE_WINDOW not in boulders_passed_closely(original_route)
    assert route.length_metres < original_route.length_metres + 0.1


# ---- Walls moved or removed -----------------------------------------------------------------------

@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_moved_maze_wall_gives_a_new_safe_route(map_name):
    original_map = load_our_map(map_name)
    original_route = plan_route(original_map, load_rulebook_mission())

    route = plan_and_check(course_changes.with_wall_moved(original_map, "M2 maze divider", (0.2, 0.0)))

    assert not np.array_equal(route.waypoints_metres, original_route.waypoints_metres)


@pytest.mark.parametrize("map_name", BOTH_MAPS)
def test_removed_maze_wall_gives_a_shorter_route(map_name):
    original_map = load_our_map(map_name)
    original_route = plan_route(original_map, load_rulebook_mission())

    route = plan_and_check(course_changes.without_wall(original_map, "M2 maze divider"))

    assert route.length_metres < original_route.length_metres - 1.0
