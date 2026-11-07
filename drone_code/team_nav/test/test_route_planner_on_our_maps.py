"""Tests the route planner on the three maps we keep: the organizers' drawing, our phone scan,
and the scan with every wall trusted.

Each route is checked by route_checks.py, which measures it again from the map instead of
trusting the planner's own numbers.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from team_nav.route_planner import RoutePlanningError, plan_route

from our_files import ALL_OUR_MAPS, DRAWING_MAP, SCAN_MAP, load_our_map, load_rulebook_mission
from route_checks import check_route, closest_approach_to_boulder_metres, first_point_with_whole_drone_past, points_along_route

# The maze is the second of the mission's checkpoints: first corridor, maze, tunnel, chicane.
MAZE_CHECKPOINT_NUMBER = 2
# Our own choice for the boulder test below. Our maps have 5 to 8 boulders, and the planner
# can't keep its full 25 cm from all of them: on the scan, the level run through the window and
# the straight run out of it pass 14 and 9 cm from the boulder just past the window. This test
# makes sure no change to the planner or the maps lets the laser pass over a boulder without
# anyone noticing.
LEAST_DISTANCE_FROM_ANY_BOULDER_EDGE_METRES = 0.05


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
def test_route_from_the_launch_mark_is_safe_and_goes_through_the_window(map_name):
    course_map = load_our_map(map_name)
    mission = load_rulebook_mission()

    route = plan_route(course_map, mission)

    check_route(route, course_map, mission, course_map.launch_mark_centre_metres)
    lines_used = [line_name for _, line_name, _ in route.checkpoints_cleared]
    assert lines_used == ["end of first corridor", "Gate A", "tunnel exit", "through the window"]


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
def test_route_crosses_the_window_at_its_middle_height(map_name):
    course_map = load_our_map(map_name)
    _, window = course_map.opening_named("window")

    route = plan_route(course_map, load_rulebook_mission())

    highest_point_metres = float(np.max(route.waypoints_metres[:, 2]))
    lowest_point_metres = float(np.min(route.waypoints_metres[:, 2]))
    window_middle_metres = window.centre_height_metres()
    # The route changes height only at the window, so the window's middle height is the
    # highest or the lowest point of the route.
    assert np.isclose(highest_point_metres, window_middle_metres) or np.isclose(lowest_point_metres, window_middle_metres)


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
def test_route_never_takes_the_laser_over_a_boulder(map_name):
    course_map = load_our_map(map_name)

    route = plan_route(course_map, load_rulebook_mission())

    for boulder in course_map.low_obstacles:
        closest_metres = closest_approach_to_boulder_metres(route.waypoints_metres, boulder)
        assert closest_metres >= LEAST_DISTANCE_FROM_ANY_BOULDER_EDGE_METRES, (
            f"The drone's centre comes within {closest_metres:.2f} m of the edge of {boulder.name}."
        )


def test_scan_route_climbs_to_the_scanned_window():
    # Our scan puts the window 0.81 to 1.56 m above the floor, so its middle is about 1.19 m,
    # well above the 0.8 m the drone flies at elsewhere. The drawing puts it lower.
    route = plan_route(load_our_map(SCAN_MAP), load_rulebook_mission())

    assert np.max(route.waypoints_metres[:, 2]) == pytest.approx(1.185, abs=0.01)


@pytest.mark.parametrize("map_name", (DRAWING_MAP, SCAN_MAP))
def test_replanning_after_the_maze_clears_only_the_remaining_checkpoints(map_name):
    course_map = load_our_map(map_name)
    mission = load_rulebook_mission()
    first_route = plan_route(course_map, mission)
    _, _, maze_waypoint_number = first_route.checkpoints_cleared[MAZE_CHECKPOINT_NUMBER - 1]
    start_after_maze_metres = first_route.waypoints_metres[maze_waypoint_number][:2]

    route = plan_route(course_map, mission, start_after_maze_metres, checkpoints_already_cleared=MAZE_CHECKPOINT_NUMBER)

    check_route(route, course_map, mission, start_after_maze_metres, checkpoints_already_cleared=MAZE_CHECKPOINT_NUMBER)
    assert route.length_metres < first_route.length_metres


@pytest.mark.parametrize("map_name", (DRAWING_MAP, SCAN_MAP))
def test_start_inside_a_wall_gives_a_clear_error(map_name):
    course_map = load_our_map(map_name)
    divider = course_map.wall_named("M1 maze divider")
    middle_of_divider_metres = (divider.start_metres + divider.end_metres) / 2.0

    with pytest.raises(RoutePlanningError, match="The start .* has only 0.00 m of room, next to the M1 maze divider"):
        plan_route(course_map, load_rulebook_mission(), middle_of_divider_metres)


def test_route_checks_notice_a_route_through_a_wall():
    # A check on the checks: a straight line from the launch mark to the landing pad crosses
    # the barrier, and clears no checkpoints, so check_route must reject it.
    course_map = load_our_map(DRAWING_MAP)
    route = plan_route(course_map, load_rulebook_mission())
    route.waypoints_metres = np.array([route.waypoints_metres[0], route.waypoints_metres[-1]])

    with pytest.raises(AssertionError, match="passes 0.00 m from the Barrier"):
        check_route(route, course_map, load_rulebook_mission(), course_map.launch_mark_centre_metres)


def test_whole_drone_past_line_needs_the_centre_one_radius_beyond_it():
    course_map = load_our_map(DRAWING_MAP)
    line = course_map.checkpoint_line_named("end of first corridor")   # Runs east-west; its far side is north.
    middle_metres = (line.start_metres + line.end_metres) / 2.0
    on_the_line = np.append(middle_metres, 0.8)
    just_short = on_the_line + np.array([0.0, 0.29, 0.0])
    just_past = on_the_line + np.array([0.0, 0.31, 0.0])

    assert first_point_with_whole_drone_past(line, points_along_route(np.array([on_the_line, just_short]))) is None
    assert first_point_with_whole_drone_past(line, np.array([on_the_line, just_short, just_past])) == 2


@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
def test_the_drone_stops_well_away_from_boulders_at_the_window(map_name):
    # The drone stops at the window's line-up point, and again above it once it has climbed to
    # the window's height. Both stops must keep clear of every boulder in the drone's map and in
    # the organizers' drawing, as course_run.py plans it. Until 2026-11-07 the line-up point was
    # 13 cm from the boulder in front of the window on the scan.
    drawing = load_our_map(DRAWING_MAP)
    route = plan_route(load_our_map(map_name), load_rulebook_mission(), other_possible_boulders=drawing.low_obstacles)

    window_level_end_number = [number for _, line_name, number in route.checkpoints_cleared if line_name == "through the window"][0]
    window_stops = {window_level_end_number - 2, window_level_end_number - 1}
    stops_too_close = [stop for stop in route.stops_near_boulders if stop[0] in window_stops]
    assert stops_too_close == []
