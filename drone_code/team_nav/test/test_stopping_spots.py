"""Tests stopping_spots.py: which waypoints the drone stops at, and how stops are moved away
from boulders.

The first tests use small made-up routes in open space, so the expected answer can be worked
out by hand. The last tests plan real routes on the maps we keep, with every boulder from every
map counted as a possible boulder, the way the flight program plans.

How to run these tests is in the package's README.
"""
import numpy as np
import pytest

from team_nav.boulder_distances import distance_to_nearest_boulder_metres
from team_nav.course_map import CheckpointLine, LowObstacle
from team_nav.route_planner import COMFORTABLE_ROOM_METRES, LASER_KEEP_OFF_BOULDER_METRES, plan_route
from team_nav.stopping_spots import (
    LARGEST_STOP_MOVE_METRES, MAY_MOVE_ANYWHERE_NEARBY, NEW_STOP_FURTHER_BACK, STAYS_WHERE_IT_IS,
    STOP_KEEP_OFF_BOULDER_METRES, HowWaypointMayMove, find_stopping_spots, keep_stops_away_from_boulders,
)

from our_files import ALL_OUR_MAPS, SCAN_MAP, boulders_from_every_map, load_our_map, load_rulebook_mission, load_side_gap_first_mission
from route_checks import check_route

CRUISE_HEIGHT_METRES = 0.8
# Our own choice: a little allowance for the 2.5 cm steps the stops are moved in.
MOVE_STEP_ALLOWANCE_METRES = 0.03
# Measured on 2026-11-07: with stops moved, the side-gap route on our scan stops no closer than
# 0.38 m to any boulder's edge. Before stops were moved, the closest was 0.23 m.
CLOSEST_STOP_TO_A_BOULDER_ON_THE_SIDE_GAP_ROUTE_METRES = 0.35


def boulder_at(east_metres, north_metres, radius_metres=0.2):
    return LowObstacle("test boulder", np.array([east_metres, north_metres]), radius_metres, 0.5, "made up for a test")


def distance_to_boulders_metres(point_metres, boulders):
    distances_metres, _ = distance_to_nearest_boulder_metres(np.asarray(point_metres)[np.newaxis, :2], boulders)
    return float(distances_metres[0])


def move_stops(waypoints, rules, boulders, checkpoint_lines=()):
    """Moves stops in open space: no walls, so only boulders and checkpoint lines matter."""
    return keep_stops_away_from_boulders(
        np.array(waypoints, dtype=float), rules, boulders, shapes=[], checkpoint_lines=list(checkpoint_lines),
        comfortable_room_metres=COMFORTABLE_ROOM_METRES, moving_keep_off_metres=LASER_KEEP_OFF_BOULDER_METRES,
    )


# ---- Which waypoints are stops ----------------------------------------------------------------

def test_a_sharp_turn_is_a_stop_and_a_slight_one_is_not():
    waypoints = [(0, 0, 0.8), (1, 0, 0.8), (2, 0.1, 0.8), (2, 1, 0.8)]   # 6 degrees, then 84 degrees.

    assert find_stopping_spots(waypoints) == [True, False, True, True]


def test_both_ends_of_a_leg_that_only_climbs_are_stops():
    waypoints = [(0, 0, 0.8), (1, 0, 0.8), (1, 0, 1.2), (2, 0, 1.2)]

    assert find_stopping_spots(waypoints) == [True, True, True, True]


# ---- Moving stops in open space ---------------------------------------------------------------

def test_corner_next_to_a_boulder_is_moved_clear_of_it():
    waypoints = [(0, 0, 0.8), (2, 0, 0.8), (2, 2, 0.8)]
    boulders = [boulder_at(2.3, -0.3)]  # The corner is 0.22 m from its edge.
    rules = [HowWaypointMayMove(STAYS_WHERE_IT_IS), HowWaypointMayMove(MAY_MOVE_ANYWHERE_NEARBY), HowWaypointMayMove(STAYS_WHERE_IT_IS)]

    result = move_stops(waypoints, rules, boulders)

    moved_corner = result.waypoints_metres[1]
    assert distance_to_boulders_metres(moved_corner, boulders) >= STOP_KEEP_OFF_BOULDER_METRES
    assert np.linalg.norm(moved_corner[:2] - np.array([2.0, 0.0])) <= LARGEST_STOP_MOVE_METRES
    assert moved_corner[2] == CRUISE_HEIGHT_METRES


def test_line_up_point_gets_a_new_stop_further_back_along_the_crossing():
    # The crossing runs east from the line-up point at (0, 0), climbing to an opening.
    waypoints = [(-1, 1, 0.8), (0, 0, 0.8), (1, 0, 1.2), (2, 0, 1.2)]
    boulders = [boulder_at(0.0, -0.35)]  # The line-up point is 0.15 m from its edge.
    rules = [
        HowWaypointMayMove(STAYS_WHERE_IT_IS),
        HowWaypointMayMove(NEW_STOP_FURTHER_BACK, np.array([1.0, 0.0])),
        HowWaypointMayMove(STAYS_WHERE_IT_IS),
        HowWaypointMayMove(STAYS_WHERE_IT_IS),
    ]

    result = move_stops(waypoints, rules, boulders)

    new_stop, old_line_up = result.waypoints_metres[1], result.waypoints_metres[2]
    assert len(result.waypoints_metres) == 5
    assert np.allclose(old_line_up, (0, 0, 0.8))
    # Straight back along the crossing, as far as allowed, because even that isn't quite far enough.
    assert np.allclose(new_stop, (-LARGEST_STOP_MOVE_METRES, 0, 0.8))
    assert find_stopping_spots(result.waypoints_metres)[2] is False
    assert result.new_number_of_each_old_waypoint == [0, 2, 3, 4]


def test_stop_does_not_move_across_a_checkpoint_line():
    waypoints = [(0, 0, 0.8), (2, 0, 0.8), (2, 2, 0.8)]
    boulders = [boulder_at(2.3, -0.3)]
    line_just_before_the_corner = CheckpointLine(
        "test line", np.array([1.9, -1.0]), np.array([1.9, 1.0]), np.array([3.0, 0.0]), "made up for a test",
    )
    rules = [HowWaypointMayMove(STAYS_WHERE_IT_IS), HowWaypointMayMove(MAY_MOVE_ANYWHERE_NEARBY), HowWaypointMayMove(STAYS_WHERE_IT_IS)]

    result = move_stops(waypoints, rules, boulders, [line_just_before_the_corner])

    # Ending up exactly on the line is allowed: that isn't across it.
    assert result.waypoints_metres[1][0] >= 1.9 - 1e-9


def test_moved_stop_keeps_its_legs_clear_of_another_boulder():
    waypoints = [(0, 0, 0.8), (2, 0, 0.8), (2, 2, 0.8)]
    near_the_corner = boulder_at(2.3, -0.3)
    # Where the corner would most like to go, up and to the left, there is another boulder.
    in_the_way = boulder_at(1.55, 0.5, radius_metres=0.1)
    rules = [HowWaypointMayMove(STAYS_WHERE_IT_IS), HowWaypointMayMove(MAY_MOVE_ANYWHERE_NEARBY), HowWaypointMayMove(STAYS_WHERE_IT_IS)]

    result = move_stops(waypoints, rules, [near_the_corner, in_the_way])

    corner = result.waypoints_metres[1]
    for leg_start, leg_end in ((result.waypoints_metres[0], corner), (corner, result.waypoints_metres[2])):
        points_on_leg = np.linspace(leg_start[:2], leg_end[:2], 200)
        closest_metres, _ = distance_to_nearest_boulder_metres(points_on_leg, [in_the_way])
        assert closest_metres.min() >= LASER_KEEP_OFF_BOULDER_METRES - MOVE_STEP_ALLOWANCE_METRES


# ---- Our maps -----------------------------------------------------------------------------------

@pytest.mark.parametrize("map_name", ALL_OUR_MAPS)
@pytest.mark.parametrize("load_mission", [load_rulebook_mission, load_side_gap_first_mission])
def test_every_stop_near_a_possible_boulder_is_reported(map_name, load_mission):
    course_map = load_our_map(map_name)
    mission = load_mission()
    possible_boulders = boulders_from_every_map()

    route = plan_route(course_map, mission, other_possible_boulders=possible_boulders)

    check_route(route, course_map, mission, course_map.launch_mark_centre_metres)
    reported = {waypoint_number for waypoint_number, _, _ in route.stops_near_boulders}
    is_stop = find_stopping_spots(route.waypoints_metres)
    for number, waypoint in enumerate(route.waypoints_metres):
        if is_stop[number] and distance_to_boulders_metres(waypoint, possible_boulders) < STOP_KEEP_OFF_BOULDER_METRES:
            assert number in reported, f"The stop at waypoint {number + 1} is near a boulder but isn't reported."


def test_side_gap_route_on_our_scan_stops_well_away_from_every_boulder():
    course_map = load_our_map(SCAN_MAP)
    possible_boulders = boulders_from_every_map()

    route = plan_route(course_map, load_side_gap_first_mission(), other_possible_boulders=possible_boulders)

    lines_used = [line_name for _, line_name, _ in route.checkpoints_cleared]
    assert lines_used[-1] == "Gate B"
    is_stop = find_stopping_spots(route.waypoints_metres)
    for number, waypoint in enumerate(route.waypoints_metres):
        if is_stop[number]:
            assert distance_to_boulders_metres(waypoint, possible_boulders) >= CLOSEST_STOP_TO_A_BOULDER_ON_THE_SIDE_GAP_ROUTE_METRES
