"""The drift correction in flight (mission.py with drift_correction on), against the pretend drone
with camera tracking drifting: at each stop corner, before turning, the depth straight ahead is
compared with a pretend map, the position estimate moves along the heading, every correction and
every skip is logged, and the drone then really reaches the next waypoint."""
import math

import pytest

from fake_drone import FakeDrone, fly
from team_nav.mission import Mission
from team_nav.mission_settings import MissionSettings

# A pretend course: one wall across the route, 3 m in front of the takeoff spot (heading 0 = East).
WALL_FORWARD_METRES = 3.0
WALL_NAME = "the pretend wall"
ROUTE = [(2.0, 0.0), (2.0, -1.0)]
# The drift grows with the distance flown forward, reaching its full size here, so that it never
# looks like a jump to the watchdog.
FULL_DRIFT_AFTER_METRES = 1.5


class PretendMapRanging:
    """Distance to WALL_FORWARD_METRES along the start heading; nothing in any other direction."""

    def __init__(self):
        self.asked = []

    def distance_ahead(self, forward_metres, left_metres, heading_from_start_radians, height_metres):
        self.asked.append((forward_metres, left_metres, heading_from_start_radians, height_metres))
        if abs(heading_from_start_radians) > math.radians(5.0):
            return math.inf, None
        return WALL_FORWARD_METRES - forward_metres, WALL_NAME


def drift_settings(on=True):
    settings = MissionSettings()
    settings.drift_correction = on
    return settings


def fly_with_drift(error_metres, on=True, depth=True):
    """Camera tracking reports the drone further back (toward -x) than it really is, by up to
    error_metres once it has flown FULL_DRIFT_AFTER_METRES forward. The depth sees the real wall
    from the real position. Returns the mission, the drone, the record, and the corrections seen."""
    mission = Mission(drift_settings(on), mission_kind="route", route_waypoints=ROUTE,
                      map_ranging=PretendMapRanging() if on else None)
    drone = FakeDrone()
    seen = set()

    def drift_and_depth(now_seconds, mission, drone):
        seen.add(mission.position_correction_metres)
        if mission.phase == "flying_route":
            flown = min(max(drone.position[0], 0.0), FULL_DRIFT_AFTER_METRES)
            # The drift only ever grows: min keeps the more negative (further back) of the two.
            drone.tracking_error_metres = [min(-error_metres * flown / FULL_DRIFT_AFTER_METRES,
                                               drone.tracking_error_metres[0]), 0.0]
        if depth and abs(drone.heading_radians) < math.radians(5.0):
            mission.observe_depth_ahead(now_seconds, WALL_FORWARD_METRES - drone.position[0])
        elif depth:
            mission.observe_depth_ahead(now_seconds, None)
    record = fly(mission, drone, 90.0, every_step=drift_and_depth)
    return mission, drone, record, seen


def corrections(mission):
    return [text for _, _, text in mission.history if text.startswith("drift correction")]


def true_forward_error_at_the_end(drone):
    """How far past the last waypoint the drone really is, along the first leg's direction (x),
    the direction the correction works in. (Sideways, the route's own arrive distance applies.)"""
    return drone.position[0] - ROUTE[-1][0]


def test_without_correction_the_drift_carries_the_drone_past_the_waypoints():
    mission, drone, record, seen = fly_with_drift(0.15, on=False)
    assert corrections(mission) == []
    assert true_forward_error_at_the_end(drone) == pytest.approx(0.15, abs=0.01)


def test_the_correction_at_the_first_corner_removes_the_drift_along_the_heading():
    mission, drone, record, seen = fly_with_drift(0.15)
    logged = corrections(mission)
    assert logged[0].startswith("drift correction at the takeoff spot: none, the map's wall straight ahead")
    assert logged[1].startswith("drift correction at waypoint 1 of 2: depth ")
    assert logged[1].endswith("to the pretend wall: moved the estimate +0.15 m along the heading; total correction "
                              "now (+0.15, +0.00) m")
    assert true_forward_error_at_the_end(drone) == pytest.approx(0.0, abs=0.01)
    assert "route: route complete: reached waypoint 2 of 2" in [text for _, _, text in mission.history]


def test_the_correction_happens_before_the_turn():
    mission, drone, record, seen = fly_with_drift(0.15)
    history = [text for _, _, text in mission.history]
    corrected = next(index for index, text in enumerate(history) if text.startswith("drift correction at waypoint 1"))
    turning = history.index("route: turning to face waypoint 2 of 2")
    reached = history.index("route: reached waypoint 1 of 2")
    assert reached < corrected < turning


def test_a_large_drift_is_corrected_at_most_0_2_metres_at_a_time():
    mission, drone, record, seen = fly_with_drift(0.3)
    assert "moved the estimate +0.20 m along the heading" in corrections(mission)[1]
    assert true_forward_error_at_the_end(drone) == pytest.approx(0.1, abs=0.01)


def test_without_a_depth_reading_the_correction_is_skipped_and_says_why():
    mission, drone, record, seen = fly_with_drift(0.15, depth=False)
    assert corrections(mission)[1] == "drift correction at waypoint 1 of 2: none, no good depth reading straight ahead"
    assert true_forward_error_at_the_end(drone) == pytest.approx(0.15, abs=0.01)


def test_an_old_depth_reading_counts_as_none():
    settings = drift_settings()
    mission = Mission(settings, mission_kind="route", route_waypoints=ROUTE, map_ranging=PretendMapRanging())
    drone = FakeDrone()

    def depth_once(now_seconds, mission, drone):
        if now_seconds == 0.0:
            mission.observe_depth_ahead(now_seconds, 1.0)
    fly(mission, drone, 60.0, every_step=depth_once)
    assert corrections(mission)[1].endswith("none, no good depth reading straight ahead")


def test_the_drift_correction_needs_a_map():
    with pytest.raises(ValueError, match="The drift correction needs a course map"):
        Mission(drift_settings(), mission_kind="route", route_waypoints=ROUTE)


def test_the_correction_is_cleared_once_the_run_is_over():
    mission, drone, record, seen = fly_with_drift(0.15)
    assert any(correction[0] == pytest.approx(0.15) for correction in seen)
    assert "disarmed on the ground: ready for another run" in [text for _, _, text in mission.history]
    assert mission.position_correction_metres == (0.0, 0.0)
