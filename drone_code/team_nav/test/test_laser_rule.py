"""laser_rule.py: which legs upset the bottom laser (a window crossed, a height change, a boulder
passed over), and which waypoints within 8 s of flight after them must be flown through."""
import json
import types
from pathlib import Path

import numpy as np
import pytest

from team_nav.course_map import load_course_map
from team_nav.laser_rule import LegEvent, legs_that_upset_the_laser, waypoints_to_fly_through
from team_nav.route_flight import split_waypoints
from team_nav.start_frame import StartFrame

PACKAGE = Path(__file__).resolve().parents[1]
CRUISE_METRES = 0.8
# A pretend course on the start frame's own axes (launch mark at 0, nose along +x): a wall across
# x = 2 from y = -2 to y = 2, with an opening 1 to 2 m along it (y = -1 to y = 0), and one boulder.
WALL = types.SimpleNamespace(name="the wall", start_metres=np.array([2.0, -2.0]), end_metres=np.array([2.0, 2.0]),
                             openings=[types.SimpleNamespace(name="window", start_along_wall_metres=1.0,
                                                             end_along_wall_metres=2.0)])
BOULDER = types.SimpleNamespace(name="the boulder", centre_metres=np.array([1.0, 3.0]), radius_metres=0.23)
COURSE = types.SimpleNamespace(walls=[WALL], low_obstacles=[BOULDER])
START = StartFrame((0.0, 0.0), 0.0)


def events_for(waypoints, heights=None):
    heights = heights or [None] * len(waypoints)
    return [(event.leg, event.reason) for event in legs_that_upset_the_laser(COURSE, START, waypoints, heights, CRUISE_METRES)]


def test_a_leg_through_the_opening_is_a_window_leg_and_one_through_the_solid_wall_is_not():
    # The opening runs from y = -1 to y = 0 at x = 2 (1 to 2 m along the wall from y = -2).
    assert events_for([(3.0, -0.5)]) == [(0, "crosses the window in the wall")]
    assert events_for([(3.0, 1.5)]) == []
    assert events_for([(1.5, -0.5)]) == []                                  # stops short of the wall


def test_a_leg_within_the_boulder_radius_plus_the_laser_footprint_passes_over_it():
    assert events_for([(0.0, 3.0), (2.0, 3.0)]) == [(1, "passes over the boulder")]
    assert events_for([(0.0, 3.4), (2.0, 3.4)]) == [(1, "passes over the boulder")]    # 0.40 <= 0.23 + 0.2
    assert events_for([(0.0, 3.5), (2.0, 3.5)]) == []                                  # 0.50 m away


def test_a_change_of_height_is_an_upsetting_leg_including_the_first():
    assert events_for([(0.5, 0.0), (1.0, 0.0), (1.5, 0.0)], [0.6, 0.6, None]) == [
        (0, "changes height from 0.80 to 0.60 m"), (2, "changes height from 0.60 to 0.80 m")]


def test_waypoints_within_8_s_at_0_5_m_s_after_the_leg_are_flown_through_and_the_last_still_stops():
    waypoints = [(1.0, 0.0), (2.0, 0.0), (3.5, 0.0), (5.0, 0.0), (6.0, 0.0), (7.0, 0.0)]
    must, last_note = waypoints_to_fly_through(waypoints, [LegEvent(1, "crosses the window")], 0.5, 8.0)
    # From waypoint 2 (the leg's end): 1.5 m to 3, 3.0 m to 4, 4.0 m to 5 (= 8 s), 5.0 m to 6.
    assert sorted(must) == [1, 2, 3, 4]
    assert must[1] == "within 8 s after the leg into waypoint 2, which crosses the window"
    assert last_note is None
    must, last_note = waypoints_to_fly_through(waypoints, [LegEvent(4, "passes over a boulder")], 0.5, 8.0)
    assert sorted(must) == [4]
    assert last_note == ("waypoint 6 is the last, so the drone still stops there (within 8 s after the leg into "
                         "waypoint 5, which passes over a boulder)")


@pytest.mark.parametrize("route_file, map_file, upsetting, flown_through", [
    ("route_waypoints.json", "organizers_drawing.json", [], []),
    ("route_waypoints_drawing.json", "organizers_drawing.json", [18, 19, 20, 21], [18, 19, 20, 21]),
    ("route_waypoints_scan.json", "phone_scan_2026-11-06.json", [21, 22, 23, 24], [21, 22, 23, 24]),
])
def test_the_real_routes(route_file, map_file, upsetting, flown_through):
    """The team's Gate B route has no window and no boulder; the planned window routes do."""
    course_map = load_course_map(PACKAGE / "course_maps" / map_file)
    start = StartFrame(course_map.launch_mark_centre_metres, 90.0)
    with open(PACKAGE / "missions" / route_file, encoding="utf-8") as opened:
        places, heights = split_waypoints(json.load(opened)["waypoints_forward_left_metres"])
    events = legs_that_upset_the_laser(course_map, start, places, heights, CRUISE_METRES)
    assert [event.leg + 1 for event in events] == upsetting
    must, _ = waypoints_to_fly_through(places, events, 0.5, 8.0)
    assert sorted(index + 1 for index in must) == flown_through
    reasons = " ".join(event.reason for event in events)
    if upsetting:
        assert "crosses the window in C2a chicane wall (window)" in reasons and "passes over Boulder" in reasons
