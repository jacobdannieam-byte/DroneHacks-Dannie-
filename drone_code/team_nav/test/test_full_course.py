"""Check route geometry and measured-arrival behavior without sending ROS messages."""
from dataclasses import replace
from types import SimpleNamespace

import numpy as np
import pytest

from our_files import load_our_map
from team_nav.practice_flights.full_course import FullCourseFlight
from team_nav.practice_flights.full_course_route import route_waypoints, CHECKPOINT_LINE_NAMES
from test_flight_steps import PretendLink


def test_sketch_route_clears_all_lines_with_actual_propeller_discs():
    course_map = load_our_map('sketch_2026-11-08')
    points = route_waypoints(course_map)
    # Separate calculation with circles at the measured motors rather than the rectangular
    # envelope used by the route validator. North-facing body: east=-left, north=forward.
    angles = np.linspace(0, 2 * np.pi, 64)
    propellers = []
    for forward, left in ((.08, .16), (.08, -.16), (-.15, .14), (-.15, -.14)):
        propellers.extend(np.column_stack((-left + .0889 * np.cos(angles),
                                           forward + .0889 * np.sin(angles))))
    propellers = np.array(propellers)
    next_line = 0
    for point in points:
        if next_line == len(CHECKPOINT_LINE_NAMES):
            break
        line = course_map.checkpoint_line_named(CHECKPOINT_LINE_NAMES[next_line])
        along = line.end_metres - line.start_metres
        normal = np.array([-along[1], along[0]])
        if normal @ (line.far_side_point_metres - line.start_metres) < 0:
            normal = -normal
        if np.all((point[:2] + propellers - line.start_metres) @ normal > 0):
            next_line += 1
    assert next_line == 4
    assert np.linalg.norm(points[-1, :2] + propellers - course_map.landing_pad_centre_metres, axis=1).max() < course_map.landing_pad_outer_radius_metres


def test_route_refuses_a_boulder_moved_onto_the_laser_path():
    course_map = load_our_map('sketch_2026-11-08')
    course_map.low_obstacles.append(replace(course_map.low_obstacles[0], centre_metres=np.array([-2.51, 1.5])))
    with pytest.raises(ValueError, match='bottom laser'):
        route_waypoints(course_map)


def test_leg_waits_for_measured_arrival_and_keeps_target_lead_short():
    link = PretendLink(height_metres=.75)
    position = np.array([0., 0., .75])
    last_target = position.copy()
    sends = []
    def send(target, heading_on_course_radians):
        nonlocal last_target
        last_target = target.copy()
        sends.append((link.seconds, target.copy(), position.copy()))
    def cycle():
        # The aircraft takes three seconds to start moving, then responds slowly.
        if link.seconds >= 3:
            position[:] += .08 * (last_target - position)
        link.seconds += .1
    link.wait_one_cycle = cycle
    flight = object.__new__(FullCourseFlight)
    flight.link = link
    flight.watch = SimpleNamespace(drone_on_course=lambda: position.copy(), check=lambda: False,
                                   send_target_on_course=send)
    flight.fly_leg(position.copy(), np.array([1., 0., .75]))
    assert link.seconds > 7
    assert np.linalg.norm(position - [1., 0., .75]) < .025
    assert max(np.linalg.norm(t - p) for _, t, p in sends) <= .276
