import json
import math
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import pytest
from our_files import MISSIONS_FOLDER, load_our_map
from test_flight_steps import PretendLink
from team_nav.practice_flights.five_waypoint_plan import validate_plan, angle_difference, segments, rotated_footprint
from team_nav.course_map import shapes_blocking_between_heights, distance_to_shapes_metres
from team_nav.practice_flights.five_waypoints import FiveWaypointFlight, turn_towards
from team_nav.practice_flights.forward_depth import ForwardDepth, obstacle_reason
from team_nav.flight_steps import RunMustStop
from team_nav.start_frame import StartFrame
from team_nav.height_keeper import HeightKeeper


def make_flight():
    flight = object.__new__(FiveWaypointFlight)
    flight.height_keeper = HeightKeeper()
    return flight


def plan():
    return json.loads((Path(MISSIONS_FOLDER)/'five_waypoints.json').read_text())


def test_five_targets_clear_turns_checkpoints_and_pad():
    validate_plan(plan(), load_our_map('sketch_2026-11-08'))


def test_chicane_exit_has_margin_for_observed_northward_drift():
    # Regression for the failed chicane trial: 25 cm north and 3 cm east/west error.
    course_map = load_our_map('sketch_2026-11-08')
    shapes = shapes_blocking_between_heights(course_map, .55, .95)
    for stage, start, end, previous, heading, reverse in segments(plan()):
        if stage['name'] != 'Chicane exit' or start[0] > .4:
            continue
        turn = angle_difference(heading, previous)
        swept = np.concatenate([start[:2]+rotated_footprint(angle)
                                for angle in np.linspace(previous, previous+turn, 60)])
        travel = (np.linspace(start[:2], end[:2], 60)[:,None,:]+rotated_footprint(heading)).reshape(-1,2)
        for east_error in [-.03, 0, .03]:
            for points in [swept, travel]:
                distance, _ = distance_to_shapes_metres(points+[east_error,.25], shapes)
                assert distance.min() >= .05


def test_turning_at_first_checkpoint_without_retreat_is_rejected():
    mission = plan()
    mission['stages'][1]['points'].pop(0)
    mission['stages'][1]['reverse_first_segment'] = False
    with pytest.raises(ValueError, match='Unsafe turn'):
        validate_plan(mission, load_our_map('sketch_2026-11-08'))


def test_yaw_wrap_takes_short_path_at_limited_rate():
    result = turn_towards(math.radians(179), math.radians(-170), .1)
    assert math.degrees(angle_difference(result, math.radians(179))) == pytest.approx(1.5)


def test_retreat_cannot_move_sideways_into_unobserved_space():
    mission = plan()
    mission['stages'][1]['points'][0] = [-2.4, 2.2, .75]
    with pytest.raises(ValueError, match='retrace'):
        validate_plan(mission, load_our_map('sketch_2026-11-08'))


def test_depth_blocks_forward_obstacle_but_not_floor_or_side():
    position = np.array([0.,0.,.75])
    goal = np.array([1.,0.,.75])
    assert obstacle_reason(np.tile([.3,0,.75], (4,1)), position, goal, 0)
    assert obstacle_reason(np.tile([.3,0,0], (4,1)), position, goal, 0) is None
    assert obstacle_reason(np.tile([.3,.5,.75], (4,1)), position, goal, 0) is None


def test_depth_missing_and_stale_hold_even_retreat():
    depth = object.__new__(ForwardDepth)
    depth.arrival = None
    assert depth.reason_to_hold(None,None,0,True)
    depth.arrival, depth.stamp = 0, 0
    depth.link = SimpleNamespace(now_seconds=lambda: 2,
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2e9))))
    assert depth.reason_to_hold(None,None,0,True) == 'depth picture is stale'


def test_recent_picture_from_before_turn_does_not_release_motion():
    depth = object.__new__(ForwardDepth)
    depth.arrival = depth.stamp = 2
    depth.picture_heading = math.pi/2
    depth.points = np.empty((0,3))
    depth.link = SimpleNamespace(now_seconds=lambda: 2.1,
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=2.1e9))))
    position = np.array([0.,0.,.75])
    assert depth.reason_to_hold(position, position, 0) == 'waiting for depth facing the new direction'
    depth.picture_heading = 0
    assert depth.reason_to_hold(position, position, 0) is None


def test_turn_waits_for_measured_heading_and_holds_position():
    flight = make_flight()
    link = flight.link = PretendLink(.75)
    position = np.array([0.,0.,.75])
    heading = 0.
    command = 0.
    def send(target, heading_on_course_radians):
        nonlocal command
        np.testing.assert_array_equal(target, position)
        command = heading_on_course_radians
    def cycle():
        nonlocal heading
        if link.seconds > 3:
            heading += .15*angle_difference(command, heading)
        link.seconds += .1
    link.wait_one_cycle = cycle
    flight.watch = SimpleNamespace(check=lambda: False, drone_on_course=lambda: position.copy(),
        drone_heading_on_course=lambda: heading, send_target_on_course=send)
    flight.turn_and_settle(position, math.pi/2)
    assert abs(angle_difference(math.pi/2, heading)) < math.radians(5)
    assert link.seconds > 7


def test_translation_waits_for_depth_and_measured_arrival():
    flight = make_flight()
    link = flight.link = PretendLink(.75)
    flight.log = link.log
    position = np.array([0.,0.,.75])
    target = position.copy()
    def send(value, heading_on_course_radians):
        nonlocal target
        if link.seconds < 3:
            np.testing.assert_array_equal(value, [0,0,.75])
        target = value.copy()
    def cycle():
        position[:] += .1*(target-position)
        link.seconds += .1
    link.wait_one_cycle = cycle
    flight.watch = SimpleNamespace(check=lambda: False, drone_on_course=lambda: position.copy(),
        drone_heading_on_course=lambda: 0, send_target_on_course=send)
    flight.depth = SimpleNamespace(reason_to_hold=lambda *args: 'waiting' if link.seconds<3 else None)
    flight.advance(position.copy(), np.array([1.,0.,.75]), 0, False)
    assert np.linalg.norm(position-[1,0,.75]) < .025
    assert link.seconds > 8


def test_persistent_obstacle_stops_without_advancing():
    flight = make_flight()
    link = flight.link = PretendLink(.75)
    flight.log = link.log
    position = np.array([0.,0.,.75])
    def send(target, heading_on_course_radians):
        np.testing.assert_array_equal(target, position)
    flight.watch = SimpleNamespace(check=lambda: False, drone_on_course=lambda: position.copy(),
        drone_heading_on_course=lambda: 0, send_target_on_course=send)
    flight.depth = SimpleNamespace(reason_to_hold=lambda *args: 'obstacle')
    with pytest.raises(RunMustStop, match='obstacle'):
        flight.advance(position, np.array([1.,0.,.75]), 0, False)
    assert link.seconds >= 20


def test_depth_callback_transforms_points_using_stamped_pose(monkeypatch):
    from team_nav.practice_flights import forward_depth
    monkeypatch.setattr(forward_depth, 'points_from_message', lambda message: np.array([[1.,0.,0.]]))
    depth = object.__new__(ForwardDepth)
    depth.frame = StartFrame([-2., .9], 90)
    depth.frame.drift_correction_metres = np.array([.1,.2])
    depth.stamp = None
    depth.messages_used = 0
    depth.link = SimpleNamespace(launch=SimpleNamespace(tracked_position_metres=np.zeros(3)),
        tracked_pose_at=lambda stamp: (np.array([1.,2.,.75]), np.array([0.,0.,0.,1.])),
        now_seconds=lambda: 10, log=SimpleNamespace(sample=lambda *args, **kwargs: None),
        node=SimpleNamespace(get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10e9))))
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)))
    depth.receive(message)
    np.testing.assert_allclose(depth.frame.drift_correction_metres, [.1,.2])
    np.testing.assert_allclose(depth.points, [[-3.9,3.1,.75]])
    assert depth.messages_used == 1
    assert depth.picture_heading == pytest.approx(math.pi/2)


def test_height_feedback_is_bounded_and_ignores_missing_tracking():
    flight = make_flight()
    flight.link = SimpleNamespace(height_correction_metres=0.)
    for _ in range(100):
        flight.correct_height(.75, np.array([0.,0.,.60]), .1)
    assert flight.link.height_correction_metres == pytest.approx(.15)
    flight.correct_height(.75, None, .1)
    assert flight.link.height_correction_metres == pytest.approx(.15)
