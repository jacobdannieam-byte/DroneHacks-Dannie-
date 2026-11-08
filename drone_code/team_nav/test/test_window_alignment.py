"""Check measured jamb alignment, absent evidence, and the corrected full route geometry."""
import math
import numpy as np
import pytest
from our_files import load_our_map
from team_nav.window_alignment import observe_window
from team_nav.practice_flights.corrected_course import first_two_plan
from team_nav.practice_flights.window_route import full_window_plan,window_geometry


def wall_points(wall, shift=(0.,0.), one_side=False, solid=False):
    opening = wall.openings[0]
    centre = wall.point_along_wall_metres((opening.start_along_wall_metres+opening.end_along_wall_metres)/2)
    along = wall.direction();normal = np.array([-along[1],along[0]])
    width = opening.end_along_wall_metres-opening.start_along_wall_metres
    side,height = np.meshgrid(np.arange(-.7,.7,.005),np.arange(1.25,1.47,.008))
    side,height = side.ravel(),height.ravel()
    valid = np.ones(len(side),dtype=bool) if solid else np.abs(side)>width/2
    if one_side:
        valid &= side < 0
    side,height = side[valid],height[valid]
    noise = np.random.default_rng(7).normal(0,.002,len(side))
    xy = centre+side[:,None]*along+(-wall.thickness_metres/2+noise[:,None])*normal+shift
    return np.column_stack((xy,height))


def test_stereo_jambs_recover_tracking_translation():
    wall = next(w for w in load_our_map('real_course_2026-11-08').walls if w.openings)
    for shift in [(.02,-.20),(-.08,.10),(0.,0.)]:
        observation = observe_window(wall_points(wall,shift),wall)
        assert observation is not None
        np.testing.assert_allclose(observation.correction_metres,-np.array(shift),atol=.012)
        assert abs(observation.width_metres-.6038)<.02


def test_one_jamb_solid_wall_and_missing_depth_cannot_confirm_window():
    wall = next(w for w in load_our_map('real_course_2026-11-08').walls if w.openings)
    for points in [wall_points(wall,one_side=True),wall_points(wall,solid=True),np.empty((0,3))]:
        assert observe_window(points,wall) is None


def test_full_plan_keeps_five_goals_and_crosses_at_checked_height():
    course_map = load_our_map('real_course_2026-11-08')
    plan = full_window_plan(course_map,first_two_plan(course_map))
    assert len(plan['stages'])==5
    stage = plan['stages'][3]
    index = stage['align_at_point']
    before,after = np.array(stage['points'][index]),np.array(stage['points'][index+1])
    assert before[2]==after[2]==1.3626
    geometry = window_geometry(course_map,before[2])
    heading = plan['window_heading_radians']
    room = geometry.footprint_clearance(np.linspace(before[:2],after[:2],200),heading)
    assert room.min()>=.05
    normal = np.array([math.cos(heading),math.sin(heading)])
    along = np.array([-normal[1],normal[0]])
    off_line = np.linspace(before[:2],after[:2],200)+.08*along
    assert geometry.footprint_clearance(off_line,heading).min()<0


@pytest.mark.parametrize('mode', ['valid','repeated','stale','wrong_heading','missing_jamb','off_height','off_centre','unstable'])
def test_alignment_gate_needs_five_fresh_facing_observations(monkeypatch,mode):
    from types import SimpleNamespace
    from team_nav.practice_flights import window_flight
    from team_nav.window_alignment import WindowObservation
    course_map = load_our_map('real_course_2026-11-08')
    target = np.array([.34,-2.36,1.3626]);heading = 2.79
    inspection_heading = heading+window_flight.INSPECTION_BIAS_RADIANS
    clock = [100.]
    events,targets = [],[]
    flight = window_flight.WindowFlight.__new__(window_flight.WindowFlight)
    flight.course_map = course_map
    flight.start_frame = SimpleNamespace(drift_correction_metres=np.zeros(2))
    flight.correct_height = lambda *args:None
    flight.log = SimpleNamespace(event=lambda *args,**kwargs:events.append(args[0]))
    flight.link = SimpleNamespace(now_seconds=lambda:clock[0],node=SimpleNamespace(
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=int(clock[0]*1e9)))))
    flight.depth = SimpleNamespace(stamp=100.,arrival=100.,picture_heading=inspection_heading,points=np.empty((0,3)))
    measured_position = target.copy()
    if mode=='off_height':
        measured_position[2] += .04
    if mode=='off_centre':
        measured_position[0] += .03
    flight.watch = SimpleNamespace(drone_on_course=lambda:measured_position.copy(),drone_heading_on_course=lambda:inspection_heading,
                                  send_target_on_course=lambda point,**kwargs:targets.append(point.copy()))
    residual = np.array([.02,0.]) if mode=='unstable' else np.zeros(2)
    observation = None if mode=='missing_jamb' else WindowObservation(residual,.604,.005,200)
    monkeypatch.setattr(window_flight,'observe_window',lambda *args:observation)
    def fly_until(link,finished,send,*args):
        for index in range(8):
            clock[0] = 100.+index*.1
            flight.depth.stamp = 100. if mode=='repeated' else clock[0]
            if mode=='stale':
                flight.depth.stamp -= 2.
            flight.depth.arrival = clock[0]
            flight.depth.picture_heading = inspection_heading+.2 if mode=='wrong_heading' else inspection_heading
            send()
            if finished():
                return
        raise window_flight.flight_steps.RunMustStop('Test alignment timed out.')
    monkeypatch.setattr(window_flight.flight_steps,'fly_until',fly_until)
    if mode=='valid':
        flight.align_window(target,heading)
        assert events.count('window_observation')==5
        assert 'window_aligned' in events
    else:
        with pytest.raises(window_flight.flight_steps.RunMustStop):
            flight.align_window(target,heading)
        assert 'window_aligned' not in events
    assert all(np.array_equal(point,target) for point in targets)


def test_height_offset_uses_sensor_difference_not_climb_target_error():
    from types import SimpleNamespace
    from team_nav.height_keeper import HeightKeeper
    from team_nav.practice_flights.window_flight import WindowFlight
    flight = WindowFlight.__new__(WindowFlight)
    flight.link = SimpleNamespace(height_correction_metres=-.15,px4_height_risen_metres=lambda:.78)
    flight.height_keeper = HeightKeeper()
    flight.height_keeper.correction_metres = -.15
    for step in range(100):
        flight.correct_height(1.36,np.array([0.,0.,.75]),.1)
    assert np.isclose(flight.link.height_correction_metres,.03,atol=.0001)
    flight.link.px4_height_risen_metres = lambda:1.5
    for step in range(100):
        flight.correct_height(1.36,np.array([0.,0.,.75]),.1)
    assert flight.link.height_correction_metres<=.15
    flight.link.px4_height_risen_metres = lambda:None
    previous = flight.link.height_correction_metres
    flight.correct_height(1.36,np.array([0.,0.,.75]),.1)
    assert flight.link.height_correction_metres==previous


def test_left_eye_depth_is_shifted_to_body_centre_using_calibrated_baseline():
    from team_nav.depth_camera import left_eye_points_to_body
    points = np.array([[1.,0.,0.],[2.,-.2,.1]])
    body = left_eye_points_to_body(points,.07515)
    np.testing.assert_allclose(body-points,[[.1053,.037575,.0304],[.1053,.037575,.0304]])
    np.testing.assert_allclose(points,[[1.,0.,0.],[2.,-.2,.1]])


@pytest.mark.parametrize('timeout',[True,False])
def test_only_alignment_timeout_uses_checked_retreat(timeout):
    from types import SimpleNamespace
    from team_nav.practice_flights.window_flight import WindowFlight
    from team_nav.flight_steps import RunMustStop
    flight = WindowFlight.__new__(WindowFlight)
    flight.plan = {'alignment_retreat_metres':[.35,-1.65,1.3626]}
    flight.link = SimpleNamespace(stage='window alignment')
    flight.log = SimpleNamespace(event=lambda *args,**kwargs:None)
    actions = []
    message = ('Waited 60 seconds for five consistent stereo observations.' if timeout
               else 'Camera tracking is stale. Check the camera.')
    def failed(*args):
        raise RunMustStop(message)
    flight.align_window = failed
    flight.turn_and_settle = lambda *args:actions.append('turn')
    flight.advance = lambda *args:actions.append('advance')
    with pytest.raises(RunMustStop,match=message):
        flight.align_or_retreat(np.array([.34,-2.36,1.3626]),2.79)
    assert actions == (['turn','advance'] if timeout else [])


def test_sparse_stereo_smearing_does_not_move_dense_jamb_edges():
    wall = next(w for w in load_our_map('real_course_2026-11-08').walls if w.openings)
    opening = wall.openings[0]
    centre = wall.point_along_wall_metres((opening.start_along_wall_metres+opening.end_along_wall_metres)/2)
    along = wall.direction()
    normal = np.array([-along[1],along[0]])
    side,height = np.meshgrid(np.arange(-.28,.28,.008),np.linspace(1.28,1.44,4))
    xy = centre+side.ravel()[:,None]*along-wall.thickness_metres/2*normal
    sparse_smearing = np.column_stack((xy,height.ravel()))
    measured = observe_window(np.vstack((wall_points(wall),sparse_smearing)),wall)
    assert measured is not None
    np.testing.assert_allclose(measured.correction_metres,[0.,0.],atol=.012)
    assert abs(measured.width_metres-.6038)<.02
