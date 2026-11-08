import json
from pathlib import Path
from dataclasses import replace
import pytest

from refnav.full_course_model import FullCoursePlan, FullCourseSelector
from refnav.model_flight import REFERENCE
from refnav.dynamics import DroneState


def config():return json.loads(Path('scenarios/model_flight/full_course.json').read_text())


def test_full_route_and_unchanged_native_speed_limits():
    raw=config();plan=FullCoursePlan(**raw['plan']);selector=FullCourseSelector(raw['model_flight'])
    assert len(selector.route.waypoints_metres)==23
    assert len(raw['model_flight']['waypoints_ned_m'])==22
    assert plan.actual_speed_limit_mps==.5 and plan.trial_wall_limit_s==600
    with pytest.raises(ValueError):replace(plan,actual_speed_limit_mps=.6)
    with pytest.raises(ValueError):replace(plan,trial_wall_limit_s=1000)
    bad=dict(raw['model_flight'],waypoints_ned_m=raw['model_flight']['waypoints_ned_m'][:-1])
    with pytest.raises(ValueError):FullCourseSelector(bad)


def test_dynamic_height_slab_and_map_checked_commands():
    raw=config();selector=FullCourseSelector(raw['model_flight'],monotonic=lambda:0.)
    state=DroneState(REFERENCE,1.,(0.,0.,-.8),(0.,0.,0.),0.,0.)
    target,row=selector.choose(state,raw['model_flight']['waypoints_ned_m'][0],1_000_000_000)
    selected=next(r for r in row['candidates'] if r['candidate_id']==row['selected'])
    assert selected['pinn']['map_margin_m']>=0
    # Eastward from launch intersects M1; it cannot be chosen by the predictor.
    bad=replace(target,position_m=(0.,.45,-.8))
    assert not selector.evaluate(selector.model,state,bad,raw['model_flight']['waypoints_ned_m'][0])['eligible']
    with pytest.raises(ValueError,match='disabled'):
        FullCourseSelector(dict(raw['model_flight'],registration_enabled=True))


def test_full_course_approval_cannot_be_prefix_approval(tmp_path):
    from examples.model_flight_owner import approval_for
    path=tmp_path/'approval.json'
    path.write_text(json.dumps(dict(mode='hybrid_model_trial',expires_utc='2099-01-01T00:00:00Z',trial_id='hybrid-pinn-classical-prefix-001')))
    with pytest.raises(ValueError):approval_for(path,{'trial_id':'hybrid-pinn-full-course-001'},'scenarios/model_flight/full_course_recorder.json')


def test_model_requires_current_observed_center_volume_when_native_guard_enabled():
    raw=config();selector=FullCourseSelector(raw['model_flight'],monotonic=lambda:0.)
    state=DroneState(REFERENCE,1.,(0.,0.,-.8),(0.,0.,0.),0.,0.)
    selector.require_depth=True
    goal=raw['model_flight']['waypoints_ned_m'][0]
    with pytest.raises(ValueError,match='No model candidate'):
        selector.choose(state,goal,1_000_000_000)
    selector.depth_admission={'allowed':True,'admitted_center_volume':{
        'origin':[0,0,-.8],'direction':[1,0,0],'lateral_limit_m':.1,
        'vertical_limit_m':.05,'forward_limit_m':.5}}
    target,_=selector.choose(state,goal,1_000_000_000)
    outside=replace(target,position_m=(.1,.15,-.8))
    assert not selector.evaluate(selector.model,state,outside,goal)['eligible']


def test_missing_depth_holds_then_lands_without_model_fallback(tmp_path):
    from refnav.full_course_model import FullCourseFlight
    from refnav.model_flight import DecisionLog
    from refnav.calibration_trial import Observation
    raw=config();log=DecisionLog(tmp_path/'decisions')
    driver=FullCourseFlight(FullCoursePlan(**raw['plan']),raw['model_flight'],log)
    obs=Observation((0,0,-.8),(0,0,0),1_000_000_000,1_000_000_000,True,False,14,0.)
    goal=raw['model_flight']['waypoints_ned_m'][0]
    first=driver.admit_goal(obs,goal,1_000_000_000,1_000_000_000)
    assert first.velocity_ned_mps==(0,0,0) and driver.model_commands==0
    last=driver.admit_goal(obs,goal,4_000_000_000,4_000_000_000)
    assert last.command=='land'
    log.close()
