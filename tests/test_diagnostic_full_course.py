import json
from pathlib import Path
from dataclasses import replace
import pytest
from refnav.full_course_model import FullCourseFlight,FullCoursePlan,FullCourseSelector
from refnav.model_flight import DecisionLog,REFERENCE
from refnav.dynamics import DroneState


def raw():return json.loads(Path('scenarios/model_flight/full_course_diagnostic.json').read_text())


def test_diagnostic_logs_rejected_candidate_without_blocking_it():
    config=raw()['model_flight'];s=FullCourseSelector(config,monotonic=lambda:0.)
    state=DroneState(REFERENCE,1.,(0.,0.,-.8),(0.,0.,0.),0.,0.)
    command,_=s.choose(state,config['waypoints_ned_m'][0],10**9)
    row=s.evaluate(s.model,state,replace(command,position_m=(0.,.45,-.8)),config['waypoints_ned_m'][0])
    assert row['eligible'] and not row['would_admit_with_bounds'] and row['diagnostic']


def test_diagnostic_driver_refuses_nonisolated_execution(tmp_path,monkeypatch):
    monkeypatch.setattr('examples.gazebo_container.isolated',lambda:False)
    a=raw();log=DecisionLog(tmp_path/'log')
    with pytest.raises(ValueError,match='isolated-simulator'):
        FullCourseFlight(FullCoursePlan(**a['plan']),a['model_flight'],log)
    log.close()


def test_discard_replans_without_claiming_a_publication(tmp_path,monkeypatch):
    from refnav.calibration_trial import Observation,NS
    from refnav.core import NAV_OFFBOARD
    from examples.audit_model_flight import audit
    monkeypatch.setattr('examples.gazebo_container.isolated',lambda:True)
    a=raw();log=DecisionLog(tmp_path/'log')
    driver=FullCourseFlight(FullCoursePlan(**a['plan']),a['model_flight'],log)
    driver.selector.monotonic=lambda:0.
    driver.phase='transit';driver._phase_time=(NS,NS)
    commands=[]
    for i in range(12):
        stamp=NS+i*50_000_000
        obs=Observation((0.,0.,-.8),(0.,0.,0.),stamp,stamp,True,False,NAV_OFFBOARD,0.)
        intent=driver.step(stamp,stamp,obs,admitted=True)
        assert intent.position_ned_m is not None
        if i==0:
            assert driver.discard_model_intent(intent,reason='observation_superseded',ros_ns=stamp,elapsed_wall_ns=10_000_000)
            assert driver.model_commands==0 and driver._pending is None
        else:
            driver.published(intent,stamp,stamp//1000,wall_ns=stamp)
            commands.append(dict(timestamp=stamp//1000,position=list(intent.position_ned_m),velocity=[float('nan')]*3,yaw=intent.yaw_rad))
    driver.step(stamp+50_000_000,stamp+50_000_000,obs,admitted=False)
    assert driver.result=='owner_admission_lost'
    log.close()
    rows=[json.loads(line) for line in (tmp_path/'log').read_text().splitlines()]
    result=audit(rows,commands,a['model_flight'])
    assert result['model_command_proof']=='PASS' and result['discarded_decisions']==1
    assert 'unpublished_model_decision' in audit([r for r in rows if r['kind']!='discarded'],commands,a['model_flight'])['faults']
