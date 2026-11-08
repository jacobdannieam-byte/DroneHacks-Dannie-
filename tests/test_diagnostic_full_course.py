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
