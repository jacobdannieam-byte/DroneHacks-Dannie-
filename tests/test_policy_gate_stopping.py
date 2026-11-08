from dataclasses import replace
import math
import pytest
from refnav.stopping_envelope import StoppingEnvelope
from examples.hybrid_mission_smoke import runtime_fixture
from test_hybrid_runtime import start_navigation

def envelope(**changes):
    fields=dict(horizontal_radius_m=.16,half_height_m=.1,max_speed_mps=.5,
                reaction_s=.1,braking_distance_m=.1,evidence_sha256='a'*64)
    fields.update(changes)
    return StoppingEnvelope(**fields)

def test_body_encloses_height_and_stop_includes_reaction():
    value=envelope()
    assert value.body_radius_m==pytest.approx(math.hypot(.16,.1))
    assert value.stop_distance_m==pytest.approx(.15)

@pytest.mark.parametrize('change',[{'max_speed_mps':.51},{'reaction_s':float('nan')},
    {'half_height_m':0},{'braking_distance_m':-1},{'braking_distance_m':3.}, {'evidence_sha256':'unmeasured'}])
def test_unmeasured_or_invalid_bounds_refused(change):
    with pytest.raises(ValueError): envelope(**change)

def test_actual_executor_applies_stop_volume_on_first_proposal():
    runtime=runtime_fixture()
    runtime.executor.stopping_envelope=envelope(braking_distance_m=1.5)
    sink=start_navigation(runtime)
    assert not any(row['setpoint'] and row['phase']=='go' for row in sink.messages)
    assert any(row['event']=='plan' and row['reason']=='eligible_for_offline_review' for row in runtime.records)
    assert any(row['event']=='owner_decision' and row['reason']=='execution_sweep:unobserved_swept_volume'
               for row in runtime.records)

def test_native_activation_refuses_missing_stopping_measurement(tmp_path, monkeypatch):
    from examples.gazebo_hybrid_adapter import IsolationApproval
    from examples.hybrid_launch import run_reviewed_sitl
    runtime=runtime_fixture(); monkeypatch.setenv('ROS_DOMAIN_ID','23')
    with pytest.raises(ValueError,match='measured stopping envelope'):
        run_reviewed_sitl(runtime,IsolationApproval('fixture',23,'offline',lambda *_:True),
                          ros_args=(),output=tmp_path/'not-created.json')
    assert not (tmp_path/'not-created.json').exists()
