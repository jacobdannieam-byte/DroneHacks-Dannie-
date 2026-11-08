from copy import deepcopy
from dataclasses import replace
import json
from pathlib import Path

import pytest

from examples.audit_model_flight import audit
from examples.model_flight_owner import approval_for, geometry_review
from refnav.calibration_trial import CalibrationPlan, Observation, NS
from refnav.core import NAV_OFFBOARD, NAV_AUTO_LAND
from refnav.dynamics import DroneState
from refnav.model_flight import DecisionLog, ModelFlight, ModelSelector, REFERENCE


def profile():
    return json.loads(Path('scenarios/model_flight/h1_pinn_prefix.json').read_text())


def state(n=0.,v=0.):
    return DroneState(REFERENCE,1.,(n,n*.0988077117,-.8),(v,v*.0988077117,0.),0.,0.)


def test_real_checkpoint_drives_selection_not_fixed_route_command():
    config=profile()['model_flight']; selector=ModelSelector(config,monotonic=lambda:0.)
    choices=[]
    for n,v in [(0.,0.),(.20,.1),(.27,.1),(.29,0.)]:
        target,row=selector.choose(state(n,v),config['waypoints_ned_m'][0],NS)
        eligible=[r for r in row['candidates'] if r['pinn']['eligible']]
        assert row['selected']==min(eligible,key=lambda r:(r['pinn']['score'],r['candidate_id']))['candidate_id']
        assert row['model_sha256']==config['checkpoint_sha256']
        choices.append(row['selected'])
    assert len(set(choices))>1
    # Causal intervention at the predictor output changes the command selected.
    original=selector.evaluate
    def changed(model,s,c,g):
        row=original(model,s,c,g)
        if model is selector.model:row['score']=-row['score']
        return row
    before,_=selector.choose(state(),config['waypoints_ned_m'][0],NS)
    selector.evaluate=changed
    after,_=selector.choose(state(),config['waypoints_ned_m'][0],NS)
    assert before.position_m!=after.position_m


def test_rejects_stale_input_bad_checkpoint_and_slow_inference():
    config=profile()['model_flight'];bad=dict(config,checkpoint_sha256='0'*64)
    with pytest.raises(ValueError,match='identity'):ModelSelector(bad)
    selector=ModelSelector(config,monotonic=lambda:0.)
    with pytest.raises(ValueError,match='stale'):selector.choose(state(),config['waypoints_ned_m'][0],NS+100_000_001)
    times=iter([0.,.04]);selector.monotonic=lambda:next(times)
    with pytest.raises(ValueError,match='deadline'):selector.choose(state(),config['waypoints_ned_m'][0],NS)


def test_corridor_prior_and_actual_guard_do_not_enable_registration(tmp_path):
    raw=profile();assert geometry_review(raw)['minimum_remaining_horizontal_margin_m']>0
    raw['model_flight']['registration_enabled']=True
    with pytest.raises(ValueError,match='Registration'):geometry_review(raw)
    raw=profile();log=DecisionLog(tmp_path/'events');driver=ModelFlight(CalibrationPlan(**raw['plan']),raw['model_flight'],log)
    driver.phase='transit';driver._phase_time=(NS,NS)
    obs=Observation((.2,.1,-.8),(0.,0.,0.),NS,NS,True,False,NAV_OFFBOARD,0.)
    intent=driver.step(NS,NS,obs,admitted=True)
    assert intent.command=='land' and driver.model_commands==0
    log.close()


def recorded_decisions(tmp_path):
    raw=profile();config=raw['model_flight'];log=DecisionLog(tmp_path/'decisions.jsonl')
    selector=ModelSelector(config,monotonic=lambda:0.)
    driver=ModelFlight(CalibrationPlan(**raw['plan']),config,log,selector=selector)
    driver.phase='transit';driver._phase_time=(NS,NS);commands=[]
    for i in range(12):
        stamp=NS+i*50_000_000
        obs=Observation((0.,0.,-.8),(0.,0.,0.),stamp,stamp,True,False,NAV_OFFBOARD,0.)
        intent=driver.step(stamp,stamp,obs,admitted=True)
        assert intent.position_ned_m is not None and intent.velocity_ned_mps is None
        driver.published(intent,stamp,stamp//1000,wall_ns=stamp)
        commands.append(dict(timestamp=stamp//1000,position=list(intent.position_ned_m),velocity=[float('nan')]*3,yaw=intent.yaw_rad))
    log.close()
    return [json.loads(line) for line in (tmp_path/'decisions.jsonl').read_text().splitlines()],commands,config


def test_independent_replay_matches_actual_publications_and_detects_tampering(tmp_path):
    rows,commands,config=recorded_decisions(tmp_path)
    assert audit(rows,commands,config)['model_command_proof']=='PASS'
    changed=deepcopy(commands);changed[3]['position'][0]+=.01
    assert 'raw_command_mismatch' in audit(rows,changed,config)['faults']
    assert 'raw_command_count' in audit(rows,commands[:-1],config)['faults']
    changed=deepcopy(rows);changed[0]['selected']='0'
    assert 'model_selection_replay' in audit(changed,commands,config)['faults']
    assert 'decision_log_hash_chain' in audit(changed,commands,config)['faults']


def test_c1_approval_cannot_arm_model_trial(tmp_path):
    p=tmp_path/'approval';p.write_text(json.dumps(dict(mode='calibration_trial',expires_utc='2099-01-01T00:00:00Z')))
    with pytest.raises(ValueError,match='Hybrid'):
        approval_for(p,{},'scenarios/model_flight/h1_recorder.json')


def test_model_fault_lands_without_classical_fallback(tmp_path):
    raw=profile();log=DecisionLog(tmp_path/'events');driver=ModelFlight(CalibrationPlan(**raw['plan']),raw['model_flight'],log)
    driver.phase='transit';driver._phase_time=(NS,NS)
    def reject(*args):raise ValueError('inference_failure')
    driver.selector.choose=reject
    obs=Observation((0.,0.,-.8),(0.,0.,0.),NS,NS,True,False,NAV_OFFBOARD,0.)
    intent=driver.step(NS,NS,obs,admitted=True)
    assert intent.command=='land' and intent.position_ned_m is None
    assert driver.recovery_reason=='model_rejected:inference_failure'
    log.close()
