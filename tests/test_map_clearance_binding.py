"""Deployment admission and final-output checks, without ROS or transport."""
from dataclasses import asdict
from hashlib import sha256
import json
from types import SimpleNamespace as S

import pytest

from examples.bind_map_clearance import bind
from examples.h0_hybrid_smoke import smoke_inputs
from examples.hybrid_mission_smoke import runtime_fixture
from refnav.core import Output
from refnav.map_assisted_clearance import alignment_identity, SCOPE


def documents(tmp_path):
    inputs=smoke_inputs()
    reference=asdict(inputs['alignment'].map)
    box=dict(lower_m=[-3.,-3.,-3.],upper_m=[3.,3.,3.])
    survey=dict(schema='dronehacks.participant_clearance_map.v1',permitted_use='controlled_sitl',
                review_id='fixture-only',reference=reference,surveyed_empty_boxes=[box])
    participant=tmp_path/'participant.json'; participant.write_text(json.dumps(survey))
    participant_sha=sha256(participant.read_bytes()).hexdigest()
    certificate=dict(scope=SCOPE,participant_map_sha256=participant_sha,review_id='fixture-only',
        reference=reference,alignment_sha256=alignment_identity(inputs['alignment']),
        bounds_min_m=box['lower_m'],bounds_max_m=box['upper_m'],map_from_prior_yaw_rad=0.,
        prior_origin_in_map_m=[0.,0.,0.],position_error_m=.01,angle_error_rad=.01,
        valid_from_ns=1,valid_until_ns=100_000_000_000,stopping_evidence_sha256='a'*64)
    path=tmp_path/'certificate.json'; path.write_text(json.dumps(certificate))
    settings=dict(certificate_file=path.name,certificate_sha256=sha256(path.read_bytes()).hexdigest(),
        participant_map_file=participant.name,participant_map_sha256=participant_sha)
    runtime=S(adapter=S(simulated_estimator=True),loop=S(replay=S(profile=S(reference=inputs['alignment'].map))),
              map_clearance=None)
    return runtime,settings,participant,path,certificate


def test_binding_pins_sources_but_cannot_create_live_lease(tmp_path):
    runtime,settings,participant,_,_=documents(tmp_path)
    assert bind(runtime,settings,tmp_path) is runtime.map_certificate
    assert runtime.map_clearance is None and runtime.map_source_valid()
    participant.write_text('{}')
    assert runtime.map_source_valid() is False


def test_binding_rejects_changed_certificate_and_missing_positive_survey(tmp_path):
    runtime,settings,participant,path,certificate=documents(tmp_path)
    path.write_text('{}')
    with pytest.raises(ValueError,match='source identity'): bind(runtime,settings,tmp_path)
    survey=json.loads(participant.read_text()); survey['surveyed_empty_boxes']=[]
    participant.write_text(json.dumps(survey))
    settings['participant_map_sha256']=certificate['participant_map_sha256']=sha256(participant.read_bytes()).hexdigest()
    path.write_text(json.dumps(certificate)); settings['certificate_sha256']=sha256(path.read_bytes()).hexdigest()
    with pytest.raises(ValueError,match='positive participant survey'): bind(runtime,settings,tmp_path)


@pytest.mark.parametrize('change', ('survey_frame','prior_yaw','prior_origin'))
def test_binder_rejects_numeric_box_reinterpreted_in_another_frame(tmp_path,change):
    runtime,settings,participant,path,certificate=documents(tmp_path)
    if change=='survey_frame':
        survey=json.loads(participant.read_text()); survey['reference']['frame_id']='different'
        participant.write_text(json.dumps(survey))
        settings['participant_map_sha256']=certificate['participant_map_sha256']=sha256(participant.read_bytes()).hexdigest()
    elif change=='prior_yaw': certificate['map_from_prior_yaw_rad']=.1
    else: certificate['prior_origin_in_map_m']=[.1,0.,0.]
    path.write_text(json.dumps(certificate)); settings['certificate_sha256']=sha256(path.read_bytes()).hexdigest()
    with pytest.raises(ValueError,match='same-frame survey'): bind(runtime,settings,tmp_path)


def test_final_fence_checks_actual_output_and_original_launch_phase():
    runtime=runtime_fixture(strategy='hybrid_shadow')
    calls=[]
    runtime.launch_guard=S(check_output=lambda *args,**kw: calls.append((args[3],kw['phase'])) or 'exact_target_rejected')
    runtime._publication_phase='hover'
    out=Output('go',(1.,2.,-1.,0.),stream=True)
    assert runtime.publication_reason(S(),S(),out,1_000_000_000)=='exact_target_rejected'
    assert calls==[(out,'hover')]


def test_final_fence_never_converts_profile_to_lease_or_landing_authority(monkeypatch):
    runtime=runtime_fixture(strategy='hybrid_shadow')
    runtime.map_certificate=object()
    out=Output('go',(0.,0.,-1.,0.),stream=True)
    assert runtime.publication_reason(S(),S(),out,1_000_000_000)=='map_clearance_lease_unavailable'
    monkeypatch.setattr(runtime,'_map_inputs',lambda inputs,now:(None,{}))
    landing=Output('land',commands=['land'])
    assert runtime.publication_reason(S(),S(),landing,1_000_000_000)=='map_assisted_recovery_unqualified'
    runtime.recovery_guard=lambda *args:'recovery_geometry_rejected'
    assert runtime.publication_reason(S(),S(),landing,1_000_000_000)=='recovery_geometry_rejected'


def test_final_go_fence_passes_exact_target_to_stopping_check(monkeypatch):
    import refnav.hybrid_integration as integration
    import examples.hybrid_runtime as runtime_module
    runtime=runtime_fixture(strategy='hybrid_shadow')
    runtime.map_certificate=object()
    inputs=smoke_inputs()
    inputs.update(state_received_ns=1,estimator_reset_counter=0,estimator_healthy=True)
    monkeypatch.setattr(runtime,'_map_inputs',lambda old,now:(None,inputs))
    monkeypatch.setattr(integration,'_input_reason',lambda *args:None)
    seen=[]
    envelope=object()
    runtime.executor.stopping_envelope=envelope
    def sweep(state,target,snapshot,alignment,bounds,now_ns,*,stopping_envelope):
        seen.append((state,target,snapshot,alignment,bounds,now_ns,stopping_envelope))
        return 'outside_tube'
    monkeypatch.setattr(runtime_module,'target_sweep',sweep)
    out=Output('go',(1.,2.,-1.,.2),stream=True)
    assert runtime.publication_reason(S(),S(),out,1_000_000_000)=='outside_tube'
    assert seen==[(inputs['state'],(1.,2.,-1.),inputs['snapshot'],inputs['alignment'],
                   inputs['vehicle'],1_000_000_000,envelope)]


def test_post_verifier_freshness_rejects_elapsed_state_and_both_lease_expiries():
    from refnav.trajectory_evaluation import state_capture_ns
    from refnav.map_assisted_clearance import MapAssistedClearance
    from dataclasses import replace
    runtime=runtime_fixture(strategy='hybrid_shadow')
    inputs=smoke_inputs(); captured=state_capture_ns(inputs['state'],inputs['alignment'])
    inputs['snapshot']=replace(inputs['snapshot'],metric=replace(inputs['snapshot'].metric,
        capture_ns=captured,received_ns=captured,query_ns=captured))
    runtime.map_certificate=object(); runtime._publication_inputs=inputs
    wall=[100]
    runtime.map_clearance=S(observation_reason=MapAssistedClearance.observation_reason,
        monotonic_ns=lambda:wall[0],lease=S(valid_from_ns=captured,
        valid_until_ns=captured+50_000_001,wall_from_ns=100,wall_until_ns=200))
    assert runtime.publication_freshness_reason(captured+50_000_000) is None
    assert runtime.publication_freshness_reason(captured+50_000_001)=='publication_state_stale'
    runtime.map_clearance.lease.valid_until_ns=captured+20
    assert runtime.publication_freshness_reason(captured+20)=='publication_clearance_expired'
    wall[0]=200
    assert runtime.publication_freshness_reason(captured)=='publication_clearance_expired'
    wall[0]=100
    metric=inputs['snapshot'].metric
    inputs['snapshot']=replace(inputs['snapshot'],metric=replace(metric,capture_ns=captured-250_000_001,
        received_ns=captured-250_000_001))
    # State and both leases remain fresh; only retained depth has expired.
    assert runtime.publication_freshness_reason(captured)=='prior_live_observation_unavailable'
