"""Synthetic authority and H1 fixtures. No motion or measured stopping evidence."""

from dataclasses import asdict, replace
from hashlib import sha256
import json
import math

import numpy as np
import pytest

from examples.h0_hybrid_smoke import smoke_inputs
from refnav.core import NAV_OFFBOARD, Output, Vehicle
from refnav.hybrid_execution import HeldExecutor
from refnav.hybrid_gateway import target_sweep
from refnav.hybrid_integration import assess
from refnav.hybrid_mission import map_position
from refnav.map_assisted_clearance import (
    AUTHORIZED_STATIC, SCOPE, ClearanceCertificate, ExclusionLease, MapAssistedClearance,
    alignment_identity, clearance_sweep,
)
from refnav.sitl_review import validate_receipt
from refnav.stopping_envelope import StoppingEnvelope
from refnav.trajectory_evaluation import MetricGoal
from refnav.world_model.spatial_input import NS
from refnav.world_model.spatial_volume import Cell


@pytest.fixture
def case():
    inputs = smoke_inputs()
    center = map_position(inputs['state'], inputs['alignment'])
    snapshot = inputs['snapshot']
    cells = np.zeros_like(snapshot.metric._cells)
    metric = replace(snapshot.metric, _cells=cells, _stamps=np.full_like(snapshot.metric._stamps, inputs['now_ns']))
    inputs.update(snapshot=replace(snapshot, metric=metric), goal=None,
                  objective=MetricGoal(metric.reference, (center[0] + .18, *center[1:])))
    envelope = StoppingEnvelope(.08, .05, .5, .10, .08, 'a' * 64)
    cert = dict(scope=SCOPE, participant_map_sha256='b' * 64, review_id='synthetic_fixture_only',
                reference=asdict(metric.reference), alignment_sha256=alignment_identity(inputs['alignment']),
                bounds_min_m=(-1., -1., -1.), bounds_max_m=(1., 1., 1.), map_from_prior_yaw_rad=0.,
                prior_origin_in_map_m=center, position_error_m=.01, angle_error_rad=.01,
                valid_from_ns=0, valid_until_ns=2 * NS, stopping_evidence_sha256=envelope.evidence_sha256)
    return inputs, cert, envelope


def certificate(fields):
    raw = json.dumps(fields).encode()
    return ClearanceCertificate.from_json(raw, expected_sha256=sha256(raw).hexdigest())


def clearance(fields, envelope, *, verify=lambda _: True, wall=lambda: NS):
    cert = certificate(fields)
    lease = ExclusionLease('synthetic-lease', cert.certificate_sha256, 0, NS, 0, 2 * NS)
    return MapAssistedClearance(cert, lease, verify, stopping_envelope=envelope, monotonic_ns=wall)


def test_prior_does_not_relabel_or_mutate_unknown_observations(case):
    inputs, fields, envelope = case
    prior = clearance(fields, envelope)
    raw = inputs['snapshot'].metric
    assert assess(**inputs).validation_reason == 'no_verified_progress'
    attached = prior.attach(inputs)
    assert attached['snapshot'].metric is raw
    assert attached['snapshot'].semantic is inputs['snapshot'].semantic
    result = assess(**attached)
    assert result.validation_reason == 'eligible_for_offline_review'
    assert result.receipt.clearance_identity == prior.identity
    plan = result.receipt.plan
    winner = next(row for row in plan.evaluation.candidates if row.candidate_id == plan.ranked_ids[0])
    assert winner.evidence == AUTHORIZED_STATIC and winner.prior_backed_segments > 0
    assert winner.predicted_peak_speed_mps <= envelope.max_speed_mps
    assert np.all(raw.cells_at(inputs['now_ns'], raw.reference) == Cell.UNKNOWN)


@pytest.mark.parametrize('hazard', ['occupied', 'scope', 'reset', 'stale', 'ros_expiry', 'wall_expiry', 'intrusion'])
def test_prior_refuses_unsafe_evidence(case, hazard):
    inputs, fields, envelope = case
    if hazard == 'scope':
        # The raw rolling grid may have free observations outside the reviewed prior.
        fields['bounds_max_m'] = (.15, 1., 1.)
        inputs['snapshot'].metric._cells[:] = Cell.FREE
    if hazard == 'occupied':
        raw = inputs['snapshot'].metric
        center = fields['prior_origin_in_map_m']
        index = np.floor(np.array(center) / raw.policy.resolution_m).astype(int) - raw.origin_index
        raw._cells[tuple(index)] = Cell.OCCUPIED
    prior = clearance(fields, envelope, verify=lambda _: hazard != 'intrusion',
                      wall=lambda: 2 * NS if hazard == 'wall_expiry' else NS)
    if hazard == 'reset':
        inputs.update(alignment=replace(inputs['alignment'], estimator_reset_counter=1), estimator_reset_counter=1)
    if hazard == 'stale':
        inputs['snapshot'] = replace(inputs['snapshot'], metric=replace(inputs['snapshot'].metric, capture_ns=0))
        inputs['now_ns'] = 300_000_001
    if hazard == 'ros_expiry':
        prior = replace(prior, lease=replace(prior.lease, valid_until_ns=inputs['now_ns']))
    attached = prior.attach(inputs)
    result = assess(**attached)
    assert result.validation_reason != 'eligible_for_offline_review'
    assert not result.receipt.plan.ranked_ids
    if hazard in ('occupied', 'scope'):
        rows = result.receipt.plan.evaluation.candidates
        expected = 'occupied_swept_volume' if hazard == 'occupied' else 'outside_certified_volume'
        assert all(expected in row.reasons for row in rows)


def test_certificate_hash_transform_and_whole_stopping_sweep(case):
    inputs, fields, envelope = case
    with pytest.raises(ValueError, match='bytes do not match'):
        ClearanceCertificate.from_json(json.dumps(fields).encode(), expected_sha256='0' * 64)
    # The surveyed long axis points map +X only after applying the explicit +90° rotation.
    fields.update(bounds_min_m=(-.5, -.62, -.5), bounds_max_m=(.5, .50, .5),
                  map_from_prior_yaw_rad=math.pi / 2, position_error_m=.02)
    prior = clearance(fields, envelope)
    snapshot = prior.attach(inputs)['snapshot']
    center = fields['prior_origin_in_map_m']
    end = (center[0] + .48, *center[1:])
    sweep = clearance_sweep(snapshot, (center, end), alignment=inputs['alignment'], now_ns=inputs['now_ns'],
                            vehicle_radius_m=.08, margin_m=.01)
    assert sweep.status == AUTHORIZED_STATIC and sweep.observed_free_cells == 0
    state, bounds, alignment, now = (inputs[key] for key in ('state', 'vehicle', 'alignment', 'now_ns'))
    target = (state.position_m[0] + .48, *state.position_m[1:])
    assert envelope.check(state, snapshot, alignment, bounds, now) is None
    assert envelope.check(state, snapshot, alignment, bounds, now, target=target) == (
        'stopping_volume_unverified:outside_certified_volume')
    assert target_sweep(state, target, snapshot, alignment, bounds, now) == (
        'target_sweep_rejected:outside_certified_volume')


def test_receipt_and_held_target_are_bound_to_current_lease(case):
    inputs, fields, envelope = case
    monitoring = {'clear': True}
    prior = clearance(fields, envelope, verify=lambda _: monitoring['clear'])
    attached = prior.attach(inputs)
    result = assess(**attached)
    now, state, alignment = (inputs[key] for key in ('now_ns', 'state', 'alignment'))
    review_inputs = dict(state=state, alignment=alignment, metric=inputs['snapshot'].metric, now_ns=now)
    assert validate_receipt(result.receipt, **review_inputs) == 'proposal_clearance_changed'
    changed = replace(prior, lease=replace(prior.lease, wall_until_ns=prior.lease.wall_until_ns - 1))
    assert validate_receipt(result.receipt, **review_inputs, clearance=changed) == 'proposal_clearance_changed'
    executor = HeldExecutor(enabled=True, monotonic=lambda: 0.)
    executor.stopping_envelope = envelope
    pose = (*state.position_m, state.yaw_rad)
    owner = dict(key=(0, 0), now_ns=now, owner_output=Output('go', pose, stream=True),
                 vehicle=Vehicle(pos=state.position_m, heading=state.yaw_rad, pos_ok=True, armed=True, landed=False,
                                 nav_state=NAV_OFFBOARD), ready=True, generation=1)
    first = executor.step(inputs=attached, assessment=result, **owner)
    assert first.reason == 'hybrid_position_selected' and first.setpoint != pose
    assert executor.step(inputs=attached, assessment=None, **owner).reason == 'held_target_revalidated'
    assert executor.step(inputs=changed.attach(inputs), assessment=None, **owner).setpoint == pose
    assert executor.held is None
    assert executor.step(inputs=attached, assessment=result, **owner).setpoint == first.setpoint
    monitoring['clear'] = False
    assert validate_receipt(result.receipt, **review_inputs, clearance=prior) == 'prior_exclusion_unverified'
    decision = executor.step(inputs=attached, assessment=None, **owner)
    assert decision.reason == 'prior_exclusion_unverified' and decision.setpoint is None
    assert executor.held is None


def test_observed_only_behavior_and_speed_gate(case):
    inputs, fields, envelope = case
    state = inputs['state']
    raw = inputs['snapshot'].metric
    raw._cells[:] = Cell.FREE
    assert assess(**inputs).validation_reason == 'eligible_for_offline_review'
    prior = clearance(fields, envelope)
    inputs['state'] = replace(state, velocity_mps=(.51, 0., 0.))
    result = assess(**prior.attach(inputs))
    assert result.validation_reason == 'no_verified_progress'
    assert all(row.reasons == ('stopping_speed_outside_measurement',)
               for row in result.receipt.plan.evaluation.candidates)


def test_lease_expiring_inside_verification_refuses(case):
    inputs, fields, envelope = case
    readings = iter((NS, 2 * NS))
    prior = clearance(fields, envelope, wall=lambda: next(readings))
    assert prior.reason(inputs['alignment'], inputs['now_ns']) == 'prior_wall_lease_expired_or_future'


def test_direct_sweep_cannot_override_wrong_live_metric_reference(case):
    inputs, fields, envelope = case
    prior = clearance(fields, envelope)
    snapshot = prior.attach(inputs)['snapshot']
    metric = snapshot.metric
    wrong_metric = replace(metric, reference=replace(metric.reference, epoch=metric.reference.epoch + 1))
    sweep = clearance_sweep(replace(snapshot, metric=wrong_metric), (fields['prior_origin_in_map_m'],),
                            alignment=inputs['alignment'], now_ns=inputs['now_ns'],
                            vehicle_radius_m=.08, margin_m=.01)
    assert sweep.status == 'unknown' and sweep.reason == 'prior_live_observation_unavailable'
