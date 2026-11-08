"""Synthetic policy checks; observation waits never authorize flight."""
from copy import deepcopy
from dataclasses import replace

import pytest

from examples.h0_hybrid_smoke import smoke_inputs
from examples.unified_spatial_replay import calibration
from refnav.hybrid_integration import assess
from refnav.hybrid_mission import CourseMission, Stage
from refnav.reasoning import build_context
from refnav.reasoning.inspection import make_inputs
from refnav.reasoning.selection import create_reasoner
from refnav.trajectory_evaluation import GoalBinding
from refnav.world_model.spatial_volume import Cell


def reobservation_inputs():
    inputs = smoke_inputs()
    old = inputs['goal']
    camera = old.selection.inputs
    source = deepcopy(inputs['snapshot'].semantic)
    for entity in source['entities']:
        if entity['entity_id'] == old.proposal.target_entity_id:
            entity['geometry']['center'] = [p + 1.2 * row[2] for p, row in
                                           zip(camera.position_metres, camera.rotation_map_optical)]
    snapshot = replace(inputs['snapshot'], semantic=source)
    context = build_context(source, now=old.context.context_created_at, reference=old.context.reference)
    auxiliary = make_inputs(context, snapshot, inputs['state'], inputs['alignment'], calibration())
    proposal, selection = create_reasoner().select(context, auxiliary)
    return inputs | {'snapshot': snapshot, 'goal': GoalBinding(context, proposal, selection)}


def test_verified_stationary_wait_has_no_motion_ids_or_progress():
    result = assess(**reobservation_inputs())
    plan = result.receipt.plan
    assert plan.decision == 'observation_wait_requested'
    assert result.validation_reason == 'observation_wait_verified'
    assert not plan.ranked_ids
    hold = next(row for row in plan.evaluation.candidates if row.candidate_id == 'hold')
    assert hold.progress_m == 0 and hold.evidence == 'verified_observed_volume'
    assert plan.deadline_monotonic_s is not None


@pytest.mark.parametrize('failure', ['stale', 'unhealthy', 'forged', 'unknown', 'occupied',
                                    'insufficient_clearance', 'cancelled', 'deadline'])
def test_invalid_observation_cannot_request_verified_wait(failure):
    inputs = reobservation_inputs()
    if failure == 'stale':
        inputs['now_ns'] += 400_000_000
    elif failure == 'unhealthy':
        inputs['estimator_healthy'] = False
    elif failure == 'forged':
        inputs['goal'] = replace(inputs['goal'], selection=replace(
            inputs['goal'].selection, selected_entity_id='forged'))
    elif failure in ('unknown', 'occupied'):
        snapshot = inputs['snapshot']
        cells = snapshot.metric._cells.copy()
        cells[:] = Cell.UNKNOWN if failure == 'unknown' else Cell.OCCUPIED
        cells.setflags(write=False)
        inputs['snapshot'] = replace(snapshot, metric=replace(snapshot.metric, _cells=cells))
    elif failure == 'insufficient_clearance':
        inputs['vehicle'] = replace(inputs['vehicle'], radius_m=1.5)
    elif failure == 'deadline':
        ticks = iter((0., .1))
        inputs['monotonic'] = lambda: next(ticks, .1)
    else:
        inputs['cancelled'] = lambda: True
    result = assess(**inputs)
    assert result.receipt.plan.decision == 'safe_stop_requested'
    assert result.validation_reason != 'observation_wait_verified'


def test_wait_does_not_complete_checkpoint_or_renew_observation_deadline():
    inputs = reobservation_inputs()
    stage = Stage('observe', 'landing', inputs['objective'], landing_site_review='fixture')
    mission = CourseMission((stage,), observation_s=.25)
    now = inputs['now_ns']
    assert mission.select(now, inputs, inputs['goal']) is not None
    for elapsed in (0, 100_000_000, 249_999_999):
        # Exercise production select/result cadence with distinct frame IDs.
        snapshot = inputs['snapshot']
        frame = replace(snapshot, metric=replace(snapshot.metric, observation_id='wait-' + str(elapsed)))
        mission.select(now + elapsed, inputs | {'snapshot': frame}, inputs['goal'])
        assert mission._unavailable == now
        mission.result(now + elapsed, 'observation_wait_verified')
        assert mission.state == 'OBSERVATION_WAIT'
        assert mission.index == 0 and mission.land_reason is None
        assert mission._arrival_hits == 0 and mission._failures == 0
    mission.result(now + 250_000_000, 'observation_wait_verified')
    assert mission.state == 'MISSION_ABORT'
    assert mission.land_reason == 'inspection_exhausted'
