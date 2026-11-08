"""Incoming sensor frames cannot stand in for completed inspection actions."""
from dataclasses import replace

from examples.h0_hybrid_smoke import smoke_inputs
from examples.h1_fixture import profile
from examples.h1_observation import HybridBagReplay
from refnav.reasoning.inspection import Attempt, candidates


def setup_history():
    inputs = smoke_inputs()
    context, snapshot = inputs['goal'].context, inputs['snapshot']
    replay = HybridBagReplay(profile())
    replay._inspection_history(context, snapshot, context.mission_goal, 'first')
    entity = next(e for e in context.entities
                  if e.entity_id == inputs['goal'].selection.selected_entity_id)
    return replay, context, snapshot, entity, inputs


def select_then_observe(replay, context, snapshot, entity, index):
    replay._inspection_last = (context.reference, context.mission_goal,
                              snapshot.metric.observation_id, entity.entity_id,
                              entity.missing_evidence)
    updated = replace(snapshot, metric=replace(snapshot.metric, observation_id=f'frame-{index}'))
    replay._inspection_history(context, updated, context.mission_goal, 'first')
    return updated


def test_unexecuted_or_refused_selections_do_not_spend_attempts_on_new_frames():
    replay, context, snapshot, entity, inputs = setup_history()
    for index in range(20):
        snapshot = select_then_observe(replay, context, snapshot, entity, index)
    assert replay._inspection_attempts == ()
    assert any(c.entity_id == entity.entity_id for c in
               candidates(context, replace(inputs['goal'].selection.inputs,
                                           attempts=replay._inspection_attempts)))


def test_reduced_missing_evidence_records_acquisition_once():
    replay, context, snapshot, entity, _ = setup_history()
    reduced = replace(entity, missing_evidence=entity.missing_evidence[1:])
    updated_context = replace(context, entities=tuple(
        reduced if e.entity_id == entity.entity_id else e for e in context.entities))
    snapshot = select_then_observe(replay, updated_context, snapshot, entity, 1)
    assert replay._inspection_attempts == (
        Attempt(entity.entity_id, context.context_created_at, 'acquired'),)
    replay._inspection_history(updated_context, snapshot, context.mission_goal, 'first')
    assert len(replay._inspection_attempts) == 1


def test_explicit_failed_attempts_still_exhaust_candidate_until_stage_reset():
    replay, context, snapshot, entity, inputs = setup_history()
    replay._inspection_attempts = (
        Attempt(entity.entity_id, context.context_created_at, 'failed'),) * 3
    select_then_observe(replay, context, snapshot, entity, 1)
    options = candidates(context, replace(inputs['goal'].selection.inputs,
                                          attempts=replay._inspection_attempts))
    assert not any(c.entity_id == entity.entity_id for c in options)
    replay._inspection_history(context, snapshot, context.mission_goal, 'next')
    assert replay._inspection_attempts == ()
