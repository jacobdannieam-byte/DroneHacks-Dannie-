import copy
from dataclasses import FrozenInstanceError, replace
import json

import pytest

from examples.semantic_world_model_demo import observation
from refnav.reasoning import (ContextConfig, ContractError, MissionGoal, ReasoningContext,
                              WorldReference, build_context)
from refnav.world_model import WorldState


def test_context_preserves_ids_geometry_provenance_and_unknowns(reasoning_snapshot, reasoning_context):
    context = reasoning_context
    assert context.context_valid
    assert {e.entity_id for e in context.entities} == {e['entity_id'] for e in reasoning_snapshot['entities']}
    opening = next(e for e in context.entities if e.semantic_type == 'opening')
    assert opening.geometry.height_m is None
    assert opening.traversability == 'unknown'
    assert opening.connected_regions == ()
    assert len(opening.relations) == 2
    assert all(r.relation_type == 'bounded_by' for r in opening.relations)
    assert opening.evidence[0].observation_id == 'frame_0.200000'
    assert opening.evidence[0].position_sigma_m == .02
    assert {'upper_boundary', 'full_volume_coverage', 'validated_swept_path'} <= set(opening.missing_evidence)


def test_nested_contracts_are_immutable_and_detached(reasoning_snapshot, reasoning_context):
    assert not hasattr(reasoning_context, '__dict__')
    before = reasoning_context.to_json()
    reasoning_snapshot['entities'][0]['geometry']['center'][0] = 1000.
    assert reasoning_context.to_json() == before
    with pytest.raises(FrozenInstanceError):
        reasoning_context.entities[0].geometry.width_m = 3.
    with pytest.raises(TypeError):
        reasoning_context.entities[0].geometry.center[0] = 3.


def test_serialization_roundtrip_and_json_duplicate_keys(reasoning_context):
    assert ReasoningContext.from_json(reasoning_context.to_json()) == reasoning_context
    with pytest.raises(ContractError, match='duplicate_json_key'):
        ReasoningContext.from_json('{"schema_version":1,"schema_version":1}')
    doc = reasoning_context.to_dict()
    doc['entities'][0]['geometry']['width_m'] += .1
    with pytest.raises(ContractError, match='context_hash_mismatch'):
        ReasoningContext.from_json(json.dumps(doc))


def test_missing_empty_and_uninitialized_world():
    ref = WorldReference()
    assert not build_context(None, now=0., reference=ref).context_valid
    world = WorldState()
    assert 'missing_world_timestamp' in build_context(world, now=0., reference=ref).invalid_reasons
    world.tick(0.)
    empty = build_context(world, now=0., reference=ref)
    assert empty.context_valid and empty.entities == ()


@pytest.mark.parametrize('key,value,reason', [
    ('schema_version',2,'unsupported_world_schema'), ('schema_version',True,'unsupported_world_schema'),
    ('frame_id','map','world_reference_mismatch'), ('epoch',1,'world_reference_mismatch'),
    ('clock_id','system','world_reference_mismatch'), ('reset_required',True,'world_reference_reset'),
    ('timestamp',-5.,'stale_or_future_world_snapshot'), ('timestamp',1.,'stale_or_future_world_snapshot'),
    ('timestamp',None,'missing_world_timestamp')])
def test_invalid_world_metadata_is_fail_closed(reasoning_snapshot, key, value, reason):
    reasoning_snapshot[key] = value
    context = build_context(reasoning_snapshot, now=.2, reference=WorldReference())
    assert context.invalid_reasons == (reason,)
    assert context.entities == ()


def test_entity_reference_duplicate_ids_and_dangling_relations_rejected(reasoning_snapshot):
    doc = copy.deepcopy(reasoning_snapshot)
    doc['entities'][0]['epoch'] = 3
    assert 'entity_reference_mismatch' in build_context(doc, now=.2, reference=WorldReference()).invalid_reasons
    doc = copy.deepcopy(reasoning_snapshot)
    doc['entities'].append(copy.deepcopy(doc['entities'][0]))
    assert 'duplicate_or_invalid_entity_id' in build_context(doc, now=.2, reference=WorldReference()).invalid_reasons
    doc = copy.deepcopy(reasoning_snapshot)
    next(e for e in doc['entities'] if e['semantic_type'] == 'opening')['relations'][0]['target'] = 'invented_wall'
    assert 'ungrounded_relationship' in build_context(doc, now=.2, reference=WorldReference()).invalid_reasons


@pytest.mark.parametrize('change,reason', [
    ({'traversability':'safe'},'unsafe_traversability_claim'),
    ({'is_safe':True},'unsafe_traversability_claim'),
    ({'connected_regions':['exit']},'unsupported_connectivity'),
    ({'geometric_confidence':2.},'invalid_confidence'),
    ({'status':'invalidated','contradictions':1},'missing_blocking_evidence'),
    ({'relations':[{'type':'connected_to','target':'exit','status':'verified'}]},'unsupported_relationship')])
def test_unsupported_semantic_promotions_rejected(reasoning_snapshot, change, reason):
    opening = next(e for e in reasoning_snapshot['entities'] if e['semantic_type'] == 'opening')
    opening.update(change)
    assert reason in build_context(reasoning_snapshot, now=.2, reference=WorldReference()).invalid_reasons


def test_stale_entity_is_distinct_from_stale_context(reasoning_replay):
    doc = reasoning_replay[4]['snapshot']
    context = build_context(doc, now=6.5, reference=WorldReference())
    assert context.context_valid
    assert next(e for e in context.entities if e.semantic_type == 'opening').status == 'stale'
    expired = build_context(doc, now=8., reference=WorldReference())
    assert not expired.context_valid


def test_semantic_api_matches_snapshot_subset_without_mutation_or_serialization(monkeypatch):
    world = WorldState()
    world.process(observation(), 0.)
    before = world.to_json()
    normal = world.snapshot(.2)
    compact = world.semantic_snapshot(.2)
    assert compact['timestamp'] == 0. and compact['query_timestamp'] == .2
    for summary in compact['entities']:
        original = next(e for e in normal['entities'] if e['entity_id'] == summary['entity_id'])
        for key in ('geometry','status','age_s','geometric_confidence','semantic_confidence'):
            assert summary[key] == original[key]
    compact['entities'][0]['geometry']['center'][0] = 100.
    assert world.to_json() == before
    monkeypatch.setattr(world, 'snapshot', lambda *args: pytest.fail('full snapshot used'))
    monkeypatch.setattr(world, 'to_json', lambda: pytest.fail('world serialized'))
    assert build_context(world, now=.2, reference=WorldReference()).context_valid


def test_entity_and_byte_caps_keep_explicit_omissions(reasoning_snapshot):
    config = ContextConfig(max_entities=1, max_context_bytes=4096)
    context = build_context(reasoning_snapshot, now=.2, reference=WorldReference(), config=config)
    assert context.context_valid and len(context.entities) == 1
    assert context.entities[0].semantic_type == 'opening'
    assert context.entities[0].relations == ()
    assert 'supporting_entities_omitted' in context.entities[0].missing_evidence
    assert context.omitted_entities == 2
    assert len(context.to_json().encode()) <= config.max_context_bytes
    small = build_context(reasoning_snapshot, now=.2, reference=WorldReference(), config=ContextConfig(max_context_bytes=1024))
    assert len(small.to_json().encode()) <= 1024
    assert small.entities == () and small.omitted_entities == 3


def test_bound_context_deterministic_for_permuted_snapshot(reasoning_snapshot):
    first = build_context(reasoning_snapshot, now=.2, reference=WorldReference())
    reasoning_snapshot['entities'].reverse()
    assert build_context(reasoning_snapshot, now=.2, reference=WorldReference()) == first


def test_context_size_limits_and_unknown_mission_targets(reasoning_snapshot):
    with pytest.raises(ContractError):
        ContextConfig(max_context_bytes=100)
    with pytest.raises(ContractError, match='wire_size_exceeded'):
        ReasoningContext.from_json(' ' * 65537)
    context = build_context(reasoning_snapshot, MissionGoal(target_entity_id='invented'), now=.2, reference=WorldReference())
    assert context.invalid_reasons == ('unknown_mission_target',)


def test_reset_latch_invalidates_context_but_preserves_old_snapshot_contract():
    world = WorldState()
    world.process(observation(), 0.)
    old = world.snapshot()
    world.process(observation(.1, epoch=1), .1)
    assert not build_context(world, now=.1, reference=WorldReference()).context_valid
    assert old['reset_required'] is False
    world.reset_reference('odom', 1)
    world.process(observation(0., epoch=1), 0.)
    assert not build_context(world, now=0., reference=WorldReference()).context_valid
    assert build_context(world, now=0., reference=WorldReference(epoch=1)).context_valid


def test_unknown_height_and_volume_do_not_become_known_from_confirmation_label(reasoning_snapshot):
    opening = next(e for e in reasoning_snapshot['entities'] if e['semantic_type'] == 'opening')
    opening.update(status='confirmed',missing_evidence=[])
    context = build_context(reasoning_snapshot,now=.2,reference=WorldReference())
    candidate = next(e for e in context.entities if e.semantic_type == 'opening')
    assert candidate.geometry.height_m is None and candidate.traversability == 'unknown'
    assert {'upper_boundary','full_volume_coverage'} <= set(candidate.missing_evidence)


def test_numeric_boolean_and_nonfinite_confidence_rejected(reasoning_snapshot):
    for value in (True,float('nan'),float('inf')):
        doc = copy.deepcopy(reasoning_snapshot)
        doc['entities'][0]['geometric_confidence'] = value
        assert not build_context(doc,now=.2,reference=WorldReference()).context_valid
