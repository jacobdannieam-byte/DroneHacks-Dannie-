import copy

from refnav.reasoning import DeterministicReasoner, MissionGoal, WorldReference, build_context, validate_proposal


def test_replay_decisions_reuse_id_and_reflect_new_evidence(reasoning_replay):
    proposals, contexts = [], []
    for frame in reasoning_replay:
        context = build_context(frame['snapshot'], now=frame['snapshot']['timestamp'], reference=WorldReference())
        proposal = DeterministicReasoner().propose(context)
        contexts.append(context)
        proposals.append(proposal)
        assert validate_proposal(proposal, context, now=context.context_created_at, reference=WorldReference()).valid
    assert [p.intent for p in proposals] == ['insufficient_information', *['observe_more'] * 5, 'reject_blocked_candidate']
    assert {p.target_entity_id for p in proposals[1:]} == {'opening_0003'}
    assert 'reobserve_stale_belief' in proposals[4].reason_codes
    assert 'blocking_evidence' in proposals[-1].reason_codes
    assert contexts[3].entities[0].last_observed == .4
    assert len({p.proposal_id for p in proposals}) == 7


def test_identical_context_produces_identical_proposal(reasoning_context):
    reasoner = DeterministicReasoner()
    assert reasoner.propose(reasoning_context) == reasoner.propose(reasoning_context)
    assert reasoner.propose(reasoning_context).target_entity_id == 'opening_0003'


def test_no_fabricated_geometry_connectivity_or_commands(reasoning_context):
    proposal = DeterministicReasoner().propose(reasoning_context)
    assert all(e.connected_regions == () for e in reasoning_context.entities)
    assert next(e for e in reasoning_context.entities if e.semantic_type == 'opening').geometry.height_m is None
    assert {'upper_boundary','full_volume_coverage','validated_swept_path'} <= set(proposal.missing_information)
    assert not any(k in proposal.to_dict() for k in ('setpoint','trajectory','velocity','vehicle_command'))


def test_invalid_context_returns_only_refusal(reasoning_snapshot):
    reasoning_snapshot['epoch'] = 2
    context = build_context(reasoning_snapshot, now=.2, reference=WorldReference())
    proposal = DeterministicReasoner().propose(context)
    assert proposal.intent == 'insufficient_information'
    assert proposal.target_entity_id is None and not proposal.source_entity_ids


def test_multiple_candidates_rank_evidence_with_deterministic_tie_break(reasoning_snapshot):
    doc = reasoning_snapshot
    original = next(e for e in doc['entities'] if e['semantic_type'] == 'opening')
    other = copy.deepcopy(original)
    other.update(entity_id='opening_0008', observations=5)
    other['geometry']['center'][1] += 3.
    doc['entities'].append(other)
    first = build_context(doc, now=.2, reference=WorldReference())
    proposal = DeterministicReasoner().propose(first)
    assert proposal.target_entity_id == 'opening_0008'
    assert 'alternatives_available' in proposal.reason_codes
    assert proposal.source_entity_ids == ('opening_0003','opening_0008')
    doc['entities'].reverse()
    assert DeterministicReasoner().propose(build_context(doc, now=.2, reference=WorldReference())) == proposal


def test_known_mission_target_changes_ranking_without_inventing_region_link(reasoning_snapshot):
    original = next(e for e in reasoning_snapshot['entities'] if e['semantic_type'] == 'opening')
    other = copy.deepcopy(original)
    other.update(entity_id='opening_0009', observations=8)
    reasoning_snapshot['entities'].append(other)
    context = build_context(reasoning_snapshot, MissionGoal(target_entity_id=original['entity_id']), now=.2,
                            reference=WorldReference())
    proposal = DeterministicReasoner().propose(context)
    assert proposal.target_entity_id == original['entity_id']
    assert 'mission_target_known' in proposal.reason_codes
    assert 'region_connectivity' in proposal.missing_information
