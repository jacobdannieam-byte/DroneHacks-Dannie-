from dataclasses import replace

import pytest

from refnav.reasoning import (ContractError, DeterministicReasoner, GoalProposal, WorldReference,
                              build_context, validate_proposal)
from refnav.reasoning.schema import make_proposal


def test_validator_alone_certifies_only_non_executable_result(reasoning_context):
    proposal = DeterministicReasoner().propose(reasoning_context)
    assert proposal.validation_status == 'unvalidated'
    result = validate_proposal(proposal, reasoning_context, now=.2, reference=WorldReference())
    assert result.valid and result.proposal.validation_status == 'valid_non_executable'
    assert GoalProposal.from_json(result.proposal.to_json()) == result.proposal
    assert proposal.validation_status == 'unvalidated'


@pytest.mark.parametrize('key,value,reason', [
    ('target_entity_id','invented','unknown_target_entity'), ('schema_version',2,'unsupported_proposal_schema'),
    ('intent','approach','unsupported_intent'), ('intent','arm','unsupported_intent'),
    ('context_reference','ctx_old','outdated_context_reference'), ('proposal_id','fake','proposal_hash_mismatch'),
    ('reason_codes',['height_confirmed'],'unsupported_reason_code'),
    ('missing_information',[],'missing_evidence_suppressed'),
    ('source_entity_ids',['invented'],'unknown_source_entity')])
def test_adversarial_proposal_fields_fail_closed(reasoning_context, key, value, reason):
    doc = DeterministicReasoner().propose(reasoning_context).to_dict()
    doc[key] = value
    result = validate_proposal(doc, reasoning_context, now=.2, reference=WorldReference())
    assert not result.valid and result.proposal is None
    assert reason in result.reason_codes


@pytest.mark.parametrize('field', ['setpoint','trajectory','vehicle_command','velocity','yaw_rate','is_safe'])
def test_no_executable_extra_payload_fields(reasoning_context, field):
    doc = DeterministicReasoner().propose(reasoning_context).to_dict()
    doc[field] = [1.,2.,3.]
    assert 'unknown_or_missing_fields' in validate_proposal(doc, reasoning_context, now=.2,
                                                          reference=WorldReference()).reason_codes


def test_nested_extra_payload_and_self_asserted_validation_do_not_bypass_guard(reasoning_context):
    doc = DeterministicReasoner().propose(reasoning_context).to_dict()
    doc['reference']['command'] = 'arm'
    assert not validate_proposal(doc, reasoning_context, now=.2, reference=WorldReference()).valid
    doc = DeterministicReasoner().propose(reasoning_context).to_dict()
    doc.update(validation_status='valid_non_executable', target_entity_id='fake')
    assert not validate_proposal(doc, reasoning_context, now=.2, reference=WorldReference()).valid


@pytest.mark.parametrize('now', [.1, .71, 2., float('nan')])
def test_freshness_validation_is_independent_of_proposal(reasoning_context, now):
    proposal = DeterministicReasoner().propose(reasoning_context)
    assert not validate_proposal(proposal, reasoning_context, now=now, reference=WorldReference()).valid


@pytest.mark.parametrize('reference', [WorldReference('map'),WorldReference(epoch=1),WorldReference(clock_id='system')])
def test_reference_mismatch_rejected(reasoning_context, reference):
    proposal = DeterministicReasoner().propose(reasoning_context)
    assert 'reference_mismatch' in validate_proposal(proposal, reasoning_context, now=.2, reference=reference).reason_codes


def test_old_proposal_rejected_after_reobservation(reasoning_replay):
    old = build_context(reasoning_replay[1]['snapshot'], now=.2, reference=WorldReference())
    new = build_context(reasoning_replay[2]['snapshot'], now=.4, reference=WorldReference())
    proposal = DeterministicReasoner().propose(old)
    assert 'outdated_context_reference' in validate_proposal(proposal,new,now=.4,reference=WorldReference()).reason_codes


def test_blocked_target_cannot_be_investigated_and_missing_evidence_cannot_be_hidden(reasoning_replay):
    context = build_context(reasoning_replay[-1]['snapshot'], now=7.4, reference=WorldReference())
    opening = next(e for e in context.entities if e.semantic_type == 'opening')
    forged = make_proposal(context, 'investigate_candidate', opening.entity_id,
                           reasons=('candidate_passage_exists','candidate_requires_investigation'),
                           missing=opening.missing_evidence, sources=(opening.entity_id,))
    result = validate_proposal(forged,context,now=7.4,reference=WorldReference())
    assert 'invalidated_target_requires_rejection' in result.reason_codes
    assert 'reason_inconsistent_with_context' in result.reason_codes


def test_validator_accepts_independently_authored_non_executable_proposal(reasoning_context):
    opening = next(e for e in reasoning_context.entities if e.semantic_type == 'opening')
    proposal = make_proposal(reasoning_context, 'investigate_candidate', opening.entity_id,
                             reasons=('candidate_passage_exists','candidate_requires_investigation'),
                             missing=opening.missing_evidence, sources=(opening.entity_id,))
    assert validate_proposal(proposal, reasoning_context, now=.2, reference=WorldReference()).valid
    assert proposal.intent != DeterministicReasoner().propose(reasoning_context).intent


def test_invalid_context_and_falsified_reason_rejected(reasoning_snapshot, reasoning_context):
    reasoning_snapshot['reset_required'] = True
    invalid = build_context(reasoning_snapshot, now=.2, reference=WorldReference())
    assert not validate_proposal(DeterministicReasoner().propose(invalid),invalid,now=.2,reference=WorldReference()).valid
    opening = next(e for e in reasoning_context.entities if e.semantic_type == 'opening')
    false = make_proposal(reasoning_context,'observe_more',opening.entity_id,
                          reasons=('candidate_passage_exists','blocking_evidence'),
                          missing=opening.missing_evidence,sources=(opening.entity_id,))
    assert 'reason_inconsistent_with_context' in validate_proposal(false,reasoning_context,now=.2,reference=WorldReference()).reason_codes


def test_review_alternative_requires_two_opening_sources(reasoning_context):
    opening = next(e for e in reasoning_context.entities if e.semantic_type == 'opening')
    wall = next(e for e in reasoning_context.entities if e.semantic_type == 'wall')
    proposal = make_proposal(reasoning_context,'review_alternative',opening.entity_id,
                             reasons=('candidate_passage_exists','alternatives_available','review_ranked_alternative'),
                             missing=opening.missing_evidence,sources=(opening.entity_id,wall.entity_id))
    assert 'grounded_alternatives_required' in validate_proposal(proposal,reasoning_context,now=.2,reference=WorldReference()).reason_codes


def test_valid_context_tampering_and_raw_duplicate_entities_rejected(reasoning_context):
    changed = replace(reasoning_context,mission_goal=replace(reasoning_context.mission_goal,intent='review_candidates'))
    result = validate_proposal(DeterministicReasoner().propose(reasoning_context),changed,now=.2,reference=WorldReference())
    assert 'context_hash_mismatch' in result.reason_codes
    doc = reasoning_context.to_dict()
    doc['entities'].append(doc['entities'][0])
    assert 'duplicate_entity_id' in validate_proposal(DeterministicReasoner().propose(reasoning_context),doc,now=.2,
                                                   reference=WorldReference()).reason_codes
