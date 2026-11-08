"""Independent goal guard: well-formed and grounded never means safe to execute."""
from dataclasses import dataclass, replace

from .schema import (Codec, ContractError, GoalProposal, ReasoningContext, WorldReference,
                     context_digest, finite, proposal_digest)


@dataclass(frozen=True, slots=True)
class ValidationResult(Codec):
    valid: bool
    reason_codes: tuple[str, ...]
    checked_at: float
    proposal: GoalProposal | None


def validate_proposal(proposal, context, *, now, reference, max_context_age_s=.5, max_snapshot_age_s=1.):
    """Accept a DTO or strict wire dictionary. Validate without invoking the reasoner.

    The context must originate at the trusted builder; hashes bind content, not
    authenticate a sender. A future external transport needs its own authentication.
    """
    errors = set()
    if not finite(now) or not finite(max_context_age_s) or not finite(max_snapshot_age_s):
        return ValidationResult(False, ('invalid_validation_time',), 0., None)
    try:
        if not isinstance(context, ReasoningContext):
            context = ReasoningContext.from_dict(context)
        if not isinstance(proposal, GoalProposal):
            proposal = GoalProposal.from_dict(proposal)
    except (ContractError, TypeError, ValueError) as error:
        code = str(error) if isinstance(error, ContractError) else 'malformed_proposal'
        return ValidationResult(False, (code,), float(now), None)
    if context.context_reference != context_digest(context):
        errors.add('context_hash_mismatch')
    if not context.context_valid:
        errors.add('invalid_context')
    if (not isinstance(reference, WorldReference) or context.reference != reference
            or proposal.reference != reference):
        errors.add('reference_mismatch')
    if proposal.context_reference != context.context_reference:
        errors.add('outdated_context_reference')
    if (not 0 <= now - context.context_created_at <= max_context_age_s
            or now > context.expires_at or max_context_age_s <= 0):
        errors.add('context_expired_or_future')
    if (context.snapshot_timestamp is None or max_snapshot_age_s <= 0
            or not 0 <= now - context.snapshot_timestamp <= max_snapshot_age_s):
        errors.add('world_snapshot_expired_or_future')
    if proposal.proposal_id != proposal_digest(proposal):
        errors.add('proposal_hash_mismatch')
    if context.omitted_entities and 'omitted_context_entities' not in proposal.missing_information:
        errors.add('context_omissions_suppressed')
    entities = {e.entity_id: e for e in context.entities}
    if any(v not in entities for v in proposal.source_entity_ids):
        errors.add('unknown_source_entity')
    for entity in context.entities:
        if any(r.target_entity_id not in entities or entities[r.target_entity_id].semantic_type != 'wall'
               or entity.semantic_type != 'opening' for r in entity.relations):
            errors.add('ungrounded_relationship')
        if abs(entity.age_s - (context.context_created_at - entity.last_observed)) > 1e-6:
            errors.add('entity_age_mismatch')
    target = entities.get(proposal.target_entity_id)
    if proposal.target_entity_id is not None and target is None:
        errors.add('unknown_target_entity')
    reasons = set(proposal.reason_codes)
    if proposal.intent == 'insufficient_information':
        if proposal.target_entity_id is not None:
            errors.add('unexpected_target')
    elif target is None or target.semantic_type != 'opening':
        errors.add('opening_target_required')
    else:
        if target.entity_id not in proposal.source_entity_ids:
            errors.add('target_missing_from_sources')
        if not set(target.missing_evidence) <= set(proposal.missing_information):
            errors.add('missing_evidence_suppressed')
        current_age = now - target.last_observed
        if target.status == 'invalidated':
            if proposal.intent != 'reject_blocked_candidate':
                errors.add('invalidated_target_requires_rejection')
        elif proposal.intent == 'reject_blocked_candidate':
            errors.add('target_not_blocked')
        needs_reobservation = target.status in ('stale', 'uncertain') or current_age > context.stale_after_s
        if needs_reobservation and proposal.intent not in ('observe_more', 'reject_blocked_candidate'):
            errors.add('stale_target_requires_reobservation')
        if proposal.intent in ('observe_more', 'investigate_candidate', 'review_alternative'):
            if not target.evidence:
                errors.add('missing_supporting_evidence')
        if proposal.intent == 'reject_blocked_candidate':
            if not any(e.kind == 'blocking_evidence' and e.rays is not None and e.rays >= 12
                       and e.blocked_fraction is not None and e.blocked_fraction >= .6 for e in target.evidence):
                errors.add('missing_blocking_evidence')
            if 'blocking_evidence' not in reasons:
                errors.add('required_reason_missing')
        elif 'candidate_passage_exists' not in reasons:
            errors.add('required_reason_missing')
        if proposal.intent == 'observe_more':
            if target.geometry.height_m is None and 'height_unconfirmed' not in reasons:
                errors.add('required_reason_missing')
            if needs_reobservation and 'reobserve_stale_belief' not in reasons:
                errors.add('required_reason_missing')
            if 'clearance_unconfirmed' not in reasons:
                errors.add('required_reason_missing')
        if proposal.intent == 'review_alternative':
            opening_sources = [v for v in proposal.source_entity_ids if v in entities and entities[v].semantic_type == 'opening']
            if len(opening_sources) < 2:
                errors.add('grounded_alternatives_required')
            if not {'alternatives_available','review_ranked_alternative'} <= reasons:
                errors.add('required_reason_missing')
    facts = {
        'no_supported_inspection': proposal.intent == 'insufficient_information' and target is None,
        'context_invalid': not context.context_valid,
        'no_relevant_entities': not any(e.semantic_type == 'opening' for e in entities.values()),
        'candidate_passage_exists': target is not None and target.semantic_type == 'opening',
        'height_unconfirmed': target is not None and target.geometry.height_m is None,
        'clearance_unconfirmed': target is not None and target.traversability in ('unknown','insufficient_clearance'),
        'reobserve_stale_belief': target is not None and (target.status in ('stale','uncertain')
                                                       or now - target.last_observed > context.stale_after_s),
        'blocking_evidence': target is not None and target.status == 'invalidated',
        'alternatives_available': len([v for v in proposal.source_entity_ids if v in entities
                                      and entities[v].semantic_type == 'opening']) >= 2,
        'candidate_requires_investigation': target is not None and target.status not in ('invalidated','stale','uncertain'),
        'review_ranked_alternative': target is not None and len(proposal.source_entity_ids) >= 2,
        'mission_target_known': target is not None and context.mission_goal.target_entity_id == target.entity_id,
        'region_connectivity_unknown': context.mission_goal.intent == 'reach_region',
    }
    if any(not facts[code] for code in reasons):
        errors.add('reason_inconsistent_with_context')
    if errors:
        return ValidationResult(False, tuple(sorted(errors)), float(now), None)
    return ValidationResult(True, ('valid_non_executable',), float(now),
                            replace(proposal, validation_status='valid_non_executable'))
