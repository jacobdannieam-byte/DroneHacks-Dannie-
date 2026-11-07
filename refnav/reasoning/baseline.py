"""Deterministic evidence-seeking semantic policy. Produces no motion commands."""
from .schema import context_digest, make_proposal


def ranked_candidates(context):
    """Known mission target, current before stale before blocked, then age/evidence/confidence/ID.

    Distance and unverified connectivity never predict mission progress.
    """
    return tuple(sorted((e for e in context.entities if e.semantic_type == 'opening'), key=lambda e: (
        e.entity_id != context.mission_goal.target_entity_id,
        2 if e.status == 'invalidated' else 1 if e.status in ('stale', 'uncertain') else 0,
        e.age_s, -min(e.observations, 8), -e.geometric_confidence, -e.semantic_confidence,
        len(e.missing_evidence), e.entity_id)))


class DeterministicReasoner:
    def propose(self, context):
        if not context.context_valid or context.context_reference != context_digest(context):
            return make_proposal(context, 'insufficient_information', reasons=('context_invalid',),
                                 missing=context.invalid_reasons or ('context_integrity',))
        if context.mission_goal.intent == 'reach_region':
            return make_proposal(context, 'insufficient_information', reasons=('region_connectivity_unknown',),
                                 missing=('verified_region_connectivity',), sources=(context.mission_goal.target_entity_id,))
        candidates = ranked_candidates(context)
        if not candidates:
            return make_proposal(context, 'insufficient_information', reasons=('no_relevant_entities',),
                                 missing=('candidate_observation',) + (('omitted_context_entities',) if context.omitted_entities else ()))
        target = candidates[0]
        reasons = ['candidate_passage_exists']
        sources = [target.entity_id]
        missing = target.missing_evidence + (('omitted_context_entities',) if context.omitted_entities else ())
        if len(candidates) > 1:
            reasons.append('alternatives_available')
            sources.append(candidates[1].entity_id)
        if target.entity_id == context.mission_goal.target_entity_id:
            reasons.append('mission_target_known')
        if target.status == 'invalidated':
            return make_proposal(context, 'reject_blocked_candidate', target.entity_id,
                                 reasons=(*reasons, 'blocking_evidence'), missing=missing, sources=sources)
        if target.status in ('stale', 'uncertain'):
            reasons.append('reobserve_stale_belief')
        if target.geometry.height_m is None:
            reasons.append('height_unconfirmed')
        reasons.append('clearance_unconfirmed')
        return make_proposal(context, 'observe_more', target.entity_id, reasons=reasons,
                             missing=missing, sources=sources)
