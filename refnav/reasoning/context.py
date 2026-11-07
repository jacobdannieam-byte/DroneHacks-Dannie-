"""Read-only WorldState/snapshot adapter. Unknown information is never filled in."""
from dataclasses import dataclass, replace
import math

from .schema import (ContractError, EvidenceSummary, GeometryEstimate, MAX_ENTITIES, MAX_WIRE_BYTES,
                     MissionGoal, ReasoningContext, Relation, SemanticEntitySummary, WorldReference,
                     canonical, finite, require, seal_context, token)


@dataclass(frozen=True, slots=True)
class ContextConfig:
    max_entities: int = 32
    max_context_bytes: int = 16384
    evidence_limit: int = 2
    max_snapshot_age_s: float = 1.0
    context_lifetime_s: float = 0.5

    def __post_init__(self):
        require(type(self.max_entities) is int and 1 <= self.max_entities <= MAX_ENTITIES, 'invalid_entity_budget')
        require(type(self.max_context_bytes) is int and 1024 <= self.max_context_bytes <= MAX_WIRE_BYTES, 'invalid_byte_budget')
        require(type(self.evidence_limit) is int and 1 <= self.evidence_limit <= 8, 'invalid_evidence_budget')
        require(finite(self.max_snapshot_age_s) and 0 < self.max_snapshot_age_s <= 5, 'invalid_snapshot_age_limit')
        require(finite(self.context_lifetime_s) and 0 < self.context_lifetime_s <= 5, 'invalid_context_lifetime')


PASSAGE_MISSING = frozenset(('full_volume_coverage', 'region_connectivity', 'validated_swept_path',
                             'vehicle_propeller_envelope', 'localization_margin', 'controller_tracking_margin'))


def _summary(item, now, query_stamp, stale_after, reference, config):
    require(item['frame_id'] == reference.frame_id and type(item['epoch']) is int
            and item['epoch'] == reference.epoch, 'entity_reference_mismatch')
    require(item.get('clock_id', reference.clock_id) == reference.clock_id, 'entity_clock_mismatch')
    last = item['last_observed']
    require(finite(last) and last <= query_stamp <= now, 'invalid_entity_timestamp')
    require(finite(item['source_timestamp']) and item['source_timestamp'] <= last, 'invalid_source_timestamp')
    require(not item.get('connected_regions'), 'unsupported_connectivity')
    require(not any(item.get(k) is True for k in ('safe', 'is_safe', 'traversable', 'fully_observed_enough')),
            'unsafe_traversability_claim')
    require(item.get('traversability', 'unknown') in ('unknown', 'blocked', 'insufficient_clearance'),
            'unsafe_traversability_claim')
    geometry = GeometryEstimate.from_dict(item['geometry'])
    age = float(now - last)
    status = item['status']
    if age > stale_after and status != 'invalidated':
        status = 'stale'
    evidence = item['evidence']
    require(isinstance(evidence, (list, tuple)) and len(evidence) <= 8, 'evidence_budget_exceeded')
    proofs = []
    for value in evidence[-config.evidence_limit:]:
        require(finite(value['stamp']) and value['stamp'] <= last, 'invalid_evidence_time')
        require(finite(value['position_sigma_m']) and finite(value['angle_sigma_rad']), 'invalid_evidence_uncertainty')
        support = value.get('support', {})
        proofs.append(EvidenceSummary(value['observation_id'], value['source'], float(value['stamp']), value['kind'],
                                      float(value['position_sigma_m']), float(value['angle_sigma_rad']),
                                      support.get('rays'), support.get('blocked_fraction')))
    missing = item['missing_evidence']
    require(isinstance(missing, (list, tuple)) and len(missing) <= 24
            and all(isinstance(v, str) for v in missing), 'invalid_missing_evidence')
    missing = set(missing)
    if item['semantic_type'] == 'opening':
        missing |= PASSAGE_MISSING
        if geometry.height_m is None:
            missing |= {'upper_boundary', 'lower_boundary'}
        if status in ('stale', 'uncertain'):
            missing.add('fresh_consistent_observation')
        if status == 'invalidated':
            missing.add('unobstructed_reobservation')
            require(item['contradictions'] >= 1 and any(e.kind == 'blocking_evidence' and e.rays is not None
                    and e.rays >= 12 and e.blocked_fraction is not None and e.blocked_fraction >= .6 for e in proofs),
                    'missing_blocking_evidence')
    raw_relations = item['relations']
    require(isinstance(raw_relations, (list, tuple)) and len(raw_relations) <= 4, 'relation_budget_exceeded')
    relations = tuple(Relation(r['type'], r['target'], r['status']) for r in raw_relations)
    if item.get('omitted_relations', 0):
        missing.add('supporting_entities_omitted')
    require(finite(item['geometric_confidence']) and finite(item['semantic_confidence']), 'invalid_confidence')
    decay = math.exp(-(now - query_stamp) / stale_after)
    return SemanticEntitySummary(item['entity_id'], item['semantic_type'], geometry, float(last), age, status,
                                 float(item['geometric_confidence']) * decay,
                                 float(item['semantic_confidence']) * decay, item['observations'], item['contradictions'],
                                 tuple(proofs), tuple(sorted(missing)), relations, (),
                                 'blocked' if status == 'invalidated' else item.get('traversability', 'unknown'))


def _prune_relations(summaries):
    present = {e.entity_id for e in summaries}
    result = []
    for entity in summaries:
        relations = tuple(r for r in entity.relations if r.target_entity_id in present)
        missing = entity.missing_evidence
        if len(relations) != len(entity.relations):
            missing = tuple(sorted(set(missing) | {'supporting_entities_omitted'}))
        result.append(replace(entity, relations=relations, missing_evidence=missing))
    return tuple(result)


def build_context(source, mission_goal=None, *, now, reference, config=None):
    """Accept a public WorldState or v1 snapshot; bind to an explicit expected reference.

    Malformed/unavailable world input yields a bounded invalid context, not geometry.
    Invalid caller configuration, mission schema or nonfinite caller time raises.
    """
    config, mission_goal = config or ContextConfig(), mission_goal or MissionGoal()
    require(isinstance(reference, WorldReference) and isinstance(mission_goal, MissionGoal), 'invalid_builder_contract')
    require(finite(now), 'invalid_context_time')
    timestamp, stale_after = None, 5.
    try:
        if hasattr(source, 'semantic_snapshot'):
            # Public configuration controls the owner's bound; no access to internal entity stores.
            doc = source.semantic_snapshot(now, max_entities=min(config.max_entities, source.config.max_entities),
                                            evidence_limit=min(config.evidence_limit, source.config.max_evidence),
                                            preferred_entity_id=mission_goal.target_entity_id)
        else:
            doc = source
        require(isinstance(doc, dict), 'missing_world_state')
        require(type(doc['schema_version']) is int and doc['schema_version'] == 1, 'unsupported_world_schema')
        require(WorldReference(doc['frame_id'], doc['epoch'], doc['clock_id']) == reference, 'world_reference_mismatch')
        require(type(doc['reset_required']) is bool and not doc['reset_required'], 'world_reference_reset')
        timestamp = doc['timestamp']
        require(finite(timestamp), 'missing_world_timestamp')
        require(0 <= now - timestamp <= config.max_snapshot_age_s, 'stale_or_future_world_snapshot')
        query_stamp = doc.get('query_timestamp', timestamp)
        require(finite(query_stamp) and timestamp <= query_stamp <= now, 'invalid_query_timestamp')
        stale_after = doc.get('stale_after_s', 5.)
        require(finite(stale_after) and stale_after > 0, 'invalid_stale_horizon')
        items = doc['entities']
        require(isinstance(items, (list, tuple)) and len(items) <= MAX_ENTITIES, 'world_entity_budget_exceeded')
        ids = [item['entity_id'] for item in items]
        require(all(token(v) for v in ids) and len(set(ids)) == len(ids), 'duplicate_or_invalid_entity_id')
        by_id = {e['entity_id']: e for e in items}
        if mission_goal.target_entity_id is not None:
            require(mission_goal.target_entity_id in by_id, 'unknown_mission_target')
            target = by_id[mission_goal.target_entity_id]
            require(target['semantic_type'] == ('region' if mission_goal.intent == 'reach_region' else 'opening'),
                    'mission_target_type_mismatch')
        ordered = sorted(items, key=lambda e: (e['entity_id'] != mission_goal.target_entity_id,
                         e['semantic_type'] != 'opening', -e['last_observed'], e['entity_id']))
        # Validate all raw snapshot entities, even if some will be omitted from the bounded output.
        summaries = [_summary(e, now, query_stamp, stale_after, reference, config) for e in ordered]
        for entity in summaries:
            for relation in entity.relations:
                require(relation.target_entity_id in by_id and by_id[relation.target_entity_id]['semantic_type'] == 'wall'
                        and entity.semantic_type == 'opening', 'ungrounded_relationship')
        total = doc.get('total_entities', len(items))
        require(type(total) is int and len(items) <= total <= 1000000, 'invalid_total_entities')
        base = ReasoningContext(1, reference, float(timestamp), float(now), float(now + config.context_lifetime_s),
                                float(stale_after), mission_goal, (), total, True, ())
        # Conservative per-record byte accounting avoids serializing the full graph repeatedly.
        available = config.max_context_bytes - len(seal_context(base).to_json().encode()) - 128
        selected = []
        for entity in summaries[:config.max_entities]:
            size = len(canonical(entity).encode()) + 64  # reserve relation-pruning metadata
            if size > available:
                break
            selected.append(entity)
            available -= size
        if mission_goal.target_entity_id is not None:
            require(any(e.entity_id == mission_goal.target_entity_id for e in selected), 'mission_target_omitted')
        context = seal_context(replace(base, entities=_prune_relations(selected), omitted_entities=total - len(selected)))
        require(len(context.to_json().encode()) <= config.max_context_bytes, 'context_byte_budget_exceeded')
        return context
    except (ContractError, ValueError, TypeError, KeyError, OverflowError) as error:
        reason = str(error) if isinstance(error, ContractError) else 'malformed_world_state'
        if not finite(timestamp):
            timestamp = None
        if not finite(stale_after) or stale_after <= 0:
            stale_after = 5.
        return seal_context(ReasoningContext(1, reference, timestamp, float(now), float(now + config.context_lifetime_s),
                                            float(stale_after), mission_goal, (), 0, False, (reason,)))
