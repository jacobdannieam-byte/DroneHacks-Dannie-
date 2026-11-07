"""Versioned, immutable, bounded machine contracts. No executable action payloads."""
from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass, replace
from functools import lru_cache
import hashlib
import json
import math
import re
import types
from typing import Protocol, get_args, get_origin, get_type_hints

SCHEMA_VERSION = 1
MAX_WIRE_BYTES = 65536
MAX_ENTITIES = 128
INTENTS = frozenset(('observe_more', 'investigate_candidate', 'reject_blocked_candidate',
                     'review_alternative', 'insufficient_information'))
REASONS = frozenset(('context_invalid', 'no_relevant_entities', 'candidate_passage_exists',
                     'height_unconfirmed', 'clearance_unconfirmed', 'reobserve_stale_belief',
                     'blocking_evidence', 'alternatives_available', 'candidate_requires_investigation',
                     'review_ranked_alternative', 'mission_target_known', 'region_connectivity_unknown'))


class ContractError(ValueError):
    """The exception text is a machine-readable failure code, never untrusted payload text."""


def require(condition, code):
    if not condition:
        raise ContractError(code)


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def token(value):
    return isinstance(value, str) and re.fullmatch(r'[A-Za-z0-9_.:/-]{1,128}', value) is not None


def codes(values):
    return (isinstance(values, tuple) and len(values) <= 32 and len(set(values)) == len(values)
            and all(isinstance(v, str) and re.fullmatch(r'[a-z][a-z0-9_]{0,63}', v) for v in values))


def wire(value):
    """A detached JSON value; avoids deepcopy/asdict and includes only declared fields."""
    if is_dataclass(value):
        return {f.name: wire(getattr(value, f.name)) for f in fields(value)}
    if isinstance(value, tuple):
        return [wire(v) for v in value]
    return value


def canonical(value):
    return json.dumps(wire(value), sort_keys=True, separators=(',', ':'), allow_nan=False)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, 'duplicate_json_key')
        result[key] = value
    return result


@lru_cache(maxsize=32)
def _hints(cls):
    return get_type_hints(cls)


def _decode(kind, value):
    origin, args = get_origin(kind), get_args(kind)
    if origin is types.UnionType:
        for candidate in args:
            try:
                return _decode(candidate, value)
            except ContractError:
                pass
        raise ContractError('invalid_field_type')
    if origin is tuple:
        require(isinstance(value, (tuple, list)) and len(value) <= MAX_ENTITIES, 'invalid_tuple')
        if len(args) == 2 and args[1] is Ellipsis:
            return tuple(_decode(args[0], v) for v in value)
        require(len(value) == len(args), 'invalid_tuple_size')
        return tuple(_decode(k, v) for k,v in zip(args, value))
    if is_dataclass(kind):
        require(isinstance(value, dict), 'invalid_object')
        require(set(value) == {f.name for f in fields(kind)}, 'unknown_or_missing_fields')
        return kind(**{name: _decode(hint, value[name]) for name,hint in _hints(kind).items()})
    if kind is float:
        require(finite(value), 'invalid_number')
        return float(value)
    require(type(value) is kind, 'invalid_field_type')
    return value


class Codec:
    __slots__ = ()

    def to_dict(self):
        return wire(self)

    def to_json(self):
        result = canonical(self)
        require(len(result.encode('utf-8')) <= MAX_WIRE_BYTES, 'wire_size_exceeded')
        return result

    @classmethod
    def from_dict(cls, value):
        try:
            result = _decode(cls, value)
            if isinstance(result, ReasoningContext):
                require(result.context_reference == context_digest(result), 'context_hash_mismatch')
            result.to_json()  # Dict-based adapters obey the same wire budget as JSON adapters.
            return result
        except (TypeError, KeyError, OverflowError, RecursionError) as error:
            raise ContractError('malformed_contract') from error

    @classmethod
    def from_json(cls, value):
        require(isinstance(value, str) and len(value.encode('utf-8')) <= MAX_WIRE_BYTES, 'wire_size_exceeded')
        try:
            doc = json.loads(value, object_pairs_hook=_unique_object,
                             parse_constant=lambda _: (_ for _ in ()).throw(ContractError('nonfinite_number')))
        except (ValueError, RecursionError) as error:
            if isinstance(error, ContractError):
                raise
            raise ContractError('invalid_json') from error
        return cls.from_dict(doc)


@dataclass(frozen=True, slots=True)
class WorldReference(Codec):
    frame_id: str = 'odom'
    epoch: int = 0
    clock_id: str = 'replay'

    def __post_init__(self):
        require(token(self.frame_id) and token(self.clock_id), 'invalid_reference')
        require(type(self.epoch) is int and self.epoch >= 0, 'invalid_epoch')


@dataclass(frozen=True, slots=True)
class MissionGoal(Codec):
    intent: str = 'investigate_candidate_passage'
    target_entity_id: str | None = None

    def __post_init__(self):
        require(self.intent in ('investigate_candidate_passage', 'reach_region', 'review_candidates'), 'invalid_mission_intent')
        require(self.target_entity_id is None or token(self.target_entity_id), 'invalid_mission_target')
        require(self.intent != 'reach_region' or self.target_entity_id is not None, 'region_target_required')


@dataclass(frozen=True, slots=True)
class GeometryEstimate(Codec):
    center: tuple[float, float, float]
    normal: tuple[float, float, float]
    width_m: float
    height_m: float | None
    uncertainty_m: float

    def __post_init__(self):
        require(all(isinstance(v, tuple) and len(v) == 3 and all(finite(x) for x in v)
                    for v in (self.center, self.normal)), 'invalid_geometry_vector')
        require(abs(sum(x*x for x in self.normal) - 1) < .001, 'invalid_surface_normal')
        require(finite(self.width_m) and self.width_m > 0, 'invalid_width')
        require(self.height_m is None or (finite(self.height_m) and self.height_m > 0), 'invalid_height')
        require(finite(self.uncertainty_m) and self.uncertainty_m >= 0, 'invalid_uncertainty')


@dataclass(frozen=True, slots=True)
class Relation(Codec):
    relation_type: str
    target_entity_id: str
    status: str

    def __post_init__(self):
        require(self.relation_type == 'bounded_by' and self.status == 'observed_geometry', 'unsupported_relationship')
        require(token(self.target_entity_id), 'invalid_relation_target')


@dataclass(frozen=True, slots=True)
class EvidenceSummary(Codec):
    observation_id: str
    source: str
    stamp: float
    kind: str
    position_sigma_m: float
    angle_sigma_rad: float
    rays: int | None = None
    blocked_fraction: float | None = None

    def __post_init__(self):
        require(token(self.observation_id) and token(self.source), 'invalid_evidence_id')
        require(self.kind in ('support', 'blocking_evidence'), 'invalid_evidence_kind')
        require(finite(self.stamp) and all(finite(v) and v >= 0 for v in
                (self.position_sigma_m, self.angle_sigma_rad)), 'invalid_evidence_uncertainty')
        require(self.rays is None or (type(self.rays) is int and self.rays >= 0), 'invalid_ray_count')
        require(self.blocked_fraction is None or (finite(self.blocked_fraction) and 0 <= self.blocked_fraction <= 1),
                'invalid_blocked_fraction')


@dataclass(frozen=True, slots=True)
class SemanticEntitySummary(Codec):
    entity_id: str
    semantic_type: str
    geometry: GeometryEstimate
    last_observed: float
    age_s: float
    status: str
    geometric_confidence: float
    semantic_confidence: float
    observations: int
    contradictions: int
    evidence: tuple[EvidenceSummary, ...]
    missing_evidence: tuple[str, ...]
    relations: tuple[Relation, ...]
    connected_regions: tuple[str, ...] = ()
    traversability: str = 'unknown'

    def __post_init__(self):
        require(token(self.entity_id), 'invalid_entity_id')
        require(self.semantic_type in ('wall', 'opening', 'corridor', 'obstacle', 'region'), 'unsupported_semantic_type')
        require(isinstance(self.geometry, GeometryEstimate), 'invalid_geometry')
        require(self.status in ('provisional', 'confirmed', 'stale', 'uncertain', 'invalidated'), 'invalid_entity_status')
        require(finite(self.last_observed) and finite(self.age_s) and self.age_s >= 0, 'invalid_entity_time')
        require(all(finite(v) and 0 <= v <= 1 for v in (self.geometric_confidence, self.semantic_confidence)), 'invalid_confidence')
        require(all(type(v) is int and 0 <= v <= 1000000000 for v in (self.observations, self.contradictions)), 'invalid_evidence_count')
        require(isinstance(self.evidence, tuple) and len(self.evidence) <= 8
                and all(isinstance(v, EvidenceSummary) for v in self.evidence), 'invalid_evidence')
        require(codes(self.missing_evidence), 'invalid_missing_evidence')
        require(isinstance(self.relations, tuple) and len(self.relations) <= 4
                and all(isinstance(v, Relation) for v in self.relations), 'invalid_relations')
        require(self.connected_regions == (), 'unsupported_connectivity')
        require(self.traversability in ('unknown', 'blocked', 'insufficient_clearance'), 'unsafe_traversability_claim')
        require(self.traversability != 'blocked' or self.status == 'invalidated', 'unsupported_blocked_claim')


@dataclass(frozen=True, slots=True)
class ReasoningContext(Codec):
    schema_version: int
    reference: WorldReference
    snapshot_timestamp: float | None
    context_created_at: float
    expires_at: float
    stale_after_s: float
    mission_goal: MissionGoal
    entities: tuple[SemanticEntitySummary, ...]
    omitted_entities: int
    context_valid: bool
    invalid_reasons: tuple[str, ...]
    context_reference: str = ''

    def __post_init__(self):
        require(type(self.schema_version) is int and self.schema_version == 1, 'unsupported_context_schema')
        require(isinstance(self.reference, WorldReference) and isinstance(self.mission_goal, MissionGoal), 'invalid_context_contract')
        require(self.snapshot_timestamp is None or finite(self.snapshot_timestamp), 'invalid_snapshot_timestamp')
        require(finite(self.context_created_at) and finite(self.expires_at)
                and 0 < self.expires_at - self.context_created_at <= 5., 'invalid_context_lifetime')
        require(finite(self.stale_after_s) and self.stale_after_s > 0, 'invalid_stale_horizon')
        require(isinstance(self.entities, tuple) and len(self.entities) <= MAX_ENTITIES
                and all(isinstance(v, SemanticEntitySummary) for v in self.entities), 'invalid_context_entities')
        require(len({e.entity_id for e in self.entities}) == len(self.entities), 'duplicate_entity_id')
        require(type(self.omitted_entities) is int and self.omitted_entities >= 0, 'invalid_omitted_count')
        require(type(self.context_valid) is bool and self.context_valid == (not self.invalid_reasons)
                and codes(self.invalid_reasons), 'invalid_context_validity')


def context_digest(context):
    doc = context.to_dict()
    doc.pop('context_reference')
    return 'ctx_' + hashlib.sha256(canonical_value(doc).encode()).hexdigest()


def canonical_value(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)


def seal_context(context):
    return replace(context, context_reference=context_digest(context))


@dataclass(frozen=True, slots=True)
class GoalProposal(Codec):
    schema_version: int
    proposal_id: str
    intent: str
    target_entity_id: str | None
    reason_codes: tuple[str, ...]
    missing_information: tuple[str, ...]
    source_entity_ids: tuple[str, ...]
    context_reference: str
    reference: WorldReference
    validation_status: str = 'unvalidated'

    def __post_init__(self):
        require(type(self.schema_version) is int and self.schema_version == 1, 'unsupported_proposal_schema')
        require(self.intent in INTENTS, 'unsupported_intent')
        require(token(self.proposal_id) and token(self.context_reference), 'invalid_proposal_reference')
        require(self.target_entity_id is None or token(self.target_entity_id), 'invalid_target_id')
        require(codes(self.reason_codes) and bool(self.reason_codes) and set(self.reason_codes) <= REASONS, 'unsupported_reason_code')
        require(codes(self.missing_information), 'invalid_missing_information')
        require(isinstance(self.source_entity_ids, tuple) and len(self.source_entity_ids) <= MAX_ENTITIES
                and len(set(self.source_entity_ids)) == len(self.source_entity_ids)
                and all(token(v) for v in self.source_entity_ids), 'invalid_source_ids')
        require(isinstance(self.reference, WorldReference), 'invalid_reference')
        require(self.validation_status in ('unvalidated', 'valid_non_executable'), 'invalid_validation_status')


def proposal_digest(proposal):
    doc = proposal.to_dict()
    doc.pop('proposal_id')
    doc.pop('validation_status')
    return 'goal_' + hashlib.sha256(canonical_value(doc).encode()).hexdigest()


def make_proposal(context, intent, target=None, reasons=(), missing=(), sources=()):
    proposal = GoalProposal(1, 'pending', intent, target, tuple(sorted(set(reasons))),
                            tuple(sorted(set(missing))), tuple(sorted(set(sources))),
                            context.context_reference, context.reference)
    return replace(proposal, proposal_id=proposal_digest(proposal))


class Reasoner(Protocol):
    def propose(self, context: ReasoningContext) -> GoalProposal:
        """Return an unvalidated, non-executable goal hypothesis."""
        ...
