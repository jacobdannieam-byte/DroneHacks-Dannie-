"""Select G0/G1/G2 without granting command authority or changing schema v1.

Auxiliary inputs and score explanations travel in a separate immutable envelope.
They are checked again when resolving the observation objective.
"""
from dataclasses import dataclass
from hashlib import sha256
from time import perf_counter
import math

from .baseline import DeterministicReasoner
from .inspection import InspectionInputs, candidates
from .schema import canonical, make_proposal, context_digest

# Our own provisional host allocation; this is not an Orin measurement.
REASONER_BUDGET_SECONDS = .005
UTILITY_WEIGHTS = (1., 2., -.25, -.75, -1., -.20)


@dataclass(frozen=True)
class ReasonerConfig:
    variant: str = 'G0'
    checkpoint: str | None = None
    checkpoint_sha256: str | None = None
    budget_seconds: float = REASONER_BUDGET_SECONDS

    def __post_init__(self):
        if (self.variant not in ('G0', 'G1', 'G2') or not math.isfinite(self.budget_seconds)
                or not 0 < self.budget_seconds <= .05):
            raise ValueError('Choose G0, G1 or G2 with a positive budget no larger than 50 ms.')


@dataclass(frozen=True)
class Selection:
    inputs: InspectionInputs
    proposal_id: str
    selected_entity_id: str | None
    model_identity: str
    scores: tuple
    reason: str
    identity: str = ''


def selection_digest(selection):
    from dataclasses import replace
    return sha256(canonical(replace(selection, identity='')).encode()).hexdigest()


def components(candidate):
    return (candidate.mission_support, candidate.information_gain_heuristic, candidate.movement_cost,
            candidate.risk_proxy, min(1., candidate.failed_attempts / 3), candidate.switching)


def abstain(context):
    missing = ('supported_inspection_viewpoint',)
    if context.omitted_entities:
        missing += ('omitted_context_entities',)
    return make_proposal(context, 'insufficient_information', reasons=('no_supported_inspection',), missing=missing)


def observation_proposal(context, entity_id):
    entity = next(e for e in context.entities if e.entity_id == entity_id)
    reasons = ['candidate_passage_exists', 'clearance_unconfirmed']
    if entity.geometry.height_m is None:
        reasons.append('height_unconfirmed')
    missing = entity.missing_evidence + (('omitted_context_entities',) if context.omitted_entities else ())
    return make_proposal(context, 'observe_more', entity_id, reasons=reasons, missing=missing, sources=(entity_id,))


class ConfiguredReasoner:
    def __init__(self, config=None):
        self.config = config or ReasonerConfig()
        self.model = None
        self.model_error = None
        if self.config.variant == 'G2':
            try:
                from .learned import GoalMLP
                self.model = GoalMLP.load(self.config.checkpoint, self.config.checkpoint_sha256)
            except (OSError, ValueError, TypeError, ImportError, RecursionError) as error:
                self.model_error = type(error).__name__

    def propose(self, context):
        # Existing interface remains valid; richer ranking requires an explicit bound input.
        return DeterministicReasoner().propose(context)

    def select(self, context, inputs, *, monotonic=perf_counter):
        from dataclasses import replace
        start = monotonic()
        options = candidates(context, inputs)
        scores, reason = (), 'baseline'
        identity = self.config.variant + ':' + sha256((canonical(self.config) + canonical(UTILITY_WEIGHTS)).encode()).hexdigest()
        proposal = DeterministicReasoner().propose(context)
        if self.config.variant != 'G0' and context.context_valid and context.context_reference == context_digest(context):
            values = tuple(sum(w * v for w, v in zip(UTILITY_WEIGHTS, components(c))) for c in options)
            reason = 'heuristic_information_gain'
            if self.config.variant == 'G2':
                try:
                    if self.model is None:
                        raise ValueError('No compatible checkpoint is available.')
                    values = self.model.score(context, options)
                    identity += ':' + self.model.identity
                    if len(values) != len(options) or not all(math.isfinite(v) for v in values):
                        raise ValueError('The learned ranker returned invalid scores.')
                    reason = 'learned_ranking'
                except (ValueError, TypeError, ArithmeticError):
                    reason = 'fresh_baseline_fallback'
            if reason != 'fresh_baseline_fallback':
                scores = tuple((c.entity_id, components(c), float(v)) for c, v in zip(options, values))
                best = min(zip(options, values), key=lambda row: (-row[1], row[0].entity_id), default=None)
                proposal = observation_proposal(context, best[0].entity_id) if best else abstain(context)
        if monotonic() - start > self.config.budget_seconds:
            proposal = DeterministicReasoner().propose(context)
            scores, reason = (), 'deadline_fresh_baseline_fallback'
        selected = proposal.target_entity_id if any(c.entity_id == proposal.target_entity_id for c in options) else None
        envelope = Selection(inputs, proposal.proposal_id, selected, identity, scores, reason)
        return proposal, replace(envelope, identity=selection_digest(envelope))


def create_reasoner(config=None):
    return ConfiguredReasoner(config)
