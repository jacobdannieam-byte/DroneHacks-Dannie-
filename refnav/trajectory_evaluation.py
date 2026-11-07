"""Offline D3 candidate evaluation. Outputs contain diagnostics only."""

from dataclasses import dataclass, replace
import math

import numpy as np

from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint, Prediction, PredictedState
from refnav.dynamics.history import MotionHistory, STEP_SECONDS
from refnav.dynamics.validation import finite_real, vector3, wrap_yaw
from refnav.reasoning import GoalProposal, ReasoningContext, WorldReference, build_context, validate_proposal
from refnav.world_model.spatial_input import MAX_LOCAL_M, NS, SpatialReference, stamp_ok
from refnav.world_model.spatial_pipeline import SpatialSnapshot
from refnav.world_model.spatial_volume import Cell, MetricSnapshot, swept_cell_bounds

MAX_CANDIDATES = 8
MAX_STEPS = 100
CLEARANCE_PROBE_M = .10


def _bounded(value, name, low, high):
    value = finite_real(value, name)
    if not low <= value <= high:
        raise ValueError(f"{name} outside supported bounds")
    return value


def _label(value):
    return isinstance(value, str) and 0 < len(value) <= 80 and bool(value.strip())


@dataclass(frozen=True)
class FrameAlignment:
    """Verified NED to gravity-aligned map transform for one reference pair."""

    ned: LocalNED
    map: SpatialReference
    capture_ns: int
    time_origin_ns: int
    map_from_enu_yaw_rad: float
    ned_origin_in_map_m: tuple
    position_error_m: float
    angle_error_rad: float
    provenance: str

    def __post_init__(self):
        if not isinstance(self.ned, LocalNED) or not isinstance(self.map, SpatialReference):
            raise ValueError("alignment needs explicit NED and map references")
        if not stamp_ok(self.capture_ns) or not stamp_ok(self.time_origin_ns) or not _label(self.provenance):
            raise ValueError("alignment needs valid timestamps and provenance")
        object.__setattr__(self, "map_from_enu_yaw_rad", wrap_yaw(finite_real(
            self.map_from_enu_yaw_rad, "map_from_enu_yaw_rad")))
        origin = vector3(self.ned_origin_in_map_m, "ned_origin_in_map_m")
        if max(map(abs, origin)) > MAX_LOCAL_M:
            raise ValueError("alignment origin outside local coordinates")
        object.__setattr__(self, "ned_origin_in_map_m", origin)
        _bounded(self.position_error_m, "alignment position error", 0., 1.)
        _bounded(self.angle_error_rad, "alignment angle error", 0., .5)


@dataclass(frozen=True)
class VehicleBounds:
    radius_m: float
    state_position_error_m: float
    tracking_error_m: float

    def __post_init__(self):
        _bounded(self.radius_m, "vehicle radius", .001, 2.)
        _bounded(self.state_position_error_m, "state position error", 0., 2.)
        _bounded(self.tracking_error_m, "tracking error", 0., 2.)


@dataclass(frozen=True)
class EvaluationPolicy:
    state_age_ns: int = 100_000_000
    map_age_ns: int = 250_000_000
    alignment_age_ns: int = 100_000_000
    alignment_skew_ns: int = 30_000_000

    def __post_init__(self):
        if any(not stamp_ok(v) or not 0 < v <= NS for v in (
                self.state_age_ns, self.map_age_ns, self.alignment_age_ns, self.alignment_skew_ns)):
            raise ValueError("freshness limits must be positive and at most one second")


@dataclass(frozen=True)
class Candidate:
    candidate_id: str
    schedule: tuple

    def __post_init__(self):
        if (not _label(self.candidate_id) or not isinstance(self.schedule, tuple)
                or not 1 <= len(self.schedule) <= MAX_STEPS
                or not all(isinstance(item, PositionSetpoint) for item in self.schedule)):
            raise ValueError("candidate needs an ID and 1–100 effective position/yaw setpoints")


@dataclass(frozen=True)
class GoalBinding:
    context: ReasoningContext
    proposal: GoalProposal

    def __post_init__(self):
        if not isinstance(self.context, ReasoningContext) or not isinstance(self.proposal, GoalProposal):
            raise ValueError("goal requires a bounded reasoning context and proposal")


@dataclass(frozen=True)
class CandidateDiagnostic:
    candidate_id: str
    status: str
    reasons: tuple[str, ...]
    evidence: str
    clearance_lower_bound_m: float | None = None
    inflation_m: float | None = None
    progress_m: float | None = None
    effort: float | None = None
    score: float | None = None
    rank: int | None = None
    occupied_segments: int = 0
    unknown_segments: int = 0


@dataclass(frozen=True)
class Evaluation:
    candidates: tuple[CandidateDiagnostic, ...]
    ranked_candidate_ids: tuple[str, ...]
    fallback_reason: str | None
    model_id: str
    map_reference: SpatialReference
    map_capture_ns: int | None
    observation_id: str | None
    calibration_id: str
    alignment_provenance: str
    goal_proposal_id: str | None


def _prefix(mask):
    counts = mask.cumsum(0, dtype=np.int32).cumsum(1, dtype=np.int32).cumsum(2, dtype=np.int32)
    return np.pad(counts, ((1, 0),) * 3)


def _box_counts(prefix, lo, hi):
    a, b, c = lo.T
    x, y, z = hi.T
    return (prefix[x, y, z] - prefix[a, y, z] - prefix[x, b, z] - prefix[x, y, c]
            + prefix[a, b, z] + prefix[a, y, c] + prefix[x, b, c] - prefix[a, b, c])


class _Geometry:
    """One detached evidence grid and two summed-volume tables per candidate batch."""

    def __init__(self, metric, now_ns):
        self.metric = metric
        cells = metric.cells_at(now_ns, metric.reference)
        self.occupied = _prefix(cells == Cell.OCCUPIED)
        # An unexpected class cannot be interpreted as free.
        self.unknown = _prefix((cells != Cell.FREE) & (cells != Cell.OCCUPIED))
        self.shape = np.asarray(cells.shape)

    def check(self, points, radius):
        lo, hi = swept_cell_bounds(points, radius, self.metric.policy.resolution_m, self.metric.origin_index)
        outside = np.any((lo < 0) | (hi > self.shape), axis=1)
        lo, hi = np.clip(lo, 0, self.shape), np.clip(hi, 0, self.shape)
        occupied = _box_counts(self.occupied, lo, hi) > 0
        unknown = outside | (_box_counts(self.unknown, lo, hi) > 0)
        return int(np.count_nonzero(occupied)), int(np.count_nonzero(unknown))


def _shared_reasons(state, metric, alignment, now_ns, policy):
    if state.reference != alignment.ned or metric.reference != alignment.map:
        return ("frame_or_reference_mismatch",)
    if state.reference.clock_id != metric.reference.clock_id:
        return ("clock_domain_mismatch",)
    if state.reference.reference_epoch != metric.reference.epoch:
        return ("reference_epoch_mismatch",)
    stamp = alignment.time_origin_ns + round(state.timestamp_s * NS) if abs(state.timestamp_s) < 1e10 else -1
    reasons = []
    if not stamp_ok(stamp) or not 0 <= now_ns - stamp <= policy.state_age_ns:
        reasons.append("stale_or_future_state")
    if (not 0 <= now_ns - alignment.capture_ns <= policy.alignment_age_ns
            or abs(stamp - alignment.capture_ns) > policy.alignment_skew_ns):
        reasons.append("stale_or_unaligned_transform")
    if (not metric.available or not stamp_ok(metric.capture_ns) or not stamp_ok(metric.query_ns)
            or not metric.capture_ns <= metric.query_ns <= now_ns):
        reasons.append("unavailable_or_future_geometry")
    elif now_ns - metric.capture_ns > policy.map_age_ns:
        reasons.append("stale_geometry")
    return tuple(reasons)


def _resolve_goal(snapshot, goal, now_ns):
    if goal is None:
        return None, None, ()
    metric = snapshot.metric
    origin = snapshot.semantic_time_origin_ns
    if not stamp_ok(origin) or origin > now_ns:
        return None, "unknown", ("invalid_semantic_time",)
    reference = WorldReference(metric.reference.frame_id, metric.reference.epoch, metric.reference.clock_id)
    now = (now_ns - origin) / NS
    context = build_context(snapshot.semantic, goal.context.mission_goal,
                            now=goal.context.context_created_at, reference=reference)
    if context.context_reference != goal.context.context_reference:
        return None, "unknown", ("semantic_context_changed",)
    checked = validate_proposal(goal.proposal, context, now=now, reference=reference)
    if not checked.valid:
        return None, "unknown", ("unvalidated_goal", *checked.reason_codes)
    if goal.proposal.intent == "reject_blocked_candidate":
        return None, "rejected", ("semantic_target_blocked",)
    if goal.proposal.target_entity_id is None:
        return None, "unknown", ("semantic_goal_unavailable",)
    target = next(entity for entity in context.entities if entity.entity_id == goal.proposal.target_entity_id)
    if target.status in ("stale", "uncertain") or now - target.last_observed > context.stale_after_s:
        return None, "unknown", ("semantic_target_stale",)
    return np.asarray(target.geometry.center), None, ()


def _forecast(candidate, state, predictor, history):
    for index, command in enumerate(candidate.schedule):
        start = state.timestamp_s + index * STEP_SECONDS
        end = state.timestamp_s + (index + 1) * STEP_SECONDS
        slack = 2 * max(math.ulp(start), math.ulp(end))
        if (command.reference != state.reference or end <= start
                or command.effective_from_s - start > slack or end - command.effective_until_s > slack):
            raise ValueError("candidate setpoint does not cover the effective interval")
    kwargs = {"history": history} if history is not None else {}
    trajectory = predictor.rollout(state, candidate.schedule, dt=STEP_SECONDS, **kwargs)
    if not isinstance(trajectory, tuple) or len(trajectory) != len(candidate.schedule):
        raise ValueError("forecast shape mismatch")
    effort = 0.
    positions = [state.position_m]
    for index, prediction in enumerate(trajectory, start=1):
        expected = state.timestamp_s + index * STEP_SECONDS
        if (not isinstance(prediction, Prediction) or not isinstance(prediction.state, PredictedState)
                or prediction.state.reference != state.reference
                or abs(prediction.state.timestamp_s - expected) > 4 * math.ulp(expected)):
            raise ValueError("forecast frame or time mismatch")
        accelerations = (*vector3(prediction.acceleration_mps2, "acceleration"),
                         finite_real(prediction.yaw_acceleration_rad_s2, "yaw acceleration"))
        effort += sum(value * value for value in accelerations) * STEP_SECONDS
        positions.append(prediction.state.position_m)
    points = np.asarray(positions)
    if not math.isfinite(effort) or np.max(np.abs(points)) > MAX_LOCAL_M:
        raise ValueError("forecast outside local bounds")
    return points, effort


def _evaluate_one(candidate, state, predictor, history, transform, alignment, vehicle, geometry, goal_point):
    try:
        points, effort = _forecast(candidate, state, predictor, history)
    except (TypeError, ValueError, OverflowError):
        return CandidateDiagnostic(candidate.candidate_id, "rejected", ("invalid_schedule_or_prediction",), "invalid")
    margin = (vehicle.state_position_error_m + vehicle.tracking_error_m + alignment.position_error_m
              + 2 * math.sin(alignment.angle_error_rad / 2) * np.linalg.norm(points, axis=1).max())
    mapped = points @ transform.T + alignment.ned_origin_in_map_m
    if margin > 2 or np.max(np.abs(mapped)) > MAX_LOCAL_M:
        return CandidateDiagnostic(candidate.candidate_id, "unknown", ("uncertainty_or_map_bound",), "unverified")
    radius = vehicle.radius_m + margin
    occupied, unknown = geometry.check(mapped, radius)
    progress = 0. if goal_point is None else float(np.linalg.norm(mapped[0] - goal_point)
                                                  - np.linalg.norm(mapped[-1] - goal_point))
    details = {"inflation_m": float(margin), "progress_m": progress, "effort": effort,
               "occupied_segments": occupied, "unknown_segments": unknown}
    if occupied:
        return CandidateDiagnostic(candidate.candidate_id, "rejected", ("occupied_swept_volume",), "occupied", **details)
    if unknown:
        return CandidateDiagnostic(candidate.candidate_id, "unknown", ("unobserved_swept_volume",), "unverified", **details)
    extra = geometry.check(mapped, radius + CLEARANCE_PROBE_M)
    clearance = CLEARANCE_PROBE_M if not any(extra) else 0.
    return CandidateDiagnostic(candidate.candidate_id, "feasible", ("observed_swept_volume",), "verified_observed_volume",
                               clearance_lower_bound_m=clearance, score=progress - .02 * effort + .1 * clearance,
                               **details)


def evaluate_candidates(state, candidates, snapshot, *, now_ns, alignment, vehicle,
                        goal=None, predictor=None, model_id=None, history=None, policy=EvaluationPolicy()):
    """Rank feasible supplied candidates against current, static observed evidence."""
    if (not isinstance(state, DroneState) or not isinstance(snapshot, SpatialSnapshot)
            or not isinstance(snapshot.metric, MetricSnapshot) or not isinstance(alignment, FrameAlignment)
            or not isinstance(vehicle, VehicleBounds) or not isinstance(policy, EvaluationPolicy)
            or not stamp_ok(now_ns) or (goal is not None and not isinstance(goal, GoalBinding))):
        raise ValueError("invalid evaluation input contract")
    if (not isinstance(candidates, tuple) or not 1 <= len(candidates) <= MAX_CANDIDATES
            or not all(isinstance(candidate, Candidate) for candidate in candidates)
            or len({item.candidate_id for item in candidates}) != len(candidates)):
        raise ValueError("supply one to eight candidates with unique IDs")
    if predictor is None:
        if model_id is not None or history is not None:
            raise ValueError("the default D0 predictor needs no override or history")
        predictor, model_id = DynamicsModel(), "d0"
    elif (not _label(model_id) or not callable(getattr(predictor, "rollout", None))
          or (history is not None and not isinstance(history, MotionHistory))):
        raise ValueError("explicit predictor requires an ID and a rollout API")

    metric = snapshot.metric
    reasons = _shared_reasons(state, metric, alignment, now_ns, policy)
    status, goal_point = "unknown", None
    if not reasons:
        goal_point, status, reasons = _resolve_goal(snapshot, goal, now_ns)
    if reasons:
        diagnostics = tuple(CandidateDiagnostic(item.candidate_id, status, reasons, "unverified")
                            for item in candidates)
    else:
        transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
        geometry = _Geometry(metric, now_ns)
        diagnostics = tuple(_evaluate_one(item, state, predictor, history, transform, alignment, vehicle,
                                         geometry, goal_point) for item in candidates)

    feasible = sorted((entry for entry in diagnostics if entry.status == "feasible"),
                      key=lambda entry: (-entry.score, entry.candidate_id))
    order = tuple(entry.candidate_id for entry in feasible)
    ranks = {name: index for index, name in enumerate(order, start=1)}
    ranked = tuple(replace(entry, rank=ranks.get(entry.candidate_id)) for entry in diagnostics)
    return Evaluation(ranked, order, None if order else "no_verified_candidate", model_id, metric.reference,
                      metric.capture_ns, metric.observation_id, metric.calibration_id,
                      alignment.provenance, None if goal is None else goal.proposal.proposal_id)
