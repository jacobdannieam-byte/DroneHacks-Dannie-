"""Compose one read-only H0 assessment from normalized, in-process inputs."""

from dataclasses import dataclass, replace
from time import perf_counter

from refnav.dynamics import DroneState
from refnav.integration_timing import measure
from refnav.predictive_planner import Plan, plan
from refnav.sitl_review import ProposalReceipt, validate_receipt
from refnav.trajectory_evaluation import (
    EvaluationPolicy, FrameAlignment, GoalBinding, MetricGoal, VehicleBounds,
    _shared_reasons, state_capture_ns,
)
from refnav.world_model.spatial_input import stamp_ok
from refnav.world_model.spatial_pipeline import SpatialSnapshot
from refnav.world_model.spatial_volume import MetricSnapshot


@dataclass(frozen=True)
class HybridAssessment:
    receipt: ProposalReceipt
    validation_reason: str
    objective: MetricGoal | None
    goal: GoalBinding | None
    vehicle_bounds: VehicleBounds


def _input_reason(state, metric, alignment, now_ns, state_received_ns,
                  estimator_reset_counter, estimator_healthy, objective):
    reasons = _shared_reasons(state, metric, alignment, now_ns, EvaluationPolicy())
    if reasons:
        return reasons[0]
    if not stamp_ok(state_received_ns) or not state_capture_ns(state, alignment) <= state_received_ns <= now_ns:
        return "invalid_state_receive_time"
    if not stamp_ok(metric.received_ns) or not metric.capture_ns <= metric.received_ns <= metric.query_ns:
        return "invalid_geometry_receive_time"
    if estimator_healthy is not True:
        return "estimator_unhealthy"
    if type(estimator_reset_counter) is not int or not 0 <= estimator_reset_counter <= 255:
        return "missing_or_invalid_estimator_reset"
    if estimator_reset_counter != alignment.estimator_reset_counter:
        return "estimator_reset_mismatch"
    if objective is None:
        return "mission_goal_missing"
    if objective.reference != metric.reference:
        return "metric_goal_reference_mismatch"
    return None


def assess(state, snapshot, *, now_ns, alignment, vehicle, objective,
           state_received_ns, estimator_reset_counter, estimator_healthy,
           goal=None, budget_s=.025, monotonic=perf_counter, cancelled=None, timings=None,
           predictor=None, model_id=None, history=None):
    """Run D5 and independently review the receipt within its wall-clock budget.

    Health describes the normalized state sample. A caller must advance its
    reference epoch on reset, including counter wrap, before forming new input.
    The returned assessment is diagnostic evidence from this call.
    """
    if (not isinstance(state, DroneState) or not isinstance(snapshot, SpatialSnapshot)
            or not isinstance(snapshot.metric, MetricSnapshot) or not isinstance(alignment, FrameAlignment)
            or not isinstance(vehicle, VehicleBounds) or not stamp_ok(now_ns)
            or (objective is not None and not isinstance(objective, MetricGoal))
            or (goal is not None and not isinstance(goal, GoalBinding))):
        raise ValueError("invalid H0 input contract")

    metric = snapshot.metric
    reason = _input_reason(state, metric, alignment, now_ns, state_received_ns,
                           estimator_reset_counter, estimator_healthy, objective)
    if reason is None:
        decision = measure(timings, "d5_planning", plan, state, snapshot, now_ns=now_ns,
                           alignment=alignment, vehicle=vehicle, objective=objective, goal=goal,
                           predictor=predictor, model_id=model_id, history=history,
                           budget_s=budget_s, monotonic=monotonic, cancelled=cancelled, timings=timings)
    else:
        decision = Plan("safe_stop_requested", reason)

    receipt = ProposalReceipt(decision, state, alignment, metric, now_ns,
                              state_received_ns=state_received_ns, estimator_reset_counter=estimator_reset_counter)
    if reason is None:
        reason = measure(timings, "receipt_validation", validate_receipt, receipt, state=state,
                         alignment=alignment, metric=metric, now_ns=now_ns)
        if decision.decision == "candidate_proposed":
            if cancelled is not None and cancelled():
                reason = "cancelled"
            elif monotonic() >= decision.deadline_monotonic_s:
                reason = "deadline_exceeded"
            if reason != "eligible_for_offline_review":
                receipt = replace(receipt, plan=Plan("safe_stop_requested", reason,
                                                     deadline_monotonic_s=decision.deadline_monotonic_s))
    return HybridAssessment(receipt, reason, objective, goal, vehicle)
