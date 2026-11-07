
"""Offline review of one planner result at the existing mission-owner boundary."""

from dataclasses import dataclass

import numpy as np

from refnav.core import NAV_OFFBOARD, Output, Vehicle
from refnav.dynamics import DroneState
from refnav.predictive_planner import MIN_PROGRESS_M, Plan
from refnav.trajectory_evaluation import CLEARANCE_PROBE_M, EvaluationPolicy, FrameAlignment, _shared_reasons
from refnav.world_model.spatial_input import stamp_ok
from refnav.world_model.spatial_volume import MetricSnapshot

MAX_HANDOFF_NS = 50_000_000


@dataclass(frozen=True)
class ProposalReceipt:
    """Bind a diagnostic proposal to its exact in-process inputs and time."""

    plan: Plan
    state: DroneState
    alignment: FrameAlignment
    metric: MetricSnapshot
    decision_ns: int


def review(receipt, *, state, alignment, metric, now_ns, owner_output, vehicle,
           owner_ready):
    """Return a review reason only. The mission owner receives no output from this function."""
    if (not isinstance(state, DroneState) or not isinstance(alignment, FrameAlignment)
            or not isinstance(metric, MetricSnapshot) or not stamp_ok(now_ns)
            or not isinstance(owner_output, Output) or not isinstance(vehicle, Vehicle)):
        return "invalid_review_input"
    if vehicle.nav_state != NAV_OFFBOARD:
        return "offboard_authority_absent"
    if (owner_ready is not True or vehicle.armed is not True or vehicle.landed is not False
            or vehicle.pos_ok is not True or owner_output.phase != "go"
            or owner_output.stream is not True or owner_output.setpoint is None or owner_output.commands):
        return "owner_gate_closed"
    if receipt is None:
        return "proposal_absent"
    if not isinstance(receipt, ProposalReceipt) or not isinstance(receipt.plan, Plan):
        return "invalid_proposal"
    if not stamp_ok(receipt.decision_ns) or not 0 <= now_ns - receipt.decision_ns < MAX_HANDOFF_NS:
        return "proposal_age_exceeded"
    reasons = _shared_reasons(state, metric, alignment, now_ns, EvaluationPolicy())
    if reasons:
        return reasons[0]
    if receipt.state != state or receipt.alignment != alignment or receipt.metric is not metric:
        return "proposal_input_changed"

    if (now_ns != receipt.decision_ns and not np.array_equal(
            metric.cells_at(receipt.decision_ns, metric.reference), metric.cells_at(now_ns, metric.reference))):
        return "geometry_evidence_changed"

    plan = receipt.plan
    if plan.decision != "candidate_proposed":
        return plan.reason
    evaluation = plan.evaluation
    if (evaluation is None or evaluation.fallback_reason is not None or not plan.ranked_ids
            or evaluation.map_reference != metric.reference or evaluation.map_capture_ns != metric.capture_ns
            or evaluation.observation_id != metric.observation_id or evaluation.calibration_id != metric.calibration_id
            or evaluation.alignment_provenance != alignment.provenance):
        return "invalid_proposal"
    first = next((row for row in evaluation.candidates if row.candidate_id == plan.ranked_ids[0]), None)
    if (first is None or first.status != "feasible" or first.evidence != "verified_observed_volume"
            or first.clearance_lower_bound_m is None or first.clearance_lower_bound_m < CLEARANCE_PROBE_M
            or first.progress_m is None or first.progress_m < MIN_PROGRESS_M or first.candidate_id == "hold"):
        return "invalid_proposal"
    return "eligible_for_offline_review"
