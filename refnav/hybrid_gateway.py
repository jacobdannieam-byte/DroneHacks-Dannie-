"""Disabled-by-default, one-shot proposal selection for the existing owner."""

from dataclasses import dataclass
import math
from time import perf_counter

import numpy as np

from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.dynamics.validation import wrap_yaw
from refnav.hybrid_integration import HybridAssessment, _input_reason
from refnav.integration_timing import measure
from refnav.map_assisted_clearance import AUTHORIZED_STATIC, clearance_sweep
from refnav.predictive_planner import MIN_PROGRESS_M, candidate_for_id, has_progress
from refnav.sitl_review import review
from refnav.trajectory_evaluation import (
    CLEARANCE_PROBE_M, VehicleBounds, evaluate_candidates, state_capture_ns,
)
from refnav.world_model.spatial_input import MAX_LOCAL_M, stamp_ok
from refnav.world_model.spatial_pipeline import SpatialSnapshot


@dataclass(frozen=True)
class GatewayDecision:
    reason: str
    setpoint: tuple | None


class HybridGateway:
    """Return a position/yaw tuple for one owner tick, with classical fallback."""

    def __init__(self, *, enabled=False, monotonic=perf_counter, timings=None, fallback_to_owner=True):
        self.enabled = enabled is True
        self.monotonic = monotonic
        self.fallback_to_owner = fallback_to_owner
        self.timings = timings
        self._pending = None

    @property
    def pending(self):
        return self._pending is not None

    def offer(self, assessment, *, owner_generation):
        self.invalidate()
        if self.enabled and isinstance(assessment, HybridAssessment) and stamp_ok(owner_generation):
            self._pending = (assessment, owner_generation)

    def invalidate(self):
        self._pending = None

    def choose(self, *, inputs, now_ns, owner_output, vehicle, owner_ready, owner_generation, simulation=False):
        pending = self._pending
        self.invalidate()
        return measure(self.timings if self.enabled else None, "owner_handoff", self._choose,
                       pending, inputs, now_ns,
                       owner_output, vehicle, owner_ready, owner_generation, simulation)

    def _choose(self, pending, inputs, now_ns, out, vehicle, owner_ready, generation, simulation):
        fallback = out.setpoint if self.fallback_to_owner else None
        if not self.enabled or simulation is not True:
            return GatewayDecision("gateway_disabled", fallback)
        if pending is None:
            return GatewayDecision("proposal_absent", fallback)
        assessment, offered_generation = pending
        if generation != offered_generation:
            return GatewayDecision("owner_generation_changed", fallback)
        if inputs is None:
            return GatewayDecision("observation_unavailable", fallback)
        snapshot, state, alignment = (inputs[key] for key in ("snapshot", "state", "alignment"))
        if not isinstance(snapshot, SpatialSnapshot):
            return GatewayDecision("invalid_review_input", fallback)
        receipt = assessment.receipt
        reason = review(receipt, state=state, alignment=alignment, metric=snapshot.metric, now_ns=now_ns,
                        owner_output=out, vehicle=vehicle, owner_ready=owner_ready, clearance=snapshot.clearance)
        if reason != "eligible_for_offline_review":
            return GatewayDecision(reason, fallback)
        bounds, objective, goal = (inputs[key] for key in ("vehicle", "objective", "goal"))
        if (goal is not None and goal.selection is not None
                and not goal.selection.model_identity.startswith('G0:')):
            return GatewayDecision('experimental_reasoner_is_shadow_only', fallback)
        if (not isinstance(bounds, VehicleBounds) or bounds != assessment.vehicle_bounds
                or objective != assessment.objective or goal != assessment.goal
                or receipt.state_received_ns != inputs["state_received_ns"]
                or receipt.estimator_reset_counter != inputs["estimator_reset_counter"]):
            return GatewayDecision("proposal_context_changed", fallback)
        reason = _input_reason(state, snapshot.metric, alignment, now_ns, inputs["state_received_ns"],
                               inputs["estimator_reset_counter"], inputs["estimator_healthy"], objective)
        if reason:
            return GatewayDecision(reason, fallback)
        position_error = math.inf if vehicle.pos is None else math.dist(vehicle.pos, state.position_m)
        if (now_ns - state_capture_ns(state, alignment) > 50_000_000
                or not math.isfinite(position_error) or position_error > bounds.state_position_error_m
                or vehicle.heading is None or not math.isfinite(vehicle.heading)
                or abs(wrap_yaw(vehicle.heading - state.yaw_rad)) > alignment.angle_error_rad):
            return GatewayDecision("owner_state_disagrees_or_stale", fallback)

        plan = receipt.plan
        deadline = plan.deadline_monotonic_s
        if deadline is None or not math.isfinite(deadline) or self.monotonic() >= deadline:
            return GatewayDecision("handoff_deadline_exceeded", fallback)
        if plan.evaluation.model_id != "d0":
            return GatewayDecision("unsupported_proposal_model", fallback)
        candidate = candidate_for_id(state, objective, alignment, plan.ranked_ids[0],
                                     goal=goal, snapshot=snapshot, now_ns=now_ns)
        if candidate is None or any(command != candidate.schedule[0] for command in candidate.schedule):
            return GatewayDecision("unsupported_position_schedule", fallback)
        checked = measure(self.timings, "gateway_d3_recheck", evaluate_candidates, state, (candidate,), snapshot,
                          now_ns=now_ns, alignment=alignment, vehicle=bounds, goal=goal or objective,
                          interrupted=lambda: "handoff_deadline_exceeded" if self.monotonic() >= deadline else None,
                          timings=self.timings)
        row = checked.candidates[0]
        if self.monotonic() >= deadline:
            return GatewayDecision("handoff_deadline_exceeded", fallback)
        if (row.status != "feasible" or row.clearance_lower_bound_m is None
                or row.clearance_lower_bound_m < CLEARANCE_PROBE_M
                or not has_progress(row)):
            return GatewayDecision("sweep_rejected:" + next(iter(row.reasons), 'unverified'), fallback)

        target = candidate.schedule[0]
        # D0's horizon can end short of the held target. Check the entire
        # requested displacement as well as the predicted curved sweep.
        reason = target_sweep(state, target.position_m, snapshot, alignment, bounds, now_ns)
        if self.monotonic() >= deadline:
            return GatewayDecision("handoff_deadline_exceeded", fallback)
        if reason:
            return GatewayDecision(reason, fallback)
        return GatewayDecision("hybrid_position_selected", (*target.position_m, target.yaw_rad))


def target_sweep(state, target, snapshot, alignment, bounds, now_ns, *, stopping_envelope=None):
    """Independently check the whole displacement to a held target."""
    points = np.asarray((state.position_m, target))
    transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
    margin = (bounds.state_position_error_m + bounds.tracking_error_m + alignment.position_error_m
              + 2 * math.sin(alignment.angle_error_rad / 2) * np.linalg.norm(points, axis=1).max())
    mapped = points @ transform.T + alignment.ned_origin_in_map_m
    clearance = snapshot.clearance
    if clearance is not None:
        if stopping_envelope is not None and stopping_envelope != clearance.stopping_envelope:
            return 'prior_stopping_evidence_mismatch'
        stopping_envelope = clearance.stopping_envelope
    radius = bounds.radius_m
    if stopping_envelope is not None:
        reason = stopping_envelope.speed_reason(math.hypot(*state.velocity_mps))
        if reason:
            return reason
        radius = stopping_envelope.sweep_radius(bounds)
    if margin > 2 or radius + CLEARANCE_PROBE_M > 2 or np.max(np.abs(mapped)) > MAX_LOCAL_M:
        return "target_uncertainty_outside_bound"
    sweep = clearance_sweep(snapshot, mapped, alignment=alignment, now_ns=now_ns,
                            vehicle_radius_m=radius + CLEARANCE_PROBE_M, margin_m=margin)
    reason = sweep.status if clearance is None else sweep.reason
    return None if sweep.status in ('verified_free', AUTHORIZED_STATIC) else 'target_sweep_rejected:' + reason
