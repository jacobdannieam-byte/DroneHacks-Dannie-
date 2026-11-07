
"""Bounded simulation-only proposal ranking over verified Phase 4 geometry."""

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin
from time import perf_counter

from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.dynamics import DroneState, PositionSetpoint
from refnav.dynamics.history import STEP_SECONDS
from refnav.dynamics.validation import finite_real, wrap_yaw
from refnav.trajectory_evaluation import (
    CLEARANCE_PROBE_M, Candidate, Evaluation, FrameAlignment, GoalBinding, MetricGoal, evaluate_candidates,
)

STEPS = 50
OPTIONS = (
    ("forward", 1., 0.),
    ("short", .50, 0.),
    ("left", .85, -pi / 5),
    ("right", .85, pi / 5),
    ("wide-left", .70, -pi / 2.8),
    ("wide-right", .70, pi / 2.8),
)
MIN_PROGRESS_M = .015


@dataclass(frozen=True)
class Plan:
    """Decision and diagnostic candidate IDs for simulation analysis."""

    decision: str
    reason: str
    ranked_ids: tuple[str, ...] = ()
    evaluation: Evaluation | None = None


def _generate(state, objective, alignment):
    transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
    offset = [p - q for p, q in zip(objective.position_m, alignment.ned_origin_in_map_m)]
    target = transform.T @ offset
    north, east, down = (t - p for t, p in zip(target, state.position_m))
    distance = min(1.2, hypot(north, east))
    bearing = atan2(east, north)
    climb = max(-.25, min(.25, down))
    start, end = state.timestamp_s, state.timestamp_s + STEPS * STEP_SECONDS

    def held(name, position, yaw):
        command = PositionSetpoint(state.reference, start, end, position, yaw)
        return Candidate(name, (command,) * STEPS)

    yield held("hold", state.position_m, state.yaw_rad)
    if distance < .015:
        if abs(climb) >= .015:
            yield held("vertical", (*state.position_m[:2], state.position_m[2] + climb), state.yaw_rad)
        return
    lateral_speed = -sin(bearing) * state.velocity_mps[0] + cos(bearing) * state.velocity_mps[1]
    lateral = max(-.85, min(.85, -.9 * lateral_speed))
    forward = distance * .55
    yield held("drift-check", (state.position_m[0] + forward * cos(bearing) - lateral * sin(bearing),
                              state.position_m[1] + forward * sin(bearing) + lateral * cos(bearing),
                              state.position_m[2] + .55 * climb), state.yaw_rad)
    for name, scale, offset in OPTIONS:
        heading = bearing + offset
        yaw_step = max(-.45, min(.45, wrap_yaw(heading - state.yaw_rad)))
        yield held(name, (state.position_m[0] + distance * scale * cos(heading),
                          state.position_m[1] + distance * scale * sin(heading),
                          state.position_m[2] + climb * scale), state.yaw_rad + yaw_step)


def plan(state, snapshot, *, now_ns, alignment, vehicle, objective, goal=None,
         predictor=None, model_id=None, history=None, min_clearance_m=CLEARANCE_PROBE_M,
         budget_s=.025, monotonic=perf_counter, cancelled=None):
    """Generate up to eight local candidates and return IDs, or request a safe stop."""
    clearance = finite_real(min_clearance_m, "minimum clearance")
    budget = finite_real(budget_s, "planner budget")
    if (not isinstance(state, DroneState) or not isinstance(alignment, FrameAlignment)
            or not isinstance(objective, MetricGoal) or (goal is not None and not isinstance(goal, GoalBinding))
            or not 0 < clearance <= CLEARANCE_PROBE_M or not 0 < budget <= .05):
        raise ValueError("invalid simulation planner contract")
    if objective.reference != alignment.map or state.reference != alignment.ned:
        return Plan("safe_stop_requested", "objective_or_state_reference_mismatch")

    deadline = monotonic() + budget

    def interrupted():
        if cancelled is not None and cancelled():
            return "cancelled"
        if monotonic() >= deadline:
            return "deadline_exceeded"
        return None

    reason = interrupted()
    if reason:
        return Plan("safe_stop_requested", reason)
    candidates = tuple(_generate(state, objective, alignment))
    evaluation = evaluate_candidates(state, candidates, snapshot, now_ns=now_ns, alignment=alignment,
                                     vehicle=vehicle, goal=objective if goal is None else goal,
                                     predictor=predictor, model_id=model_id, history=history,
                                     interrupted=interrupted)
    reason = interrupted()
    if reason or evaluation.fallback_reason in ("cancelled", "deadline_exceeded"):
        return Plan("safe_stop_requested", reason or evaluation.fallback_reason)
    eligible = [entry for entry in evaluation.candidates if entry.candidate_id != "hold" and entry.status == "feasible"
                and entry.clearance_lower_bound_m >= clearance and entry.progress_m >= MIN_PROGRESS_M]
    ordered = sorted(eligible, key=lambda entry: (-(entry.score - .10 * entry.inflation_m), entry.candidate_id))
    reason = interrupted()
    if reason:
        return Plan("safe_stop_requested", reason)
    if not ordered:
        return Plan("safe_stop_requested", "no_verified_progress", evaluation=evaluation)
    return Plan("candidate_proposed", "verified_progress", tuple(entry.candidate_id for entry in ordered), evaluation)
