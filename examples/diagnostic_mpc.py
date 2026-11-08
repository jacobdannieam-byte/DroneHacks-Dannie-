"""Offline classical/learned-model receding-horizon optimizer, diagnostic only.

Bounded coordinate search optimizes two held position/yaw setpoints. It uses the
actual closed-loop command contract and D3 constraints. No D5 ID or execution
artifact is produced, so the gateway cannot mistake this for a reviewed plan.
"""
from dataclasses import dataclass
import math
from time import perf_counter

import numpy as np

from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.dynamics import PositionSetpoint
from refnav.trajectory_evaluation import Candidate, evaluate_candidates, CLEARANCE_PROBE_M

# Our own offline optimization budget and bounds; not a runtime qualification.
ITERATIONS = 3
STEPS = 50
STEP_SECONDS = .02
MAX_DISPLACEMENT_METRES = .65
MAX_YAW_RADIANS = .45


@dataclass(frozen=True)
class DiagnosticOptimization:
    status: str
    candidate: Candidate | None
    objective_score: float | None
    evaluations: int
    elapsed_seconds: float


def schedule(state, displacement, yaw_delta, identifier):
    delta = np.asarray(displacement, dtype=float)
    if not np.isfinite(delta).all() or not math.isfinite(yaw_delta):
        raise ValueError('Optimizer proposed a nonfinite position or heading.')
    length = np.linalg.norm(delta)
    if length > MAX_DISPLACEMENT_METRES:
        delta *= MAX_DISPLACEMENT_METRES / length
    delta[2] = np.clip(delta[2], -.25, .25)
    yaw_delta = float(np.clip(yaw_delta, -MAX_YAW_RADIANS, MAX_YAW_RADIANS))
    midpoint = state.timestamp_s + .5
    end = state.timestamp_s + 1.
    first = PositionSetpoint(state.reference, state.timestamp_s, midpoint,
                             np.asarray(state.position_m) + delta * .5, state.yaw_rad + yaw_delta * .5)
    second = PositionSetpoint(state.reference, midpoint, end,
                              np.asarray(state.position_m) + delta, state.yaw_rad + yaw_delta)
    return Candidate(identifier, (first,) * 25 + (second,) * 25)


def optimize(state, snapshot, alignment, vehicle, objective, now_ns, *, predictor=None,
             model_id=None, budget_seconds=.05, monotonic=perf_counter, cancelled=None):
    if not math.isfinite(budget_seconds) or not 0 < budget_seconds <= 1:
        raise ValueError('Use a finite offline optimization budget of at most one second.')
    start = monotonic()
    deadline = start + budget_seconds
    transform = rot_z(alignment.map_from_enu_yaw_rad) @ R_ENU_NED
    target = transform.T @ (np.asarray(objective.position_m) - alignment.ned_origin_in_map_m)
    delta = target - state.position_m
    delta *= min(1., .5 / max(np.linalg.norm(delta), .001))
    center = np.array((delta[0], delta[1], delta[2], 0.))
    best, score, evaluations = None, None, 0

    def expired():
        if cancelled is not None and cancelled():
            return 'cancelled'
        return 'deadline_exceeded' if monotonic() >= deadline else None

    for iteration in range(ITERATIONS):
        reason = expired()
        if reason:
            return DiagnosticOptimization(reason, None, None, evaluations, monotonic() - start)
        parameters = [center.copy()]
        for axis in (0, 1, 3):
            for direction in (-1, 1):
                value = center.copy()
                value[axis] += direction * .15 / (iteration + 1)
                parameters.append(value)
        options = tuple(schedule(state, p[:3], p[3], f'mpc-{iteration}-{i}') for i, p in enumerate(parameters))
        try:
            result = evaluate_candidates(state, options, snapshot, now_ns=now_ns, alignment=alignment,
                                         vehicle=vehicle, goal=objective, predictor=predictor,
                                         model_id=model_id, interrupted=expired)
        except (ValueError, ArithmeticError):
            return DiagnosticOptimization('invalid_model_or_candidate', None, None, evaluations, monotonic() - start)
        evaluations += len(options)
        reason = expired()
        if reason or result.fallback_reason in ('cancelled', 'deadline_exceeded'):
            return DiagnosticOptimization(reason or result.fallback_reason, None, None, evaluations, monotonic() - start)
        eligible = [(i, row) for i, row in enumerate(result.candidates)
                    if row.status == 'feasible' and row.clearance_lower_bound_m >= CLEARANCE_PROBE_M
                    and row.progress_m >= .015 and math.isfinite(row.score)]
        if not eligible:
            return DiagnosticOptimization('infeasible', None, None, evaluations, monotonic() - start)
        index, selected = max(eligible, key=lambda pair: pair[1].score)
        center, best, score = parameters[index], options[index], selected.score
    return DiagnosticOptimization('diagnostic_only', best, score, evaluations, monotonic() - start)
