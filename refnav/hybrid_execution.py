"""Revalidate a bounded held position on every owner tick; never substitute a route target."""

from dataclasses import dataclass
import math
from time import perf_counter

from refnav.core import NAV_OFFBOARD
from refnav.dynamics import PositionSetpoint
from refnav.dynamics.history import STEP_SECONDS
from refnav.dynamics.validation import wrap_yaw
from refnav.hybrid_gateway import GatewayDecision, HybridGateway, target_sweep
from refnav.hybrid_integration import _input_reason
from refnav.predictive_planner import STEPS
from refnav.trajectory_evaluation import CLEARANCE_PROBE_M, Candidate, evaluate_candidates, state_capture_ns
from refnav.world_model.spatial_input import NS


MAX_LEAD_M = .65
MAX_SPEED_MPS = 1.
HOLD_SPEED_MPS = .15


@dataclass(frozen=True)
class Held:
    target: tuple
    generation: int
    key: tuple
    reference: object
    calibration: str
    ros_expires_ns: int
    wall_expires_s: float
    clearance_identity: tuple | None = None


class HeldExecutor:
    """A short lease over a position only. Effective PX4 intervals require separate evidence."""

    def __init__(self, *, enabled=False, lease_s=.25, monotonic=perf_counter, timings=None):
        if not .05 <= lease_s <= .30:
            raise ValueError("held target lease must be 50–300 ms")
        self.gateway = HybridGateway(enabled=enabled, monotonic=monotonic, timings=timings,
                                     fallback_to_owner=False)
        self.lease_s, self.clock, self.timings = lease_s, monotonic, timings
        self.held = None
        self.stopping_envelope = None

    def invalidate(self):
        self.held = None
        self.gateway.invalidate()

    def _owner_reason(self, inputs, now_ns, vehicle, ready):
        if not self.gateway.enabled or ready is not True or vehicle.nav_state != NAV_OFFBOARD:
            return "owner_gate_closed"
        if vehicle.armed is not True or vehicle.landed is not False or vehicle.pos_ok is not True:
            return "owner_gate_closed"
        if inputs is None:
            return "observation_unavailable"
        state, alignment, metric = inputs["state"], inputs["alignment"], inputs["snapshot"].metric
        reason = _input_reason(state, metric, alignment, now_ns, inputs["state_received_ns"],
                               inputs["estimator_reset_counter"], inputs["estimator_healthy"], inputs["objective"])
        if reason is not None:
            return reason
        clearance = inputs['snapshot'].clearance
        if clearance is not None:
            if self.stopping_envelope != clearance.stopping_envelope:
                return 'prior_stopping_evidence_mismatch'
            reason = clearance.reason(alignment, now_ns)
            if reason is not None:
                return reason
        if (now_ns - state_capture_ns(state, alignment) > 50_000_000 or vehicle.pos is None
                or not math.dist(vehicle.pos, state.position_m) <= inputs["vehicle"].state_position_error_m
                or vehicle.heading is None
                or not abs(wrap_yaw(vehicle.heading - state.yaw_rad)) <= alignment.angle_error_rad):
            return "owner_state_disagrees_or_stale"
        return None

    @staticmethod
    def _bounded(state, target, *, hold=False):
        speed = math.hypot(*state.velocity_mps)
        return (speed <= (HOLD_SPEED_MPS if hold else MAX_SPEED_MPS)
                and math.dist(state.position_m, target[:3]) <= MAX_LEAD_M
                and abs(state.position_m[2] - target[2]) <= .25 + 1e-9
                and abs(wrap_yaw(state.yaw_rad - target[3])) <= .45 + 1e-9)

    def _checked(self, target, inputs, now_ns, *, deadline, hold=False):
        state, snapshot, alignment = inputs["state"], inputs["snapshot"], inputs["alignment"]
        bounds = inputs["vehicle"]
        if not self._bounded(state, target, hold=hold):
            return "target_outside_execution_limits"
        if self.clock() >= deadline:
            return "execution_recheck_deadline"
        command = PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + STEPS * STEP_SECONDS,
                                   target[:3], target[3])
        candidate = Candidate("standby" if hold else "held", (command,) * STEPS)
        row = evaluate_candidates(state, (candidate,), snapshot, now_ns=now_ns, alignment=alignment,
                                  vehicle=bounds, goal=None if hold else inputs["goal"] or inputs["objective"],
                                  interrupted=lambda: "execution_recheck_deadline" if self.clock() >= deadline else None,
                                  timings=self.timings, stopping_envelope=self.stopping_envelope).candidates[0]
        if (row.status != 'feasible' or row.clearance_lower_bound_m is None
                or row.clearance_lower_bound_m < CLEARANCE_PROBE_M):
            return 'execution_sweep:' + next(iter(row.reasons), 'unverified')
        reason = target_sweep(state, target[:3], snapshot, alignment, bounds, now_ns,
                              stopping_envelope=self.stopping_envelope)
        return "execution_recheck_deadline" if self.clock() >= deadline else reason

    def step(self, *, inputs, assessment, key, now_ns, owner_output, vehicle, ready, generation):
        reason = self._owner_reason(inputs, now_ns, vehicle, ready)
        if reason:
            self.invalidate()
            return GatewayDecision(reason, None)
        state, metric = inputs["state"], inputs["snapshot"].metric
        clearance = inputs['snapshot'].clearance
        identity = clearance.identity if clearance is not None else None
        if assessment is not None:
            self.invalidate()
            self.gateway.offer(assessment, owner_generation=generation)
            result = self.gateway.choose(inputs=inputs, now_ns=now_ns, owner_output=owner_output,
                                         vehicle=vehicle, owner_ready=ready, owner_generation=generation,
                                         simulation=True)
            if result.setpoint is not None and self._bounded(state, result.setpoint):
                if self.stopping_envelope is not None:
                    reason = self._checked(result.setpoint, inputs, now_ns,
                                           deadline=min(assessment.receipt.plan.deadline_monotonic_s, self.clock()+.01))
                    if reason:
                        return GatewayDecision(reason, None)
                self.held = Held(result.setpoint, generation, key, metric.reference, metric.calibration_id,
                                 now_ns + round(self.lease_s * NS), self.clock() + self.lease_s, identity)
                return result
            reason = result.reason if result.setpoint is None else "target_outside_execution_limits"
        elif self.held is not None:
            held = self.held
            if ((held.generation, held.key, held.reference, held.calibration, held.clearance_identity) ==
                    (generation, key, metric.reference, metric.calibration_id, identity) and
                    now_ns < held.ros_expires_ns and self.clock() < held.wall_expires_s):
                reason = self._checked(held.target, inputs, now_ns,
                                       deadline=min(held.wall_expires_s, self.clock() + .01))
                if reason is None:
                    return GatewayDecision("held_target_revalidated", held.target)
            else:
                reason = "held_target_expired_or_changed"
            self.invalidate()
        # Explicitly verified stationary position, never a classical target or an inferred brake.
        target = (*state.position_m, state.yaw_rad)
        hold_reason = self._checked(target, inputs, now_ns, deadline=self.clock() + .01, hold=True)
        if hold_reason is None:
            return GatewayDecision("verified_stationary_hold:" + (reason or "awaiting_proposal"), target)
        return GatewayDecision("no_verified_hold:" + (reason or hold_reason), None)
