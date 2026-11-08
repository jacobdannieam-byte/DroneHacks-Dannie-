"""Explicitly approved simulator experiment, separate from the general D3 gateway.

The PINN selects actual held position commands. No Classical navigation fallback.
The participant-map corridor is a prior, never represented as observed free space.
"""
from dataclasses import asdict
from hashlib import sha256
import json
import math
import os
from pathlib import Path
from time import perf_counter

import numpy as np

from refnav.calibration_trial import CalibrationTrial, Intent, NS, ZERO
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP

REFERENCE = LocalNED('px4_local_ned', 0, 'ros_sim_time')


def digest(path):
    return sha256(Path(path).read_bytes()).hexdigest()


class DecisionLog:
    """Append-only durable pre-publication decisions and post-publication receipts."""
    def __init__(self, path):
        self.stream = Path(path).open('x', encoding='utf-8')
        self.previous = '0' * 64

    def write(self, row):
        row = dict(row, previous_sha256=self.previous)
        raw = json.dumps(row, sort_keys=True, separators=(',', ':'), allow_nan=False)
        self.previous = sha256(raw.encode()).hexdigest()
        self.stream.write(json.dumps(dict(row, sha256=self.previous), allow_nan=False) + '\n')
        self.stream.flush()
        os.fsync(self.stream.fileno())

    def close(self):
        self.stream.close()


class ModelSelector:
    def __init__(self, config, *, model=None, monotonic=perf_counter):
        self.config = config
        path = config['checkpoint']
        if digest(path) != config['checkpoint_sha256']:
            raise ValueError('Model checkpoint identity mismatch')
        self.model = model if model is not None else ResidualDynamicsModel(network=ResidualMLP.load(path))
        self.nominal = DynamicsModel()
        self.monotonic = monotonic
        self.sequence = 0

    def candidates(self, state, goal):
        delta = np.asarray(goal) - state.position_m
        horizontal = np.linalg.norm(delta[:2])
        direction = delta[:2] / max(horizontal, 1e-12)
        for index, lead in enumerate((0., .08, .14, .20)):
            step = direction * min(lead, horizontal)
            target = (state.position_m[0] + float(step[0]), state.position_m[1] + float(step[1]),
                      state.position_m[2] + float(np.clip(delta[2], -.04, .04)))
            yield str(index), PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + .6,
                                               target, state.yaw_rad)

    def evaluate(self, model, state, target, goal):
        predictions = model.rollout(state, target, dt=.02, steps=30)
        points = np.asarray([state.position_m, target.position_m] + [p.state.position_m for p in predictions])
        low, high = self.config['command_low_ned_m'], self.config['command_high_ned_m']
        inside = bool(np.all(points >= low) and np.all(points <= high)
                      and np.all(np.abs(points[:,1]-self.config['route_east_per_north']*points[:,0]) <= self.config['cross_track_limit_m']))
        peak = max(math.hypot(*state.velocity_mps), *(math.hypot(*p.state.velocity_mps) for p in predictions))
        last = predictions[-1].state
        # Terminal error plus terminal speed: residual dynamics change this cost.
        score = math.dist(last.position_m, goal) + .15 * math.hypot(*last.velocity_mps)
        return dict(score=score, eligible=inside and peak <= .35, peak_speed_mps=peak,
                    endpoint_ned_m=last.position_m, endpoint_velocity_mps=last.velocity_mps,
                    reason='eligible' if inside and peak <= .35 else 'corridor_or_predicted_speed',
                    trajectory_ned_m=[p.state.position_m for p in predictions])

    def choose(self, state, goal, now_ns):
        began = self.monotonic()
        if not 0 <= now_ns - round(state.timestamp_s * NS) <= 100_000_000:
            raise ValueError('Model input stale or future')
        candidates = list(self.candidates(state, goal))
        rows = [dict(candidate_id=key, command=asdict(command),
                     pinn=self.evaluate(self.model, state, command, goal),
                     d0=self.evaluate(self.nominal, state, command, goal)) for key, command in candidates]
        def best(kind):
            allowed = [r for r in rows if r[kind]['eligible']]
            return min(allowed, key=lambda r: (r[kind]['score'], r['candidate_id'])) if allowed else None
        chosen, nominal = best('pinn'), best('d0')
        elapsed = self.monotonic() - began
        if elapsed >= .04:
            raise ValueError('Model decision deadline exceeded')
        if chosen is None:
            raise ValueError('No model candidate admissible')
        self.sequence += 1
        row = dict(kind='decision', sequence=self.sequence, state=asdict(state), goal_ned_m=goal,
                   decision_ros_ns=now_ns, model_sha256=self.config['checkpoint_sha256'],
                   selected=chosen['candidate_id'], d0_selected=None if nominal is None else nominal['candidate_id'],
                   candidates=rows, elapsed_s=elapsed, fallback=False, registration_enabled=False)
        return candidates[int(chosen['candidate_id'])][1], row


class ModelFlight(CalibrationTrial):
    """Reuse native admission, climb, abort, LAND, authority and timeout behavior."""
    def __init__(self, plan, config, log, *, selector=None):
        super().__init__(plan)
        self.config = config
        self.log = log
        self.selector = selector or ModelSelector(config)
        self.waypoint = 0
        self.pending_model = None
        self.model_commands = 0
        self.prior_heading = None

    def step(self, ros_ns, wall_ns, observation, *, admitted):
        if observation is not None and self.phase not in ('wait','prime','mode','arm','done'):
            n,e,_ = observation.position
            if abs(e-self.config['route_east_per_north']*n) > self.config['cross_track_limit_m']:
                if self.phase == 'land':
                    self.recovery_reason = self.recovery_reason or 'model_corridor_exceeded'
                else:
                    self._operator_land = True
        intent = super().step(ros_ns,wall_ns,observation,admitted=admitted)
        return intent

    def transit(self, obs, offset, vertical_target, ros_ns, wall_ns):
        if max(self._elapsed(ros_ns, wall_ns)) >= self.plan.transit_timeout_s:
            return self._land('model_transit_deadline', ros_ns, wall_ns)
        if obs.heading_rad is None or not math.isfinite(obs.heading_rad):
            return self._land('model_heading_unavailable', ros_ns, wall_ns)
        goals = self.config['waypoints_ned_m']
        goal = goals[self.waypoint]
        if math.dist(obs.position, goal) <= .06 and math.hypot(*obs.velocity) <= .05:
            if self._settled(obs, math.hypot(*obs.velocity)):
                self.waypoint += 1
                self._settling = None
                if self.waypoint == len(goals):
                    if self.model_commands < 10:
                        return self._land('insufficient_model_authority_evidence', ros_ns, wall_ns)
                    self._enter('brake', ros_ns, wall_ns)
                    return self._issue(Intent(ZERO, event='brake_command'), ros_ns)
                goal = goals[self.waypoint]
        else:
            self._settling = None
        rate = 0.
        if self.prior_heading is not None:
            heading, stamp = self.prior_heading
            if obs.position_stamp_ns > stamp:
                rate = math.atan2(math.sin(obs.heading_rad-heading), math.cos(obs.heading_rad-heading)) / ((obs.position_stamp_ns-stamp)/NS)
        self.prior_heading = obs.heading_rad, obs.position_stamp_ns
        state = DroneState(REFERENCE, obs.position_stamp_ns / NS, obs.position, obs.velocity, obs.heading_rad, rate)
        try:
            command, decision = self.selector.choose(state, goal, ros_ns)
            self.log.write(dict(decision, waypoint=self.waypoint, phase='model_transit'))
        except (ValueError, OSError, ArithmeticError) as error:
            # No nominal or route-follower command is allowed to complete this trial.
            self.log.write(dict(kind='abort', reason=str(error), ros_ns=ros_ns, fallback=False))
            return self._land('model_rejected:' + str(error), ros_ns, wall_ns)
        self.pending_model = decision
        return self._issue(Intent(position_ned_m=command.position_m, yaw_rad=command.yaw_rad), ros_ns)

    def published(self, intent, stamp_ns, px4_issue_us, *, wall_ns=None):
        event = super().published(intent, stamp_ns, px4_issue_us, wall_ns=wall_ns)
        if intent.position_ned_m is not None:
            if self.pending_model is None:
                raise ValueError('Position publication without model decision')
            self.log.write(dict(kind='published', sequence=self.pending_model['sequence'],
                px4_issue_us=px4_issue_us, stamp_ns=stamp_ns, position_ned_m=intent.position_ned_m,
                yaw_rad=intent.yaw_rad, model_sha256=self.config['checkpoint_sha256']))
            self.pending_model = None
            self.model_commands += 1
        return event
