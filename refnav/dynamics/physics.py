"""Deterministic, reduced-order closed-loop position and heading tracking."""

from collections.abc import Iterable, Mapping, Set
from dataclasses import dataclass
from itertools import repeat
from math import isfinite, ulp

from .schema import DroneState, PositionSetpoint, PredictedState, Prediction, Vector3
from .validation import finite_real, vector3, wrap_yaw

MIN_DT_S = 0.001
MAX_DT_S = 0.05
MAX_STEPS = 250
MAX_HORIZON_S = 5.0


@dataclass(frozen=True, slots=True)
class NominalParameters:
    """Synthetic closed-loop parameters for D0 experiments."""

    position_gain_per_s: float = 1.2
    velocity_tau_s: float = 0.35
    speed_limits_mps: Vector3 = (1.2, 1.2, 0.7)
    acceleration_limits_mps2: Vector3 = (2.0, 2.0, 1.2)
    yaw_gain_per_s: float = 2.0
    yaw_tau_s: float = 0.18
    yaw_rate_limit_rad_s: float = 1.5
    yaw_acceleration_limit_rad_s2: float = 4.0

    def __post_init__(self):
        for name in ("position_gain_per_s", "velocity_tau_s", "yaw_gain_per_s", "yaw_tau_s",
                     "yaw_rate_limit_rad_s", "yaw_acceleration_limit_rad_s2"):
            value = finite_real(getattr(self, name), name)
            if value <= 0.0:
                raise ValueError(f"{name} must be positive")
            object.__setattr__(self, name, value)
        for name in ("speed_limits_mps", "acceleration_limits_mps2"):
            limits = vector3(getattr(self, name), name)
            if min(limits) <= 0.0:
                raise ValueError(f"{name} components must be positive")
            object.__setattr__(self, name, limits)


def _clip(value, limit):
    return min(limit, max(-limit, value))


@dataclass(frozen=True, slots=True)
class DynamicsModel:
    parameters: NominalParameters = NominalParameters()

    def __post_init__(self):
        if not isinstance(self.parameters, NominalParameters):
            raise ValueError("parameters must be NominalParameters")

    def predict(self, state: DroneState | PredictedState, command: PositionSetpoint,
                *, dt: float) -> Prediction:
        """Advance one step using the command effective throughout the step."""
        dt = self._validate_dt(dt)
        self._validate_state(state)
        return self._advance(state, command, dt, state.timestamp_s + dt)

    def rollout(self, initial_state: DroneState | PredictedState,
                commands: PositionSetpoint | Iterable[PositionSetpoint], *,
                dt: float, steps: int | None = None) -> tuple[Prediction, ...]:
        """Return one prediction per step. Supply steps for one held command.

        A command iterable selects one held action per integration step. Its
        length sets the horizon. A failed step aborts the entire result.
        """
        dt = self._validate_dt(dt)
        self._validate_state(initial_state)
        if isinstance(commands, PositionSetpoint):
            if type(steps) is not int:
                raise ValueError("a held command requires an integer step count")
            self._validate_horizon(steps, dt)
            schedule = repeat(commands, steps)
        else:
            if steps is not None or isinstance(commands, (str, bytes, Mapping, Set)):
                raise ValueError("a command iterable supplies one ordered action per step")
            try:
                schedule = iter(commands)
            except TypeError as error:
                raise ValueError("commands must be a setpoint or an iterable of setpoints") from error

        result = []
        state = initial_state
        for index, command in enumerate(schedule, start=1):
            self._validate_horizon(index, dt)
            end_s = initial_state.timestamp_s + index * dt
            prediction = self._advance(state, command, dt, end_s)
            result.append(prediction)
            state = prediction.state
        if not result:
            raise ValueError("rollout requires at least one step")
        return tuple(result)

    def _validate_dt(self, dt):
        dt = finite_real(dt, "dt")
        p = self.parameters
        if not MIN_DT_S <= dt <= MAX_DT_S:
            raise ValueError(f"dt must be in [{MIN_DT_S}, {MAX_DT_S}] seconds")
        if (dt > min(p.velocity_tau_s, p.yaw_tau_s) / 2
                or dt * max(p.position_gain_per_s, p.yaw_gain_per_s) > 1):
            raise ValueError("dt exceeds the integration bound for the configured response")
        return dt

    @staticmethod
    def _validate_state(state):
        if not isinstance(state, (DroneState, PredictedState)):
            raise ValueError("state must be DroneState or PredictedState")

    @staticmethod
    def _validate_horizon(steps, dt):
        if not 1 <= steps <= MAX_STEPS:
            raise ValueError(f"rollout must have between 1 and {MAX_STEPS} steps")
        if steps * dt > MAX_HORIZON_S:
            raise ValueError(f"rollout exceeds the {MAX_HORIZON_S:g} second horizon")

    def _advance(self, state, command, dt, end_s):
        if not isinstance(command, PositionSetpoint):
            raise ValueError("command must be a PositionSetpoint")
        if command.reference != state.reference:
            raise ValueError("command and state must share a frame, reference epoch and clock")

        start_s = state.timestamp_s
        if not isfinite(end_s) or end_s <= start_s:
            raise ValueError("prediction timestamp must advance")
        # Permit rounding at a grid boundary, including Unix-epoch seconds.
        slack_s = 2 * max(ulp(start_s), ulp(end_s))
        if slack_s > dt / 1024:
            raise ValueError("timestamp resolution is too coarse for dt")
        if (command.effective_from_s - start_s > slack_s
                or end_s - command.effective_until_s > slack_s):
            raise ValueError("command interval does not cover the prediction step")

        velocity_reference, acceleration, rate_reference, yaw_acceleration = self._tracking(state, command, dt)
        velocity = tuple(v + a * dt for v, a in zip(state.velocity_mps, acceleration))
        position = tuple(x + v * dt for x, v in zip(state.position_m, velocity))
        yaw_rate = state.yaw_rate_rad_s + yaw_acceleration * dt
        yaw = wrap_yaw(state.yaw_rad + yaw_rate * dt)
        predicted = PredictedState(state.reference, end_s, position, velocity, yaw, yaw_rate)
        return Prediction(predicted, velocity_reference, acceleration, rate_reference, yaw_acceleration)

    def _tracking(self, state, command, dt):
        p = self.parameters
        velocity_reference = tuple(
            _clip(p.position_gain_per_s * (target - position), limit)
            for target, position, limit in zip(command.position_m, state.position_m, p.speed_limits_mps)
        )
        acceleration = tuple(
            _clip((target - velocity) / p.velocity_tau_s, limit)
            for target, velocity, limit in zip(velocity_reference, state.velocity_mps, p.acceleration_limits_mps2)
        )
        yaw_error = wrap_yaw(command.yaw_rad - state.yaw_rad)
        rate_reference = _clip(p.yaw_gain_per_s * yaw_error, p.yaw_rate_limit_rad_s)
        yaw_acceleration = _clip((rate_reference - state.yaw_rate_rad_s) / p.yaw_tau_s,
                                 p.yaw_acceleration_limit_rad_s2)
        return velocity_reference, acceleration, rate_reference, yaw_acceleration
