"""Immutable SI state and effective position/yaw command records."""

from dataclasses import dataclass

from .validation import finite_real, vector3, wrap_yaw

Vector3 = tuple[float, float, float]


@dataclass(frozen=True, slots=True)
class LocalNED:
    """Identity of a local north/east/down frame and its caller-owned clock."""

    frame_id: str
    reference_epoch: int
    clock_id: str

    def __post_init__(self):
        for value in (self.frame_id, self.clock_id):
            if not isinstance(value, str) or not value.strip():
                raise ValueError("frame_id and clock_id must be nonempty strings")
        if type(self.reference_epoch) is not int or self.reference_epoch < 0:
            raise ValueError("reference_epoch must be a nonnegative integer")


@dataclass(frozen=True, slots=True)
class _MotionState:
    reference: LocalNED
    timestamp_s: float
    position_m: Vector3
    velocity_mps: Vector3
    yaw_rad: float
    yaw_rate_rad_s: float

    def __post_init__(self):
        if not isinstance(self.reference, LocalNED):
            raise ValueError("state requires a LocalNED reference")
        object.__setattr__(self, "timestamp_s", finite_real(self.timestamp_s, "timestamp_s"))
        object.__setattr__(self, "position_m", vector3(self.position_m, "position_m"))
        object.__setattr__(self, "velocity_mps", vector3(self.velocity_mps, "velocity_mps"))
        object.__setattr__(self, "yaw_rad", wrap_yaw(finite_real(self.yaw_rad, "yaw_rad")))
        object.__setattr__(self, "yaw_rate_rad_s", finite_real(self.yaw_rate_rad_s, "yaw_rate_rad_s"))


@dataclass(frozen=True, slots=True)
class DroneState(_MotionState):
    """Estimated physical state at its capture time, or a synthetic sample."""


@dataclass(frozen=True, slots=True)
class PredictedState(_MotionState):
    """Model output at a future time, suitable for autoregressive prediction."""


@dataclass(frozen=True, slots=True)
class PositionSetpoint:
    """Absolute NED position and heading held on [effective_from_s, effective_until_s)."""

    reference: LocalNED
    effective_from_s: float
    effective_until_s: float
    position_m: Vector3
    yaw_rad: float

    def __post_init__(self):
        if not isinstance(self.reference, LocalNED):
            raise ValueError("setpoint requires a LocalNED reference")
        start = finite_real(self.effective_from_s, "effective_from_s")
        end = finite_real(self.effective_until_s, "effective_until_s")
        if end <= start:
            raise ValueError("setpoint interval must have positive duration")
        object.__setattr__(self, "effective_from_s", start)
        object.__setattr__(self, "effective_until_s", end)
        object.__setattr__(self, "position_m", vector3(self.position_m, "position_m"))
        object.__setattr__(self, "yaw_rad", wrap_yaw(finite_real(self.yaw_rad, "yaw_rad")))


@dataclass(frozen=True, slots=True)
class Prediction:
    state: PredictedState
    velocity_reference_mps: Vector3
    acceleration_mps2: Vector3
    yaw_rate_reference_rad_s: float
    yaw_acceleration_rad_s2: float
