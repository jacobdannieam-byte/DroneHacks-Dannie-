"""D1 inference for a frozen 64×64 acceleration residual MLP."""

from dataclasses import dataclass
from zipfile import BadZipFile, ZipFile

import numpy as np

from .physics import DynamicsModel, NominalParameters, _clip
from .validation import wrap_yaw

FEATURE_VERSION = 1
FEATURE_LIMITS = (4, 4, 4, 6, 6, 6, 2, 2, 2, 3, 3, 3, 4, 4, 2, 5, 0.05)
SHAPES = {"w1": (len(FEATURE_LIMITS), 64), "b1": (64,), "w2": (64, 64), "b2": (64,),
          "w3": (64, 4), "b3": (4,), "mean": (len(FEATURE_LIMITS),), "scale": (len(FEATURE_LIMITS),)}
RESIDUAL_LIMITS = (1.0, 1.0, 0.6, 2.0)
MAX_ARCHIVE_BYTES = 32768


def residual_features(state, command, dt, tracking):
    """Features and activity gate from the current state and held command."""
    velocity_ref, acceleration, yaw_ref, yaw_acceleration = tracking
    position_error = tuple(target - value for target, value in zip(command.position_m, state.position_m))
    values = (*state.velocity_mps, *position_error, *velocity_ref, *acceleration,
              wrap_yaw(command.yaw_rad - state.yaw_rad), state.yaw_rate_rad_s, yaw_ref, yaw_acceleration, dt)
    features = tuple(_clip(value, limit) for value, limit in zip(values, FEATURE_LIMITS))
    # A component at rest with zero demand has zero residual. The gate is shared with training.
    velocity_and_rate = (*state.velocity_mps, state.yaw_rate_rad_s)
    demand = (*velocity_ref, yaw_ref)
    activity = tuple(min(1.0, 4.0 * (abs(value) + abs(target)))
                     for value, target in zip(velocity_and_rate, demand))
    return features, activity


@dataclass(frozen=True, slots=True, eq=False)
class ResidualMLP:
    w1: np.ndarray
    b1: np.ndarray
    w2: np.ndarray
    b2: np.ndarray
    w3: np.ndarray
    b3: np.ndarray
    mean: np.ndarray
    scale: np.ndarray

    def __post_init__(self):
        for name, shape in SHAPES.items():
            array = np.asarray(getattr(self, name))
            if array.shape != shape or array.dtype.kind != "f" or not np.isfinite(array).all():
                raise ValueError(f"invalid residual checkpoint array {name}")
            if np.max(np.abs(array)) > 100 or (name == "scale" and np.min(array) < 0.001):
                raise ValueError(f"residual checkpoint array {name} is out of bounds")
            frozen = np.frombuffer(array.astype(np.float32).tobytes(), dtype=np.float32).reshape(shape)
            object.__setattr__(self, name, frozen)

    @classmethod
    def load(cls, path):
        with open(path, "rb") as stream:
            stream.seek(0, 2)
            if stream.tell() > MAX_ARCHIVE_BYTES:
                raise ValueError("residual checkpoint exceeds its byte budget")
            stream.seek(0)
            try:
                with ZipFile(stream) as archive:
                    members = archive.infolist()
                    expected = {f"{name}.npy" for name in (*SHAPES, "feature_version")}
                    if ({member.filename for member in members} != expected or len(members) != len(expected)
                            or sum(member.file_size for member in members) > MAX_ARCHIVE_BYTES):
                        raise ValueError("invalid residual checkpoint archive")
                stream.seek(0)
                with np.load(stream, allow_pickle=False) as weights:
                    version = weights["feature_version"]
                    if version.shape != () or version.dtype.kind not in "iu" or version.item() != FEATURE_VERSION:
                        raise ValueError("unsupported residual feature version")
                    return cls(**{name: weights[name] for name in SHAPES})
            except BadZipFile as error:
                raise ValueError("invalid residual checkpoint archive") from error

    def _normalized_residual(self, features):
        normalized = (np.asarray(features, dtype=np.float32) - self.mean) / self.scale
        hidden = np.maximum(normalized @ self.w1 + self.b1, 0)
        hidden = np.maximum(hidden @ self.w2 + self.b2, 0)
        return np.tanh(hidden @ self.w3 + self.b3)


@dataclass(frozen=True, slots=True, kw_only=True)
class ResidualDynamicsModel(DynamicsModel):
    network: ResidualMLP

    def __post_init__(self):
        if self.parameters != NominalParameters() or not isinstance(self.network, ResidualMLP):
            raise ValueError("D1 requires the default D0 parameters and a validated residual network")

    def _validate_dt(self, dt):
        dt = DynamicsModel._validate_dt(self, dt)
        if not 0.01 <= dt <= 0.04:
            raise ValueError("D1 dt must be within the trained 10–40 ms range")
        return dt

    def _tracking(self, state, command, dt):
        nominal = DynamicsModel._tracking(self, state, command, dt)
        velocity_ref, acceleration, yaw_ref, yaw_acceleration = nominal
        features, activity = residual_features(state, command, dt, nominal)
        residual = self.network._normalized_residual(features)
        limits = (*self.parameters.acceleration_limits_mps2, self.parameters.yaw_acceleration_limit_rad_s2)
        corrected = tuple(_clip(value + float(correction) * scale * gate, limit)
                          for value, correction, scale, gate, limit in
                          zip((*acceleration, yaw_acceleration), residual, RESIDUAL_LIMITS, activity, limits))
        return velocity_ref, corrected[:3], yaw_ref, corrected[3]
