"""Frozen D2 encoders and a classical bounded-window identification baseline."""

from dataclasses import dataclass
from types import MappingProxyType
from zipfile import BadZipFile, ZipFile

import numpy as np

from .history import ACCELERATION_SCALE, HISTORY_FEATURES, HISTORY_STEPS, VELOCITY_SCALE, MotionHistory, check_step
from .physics import DynamicsModel, _clip
from .residual import FEATURE_LIMITS, RESIDUAL_LIMITS, residual_features
from .schema import DroneState

CONTEXT_SIZE = 8
ENCODERS = ("mlp", "gru32", "gru64")
MODEL_VERSION = 1
MAX_CHECKPOINT_BYTES = 131072
HEAD_WIDTHS = (48, 32)


def weight_shapes(encoder):
    if encoder not in ENCODERS:
        raise ValueError("unsupported D2 encoder")
    feature_count = len(FEATURE_LIMITS)
    first, second = HEAD_WIDTHS
    shapes = {"mean": (feature_count,), "scale": (feature_count,),
              "head1_w": (first, feature_count + CONTEXT_SIZE), "head1_b": (first,),
              "head2_w": (second, first), "head2_b": (second,),
              "out_w": (4, second), "out_b": (4,)}
    if encoder == "mlp":
        width = 32
        shapes.update(enc_w=(width, HISTORY_STEPS * HISTORY_FEATURES), enc_b=(width,))
    else:
        width = int(encoder[3:])
        shapes.update(gru_ih=(3 * width, HISTORY_FEATURES), gru_hh=(3 * width, width),
                      gru_bi=(3 * width,), gru_bh=(3 * width,))
    return {**shapes, "latent_w": (CONTEXT_SIZE, width), "latent_b": (CONTEXT_SIZE,)}


def _sigmoid(value):
    return 1 / (1 + np.exp(-np.clip(value, -40, 40)))


@dataclass(frozen=True, slots=True, init=False, eq=False)
class AdaptiveNetwork:
    _encoder: str
    _weights: MappingProxyType

    def __init__(self, encoder, weights):
        shapes = weight_shapes(encoder)
        if weights.keys() != shapes.keys():
            raise ValueError("D2 checkpoint has unexpected tensors")
        frozen = {}
        for name, shape in shapes.items():
            value = np.asarray(weights[name])
            if (value.shape != shape or value.dtype.kind != "f" or not np.isfinite(value).all()
                    or np.max(np.abs(value)) > 100 or (name == "scale" and np.min(value) < 0.0009)):
                raise ValueError(f"invalid D2 checkpoint tensor {name}")
            frozen[name] = np.frombuffer(value.astype(np.float32).tobytes(), dtype=np.float32).reshape(shape)
        object.__setattr__(self, "_encoder", encoder)
        object.__setattr__(self, "_weights", MappingProxyType(frozen))

    @property
    def encoder(self):
        return self._encoder

    @classmethod
    def load(cls, path):
        with open(path, "rb") as stream:
            stream.seek(0, 2)
            if stream.tell() > MAX_CHECKPOINT_BYTES:
                raise ValueError("D2 checkpoint exceeds the archive budget")
            stream.seek(0)
            try:
                with ZipFile(stream) as archive:
                    members = archive.infolist()
                    if (len(members) > 20 or len({m.filename for m in members}) != len(members)
                            or sum(m.file_size for m in members) > MAX_CHECKPOINT_BYTES):
                        raise ValueError("invalid D2 checkpoint archive")
                stream.seek(0)
                with np.load(stream, allow_pickle=False) as data:
                    version, encoder = data["version"], data["encoder"]
                    if version.shape != () or version.dtype.kind not in "iu" or version.item() != MODEL_VERSION:
                        raise ValueError("unsupported D2 checkpoint version")
                    if encoder.shape != () or encoder.dtype.kind != "U":
                        raise ValueError("invalid D2 encoder identifier")
                    return cls(encoder.item(), {name: data[name] for name in data.files if name not in ("version", "encoder")})
            except (BadZipFile, KeyError) as error:
                raise ValueError("invalid D2 checkpoint archive") from error

    def encode(self, matrix):
        weights = self._weights
        if self.encoder == "mlp":
            hidden = np.maximum(weights["enc_w"] @ matrix.ravel() + weights["enc_b"], 0)
        else:
            width = int(self.encoder[3:])
            hidden = np.zeros(width, dtype=np.float32)
            inputs = matrix @ weights["gru_ih"].T + weights["gru_bi"]
            for current in inputs:
                previous = weights["gru_hh"] @ hidden + weights["gru_bh"]
                gates = _sigmoid(current[:2 * width] + previous[:2 * width])
                reset, update = gates[:width], gates[width:]
                candidate = np.tanh(current[2 * width:] + reset * previous[2 * width:])
                hidden = (1 - update) * candidate + update * hidden
        return np.tanh(weights["latent_w"] @ hidden + weights["latent_b"])

    def correction(self, features, latent):
        weights = self._weights
        inputs = (np.asarray(features, dtype=np.float32) - weights["mean"]) / weights["scale"]
        first = np.maximum(weights["head1_w"] @ np.concatenate((inputs, latent)) + weights["head1_b"], 0)
        second = np.maximum(weights["head2_w"] @ first + weights["head2_b"], 0)
        return np.tanh(weights["out_w"] @ second + weights["out_b"]) * RESIDUAL_LIMITS


@dataclass(frozen=True, slots=True, kw_only=True)
class _ConditionedDynamics(DynamicsModel):
    correction: object

    def _tracking(self, state, command, dt):
        nominal = DynamicsModel._tracking(self, state, command, dt)
        velocity_ref, acceleration, yaw_ref, yaw_acceleration = nominal
        residual, activity = self.correction(state, command, dt, nominal)
        corrected = tuple(_clip(value + _clip(float(delta), residual_limit) * gate, total_limit)
                          for value, delta, gate, residual_limit, total_limit in
                          zip((*acceleration, yaw_acceleration), residual, activity,
                              RESIDUAL_LIMITS, ACCELERATION_SCALE))
        return velocity_ref, corrected[:3], yaw_ref, corrected[3]


@dataclass(frozen=True, slots=True)
class Adaptation:
    status: str
    transitions: int
    context: tuple[float, ...] = ()


class _HistoryPredictor:
    def _bound(self, state, history):
        if not isinstance(state, DroneState) or not isinstance(history, MotionHistory):
            raise ValueError("D2 requires a measured anchor and MotionHistory")
        window = history.window(state)
        if window.status != "ready":
            return DynamicsModel(), Adaptation(window.status, len(window.rows))
        correction, context = self._fit(window.matrix())
        return _ConditionedDynamics(correction=correction), Adaptation("ready", len(window.rows), context)

    def adaptation(self, state, history):
        return self._bound(state, history)[1]

    def predict(self, state, command, *, dt, history):
        check_step(dt)
        model, _ = self._bound(state, history)
        return model.predict(state, command, dt=dt)

    def rollout(self, state, commands, *, dt, history, steps=None):
        check_step(dt)
        model, _ = self._bound(state, history)
        return model.rollout(state, commands, dt=dt, steps=steps)


@dataclass(frozen=True, slots=True)
class AdaptiveDynamicsModel(_HistoryPredictor):
    network: AdaptiveNetwork

    def __post_init__(self):
        if not isinstance(self.network, AdaptiveNetwork):
            raise ValueError("D2 requires a validated adaptive network")

    def _fit(self, matrix):
        latent = self.network.encode(matrix)

        def correction(state, command, dt, nominal):
            features, gate = residual_features(state, command, dt, nominal)
            return self.network.correction(features, latent), gate

        return correction, tuple(map(float, latent))


class OnlineFitDynamicsModel(_HistoryPredictor):
    """Regularized online fit of a gain and rate damping per acceleration axis."""

    def _fit(self, matrix):
        rows = matrix[matrix[:, -1] > 0]
        nominal, observed, rate = rows[:, :4], rows[:, 4:8], -rows[:, 8:12]
        weight = 0.90 ** np.arange(len(rows) - 1, -1, -1)[:, None]
        # Ridge prior [gain=1, damping=0]. Solve the four 2×2 normal equations.
        aa = (weight * nominal ** 2).sum(axis=0) + 0.3
        bb = (weight * rate ** 2).sum(axis=0) + 0.3
        ab = (weight * nominal * rate).sum(axis=0)
        ay = (weight * nominal * observed).sum(axis=0) + 0.3
        by = (weight * rate * observed).sum(axis=0)
        det = aa * bb - ab ** 2
        gain = np.clip((bb * ay - ab * by) / det, 0.35, 1.35)
        damping = np.clip((aa * by - ab * ay) / det, 0, 1.5)

        def correction(state, command, dt, tracking):
            features, gate = residual_features(state, command, dt, tracking)
            normalized_rate = np.clip(np.array((*state.velocity_mps, state.yaw_rate_rad_s)) / VELOCITY_SCALE, -3, 3)
            acceleration = np.array((*tracking[1], tracking[3]))
            residual = (gain - 1) * acceleration - damping * normalized_rate * ACCELERATION_SCALE
            return residual, gate

        return correction, tuple(map(float, (*gain, *damping)))
