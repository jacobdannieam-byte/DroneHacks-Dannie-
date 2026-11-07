"""Optional A2 frozen features with bounded causal coefficient adaptation."""

from dataclasses import dataclass
from types import MappingProxyType
from zipfile import BadZipFile, ZipFile

import numpy as np

from .adaptive import _HistoryPredictor
from .history import ACCELERATION_SCALE, VELOCITY_SCALE
from .physics import DynamicsModel
from .residual import residual_features

VERSION = 1
SHAPES = {"w1": (8, 16), "b1": (16,), "w2": (16, 8), "b2": (8,)}
PRIOR = (.30, .30, .70, .70)
LOWER = (-.65, 0., -.35, -.35)
UPPER = (.35, 1.50, .35, .35)
FORGET = .90
HOLD_BACK = 4
MIN_ROWS = 8
MIN_EXCITATION = .12
MAX_CONDITION_BOUND = 300.
MIN_ERROR_GAIN = .0025
MAX_VALIDATION_MSE = .16
_MAX_BYTES = 16384


@dataclass(frozen=True, slots=True, init=False, eq=False)
class FeatureBasis:
    _weights: MappingProxyType

    def __init__(self, weights):
        if weights.keys() != SHAPES.keys():
            raise ValueError("unexpected A2 feature tensors")
        frozen = {}
        for name, shape in SHAPES.items():
            value = np.asarray(weights[name])
            if (value.shape != shape or value.dtype.kind != "f" or not np.isfinite(value).all()
                    or np.max(np.abs(value)) > 20):
                raise ValueError(f"invalid A2 tensor {name}")
            frozen[name] = np.frombuffer(value.astype(np.float32).tobytes(), dtype=np.float32).reshape(shape)
        object.__setattr__(self, "_weights", MappingProxyType(frozen))

    @classmethod
    def linear_only(cls):
        """Zero learned columns for an ablation with the identical online gate."""
        return cls({name: np.zeros(shape, np.float32) for name, shape in SHAPES.items()})

    @classmethod
    def load(cls, path):
        with open(path, "rb") as stream:
            stream.seek(0, 2)
            if stream.tell() > _MAX_BYTES:
                raise ValueError("A2 checkpoint exceeds archive budget")
            stream.seek(0)
            try:
                with ZipFile(stream) as archive:
                    members = archive.infolist()
                    expected = {f"{name}.npy" for name in (*SHAPES, "version")}
                    if (len(members) != len(expected) or {item.filename for item in members} != expected
                            or sum(item.file_size for item in members) > _MAX_BYTES):
                        raise ValueError("invalid A2 checkpoint archive")
                stream.seek(0)
                with np.load(stream, allow_pickle=False) as data:
                    version = data["version"]
                    if version.shape != () or version.dtype.kind not in "iu" or version.item() != VERSION:
                        raise ValueError("unsupported A2 checkpoint version")
                    return cls({name: data[name] for name in SHAPES})
            except (BadZipFile, KeyError) as error:
                raise ValueError("invalid A2 checkpoint archive") from error

    def design(self, nominal, rate):
        """Four per-axis columns. Inputs use the existing D2 history scaling."""
        weights = self._weights
        inputs = np.concatenate((nominal, rate), axis=-1).astype(np.float32, copy=False)
        hidden = np.maximum(inputs @ weights["w1"] + weights["b1"], 0)
        learned = np.tanh(hidden @ weights["w2"] + weights["b2"]).reshape(*nominal.shape, 2)
        activity = np.minimum(1., np.abs(nominal) + np.abs(rate))
        return np.concatenate((nominal[..., None], -rate[..., None], learned * activity[..., None]), axis=-1)


def _fit(design, target):
    weights = FORGET ** np.arange(len(design) - 1, -1, -1)
    gram = np.einsum("n,nai,naj->aij", weights, design, design)
    rhs = np.einsum("n,nai,na->ai", weights, design, target)
    system = gram + np.diag(PRIOR)
    # Minimum eigenvalue is at least min(PRIOR) since the data Gram is PSD.
    condition_bound = np.abs(system).sum(axis=2).max(axis=1) / min(PRIOR)
    excitation = np.maximum(gram[:, 0, 0], gram[:, 1, 1])
    coefficients = np.linalg.solve(system, rhs[..., None])[..., 0]
    bounded = np.clip(coefficients, LOWER, UPPER)
    return bounded, (excitation >= MIN_EXCITATION) & (condition_bound <= MAX_CONDITION_BOUND)


def causal_coefficients(basis, matrix):
    """Use only completed transitions. Validate on the most recent four first."""
    rows = matrix[matrix[:, -1] > 0]
    if len(rows) < MIN_ROWS:
        return np.zeros((4, 4))
    nominal, observed, rate = rows[:, :4], rows[:, 4:8], rows[:, 8:12]
    design = basis.design(nominal, rate)
    target = np.clip(observed, -1., 1.) - nominal
    prefix, usable = _fit(design[:-HOLD_BACK], target[:-HOLD_BACK])
    recent = slice(-HOLD_BACK, None)
    delta = np.clip(np.einsum("nai,ai->na", design[recent], prefix), -.5, .5)
    prediction = np.clip(nominal[recent] + delta, -1., 1.)
    # Retain raw innovations for the gate. Saturating a noisy label would hide
    # precisely the inconsistency that should prevent an adaptive correction.
    baseline_error = np.mean((observed[recent] - nominal[recent]) ** 2, axis=0)
    fit_error = np.mean((observed[recent] - prediction) ** 2, axis=0)
    usable &= ((baseline_error - fit_error >= np.maximum(MIN_ERROR_GAIN, .15 * baseline_error))
               & (fit_error <= MAX_VALIDATION_MSE))
    if not np.any(usable):
        return np.zeros((4, 4))
    coefficients, full_usable = _fit(design, target)
    return np.where((usable & full_usable)[:, None], coefficients, 0.)


@dataclass(frozen=True, slots=True)
class AdaptiveFeatureDynamics(_HistoryPredictor):
    basis: FeatureBasis

    def __post_init__(self):
        if not isinstance(self.basis, FeatureBasis):
            raise ValueError("A2 requires a validated frozen feature basis")

    def _bound(self, state, history):
        model, adaptation = _HistoryPredictor._bound(self, state, history)
        if adaptation.status == "ready" and not any(adaptation.context):
            return DynamicsModel(), adaptation
        return model, adaptation

    def _fit(self, matrix):
        coefficients = causal_coefficients(self.basis, matrix)

        def correction(state, command, dt, tracking):
            _, gate = residual_features(state, command, dt, tracking)
            nominal = np.asarray((*tracking[1], tracking[3]), dtype=np.float32) / _ACCEL
            rate = np.clip(np.asarray((*state.velocity_mps, state.yaw_rate_rad_s), dtype=np.float32) / _RATE, -3, 3)
            design = self.basis.design(nominal, rate)
            return np.einsum("ai,ai->a", design, coefficients) * _ACCEL, gate

        return correction, tuple(map(float, coefficients.ravel()))


_ACCEL = np.asarray(ACCELERATION_SCALE, np.float32)
_RATE = np.asarray(VELOCITY_SCALE, np.float32)
