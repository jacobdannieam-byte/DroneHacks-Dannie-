"""Bounded, causal records of completed estimated-motion transitions."""

from collections import deque
from dataclasses import dataclass
from math import isclose
from threading import Lock

import numpy as np

from .physics import DynamicsModel, _clip
from .schema import DroneState
from .validation import finite_real

HISTORY_STEPS = 20
MIN_HISTORY_STEPS = 4
STEP_SECONDS = 0.02
HISTORY_FEATURES = 13
ACCELERATION_SCALE = (2.0, 2.0, 1.2, 4.0)
VELOCITY_SCALE = (1.2, 1.2, 0.7, 1.5)


def check_step(dt):
    if not isclose(finite_real(dt, "dt"), STEP_SECONDS, rel_tol=0, abs_tol=1e-6):
        raise ValueError("D2 requires aligned 20 ms transitions")


@dataclass(frozen=True, slots=True)
class HistoryWindow:
    status: str
    rows: tuple[tuple[float, ...], ...] = ()

    def matrix(self):
        """Left-pad with zeros. The final column marks real observations."""
        matrix = np.zeros((HISTORY_STEPS, HISTORY_FEATURES), dtype=np.float32)
        if self.rows:
            matrix[-len(self.rows):] = self.rows
        return matrix


class MotionHistory:
    """One measured state stream. Predictions and command changes inside a step are excluded."""

    def __init__(self):
        self._rows = deque(maxlen=HISTORY_STEPS)
        self._last = None
        self._lock = Lock()
        self._nominal = DynamicsModel()

    def clear(self):
        with self._lock:
            self._rows.clear()
            self._last = None

    def observe(self, before, command, after):
        if not isinstance(before, DroneState) or not isinstance(after, DroneState):
            raise ValueError("history requires measured DroneState endpoints")
        if before.reference != after.reference:
            self.clear()
            raise ValueError("a transition cannot cross a reference reset")
        dt = after.timestamp_s - before.timestamp_s
        check_step(dt)
        nominal = self._nominal.predict(before, command, dt=dt)
        before_rates = (*before.velocity_mps, before.yaw_rate_rad_s)
        after_rates = (*after.velocity_mps, after.yaw_rate_rad_s)
        acceleration = (*nominal.acceleration_mps2, nominal.yaw_acceleration_rad_s2)
        row = (
            *(a / scale for a, scale in zip(acceleration, ACCELERATION_SCALE)),
            *(_clip((end - start) / dt / scale, 2.0)
              for start, end, scale in zip(before_rates, after_rates, ACCELERATION_SCALE)),
            *(_clip(rate / scale, 3.0) for rate, scale in zip(before_rates, VELOCITY_SCALE)),
            1.0,
        )
        with self._lock:
            if self._last is not None:
                previous = self._last.reference
                current = before.reference
                if (current.frame_id == previous.frame_id and current.clock_id == previous.clock_id
                        and current.reference_epoch < previous.reference_epoch):
                    raise ValueError("history transition belongs to an older reference epoch")
                if before.reference == self._last.reference and before.timestamp_s < self._last.timestamp_s:
                    raise ValueError("history transition is duplicate or out of order")
                if before != self._last:
                    self._rows.clear()
            self._rows.append(row)
            self._last = after

    def window(self, state):
        with self._lock:
            if self._last is None:
                return HistoryWindow("empty")
            if state.reference != self._last.reference:
                return HistoryWindow("reference_mismatch")
            if state != self._last:
                return HistoryWindow("stale_or_unaligned")
            status = "ready" if len(self._rows) >= MIN_HISTORY_STEPS else "warming_up"
            return HistoryWindow(status, tuple(self._rows))
