"""Offline randomized system-ID fixtures. Hidden response settings stay in the simulator."""

from dataclasses import dataclass
import hashlib
import json
from math import pi

import numpy as np

from refnav.dynamics import DroneState, LocalNED, PositionSetpoint

SEED = 20261106
COUNTS = {"train": 160, "validation": 24, "test": 48, "shifted": 24}
SEED_OFFSETS = {"train": 0, "validation": 1, "test": 2, "shifted": 3}
STEPS = 100
RESPONSE_NAMES = ("position_gain", "velocity_tau_s", "acceleration_scale", "drag_per_s",
                  "yaw_gain", "yaw_tau_s", "yaw_acceleration_scale")
RANGES = ((0.8, 1.5), (0.30, 0.68), (0.55, 1.15), (0.0, 0.32),
          (1.5, 2.5), (0.14, 0.32), (0.60, 1.15))
SHIFTED_RANGES = ((0.65, 1.35), (0.66, 0.95), (0.38, 0.65), (0.28, 0.52),
                  (1.2, 2.0), (0.27, 0.44), (0.45, 0.75))


@dataclass(frozen=True)
class Episode:
    identifier: str
    dt: float
    reference: LocalNED
    states: np.ndarray
    commands: tuple[PositionSetpoint, ...]
    response: dict
    change_step: int | None = None

    def sample(self, index):
        value = self.states[index]
        return DroneState(self.reference, index * self.dt, value[:3], value[3:6], value[6], value[7])

    def response_id(self):
        encoded = json.dumps(self.response, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def generate(split, *, seed=SEED, count=None, steps=STEPS, step_seconds=None, change_step=None,
             position_delay_steps=None, yaw_delay_steps=None):
    """Use separate random streams and full response configurations per episode."""
    count = COUNTS[split] if count is None else count
    rng = np.random.default_rng(np.random.SeedSequence([seed, SEED_OFFSETS[split]]))
    shifted = split == "shifted" and change_step is None
    ranges = np.array(SHIFTED_RANGES if shifted else RANGES)
    response = rng.uniform(ranges[:, 0], ranges[:, 1], size=(count, len(RESPONSE_NAMES)))
    position_delay = rng.integers(3, 7, count) if shifted else rng.integers(0, 3, count)
    yaw_delay = rng.integers(5, 10, count) if shifted else rng.integers(0, 5, count)
    for values, override in ((position_delay, position_delay_steps), (yaw_delay, yaw_delay_steps)):
        if override is not None:
            if type(override) is not int or not 0 <= override < steps or change_step is not None:
                raise ValueError("fixed delay requires a stationary episode and a valid step count")
            values.fill(override)
    if step_seconds is not None:
        dt = np.full(count, step_seconds)
    else:
        dt = rng.choice([0.01, 0.02, 0.04], count) if split == "train" else np.full(count, 0.02)
    initial = np.zeros((count, 8))
    initial[:, :3] = rng.uniform([-0.5, -0.5, -1.5], [0.5, 0.5, -0.5], (count, 3))
    initial[:, 3:6] = rng.uniform(-0.4, 0.4, (count, 3))
    initial[:, 6] = rng.uniform(-pi, pi, count)
    initial[:, 7] = rng.uniform(-0.4, 0.4, count)

    reference = LocalNED("d1_synthetic_ned", 0, "replay")
    targets = np.empty((count, steps, 4))
    schedules = []
    for episode in range(count):
        row = initial[episode]
        boundary = int(rng.integers(8, 17))
        segments = [(0, boundary, row[:3], row[6])]
        while boundary < steps:
            end = min(steps, boundary + int(rng.integers(15, 32)))
            position = row[:3] + rng.uniform([-3, -3, -1], [3, 3, 1])
            heading = rng.uniform(-pi, pi)
            segments.append((boundary, end, position, heading))
            boundary = end
        commands = []
        for start, end, position, heading in segments:
            targets[episode, start:end] = (*position, heading)
            command = PositionSetpoint(reference, start * dt[episode], end * dt[episode], position, heading)
            commands.extend([command] * (end - start))
        schedules.append(tuple(commands))

    change = None
    post_settings = [{} for _ in range(count)]
    if change_step is not None:
        if not 0 < change_step < steps:
            raise ValueError("response switch must occur inside the episode")
        shifted_post = split == "shifted"
        after_ranges = np.array(SHIFTED_RANGES if shifted_post else RANGES)
        after = rng.uniform(after_ranges[:, 0], after_ranges[:, 1], size=response.shape)
        after_position_delay = rng.integers(3, 7, count) if shifted_post else rng.integers(0, 3, count)
        after_yaw_delay = rng.integers(5, 10, count) if shifted_post else rng.integers(0, 5, count)
        change = (change_step, after, after_position_delay, after_yaw_delay)
        post_settings = [{"change_time_s": float(change_step * dt[i]),
                          "after": {**dict(zip(RESPONSE_NAMES, map(float, after[i]))),
                                    "position_delay_s": float(after_position_delay[i] * dt[i]),
                                    "yaw_delay_s": float(after_yaw_delay[i] * dt[i])}} for i in range(count)]
    traces = _simulate(initial, targets, dt, response, position_delay, yaw_delay, change=change)
    return tuple(Episode(
        f"{split}-{i:03}", float(dt[i]), reference, traces[i], schedules[i],
        {**dict(zip(RESPONSE_NAMES, map(float, response[i]))),
         "position_delay_s": float(position_delay[i] * dt[i]), "yaw_delay_s": float(yaw_delay[i] * dt[i]),
         **post_settings[i]}, change_step,
    ) for i in range(count))


def _simulate(initial, targets, dt, response, position_delay, yaw_delay, *, change=None):
    """Vectorized independent RK4 with ten substeps per prediction interval."""
    state = initial.copy()
    steps = targets.shape[1]
    traces = np.empty((len(state), steps + 1, 8))
    traces[:, 0] = state
    position_gain, velocity_tau, acceleration_scale, drag, yaw_gain, yaw_tau, yaw_scale = response.T
    velocity_limit = np.array([1.2, 1.2, 0.7])
    acceleration_limit = acceleration_scale[:, None] * np.array([2.0, 2.0, 1.2])
    heading_acceleration_limit = 4.0 * yaw_scale
    substep = dt[:, None] / 10

    def derivative(value, position, heading):
        demand = np.clip(position_gain[:, None] * (position - value[:, :3]), -velocity_limit, velocity_limit)
        velocity = value[:, 3:6]
        accel = np.clip((demand - velocity) / velocity_tau[:, None], -acceleration_limit, acceleration_limit)
        accel -= drag[:, None] * velocity
        yaw_error = (heading - value[:, 6] + pi) % (2 * pi) - pi
        yaw_demand = np.clip(yaw_gain * yaw_error, -1.5, 1.5)
        yaw_accel = np.clip((yaw_demand - value[:, 7]) / yaw_tau,
                            -heading_acceleration_limit, heading_acceleration_limit)
        return np.column_stack((velocity, accel, value[:, 7], yaw_accel))

    rows = np.arange(len(state))
    for index in range(steps):
        if change is not None and index == change[0]:
            _, after, position_delay, yaw_delay = change
            position_gain, velocity_tau, acceleration_scale, drag, yaw_gain, yaw_tau, yaw_scale = after.T
            acceleration_limit = acceleration_scale[:, None] * np.array([2.0, 2.0, 1.2])
            heading_acceleration_limit = 4.0 * yaw_scale
        position = targets[rows, np.maximum(0, index - position_delay), :3]
        heading = targets[rows, np.maximum(0, index - yaw_delay), 3]
        for _ in range(10):
            k1 = derivative(state, position, heading)
            k2 = derivative(state + substep * k1 / 2, position, heading)
            k3 = derivative(state + substep * k2 / 2, position, heading)
            k4 = derivative(state + substep * k3, position, heading)
            state += substep / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        traces[:, index + 1] = state
    return traces
