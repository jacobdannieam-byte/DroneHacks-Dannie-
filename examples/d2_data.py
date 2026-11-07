"""Shared D2 experiment settings and causal training examples."""

import numpy as np

from examples.d1_synthetic import generate
from refnav.dynamics import DynamicsModel
from refnav.dynamics.history import ACCELERATION_SCALE, MotionHistory, STEP_SECONDS
from refnav.dynamics.residual import RESIDUAL_LIMITS, residual_features

SEED = 20261107
STEPS = 150
CHANGE_STEP = 65


def episodes(split):
    if split == "train":
        common = {"steps": STEPS, "step_seconds": STEP_SECONDS}
        return (generate(split, seed=SEED, count=80, change_step=CHANGE_STEP, **common)
                + generate(split, seed=SEED + 10, count=40, **common))
    counts = {"validation": 24, "test": 32, "shifted": 24}
    return generate(split, seed=SEED, count=counts[split], steps=STEPS,
                    step_seconds=STEP_SECONDS, change_step=CHANGE_STEP)


def training_arrays(cases):
    inputs, windows, gates, targets = [], [], [], []
    nominal = DynamicsModel()
    for episode in cases:
        history = MotionHistory()
        before = episode.sample(0)
        for index, command in enumerate(episode.commands):
            after = episode.sample(index + 1)
            window = history.window(before)
            if window.status == "ready":
                tracking = nominal._tracking(before, command, episode.dt)
                features, gate = residual_features(before, command, episode.dt, tracking)
                measured = (episode.states[index + 1, [3, 4, 5, 7]]
                            - episode.states[index, [3, 4, 5, 7]]) / episode.dt
                achievable = np.clip(measured, -np.asarray(ACCELERATION_SCALE), ACCELERATION_SCALE)
                residual = (achievable - (*tracking[1], tracking[3])) / RESIDUAL_LIMITS
                inputs.append(features)
                windows.append(window.matrix())
                gates.append(gate)
                targets.append(np.clip(residual, -np.asarray(gate), gate))
            history.observe(before, command, after)
            before = after
    return tuple(np.asarray(values, dtype=np.float32) for values in (inputs, windows, gates, targets))
