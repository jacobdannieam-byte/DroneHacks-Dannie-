from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.adaptive import ENCODERS, AdaptiveDynamicsModel, AdaptiveNetwork, OnlineFitDynamicsModel, weight_shapes
from refnav.dynamics.history import HISTORY_STEPS, MotionHistory

REFERENCE = LocalNED("ned", 0, "replay")
START = DroneState(REFERENCE, 0, (0, 0, -1), (0, 0, 0), 0, 0)
COMMAND = PositionSetpoint(REFERENCE, 0, 5, (10, 0, -1), 0)
DT = 0.02
CHECKPOINT_DIR = Path(__file__).resolve().parents[1] / "artifacts" / "adaptive_dynamics_d2"


def advance(state, command, gain=1.0):
    nominal = DynamicsModel().predict(state, command, dt=DT)
    velocity = tuple(v + gain * a * DT for v, a in zip(state.velocity_mps, nominal.acceleration_mps2))
    position = tuple(x + v * DT for x, v in zip(state.position_m, velocity))
    return DroneState(state.reference, nominal.state.timestamp_s, position, velocity,
                      nominal.state.yaw_rad, nominal.state.yaw_rate_rad_s)


def fill(count, gain=1.0):
    state, history = START, MotionHistory()
    for _ in range(count):
        after = advance(state, COMMAND, gain)
        history.observe(state, COMMAND, after)
        state = after
    return state, history


def test_window_warms_up_caps_length_and_rejects_unaligned_anchors():
    state, history = fill(3)
    window = history.window(state)
    assert window.status == "warming_up"
    assert list(window.matrix()[:, -1]) == [0.0] * (HISTORY_STEPS - 3) + [1.0] * 3
    assert history.window(START).status == "stale_or_unaligned"
    after = advance(state, COMMAND)
    assert history.window(after).status == "stale_or_unaligned"
    history.observe(state, COMMAND, after)
    assert history.window(after).status == "ready"
    state, history = fill(HISTORY_STEPS + 10)
    assert len(history.window(state).rows) == HISTORY_STEPS


def test_gap_and_reference_reset_discard_prior_history():
    state, history = fill(8)
    late = replace(state, timestamp_s=state.timestamp_s + 0.04)
    after = advance(late, COMMAND)
    history.observe(late, COMMAND, after)
    assert len(history.window(after).rows) == 1
    ref = replace(REFERENCE, reference_epoch=1)
    reset = replace(START, reference=ref)
    command = replace(COMMAND, reference=ref)
    assert history.window(reset).status == "reference_mismatch"
    new = advance(reset, command)
    history.observe(reset, command, new)
    assert history.window(new).status == "warming_up"
    with pytest.raises(ValueError, match="older reference epoch"):
        history.observe(START, COMMAND, advance(START, COMMAND))
    assert len(history.window(new).rows) == 1


def test_history_rejects_predictions_duplicates_and_mid_step_commands():
    state, history = fill(5)
    with pytest.raises(ValueError, match="measured"):
        history.observe(state, COMMAND, DynamicsModel().predict(state, COMMAND, dt=DT).state)
    with pytest.raises(ValueError, match="out of order"):
        history.observe(START, COMMAND, advance(START, COMMAND))
    with pytest.raises(ValueError, match="does not cover"):
        history.observe(state, replace(COMMAND, effective_from_s=state.timestamp_s + 0.01), advance(state, COMMAND))
    assert history.window(state).status == "ready"


def test_cross_epoch_transition_requires_a_fresh_window():
    state, history = fill(5)
    invalid = replace(advance(state, COMMAND), reference=replace(REFERENCE, reference_epoch=1))
    with pytest.raises(ValueError, match="cross a reference reset"):
        history.observe(state, COMMAND, invalid)
    assert history.window(state).status == "empty"


@pytest.mark.parametrize("encoder", ENCODERS)
def test_shipped_models_are_finite_bounded_and_keep_equilibrium(encoder):
    model = AdaptiveDynamicsModel(AdaptiveNetwork.load(CHECKPOINT_DIR / f"adaptive_{encoder}.npz"))
    state, history = fill(10, gain=0.5)
    context = model.adaptation(state, history)
    assert context.status == "ready" and len(context.context) == 8
    assert max(map(abs, context.context)) <= 1.0
    step = model.predict(state, COMMAND, dt=DT, history=history)
    assert all(abs(a) <= cap for a, cap in zip(step.acceleration_mps2, (2.0, 2.0, 1.2)))
    assert abs(step.yaw_acceleration_rad_s2) <= 4.0
    hold = replace(COMMAND, position_m=START.position_m)
    state, history = START, MotionHistory()
    for _ in range(8):
        after = advance(state, hold)
        history.observe(state, hold, after)
        state = after
    trajectory = model.rollout(state, hold, dt=DT, steps=100, history=history)
    assert all(step.state.position_m == START.position_m and step.state.velocity_mps == (0, 0, 0)
               for step in trajectory)


def test_rollout_encodes_once_and_does_not_feed_predictions_back_into_history(monkeypatch):
    state, history = fill(8, gain=0.5)
    network = AdaptiveNetwork.load(CHECKPOINT_DIR / "adaptive_gru64.npz")
    model = AdaptiveDynamicsModel(network)
    original = AdaptiveNetwork.encode
    calls = []

    def counted(self, matrix):
        calls.append(matrix.copy())
        return original(self, matrix)

    monkeypatch.setattr(AdaptiveNetwork, "encode", counted)
    before = history.window(state)
    rollout = model.rollout(state, COMMAND, dt=DT, steps=20, history=history)
    assert len(calls) == 1 and len(rollout) == 20
    assert history.window(state) == before
    assert model.adaptation(START, history).status == "stale_or_unaligned"
    assert model.predict(START, COMMAND, dt=DT, history=history) == DynamicsModel().predict(START, COMMAND, dt=DT)
    with pytest.raises(ValueError, match="20 ms"):
        model.predict(state, COMMAND, dt=0.04, history=history)


def test_online_fit_learns_a_simple_acceleration_loss():
    state, history = fill(20, gain=0.5)
    model = OnlineFitDynamicsModel()
    expected = advance(state, COMMAND, gain=0.5).velocity_mps[0]
    nominal = DynamicsModel().predict(state, COMMAND, dt=DT).state.velocity_mps[0]
    fitted = model.predict(state, COMMAND, dt=DT, history=history).state.velocity_mps[0]
    assert abs(fitted - expected) < abs(nominal - expected)


def test_checkpoint_rejects_bad_version_and_nonfinite_weights(tmp_path):
    weights = {name: np.zeros(shape, dtype=np.float32) for name, shape in weight_shapes("mlp").items()}
    weights["scale"].fill(1)
    path = tmp_path / "model.npz"
    np.savez(path, version=np.int32(9), encoder=np.array("mlp"), **weights)
    with pytest.raises(ValueError, match="version"):
        AdaptiveNetwork.load(path)
    weights["out_b"][0] = np.nan
    with pytest.raises(ValueError, match="out_b"):
        AdaptiveNetwork("mlp", weights)
