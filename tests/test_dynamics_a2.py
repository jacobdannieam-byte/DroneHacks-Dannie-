from dataclasses import replace
from hashlib import sha256
import json

import numpy as np
import pytest

from examples.a1_data import digest
from examples.evaluate_adaptive_dynamics_a2 import load_candidate, verify_holdout
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.adaptive_features import (
    AdaptiveFeatureDynamics, FeatureBasis, LOWER, SHAPES, UPPER, VERSION, causal_coefficients,
)
from refnav.dynamics.history import MotionHistory

REF = LocalNED("ned", 0, "replay")
START = DroneState(REF, 0, (0., 0., -1.), (0., 0., 0.), 0., 0.)
SETPOINT = PositionSetpoint(REF, 0, 4, (10., 0., -1.), 0.)
DT = .02


def advance(state, command, gain):
    step = DynamicsModel().predict(state, command, dt=DT)
    velocity = tuple(v + DT * gain * a for v, a in zip(state.velocity_mps, step.acceleration_mps2))
    position = tuple(x + DT * v for x, v in zip(state.position_m, velocity))
    return DroneState(state.reference, step.state.timestamp_s, position, velocity,
                      step.state.yaw_rad, step.state.yaw_rate_rad_s)


def history_for(count, gain, command=SETPOINT):
    state, history = START, MotionHistory()
    for _ in range(count):
        after = advance(state, command, gain)
        history.observe(state, command, after)
        state = after
    return state, history


def test_active_fit_improves_repeatable_acceleration_loss_and_preserves_caps():
    model = AdaptiveFeatureDynamics(FeatureBasis.linear_only())
    state, history = history_for(20, .5)
    prior = history.window(state)
    context = np.asarray(model.adaptation(state, history).context).reshape(4, 4)
    assert np.any(context[0]) and not np.any(context[1:])
    assert np.all(context >= LOWER) and np.all(context <= UPPER)
    nominal = DynamicsModel().predict(state, SETPOINT, dt=DT)
    adaptive = model.predict(state, SETPOINT, dt=DT, history=history)
    target = advance(state, SETPOINT, .5)
    assert abs(adaptive.state.velocity_mps[0] - target.velocity_mps[0]) < abs(
        nominal.state.velocity_mps[0] - target.velocity_mps[0])
    trajectory = model.rollout(state, SETPOINT, dt=DT, steps=50, history=history)
    assert all(np.all(np.abs(step.acceleration_mps2) <= (2., 2., 1.2))
               and abs(step.yaw_acceleration_rad_s2) <= 4. for step in trajectory)
    assert history.window(state) == prior


@pytest.mark.parametrize("count,gain", [(6, .5), (20, 1.)])
def test_warmup_and_nominal_evidence_use_exact_d0(count, gain):
    model = AdaptiveFeatureDynamics(FeatureBasis.linear_only())
    state, history = history_for(count, gain)
    assert model.predict(state, SETPOINT, dt=DT, history=history) == DynamicsModel().predict(state, SETPOINT, dt=DT)
    assert model.predict(START, SETPOINT, dt=DT, history=history) == DynamicsModel().predict(START, SETPOINT, dt=DT)


def test_model_keeps_equilibrium_even_with_nonzero_frozen_weights():
    model = AdaptiveFeatureDynamics(FeatureBasis({name: np.full(shape, .1) for name, shape in SHAPES.items()}))
    hold = replace(SETPOINT, position_m=START.position_m)
    state, history = history_for(20, 1., hold)
    assert model.rollout(state, hold, dt=DT, steps=30, history=history) == DynamicsModel().rollout(
        state, hold, dt=DT, steps=30)
    with pytest.raises(ValueError, match="20 ms"):
        model.predict(state, hold, dt=.04, history=history)


def test_recent_innovations_fail_closed_on_inconsistent_history():
    matrix = np.zeros((20, 13), dtype=np.float32)
    matrix[:, 0], matrix[:, 4], matrix[:, -1] = .8, .4, 1.
    basis = FeatureBasis.linear_only()
    assert np.any(causal_coefficients(basis, matrix)[0])
    matrix[-4:, 4] = (-1.6, 1.8, -1.6, 1.8)
    assert not np.any(causal_coefficients(basis, matrix))


def test_reference_discontinuity_and_gap_force_fresh_history():
    model = AdaptiveFeatureDynamics(FeatureBasis.linear_only())
    state, history = history_for(20, .5)
    after = replace(advance(state, SETPOINT, .5), reference=replace(REF, reference_epoch=1))
    with pytest.raises(ValueError, match="cross a reference reset"):
        history.observe(state, SETPOINT, after)
    command = replace(SETPOINT, reference=after.reference)
    assert model.predict(after, command, dt=DT, history=history) == DynamicsModel().predict(after, command, dt=DT)
    later = replace(after, timestamp_s=after.timestamp_s + .1)
    measured = advance(later, command, .5)
    history.observe(later, command, measured)
    assert model.adaptation(measured, history).status == "warming_up"


def test_frozen_basis_validates_checkpoint_and_has_no_bias_at_rest(tmp_path):
    weights = {name: np.full(shape, .1) for name, shape in SHAPES.items()}
    path = tmp_path / "basis.npz"
    np.savez(path, version=np.int32(VERSION), **weights)
    basis = FeatureBasis.load(path)
    assert not np.any(basis.design(np.zeros(4), np.zeros(4)))
    with pytest.raises(ValueError):
        basis._weights["w1"].setflags(write=True)
    np.savez(path, version=np.int32(999), **weights)
    with pytest.raises(ValueError, match="version"):
        FeatureBasis.load(path)
    weights["b2"][0] = np.nan
    np.savez(path, version=np.int32(VERSION), **weights)
    with pytest.raises(ValueError, match="invalid A2 tensor"):
        FeatureBasis.load(path)


def test_holdout_cannot_reuse_trajectories_or_nonnominal_regimes():
    first = {"test": {"switched": [{"trace": "a", "regimes": ["r"]}]}}
    with pytest.raises(ValueError, match="trajectory"):
        verify_holdout(first, first)
    other = {"test": {"switched": [{"trace": "b", "regimes": ["r"]}]}}
    with pytest.raises(ValueError, match="regime"):
        verify_holdout(first, other)
    verify_holdout(first, {"test": {"switched": [{"trace": "b", "regimes": ["s"]}]}})


def test_checkpoint_provenance_survives_json_checkout_line_endings(tmp_path):
    weights = {name: np.zeros(shape) for name, shape in SHAPES.items()}
    checkpoint = tmp_path / "feature_basis.npz"
    np.savez(checkpoint, version=np.int32(VERSION), **weights)
    d2 = {"response_ids": {"train": ["a"], "validation": ["b"]}}
    record = {
        "checkpoint_sha256": sha256(checkpoint.read_bytes()).hexdigest(),
        "d2_training_manifest_sha256": digest(d2),
        "split_fingerprints": {"train": [{"response": "a"}], "validation": [{"response": "b"}]},
    }
    (tmp_path / "training.json").write_text(json.dumps(record), encoding="utf-8")
    alternate_checkout = json.loads(json.dumps(d2, indent=2).replace("\n", "\r\n"))
    assert load_candidate(tmp_path, alternate_checkout)[1] == record["checkpoint_sha256"]
    with pytest.raises(ValueError, match="manifest hash mismatch"):
        load_candidate(tmp_path, {**d2, "unexpected": 1})
