from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from refnav.dynamics import DroneState, DynamicsModel, LocalNED, NominalParameters, PositionSetpoint
from refnav.dynamics.residual import FEATURE_VERSION, SHAPES, ResidualDynamicsModel, ResidualMLP

REFERENCE = LocalNED("local_ned", 0, "replay")
START = DroneState(REFERENCE, 0.0, (0, 0, -1), (0, 0, 0), 0, 0)
COMMAND = PositionSetpoint(REFERENCE, 0.0, 5.0, (0.2, -0.2, -1), 0.12)


def weights():
    arrays = {name: np.zeros(shape, dtype=np.float32) for name, shape in SHAPES.items()}
    arrays["scale"].fill(1)
    return arrays


def constant_network(output):
    arrays = weights()
    arrays["b3"][:] = output
    return ResidualMLP(**arrays)


def test_zero_residual_reproduces_d0_for_the_same_schedule():
    model = ResidualDynamicsModel(network=constant_network(0))
    commands = [COMMAND] * 10 + [replace(COMMAND, position_m=(-1, 1, -1.4))] * 15
    assert model.rollout(START, commands, dt=0.02) == DynamicsModel().rollout(START, commands, dt=0.02)


def test_residual_changes_derivatives_and_uses_d0_integration():
    model = ResidualDynamicsModel(network=constant_network(100))
    step = model.predict(START, COMMAND, dt=0.02)
    nominal = DynamicsModel().predict(START, COMMAND, dt=0.02)
    assert step.acceleration_mps2[0] > nominal.acceleration_mps2[0]
    assert step.yaw_acceleration_rad_s2 > nominal.yaw_acceleration_rad_s2
    assert step.acceleration_mps2[2] == 0.0
    assert step.state.velocity_mps == pytest.approx(tuple(a * 0.02 for a in step.acceleration_mps2))
    assert step.state.position_m == pytest.approx(tuple(x + v * 0.02 for x, v in
                                                       zip(START.position_m, step.state.velocity_mps)))
    assert step.state.yaw_rad == pytest.approx(step.yaw_acceleration_rad_s2 * 0.02 ** 2)


def test_nominal_caps_and_equilibrium_hold_even_for_saturated_network():
    model = ResidualDynamicsModel(network=constant_network(100))
    hard_command = replace(COMMAND, position_m=(10, 10, 10), yaw_rad=2)
    step = model.predict(START, hard_command, dt=0.02)
    assert step.acceleration_mps2 == model.parameters.acceleration_limits_mps2
    assert step.yaw_acceleration_rad_s2 == model.parameters.yaw_acceleration_limit_rad_s2
    hold = replace(COMMAND, position_m=START.position_m, yaw_rad=START.yaw_rad)
    trajectory = model.rollout(START, hold, dt=0.02, steps=250)
    assert all(step.state.position_m == START.position_m and step.state.yaw_rad == START.yaw_rad
               and step.state.velocity_mps == START.velocity_mps for step in trajectory)


def test_d1_validates_its_training_contract_and_inherits_rollout_guards():
    network = constant_network(0)
    model = ResidualDynamicsModel(network=network)
    with pytest.raises(ValueError, match="default D0 parameters"):
        ResidualDynamicsModel(parameters=NominalParameters(velocity_tau_s=0.4), network=network)
    with pytest.raises(ValueError, match="trained"):
        model.predict(START, COMMAND, dt=0.005)
    with pytest.raises(ValueError, match="share a frame"):
        model.predict(START, replace(COMMAND, reference=replace(REFERENCE, reference_epoch=1)), dt=0.02)
    with pytest.raises(ValueError, match="does not cover"):
        model.predict(START, replace(COMMAND, effective_until_s=0.01), dt=0.02)
    with pytest.raises(ValueError, match="250 steps"):
        model.rollout(START, COMMAND, dt=0.02, steps=251)


@pytest.mark.parametrize("name, invalid", [("w1", np.zeros((2, 2))), ("b3", np.full(4, np.nan)),
                                            ("scale", np.zeros(SHAPES["scale"]))])
def test_invalid_weights_are_rejected(name, invalid):
    arrays = weights()
    arrays[name] = invalid
    with pytest.raises(ValueError, match=name):
        ResidualMLP(**arrays)


def test_checkpoint_format_version_and_immutable_arrays(tmp_path):
    path = tmp_path / "weights.npz"
    arrays = weights()
    np.savez(path, feature_version=np.int32(FEATURE_VERSION), **arrays)
    network = ResidualMLP.load(path)
    with pytest.raises(ValueError):
        network.b3.setflags(write=True)
    np.savez(path, feature_version=np.int32(FEATURE_VERSION + 1), **arrays)
    with pytest.raises(ValueError, match="feature version"):
        ResidualMLP.load(path)
    np.savez(path, feature_version=np.int32(FEATURE_VERSION), unknown=np.zeros(1), **arrays)
    with pytest.raises(ValueError, match="archive"):
        ResidualMLP.load(path)


def test_shipped_checkpoint_runs_reproducibly():
    path = Path(__file__).resolve().parents[1] / "artifacts" / "adaptive_dynamics_d1" / "residual_64x64.npz"
    model = ResidualDynamicsModel(network=ResidualMLP.load(path))
    first = model.rollout(START, COMMAND, dt=0.02, steps=100)
    assert first == model.rollout(START, COMMAND, dt=0.02, steps=100)
    assert np.isfinite(first[-1].state.position_m).all()
