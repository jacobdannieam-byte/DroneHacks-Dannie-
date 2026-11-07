from dataclasses import replace
from math import inf, nan, pi

import pytest

from refnav.dynamics import (DroneState, DynamicsModel, LocalNED, NominalParameters,
                             PositionSetpoint, PredictedState)


REFERENCE = LocalNED("local_ned", 0, "replay")
REST = DroneState(REFERENCE, 0.0, (0, 0, 0), (0, 0, 0), 0, 0)
HOLD = PositionSetpoint(REFERENCE, 0.0, 5.0, (0, 0, 0), 0)


def test_zero_state_stays_at_equilibrium():
    result = DynamicsModel().predict(REST, HOLD, dt=0.02)
    assert isinstance(result.state, PredictedState)
    assert result.state.timestamp_s == 0.02
    assert result.state.position_m == result.state.velocity_mps == (0, 0, 0)
    assert result.acceleration_mps2 == (0, 0, 0)
    assert result.state.yaw_rate_rad_s == result.yaw_acceleration_rad_s2 == 0


def test_matching_tracking_reference_preserves_signed_constant_velocity():
    params = NominalParameters(position_gain_per_s=1)
    state = replace(REST, velocity_mps=(0.5, -0.3, -0.2))
    command = replace(HOLD, position_m=state.velocity_mps)
    result = DynamicsModel(params).predict(state, command, dt=0.02)
    assert result.state.velocity_mps == state.velocity_mps
    assert result.state.position_m == pytest.approx((0.01, -0.006, -0.004))
    assert result.acceleration_mps2 == (0, 0, 0)


def test_far_position_step_saturates_acceleration_and_updates_position_semi_implicitly():
    model = DynamicsModel()
    command = replace(HOLD, position_m=(5, -5, -2))
    result = model.predict(REST, command, dt=0.02)
    assert result.velocity_reference_mps == (1.2, -1.2, -0.7)
    assert result.acceleration_mps2 == (2.0, -2.0, -1.2)
    assert result.state.velocity_mps == pytest.approx((0.04, -0.04, -0.024))
    assert result.state.position_m == pytest.approx((0.0008, -0.0008, -0.00048))
    assert REST.velocity_mps == (0, 0, 0)


def test_small_step_uses_velocity_lag_before_hitting_limits():
    result = DynamicsModel().predict(REST, replace(HOLD, position_m=(0.1, 0, 0)), dt=0.02)
    assert result.velocity_reference_mps[0] == pytest.approx(0.12)
    assert result.state.velocity_mps[0] == pytest.approx(0.12 / 0.35 * 0.02)


def test_heading_wrap_takes_short_arc_and_crosses_pi():
    state = replace(REST, yaw_rad=pi - 0.005, yaw_rate_rad_s=0.5)
    command = replace(HOLD, yaw_rad=-pi + 0.08)
    result = DynamicsModel().predict(state, command, dt=0.02)
    assert result.yaw_rate_reference_rad_s == pytest.approx(0.17)
    assert result.state.yaw_rad == pytest.approx(-pi + 0.004266666666666667)
    assert result.state.yaw_rate_rad_s > 0


@pytest.mark.parametrize("dt", [0, -0.02, 0.0001, 0.051, nan, inf, True])
def test_bad_steps_fail_before_prediction(dt):
    with pytest.raises(ValueError, match="dt"):
        DynamicsModel().predict(REST, HOLD, dt=dt)


def test_step_rejects_configured_lag_that_is_too_fast_for_discretization():
    with pytest.raises(ValueError, match="integration bound"):
        DynamicsModel(NominalParameters(yaw_tau_s=0.03)).predict(REST, HOLD, dt=0.02)


@pytest.mark.parametrize("reference", [LocalNED("other_ned", 0, "replay"),
                                       LocalNED("local_ned", 1, "replay"),
                                       LocalNED("local_ned", 0, "system")])
def test_reference_binding_rejects_mixed_origin_epoch_or_clock(reference):
    with pytest.raises(ValueError, match="share a frame"):
        DynamicsModel().predict(REST, replace(HOLD, reference=reference), dt=0.02)


@pytest.mark.parametrize("start, end", [(-1, 0), (0.001, 1), (-1, 0.01)])
def test_step_requires_effective_command_for_its_entire_interval(start, end):
    command = replace(HOLD, effective_from_s=start, effective_until_s=end)
    with pytest.raises(ValueError, match="does not cover"):
        DynamicsModel().predict(REST, command, dt=0.02)


def test_coarse_clock_resolution_is_rejected():
    state = replace(REST, timestamp_s=1e15)
    command = replace(HOLD, effective_from_s=1e15, effective_until_s=1e15 + 1)
    with pytest.raises(ValueError, match="timestamp"):
        DynamicsModel().predict(state, command, dt=0.02)
