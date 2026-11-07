from dataclasses import replace
from math import isfinite, pi

import pytest

from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint


REFERENCE = LocalNED("local_ned", 4, "replay")
START = DroneState(REFERENCE, 0.0, (0, 0, 0), (0, 0, 0), 0, 0)
TARGET = PositionSetpoint(REFERENCE, 0.0, 20.0, (1, -0.6, -0.3), 0.8)


def test_held_rollout_matches_varying_interface_and_converges_at_maximum_horizon():
    model = DynamicsModel()
    held = model.rollout(START, TARGET, dt=0.02, steps=250)
    assert held == model.rollout(START, (TARGET for _ in range(250)), dt=0.02)
    assert held[-1].state.timestamp_s == 5.0
    assert held[-1].state.position_m == pytest.approx(TARGET.position_m, abs=0.005)
    assert held[-1].state.yaw_rad == pytest.approx(TARGET.yaw_rad, abs=0.005)
    assert all(isfinite(x) for step in held for x in step.state.position_m + step.state.velocity_mps)


def test_reversal_uses_each_scheduled_command_and_keeps_reference():
    model = DynamicsModel()
    forward = replace(TARGET, position_m=(10, 0, 0), effective_until_s=0.5)
    backward = replace(TARGET, position_m=(-10, 0, 0), effective_from_s=0.5)
    result = model.rollout(START, [forward] * 25 + [backward] * 25, dt=0.02)
    assert result[24].state.velocity_mps[0] > 0
    assert result[25].state.velocity_mps[0] > 0  # momentum persists across the reversal
    assert result[25].acceleration_mps2[0] < 0
    assert result[-1].state.velocity_mps[0] < result[24].state.velocity_mps[0]
    assert all(step.state.reference == REFERENCE for step in result)


def test_rollout_is_repeatable_on_local_and_unix_epoch_clocks():
    model = DynamicsModel()
    initial_time = 1_790_000_000.0
    initial = replace(START, timestamp_s=initial_time)
    command = replace(TARGET, effective_from_s=initial_time, effective_until_s=initial_time + 5)
    shifted = model.rollout(initial, command, dt=0.02, steps=250)
    assert shifted == model.rollout(initial, command, dt=0.02, steps=250)
    local = model.rollout(START, TARGET, dt=0.02, steps=250)
    assert [step.state.position_m for step in shifted] == [step.state.position_m for step in local]
    assert shifted[-1].state.timestamp_s == initial_time + 5


def test_chained_maximum_rollouts_stay_finite_and_bounded():
    model = DynamicsModel()
    state = START
    for index in range(12):
        command = replace(TARGET, effective_from_s=index * 5, effective_until_s=(index + 1) * 5,
                          position_m=(1 if index % 2 else -1, 0, -0.3), yaw_rad=(-1)**index * 2.8)
        trajectory = model.rollout(state, command, dt=0.02, steps=250)
        assert all(-pi <= step.state.yaw_rad < pi for step in trajectory)
        assert all(abs(step.state.position_m[0]) < 1.1 for step in trajectory)
        state = trajectory[-1].state
    assert state.timestamp_s == 60.0


@pytest.mark.parametrize("dt, steps", [(0.02, 0), (0.02, -1), (0.02, 251),
                                        (0.05, 101), (0.02, None), (0.02, True)])
def test_step_and_horizon_limits(dt, steps):
    with pytest.raises(ValueError):
        DynamicsModel().rollout(START, TARGET, dt=dt, steps=steps)


def test_oversized_iterable_aborts_after_bounded_consumption():
    consumed = []

    def commands():
        while True:
            consumed.append(1)
            yield TARGET

    with pytest.raises(ValueError, match="250 steps"):
        DynamicsModel().rollout(START, commands(), dt=0.02)
    assert len(consumed) == 251


@pytest.mark.parametrize("schedule", [[], [TARGET, None],
                                      [replace(TARGET, effective_until_s=0.02)] * 2])
def test_rollout_aborts_on_empty_invalid_or_uncovered_schedule(schedule):
    with pytest.raises(ValueError):
        DynamicsModel().rollout(START, schedule, dt=0.02)
