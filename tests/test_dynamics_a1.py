from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from examples.a1_data import FAULT_END, FAULT_STEP, LONG_ANCHORS, Replay, verify_splits
from examples.a1_metrics import HORIZONS, calibrate
from examples.d1_synthetic import generate
from examples.evaluate_adaptive_dynamics_a1 import Forecast, load_models, score_replay
from refnav.dynamics import DynamicsModel
from refnav.dynamics.adaptive import OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory


@pytest.fixture(scope="module")
def replay():
    episode, = generate("shifted", count=1, steps=210, change_step=65)
    return Replay("test:fixed", episode, LONG_ANCHORS)


@pytest.mark.parametrize(("fault", "expected"), [
    ("missing_history", ("empty", "empty", "empty")),
    ("dropped_samples", (None, "stale_or_unaligned", "ready")),
    ("stale_history", ("stale_or_unaligned", "warming_up", "ready")),
    ("epoch_reset", ("empty", "ready", "ready")),
    ("late_epoch_transition", ("empty", "empty", "ready")),
])
def test_public_history_fallback_and_recovery(replay, fault, expected):
    result = score_replay(replace(replay, fault=fault), {"d0": DynamicsModel(), "online_fit": OnlineFitDynamicsModel()})
    forecasts = {row.anchor: row for row in result.forecasts if row.model == "online_fit"}
    assert tuple(forecasts[label].status if label in forecasts else None
                 for label in ("after_0.4s", "fault_end", "after_0.7s")) == expected
    if fault == "dropped_samples":
        assert not set(range(FAULT_STEP, FAULT_END)).intersection(result.sample_indices)
    if fault == "late_epoch_transition":
        assert result.reset_events == {
            "cross_epoch_rejected": 1, "late_cross_epoch_rejected": 1,
            "late_cross_epoch_cleared_ready_history": 1,
        }


def test_only_completed_measured_transitions_reach_forecasts(replay, monkeypatch):
    delivered = []
    original = MotionHistory.observe

    def record(history, before, command, after):
        original(history, before, command, after)
        delivered.append(after)

    class CheckedRidge(OnlineFitDynamicsModel):
        def rollout(self, state, commands, *, dt, history, steps=None):
            assert delivered[-1] == state
            assert all(sample.timestamp_s <= state.timestamp_s for sample in delivered)
            assert commands[0].effective_from_s <= state.timestamp_s < commands[0].effective_until_s
            return super().rollout(state, commands, dt=dt, history=history, steps=steps)

    monkeypatch.setattr(MotionHistory, "observe", record)
    result = score_replay(replay, {"d0": DynamicsModel(), "online_fit": CheckedRidge()})
    assert len(result.forecasts) == 2 * len(LONG_ANCHORS)
    assert all(len(row.errors["position_m"]) == 100 for row in result.forecasts)


def test_delay_override_isolates_the_delays():
    case, = generate("test", count=1)
    delayed, = generate("test", count=1, position_delay_steps=6, yaw_delay_steps=9)
    assert case.commands == delayed.commands
    assert np.array_equal(case.states[0], delayed.states[0])
    assert not np.array_equal(case.states[1:], delayed.states[1:])
    assert delayed.response == {**case.response, "position_delay_s": 0.12, "yaw_delay_s": 0.18}
    with pytest.raises(ValueError, match="stationary episode"):
        generate("test", count=1, change_step=30, yaw_delay_steps=9)


def test_split_guard_rejects_reused_training_configuration():
    artifacts = Path(__file__).resolve().parents[1] / "artifacts"
    _, _, d1, d2 = load_models(artifacts)
    case = generate("train")[0]
    with pytest.raises(ValueError, match="split overlap for reused"):
        verify_splits({"leaked": (Replay("reused", case, {}),)}, (), d1, d2)


def test_calibration_counts_independent_episodes_and_their_worst_anchor():
    results = []
    for i in range(10):
        rows = [
            Forecast(str(i), "d0", str(anchor), 0.3, "stateless",
                     {"position_m": np.full(100, i + anchor), "heading_rad": np.full(100, i + anchor)})
            for anchor in (0, 1)
        ]
        results.append(SimpleNamespace(forecasts=rows))
    summary = calibrate(results, ("d0",))
    assert summary["episode_count"] == 10
    assert all(summary["bounds"]["d0"][str(h)]["position_m"] == 10 for h in HORIZONS)
