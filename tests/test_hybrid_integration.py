from dataclasses import replace

import pytest

from examples.h0_hybrid_smoke import smoke_inputs
from examples.unified_trajectory_replay import STAMP, fixture
from refnav import hybrid_integration
from refnav.sitl_review import validate_receipt


@pytest.fixture(scope="module")
def inputs():
    return smoke_inputs()


def test_world_and_mission_goal_reach_d5_and_independent_review(inputs):
    result = hybrid_integration.assess(**inputs)
    receipt = result.receipt
    assert result.validation_reason == "eligible_for_offline_review"
    assert receipt.plan.decision == "candidate_proposed"
    assert receipt.plan.ranked_ids[0] == "forward"
    assert receipt.plan.evaluation.goal_proposal_id == inputs["goal"].proposal.proposal_id
    assert receipt.plan.evaluation.model_id == "d0"
    assert receipt.plan.deadline_monotonic_s == .025
    assert receipt.state_received_ns == STAMP == receipt.metric.received_ns
    assert receipt.estimator_reset_counter == receipt.alignment.estimator_reset_counter == 0
    assert result.objective == inputs["objective"]
    assert result.vehicle_bounds == inputs["vehicle"]
    forged = replace(receipt, plan=replace(receipt.plan, ranked_ids=("hold",)))
    assert validate_receipt(forged, state=receipt.state, alignment=receipt.alignment,
                            metric=receipt.metric, now_ns=receipt.decision_ns) == "invalid_proposal"


def test_metric_goal_without_semantics(inputs):
    snapshot = replace(inputs["snapshot"], semantic={}, semantic_time_origin_ns=None)
    result = hybrid_integration.assess(**(inputs | {"snapshot": snapshot, "goal": None}))
    assert result.validation_reason == "eligible_for_offline_review"
    assert result.receipt.plan.evaluation.goal_proposal_id is None


@pytest.mark.parametrize("scenario, expected", [
    ("wrong_frame", "frame_or_reference_mismatch"),
    ("wrong_clock", "clock_domain_mismatch"),
    ("changed_epoch", "reference_epoch_mismatch"),
    ("stale_state", "stale_or_future_state"),
    ("stale_geometry", "stale_geometry"),
    ("future_receive", "invalid_state_receive_time"),
    ("missing_depth_receive", "invalid_geometry_receive_time"),
    ("missing_goal", "mission_goal_missing"),
    ("wrong_goal_reference", "metric_goal_reference_mismatch"),
    ("estimator_loss", "estimator_unhealthy"),
    ("reset_counter_changed", "estimator_reset_mismatch"),
    ("missing_reset", "missing_or_invalid_estimator_reset"),
])
def test_h0_stops_before_planning_on_invalid_inputs(inputs, scenario, expected):
    state, alignment, snapshot = (inputs[key] for key in ("state", "alignment", "snapshot"))
    reference = state.reference
    late = STAMP + 300_000_000
    changes = {
        "wrong_frame": {"state": replace(state, reference=replace(reference, frame_id="other_ned"))},
        "wrong_clock": {
            "state": replace(state, reference=replace(reference, clock_id="other")),
            "alignment": replace(alignment, ned=replace(reference, clock_id="other")),
        },
        "changed_epoch": {
            "state": replace(state, reference=replace(reference, reference_epoch=1)),
            "alignment": replace(alignment, ned=replace(reference, reference_epoch=1)),
        },
        "stale_state": {"now_ns": late},
        "stale_geometry": {
            "now_ns": late, "state_received_ns": late,
            "state": replace(state, timestamp_s=late / 1e9),
            "alignment": replace(alignment, capture_ns=late),
        },
        "future_receive": {"state_received_ns": STAMP + 1},
        "missing_depth_receive": {"snapshot": replace(snapshot, metric=replace(snapshot.metric, received_ns=None))},
        "missing_goal": {"objective": None},
        "wrong_goal_reference": {"objective": replace(inputs["objective"], reference=replace(alignment.map, epoch=1))},
        "estimator_loss": {"estimator_healthy": False},
        "reset_counter_changed": {"estimator_reset_counter": 1},
        "missing_reset": {"estimator_reset_counter": None},
    }[scenario]
    result = hybrid_integration.assess(**(inputs | changes))
    assert result.validation_reason == result.receipt.plan.reason == expected
    assert result.receipt.plan.decision == "safe_stop_requested"
    assert result.receipt.plan.ranked_ids == ()
    assert result.receipt.plan.evaluation is None


def test_planning_timeout_discards_candidates(inputs):
    ticks = iter((0., .026))
    result = hybrid_integration.assess(**(inputs | {"monotonic": lambda: next(ticks)}))
    assert result.validation_reason == result.receipt.plan.reason == "deadline_exceeded"
    assert not result.receipt.plan.ranked_ids


def test_validation_timeout_discards_previously_ranked_candidates(inputs, monkeypatch):
    elapsed = 0.
    check = hybrid_integration.validate_receipt

    def slow_check(*args, **kwargs):
        nonlocal elapsed
        reason = check(*args, **kwargs)
        assert reason == "eligible_for_offline_review"
        elapsed = .026
        return reason

    monkeypatch.setattr(hybrid_integration, "validate_receipt", slow_check)
    result = hybrid_integration.assess(**(inputs | {"monotonic": lambda: elapsed}))
    assert result.validation_reason == result.receipt.plan.reason == "deadline_exceeded"
    assert not result.receipt.plan.ranked_ids


def test_wall_rejects_trajectory_despite_semantic_hint(inputs):
    snapshot = replace(inputs["snapshot"], metric=fixture(blocked=True)[1].metric)
    state = replace(inputs["state"], position_m=(3.50, -.10, -1.3), velocity_mps=(.55, 0., 0.))
    result = hybrid_integration.assess(**(inputs | {"snapshot": snapshot, "state": state}))
    decision = result.receipt.plan
    assert decision.decision == "safe_stop_requested"
    assert result.validation_reason == "no_verified_progress"
    assert not decision.ranked_ids
    assert any(row.status == "rejected" and "occupied_swept_volume" in row.reasons
               for row in decision.evaluation.candidates)


def test_semantic_context_from_another_snapshot_cannot_select_a_trajectory(inputs):
    snapshot = fixture(blocked=True)[1]
    result = hybrid_integration.assess(**(inputs | {"snapshot": snapshot}))
    decision = result.receipt.plan
    assert decision.decision == "safe_stop_requested"
    assert not decision.ranked_ids
    assert all("semantic_context_changed" in row.reasons for row in decision.evaluation.candidates)


def test_h2_map_consumer_is_not_called_for_invalid_estimator(inputs):
    def forbidden(*args):
        raise AssertionError('Invalid estimator state reached map reasoning.')
    result = hybrid_integration.assess(**(inputs | {'estimator_healthy': False}), map_consumer=forbidden)
    assert result.map_reasoning is None
    assert result.receipt.plan.decision == 'safe_stop_requested'
