from dataclasses import replace
from types import SimpleNamespace as Msg

import numpy as np
import pytest

from examples.h1_fixture import DEPTH, HEALTH, MS, PX4_TOPICS, clock, depth, odometry, profile, setup, tick
from examples.hybrid_shadow_loop import ShadowLoop
from examples.hybrid_sitl import ReplayDriver, main
from refnav.core import NAV_OFFBOARD, Output, Vehicle
from refnav.hybrid_gateway import HybridGateway
from refnav.world_model.spatial_volume import Cell


def shadow(*, offset=False, **options):
    options.setdefault("monotonic", lambda: 0.)
    loop = ShadowLoop(profile(offset_rig=offset), rate_hz=20, **options)
    driver = ReplayDriver(loop, lambda _: None, auto_poll=False)
    setup(driver)
    tick(driver, 100, 0.)
    return loop, driver


def test_shadow_reuses_snapshot_and_drops_missed_ticks():
    loop, driver = shadow(offset=True)
    start = loop.poll(103 * MS, 103 * MS)
    assert start.reason == "eligible_for_offline_review"
    assert loop.poll(110 * MS, 110 * MS) is None
    later = loop.poll(153 * MS, 153 * MS)
    assert later.assessment.receipt.metric is start.assessment.receipt.metric
    tick(driver, 230, .2)
    fresh = loop.poll(233 * MS, 253 * MS)
    assert fresh.assessment.receipt.metric is not start.assessment.receipt.metric
    assert loop.skipped_ticks == 1
    assert len(loop.replay.rows) == 1
    assert fresh.report()["state_received_ns"] == 232 * MS
    assert fresh.report()["depth_capture_ns"] == 230 * MS


def test_shadow_snapshot_is_immutable_and_timing_uses_selected_depth():
    loop, driver = shadow(measure_receipt_wall=True, completion_clock=lambda: 200 * MS)
    _, inputs = loop.replay.prepare(103 * MS)
    snapshot = inputs["snapshot"]
    with pytest.raises(TypeError):
        snapshot.semantic["entities"][0]["status"] = "changed"
    assert isinstance(snapshot.semantic["entities"], tuple)
    tick(driver, 150, .05, with_depth=False)
    result = loop.poll(153 * MS, 153 * MS).report()
    assert result["depth_received_ns"] == 102 * MS
    assert result["depth_receipt_to_decision_wall_ns"] == 98 * MS
    assert result["px4_sample_us"] == 140_000
    assert result["clock_evidence_id"] == driver.loop.replay.hybrid.clock.evidence_id


def test_shadow_missing_stale_and_unhealthy_inputs_stop():
    loop = ShadowLoop(profile(), rate_hz=20)
    loop.feed("/clock", clock(100), 100 * MS, 100 * MS)
    assert loop.poll(100 * MS, 100 * MS).reason == "missing_synchronized_depth"
    loop, driver = shadow(offset=True)
    assert loop.poll(103 * MS, 103 * MS).reason == "eligible_for_offline_review"
    driver.feed("/clock", clock(205), 205 * MS)
    assert loop.poll(205 * MS, 205 * MS).reason == "stale_state_or_clock_mapping"
    driver.feed(HEALTH, Msg(data=False), 206 * MS)
    assert loop.poll(206 * MS, 255 * MS).reason == "geometry_unavailable"


def test_shadow_clock_stall_and_rewind_have_no_assessment():
    loop, driver = shadow(offset=True)
    assert loop.poll(103 * MS, 103 * MS).assessment is not None
    paused = loop.poll(103 * MS, 450 * MS)
    assert paused.reason == "shadow_clock_stalled"
    assert paused.assessment is None
    driver.feed("/clock", clock(90), 104 * MS)
    fault = loop.poll(104 * MS, 451 * MS)
    assert "clock rewind" in fault.reason
    assert fault.assessment is None


def test_shadow_reset_fences_subsequent_frames_and_close_stops_work():
    loop, driver = shadow(offset=True)
    msg = odometry(110, 0.)
    msg.reset_counter = 5
    driver.feed(PX4_TOPICS["vehicle_odometry"], msg, 112 * MS)
    assert loop.poll(112 * MS, 112 * MS).reason == "px4_odometry_reset_requires_new_epoch"
    before = loop.replay.now
    loop.close()
    driver.feed("/clock", clock(120), 120 * MS)
    assert loop.replay.now == before
    assert loop.poll(120 * MS, 120 * MS).reason == "shadow_closed"


def test_depth_overload_replaces_pending_frame_instead_of_backlogging():
    loop, driver = shadow(offset=True)
    driver.feed(DEPTH, depth(120, 0., cal=driver.profile.calibration), 122 * MS)
    driver.feed(DEPTH, depth(121, 0., cal=driver.profile.calibration), 123 * MS)
    assert loop.replay.dropped_depth == 1
    assert loop.poll(123 * MS, 123 * MS).reason == "awaiting_capture_pose"
    tick(driver, 180, 0.)
    assert loop.replay.dropped_depth == 2
    assert loop.poll(183 * MS, 183 * MS).reason == "eligible_for_offline_review"


def test_contradictory_semantic_observation_replaces_old_goal():
    loop, driver = shadow()
    old = loop.poll(103 * MS, 103 * MS).report()
    tick(driver, 220, 0., blocked=True)
    new = loop.poll(223 * MS, 223 * MS).report()
    assert old["goal_id"] != new["goal_id"]
    assert not new["ranked_ids"]
    assert all("semantic_target_blocked" in candidate["reasons"] for candidate in new["candidates"])


def test_shadow_deadline_discards_partial_ranking():
    ticks = iter((0., .026))
    loop, _ = shadow(offset=True, monotonic=lambda: next(ticks))
    result = loop.poll(103 * MS, 103 * MS).report()
    assert result["reason"] == "deadline_exceeded"
    assert not result["ranked_ids"]


@pytest.fixture
def owner_input():
    loop, _ = shadow(offset=True)
    tick_result = loop.poll(103 * MS, 103 * MS)
    assessment = tick_result.assessment
    _, inputs = loop.replay.prepare(tick_result.now_ns)
    state = inputs["state"]
    out = Output("go", (*state.position_m, state.yaw_rad), stream=True)
    vehicle = Vehicle(pos=state.position_m, heading=state.yaw_rad, pos_ok=True, armed=True,
                      landed=False, nav_state=NAV_OFFBOARD)
    return assessment, dict(inputs=inputs, now_ns=tick_result.now_ns, owner_output=out,
                            vehicle=vehicle, owner_ready=True, owner_generation=2, simulation=True)


def test_gateway_fake_sink_selects_only_one_setpoint_and_never_replays(owner_input):
    assessment, args = owner_input
    baseline = args["owner_output"].setpoint
    sink = []
    disabled = HybridGateway(monotonic=lambda: 0.)
    disabled.offer(assessment, owner_generation=2)
    sink.append(disabled.choose(**args).setpoint)
    gate = HybridGateway(enabled=True, monotonic=lambda: 0.)
    gate.offer(assessment, owner_generation=2)
    decision = gate.choose(**args)
    assert decision.reason == "hybrid_position_selected"
    sink.append(decision.setpoint)
    sink.append(gate.choose(**args).setpoint)
    assert sink[0] == sink[2] == baseline
    assert len(sink[1]) == 4 and sink[1] != baseline
    assert not gate.pending


@pytest.mark.parametrize("case, reason", [
    ("hardware", "gateway_disabled"),
    ("takeover", "offboard_authority_absent"),
    ("landing", "owner_gate_closed"),
    ("owner_change", "owner_generation_changed"),
    ("health", "owner_gate_closed"),
    ("expired", "proposal_age_exceeded"),
    ("changed_epoch", "frame_or_reference_mismatch"),
    ("calibration", "proposal_input_changed"),
    ("reset_counter", "proposal_context_changed"),
    ("deadline", "handoff_deadline_exceeded"),
    ("disagree", "owner_state_disagrees_or_stale"),
    ("heading", "owner_state_disagrees_or_stale"),
    ("invalidated", "proposal_absent"),
    ("stop", "no_verified_progress"),
])
def test_gateway_rejections_keep_exact_baseline(case, reason, owner_input):
    assessment, args = owner_input
    gate = HybridGateway(enabled=True, monotonic=(lambda: .026 if case == "deadline" else 0.))
    if case == "hardware":
        args["simulation"] = False
    elif case == "takeover":
        args["vehicle"] = replace(args["vehicle"], nav_state=4)
    elif case == "landing":
        args["owner_output"] = Output("land", commands=["land"])
    elif case == "owner_change":
        args["owner_generation"] += 1
    elif case == "health":
        args["owner_ready"] = False
    elif case == "expired":
        args["now_ns"] += 50 * MS
    elif case == "changed_epoch":
        state = args["inputs"]["state"]
        args["inputs"]["state"] = replace(state, reference=replace(state.reference, reference_epoch=1))
    elif case == "calibration":
        snapshot = args["inputs"]["snapshot"]
        args["inputs"]["snapshot"] = replace(snapshot, metric=replace(snapshot.metric, calibration_id="changed"))
    elif case == "reset_counter":
        args["inputs"]["estimator_reset_counter"] = 5
    elif case == "disagree":
        args["vehicle"] = replace(args["vehicle"], pos=(.5, -.1, -1.3))
    elif case == "heading":
        args["vehicle"] = replace(args["vehicle"], heading=.2)
    elif case == "stop":
        receipt = assessment.receipt
        stop = replace(receipt.plan, decision="safe_stop_requested", reason="no_verified_progress", ranked_ids=())
        assessment = replace(assessment, receipt=replace(receipt, plan=stop))
    gate.offer(assessment, owner_generation=2)
    if case == "invalidated":
        gate.invalidate()
    result = gate.choose(**args)
    assert result.reason == reason
    assert result.setpoint == args["owner_output"].setpoint
    assert not gate.pending


@pytest.mark.parametrize("cell, reason", [
    (Cell.UNKNOWN, "unobserved_swept_volume"), (Cell.OCCUPIED, "occupied_swept_volume"),
])
def test_gateway_rechecks_sweep_even_when_diagnostics_claim_feasible(cell, reason, owner_input):
    assessment, args = owner_input
    snapshot = args["inputs"]["snapshot"]
    metric = replace(snapshot.metric, _cells=np.full_like(snapshot.metric._cells, cell))
    args["inputs"]["snapshot"] = replace(snapshot, metric=metric)
    assessment = replace(assessment, receipt=replace(assessment.receipt, metric=metric))
    gate = HybridGateway(enabled=True, monotonic=lambda: 0.)
    gate.offer(assessment, owner_generation=2)
    result = gate.choose(**args)
    assert result.reason == "sweep_rejected:" + reason
    assert result.setpoint == args["owner_output"].setpoint


def test_gateway_checks_held_target_beyond_the_d0_horizon(owner_input):
    assessment, args = owner_input
    snapshot = args["inputs"]["snapshot"]
    cells = snapshot.metric._cells.copy()
    # The D0 rollout stops short of the obstacle in x=[1.0, 1.2].
    cells[5 - snapshot.metric.origin_index[0], :, :] = Cell.OCCUPIED
    metric = replace(snapshot.metric, _cells=cells)
    args["inputs"]["snapshot"] = replace(snapshot, metric=metric)
    assessment = replace(assessment, receipt=replace(assessment.receipt, metric=metric))
    gate = HybridGateway(enabled=True, monotonic=lambda: 0.)
    gate.offer(assessment, owner_generation=2)
    result = gate.choose(**args)
    assert result.reason == "target_sweep_rejected:occupied"
    assert result.setpoint == args["owner_output"].setpoint


def test_handoff_expiring_during_independent_validation_keeps_fallback(owner_input, monkeypatch):
    from refnav import hybrid_gateway

    assessment, args = owner_input
    now = 0.
    evaluator = hybrid_gateway.evaluate_candidates

    def slow_evaluation(*a, **kwargs):
        nonlocal now
        result = evaluator(*a, **kwargs)
        now = .026
        return result

    monkeypatch.setattr(hybrid_gateway, "evaluate_candidates", slow_evaluation)
    gate = HybridGateway(enabled=True, monotonic=lambda: now)
    gate.offer(assessment, owner_generation=2)
    result = gate.choose(**args)
    assert result.reason == "handoff_deadline_exceeded"
    assert result.setpoint == args["owner_output"].setpoint


def test_guarded_cli_only_prepares_checklist(tmp_path):
    import json

    output = tmp_path / "guarded.json"
    assert main(["guarded-review", "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert report["gateway_enabled"] is False
    assert "exact_isolated_sitl_trial_approved" in report["requires"]
