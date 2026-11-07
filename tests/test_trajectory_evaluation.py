from dataclasses import asdict, replace
import math

import numpy as np
import pytest

from examples.unified_trajectory_replay import _binding, candidate, exercise, fixture
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint
from refnav.dynamics.adaptive import OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory
from refnav.trajectory_evaluation import (
    Candidate, FrameAlignment, VehicleBounds, _Geometry, evaluate_candidates,
)
from refnav.world_model.spatial_input import SpatialReference
from refnav.world_model.spatial_pipeline import SpatialSnapshot
from refnav.world_model.spatial_volume import Cell, MapPolicy, RollingVolume

NOW = 1_000_000_000
NED = LocalNED("ned", 1, "test")
MAP = SpatialReference("map", "test", 1)
STATE = DroneState(NED, 1., (.1, .1, -.1), (0., 0., 0.), 0., 0.)
ALIGN = FrameAlignment(NED, MAP, NOW, 0, math.pi / 2, (.2, -.2, 0.), 0., 0., "rig-check")
VEHICLE = VehicleBounds(.05, 0., 0.)


def grid(cells=None, *, captured=NOW, evidence_time=None):
    volume = RollingVolume(MapPolicy(shape=(20, 20, 20)))
    volume.origin[:] = -10
    volume.cells[:] = Cell.FREE
    volume.stamps[:] = captured if evidence_time is None else evidence_time
    if cells:
        cells(volume.cells)
    metric = volume.snapshot(MAP, NOW, captured, "map-frame", "test-camera", True)
    return SpatialSnapshot(metric, {}, None)


def evaluate(items, snapshot, *, state=STATE, alignment=ALIGN, vehicle=VEHICLE, now=NOW, **kwargs):
    return evaluate_candidates(state, items, snapshot, now_ns=now, alignment=alignment, vehicle=vehicle, **kwargs)


def test_rotated_translated_ned_and_occupied_priority():
    def obstruct(cells):
        cells[:8] = Cell.OCCUPIED
        cells[:4] = Cell.UNKNOWN
    options = (candidate(STATE, "forward", (2., .1, -.1)),
               candidate(STATE, "reverse", (-2., .1, -.1)))
    report = evaluate(options, grid(obstruct))
    assert report.candidates[0].status == "rejected"
    assert report.candidates[0].reasons == ("occupied_swept_volume",)
    assert report.candidates[1].clearance_lower_bound_m == .10
    assert report.ranked_candidate_ids == ("reverse",)


def test_unknown_volume_stays_unranked_even_with_clear_opening():
    state, snapshot, alignment = fixture()
    goal = _binding(snapshot)
    candidate_path = candidate(state, "toward_opening", (4.8, -.10, -1.30), steps=100)
    assert goal.proposal.intent == "observe_more"
    unavailable = replace(snapshot, metric=replace(snapshot.metric, available=False))
    verdict = evaluate((candidate_path,), unavailable, state=state, alignment=alignment,
                       now=snapshot.metric.query_ns, goal=goal)
    assert verdict.candidates[0].status == "unknown" and verdict.fallback_reason == "no_verified_candidate"


def test_invalidated_or_changed_semantic_context_has_no_rank():
    state, snapshot, alignment = fixture()
    goal = _binding(snapshot)
    # A caller retained the old proposal after WorldState invalidated the opening.
    changed = dict(snapshot.semantic)
    changed["entities"] = [dict(entity, status="invalidated") if entity["semantic_type"] == "opening"
                           else entity for entity in changed["entities"]]
    verdict = evaluate((candidate(state, "hold", state.position_m),), replace(snapshot, semantic=changed),
                       state=state, alignment=alignment, now=snapshot.metric.query_ns, goal=goal)
    assert verdict.candidates[0].reasons == ("semantic_context_changed",)
    assert not verdict.ranked_candidate_ids


@pytest.mark.parametrize(("field", "expected"), [
    ("clock_id", "clock_domain_mismatch"), ("reference_epoch", "reference_epoch_mismatch"),
])
def test_cross_clock_and_epoch_binding_fail_closed(field, expected):
    value = "other" if field == "clock_id" else 2
    state = replace(STATE, reference=replace(NED, **{field: value}))
    report = evaluate((candidate(state, "hold", state.position_m),), grid(), state=state,
                      alignment=replace(ALIGN, ned=state.reference))
    assert report.candidates[0].reasons == (expected,)


def test_wrong_frame_stale_geometry_and_unaligned_transform():
    options = (candidate(STATE, "hold", STATE.position_m),)
    assert evaluate(options, grid(), alignment=replace(ALIGN, ned=replace(NED, frame_id="different"))
                    ).candidates[0].reasons == ("frame_or_reference_mismatch",)
    assert evaluate(options, grid(captured=NOW - 300_000_000)).candidates[0].reasons == ("stale_geometry",)
    assert evaluate(options, grid(), alignment=replace(ALIGN, capture_ns=NOW - 40_000_000)
                    ).candidates[0].reasons == ("stale_or_unaligned_transform",)


def test_expired_cells_and_outside_map_are_unknown():
    options = (candidate(STATE, "hold", STATE.position_m),)
    assert evaluate(options, grid(evidence_time=NOW - 600_000_000)).candidates[0].status == "unknown"
    report = evaluate(options, grid(), alignment=replace(ALIGN, ned_origin_in_map_m=(4., 0., 0.)))
    assert report.candidates[0].status == "unknown" and report.candidates[0].rank is None


def test_vehicle_and_uncertainty_envelope_detects_overhead_collision():
    # The nominal z-up map position is 0.1 m and the ceiling voxel starts at 0.4 m.
    snapshot = grid(lambda cells: cells.__setitem__((slice(None), slice(None), slice(12, None)), Cell.OCCUPIED))
    options = (candidate(STATE, "hold", STATE.position_m),)
    assert evaluate(options, snapshot).candidates[0].status == "feasible"
    report = evaluate(options, snapshot, vehicle=VehicleBounds(.18, .04, .10))
    assert report.candidates[0].status == "rejected"
    assert report.candidates[0].inflation_m == pytest.approx(.14)


def test_continuous_box_queries_match_phase2_sweep_even_across_unseen_middle():
    def obstruct(cells):
        cells[9, 9:11, 9:11] = Cell.UNKNOWN
        cells[12, 9:11, 9:11] = Cell.OCCUPIED
    metric = grid(obstruct).metric
    probe = _Geometry(metric, NOW)
    for path in (((-.7, 0., 0.), (.3, 0., 0.)),
                 ((-.7, 0., 0.), (.8, 0., 0.)),
                 ((-.7, 0., 0.), (-.4, 0., 0.)),
                 ((-3., 0., 0.), (-1.8, 0., 0.))):
        occupied, unknown = probe.check(np.asarray(path), .05)
        verdict = "occupied" if occupied else "unknown" if unknown else "verified_free"
        assert verdict == metric.check_sweep(path, reference=MAP, now_ns=NOW,
                                             vehicle_radius_m=.05, margin_m=0.).status


def test_effective_command_and_forecast_contracts_are_enforced_with_overrides():
    invalid = PositionSetpoint(NED, 1.01, 2., STATE.position_m, 0.)
    report = evaluate((Candidate("late", (invalid,)),), grid())
    assert report.candidates[0].status == "rejected"
    valid = candidate(STATE, "hold", STATE.position_m)
    report = evaluate((valid,), grid(), predictor=OnlineFitDynamicsModel(), history=MotionHistory(), model_id="ridge")
    assert report.model_id == "ridge" and report.ranked_candidate_ids == ("hold",)

    class WrongClock:
        def rollout(self, state, commands, *, dt):
            valid = DynamicsModel().rollout(state, commands, dt=dt)
            step = replace(valid[0], state=replace(valid[0].state, reference=replace(NED, clock_id="other")))
            return (step, *valid[1:])

    report = evaluate((valid,), grid(), predictor=WrongClock(), model_id="bad")
    assert report.candidates[0].reasons == ("invalid_schedule_or_prediction",)


def test_yaw_wrap_and_deterministic_tie_break_return_only_diagnostics():
    state = replace(STATE, yaw_rad=math.pi - .01)
    command = PositionSetpoint(NED, 1., 2., state.position_m, -math.pi + .01)
    options = (Candidate("beta", (command,) * 50), Candidate("alpha", (command,) * 50))
    report = evaluate(options, grid(), state=state)
    assert report.ranked_candidate_ids == ("alpha", "beta")
    assert report.candidates[0].effort < .01
    result = asdict(report)
    assert all(not ({"schedule", "setpoint", "trajectory", "command"} & item.keys())
               for item in result["candidates"])


def test_real_synthetic_map_replay_rejects_wall_even_with_positive_hint():
    report, run = exercise()
    assert report["observed_with_goal"]["ranked_candidate_ids"][0] == "advance"
    assert report["wall_with_opening_hint"]["candidates"][0]["status"] == "rejected"
    assert report["stale_geometry"]["fallback_reason"] == "no_verified_candidate"
    assert len(run().candidates) == 8
