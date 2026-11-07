
from dataclasses import replace

import pytest

from examples.unified_planner_ablation import StraightLine, _truth, cases
from examples.unified_trajectory_replay import STAMP, VEHICLE, fixture
from refnav.dynamics import DynamicsModel
from refnav.predictive_planner import STEPS, _generate, plan
from refnav.trajectory_evaluation import MetricGoal
from refnav.world_model.spatial_input import SpatialReference


@pytest.fixture(scope="module")
def scenarios():
    return {case.name: case for case in cases()}


def assess(case, **overrides):
    inputs = dict(now_ns=case.now_ns, alignment=case.alignment, vehicle=VEHICLE, objective=case.objective,
                  goal=case.goal, monotonic=lambda: 0.)
    return plan(case.state, case.snapshot, **{**inputs, **overrides})


def test_candidates_are_bounded_and_result_contains_diagnostic_ids_only(scenarios):
    case = scenarios["rendered_corridor"]
    candidates = tuple(_generate(case.state, case.objective, case.alignment))
    assert len(candidates) == 8 and len({candidate.candidate_id for candidate in candidates}) == 8
    assert all(len(candidate.schedule) == STEPS for candidate in candidates)
    assert all(command.reference == case.state.reference for candidate in candidates for command in candidate.schedule)
    result = assess(case)
    assert result == assess(case) and result.ranked_ids[0] == "forward"
    assert all(type(item) is str for item in result.ranked_ids)
    assert not hasattr(result, "schedule")


@pytest.mark.parametrize("name", ("rendered_wall_with_opening_hint", "unobserved", "stale", "narrow_clearance"))
def test_stale_unknown_wall_and_narrow_clearance_do_not_propose_motion(scenarios, name):
    result = assess(scenarios[name])
    assert result.decision == "safe_stop_requested" and not result.ranked_ids
    assert result.evaluation is not None
    if name == "narrow_clearance":
        assert any(row.status == "feasible" and row.clearance_lower_bound_m == 0.
                   for row in result.evaluation.candidates)
    if name == "rendered_wall_with_opening_hint":
        assert any(row.status == "rejected" for row in result.evaluation.candidates)


def test_geometric_shortcut_misses_inertial_drift_in_synthetic_oracle(scenarios):
    case = scenarios["crosswind_shifted"]
    reactive = assess(case, goal=None, predictor=StraightLine(), model_id="reactive")
    selected = reactive.ranked_ids[0]
    candidates = {item.candidate_id: item for item in _generate(case.state, case.objective, case.alignment)}
    assert _truth(case, candidates[selected])["truth_collision"]
    assert assess(case).decision == "safe_stop_requested"


def test_budget_and_cancellation_discard_partial_results(scenarios):
    case = scenarios["clear"]
    calls = []

    class Predictor:
        def rollout(self, *args, **kwargs):
            calls.append(True)
            return DynamicsModel().rollout(*args, **kwargs)

    predictor = Predictor()
    result = assess(case, predictor=predictor, model_id="test", cancelled=lambda: bool(calls))
    assert len(calls) == 1 and result.reason == "cancelled" and not result.ranked_ids
    calls.clear()
    result = assess(case, predictor=predictor, model_id="test",
                    monotonic=lambda: .026 if calls else 0.)
    assert len(calls) == 1 and result.reason == "deadline_exceeded" and not result.ranked_ids


def test_wrong_goal_reference_and_no_motion_at_destination(scenarios):
    case = scenarios["clear"]
    wrong = MetricGoal(SpatialReference("other", case.alignment.map.clock_id, case.alignment.map.epoch,
                                       case.alignment.map.body_frame), case.objective.position_m)
    assert assess(case, objective=wrong).reason == "objective_or_state_reference_mismatch"
    state, snapshot, alignment = fixture()
    objective = MetricGoal(alignment.map, (state.position_m[0], -state.position_m[1], -state.position_m[2]))
    assert plan(state, snapshot, now_ns=STAMP, alignment=alignment, vehicle=VEHICLE, objective=objective,
                monotonic=lambda: 0.).decision == "safe_stop_requested"


def test_stale_geometry_cannot_be_revived_by_fresh_state(scenarios):
    case = scenarios["clear"]
    now = case.now_ns + 300_000_000
    case = replace(case, now_ns=now, state=replace(case.state, timestamp_s=now / 1e9),
                   alignment=replace(case.alignment, capture_ns=now))
    result = assess(case)
    assert not result.ranked_ids
    assert result.evaluation.candidates[0].reasons == ("stale_geometry",)
