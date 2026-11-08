"""Strategy identities affect real consumers; mismatched experiments fail closed."""
from copy import deepcopy
from dataclasses import replace
from hashlib import sha256
from pathlib import Path

import pytest

from examples.h1_observation import HybridBagReplay
from examples.strategy_comparison import CONDITIONS, compare
from refnav.strategies import StrategyConfig


def test_selector_default_and_invalid_combinations():
    assert StrategyConfig().reasoner().variant == 'G0'
    assert StrategyConfig('hybrid_information_gain').reasoner().variant == 'G1'
    for config in ({'identifier': 'classical'}, {'identifier': 'hybrid_learned_ranker'},
                   {'schema_version': True}, {'predictor_checkpoint': 'extra'}):
        with pytest.raises(ValueError):
            StrategyConfig(**config)


def test_pinned_physics_model_and_explicit_load_fallback():
    path = Path('artifacts/goal_comparison/predictors/physics_11.npz')
    config = StrategyConfig('hybrid_physics_informed', predictor_checkpoint=str(path),
                            predictor_sha256=sha256(path.read_bytes()).hexdigest())
    model, identity, fallback = config.load_predictor()
    assert model is not None and identity.endswith(config.predictor_sha256) and fallback is None
    broken = replace(config, predictor_sha256='0' * 64)
    assert broken.load_predictor() == (None, None, 'predictor_load_fallback:ValueError')
    assert broken.identity != config.identity


def test_external_runs_require_matching_conditions_and_keep_missing_metrics():
    first = {'strategy': 'S1', 'run_id': 'recorded', 'revision': 'a' * 40,
             'configuration_sha256': 'b' * 64, 'conditions': {key: 'same' for key in CONDITIONS},
             'metrics': {'completion': True, 'touches': 0}}
    second = deepcopy(first)
    second.update(strategy='S2', run_id='hybrid')
    result = compare([first, second])
    assert result['comparable'] and result['runs'][0]['metrics']['useful_evidence'] is None
    second['conditions']['seed'] = None
    assert not compare([first, second])['comparable']
    second['conditions']['seed'] = 'different'
    assert not compare([first, second])['comparable']
    assert not compare([first, first])['comparable']


def test_mpc_cancellation_discards_schedule():
    from examples.diagnostic_mpc import optimize
    from examples.unified_planner_ablation import cases
    from examples.unified_trajectory_replay import VEHICLE
    case = next(c for c in cases() if c.name == 'clear')
    result = optimize(case.state, case.snapshot, case.alignment, VEHICLE, case.objective,
                      case.now_ns, cancelled=lambda: True, monotonic=lambda: 0.)
    assert result.status == 'cancelled' and result.candidate is None and result.evaluations == 0


def test_mpc_solver_failure_discards_schedule():
    from examples.diagnostic_mpc import optimize
    from examples.unified_planner_ablation import cases
    from examples.unified_trajectory_replay import VEHICLE
    class FailedSolver:
        def rollout(self, *args, **kwargs):
            raise RuntimeError('solver failed')
    case = next(c for c in cases() if c.name == 'clear')
    result = optimize(case.state, case.snapshot, case.alignment, VEHICLE, case.objective,
                      case.now_ns, predictor=FailedSolver(), model_id='failure_fixture', monotonic=lambda: 0.)
    assert result.status == 'solver_failure' and result.candidate is None


def test_h1_passes_selected_predictor_to_actual_d5_consumer():
    from examples.h1_fixture import profile, setup, tick
    from refnav.hybrid_integration import assess
    path = Path('artifacts/goal_comparison/predictors/physics_11.npz')
    config = StrategyConfig('hybrid_physics_informed', predictor_checkpoint=str(path),
                            predictor_sha256=sha256(path.read_bytes()).hexdigest())
    replay = HybridBagReplay(replace(profile(offset_rig=True), strategy_config=config, reasoner=config.reasoner()))
    setup(replay)
    tick(replay, 100, 0.)
    reason, inputs = replay.prepare(replay.now)
    assert reason is None and inputs['predictor'] is replay.predictor
    result = assess(**inputs, monotonic=lambda: 0.)
    assert result.receipt.plan.evaluation.model_id == replay.predictor_id
    assert result.receipt.plan.evaluation.model_id.startswith('physics:')


def test_yaw_visibility_is_bounded_and_s0_does_not_acquire_new_primitive():
    from examples.goal_synthetic import InspectionEpisode
    from refnav.reasoning.inspection import candidates
    from refnav.reasoning.selection import create_reasoner
    context, auxiliary = InspectionEpisode(201).observe()
    # Narrow horizontal camera view makes observed off-axis openings require a turn.
    cal = replace(auxiliary.calibration, K=(2000., auxiliary.calibration.K[1],
                                           auxiliary.calibration.K[2], auxiliary.calibration.K[3]))
    auxiliary = replace(auxiliary, calibration=cal)
    options = candidates(context, auxiliary)
    assert any(option.mode == 'yaw' for option in options)
    assert all(abs(option.yaw_delta_map_rad) <= .45 for option in options)
    _, selection = create_reasoner().select(context, auxiliary, monotonic=lambda: 0.)
    yaw_ids = {option.entity_id for option in options if option.mode == 'yaw'}
    assert selection.selected_entity_id not in yaw_ids


def test_yaw_reconstructs_and_checks_geometry_without_checkpoint_progress():
    from examples.h0_hybrid_smoke import smoke_inputs
    from refnav.reasoning.selection import create_reasoner
    from refnav.trajectory_evaluation import GoalBinding
    from refnav.hybrid_integration import assess
    from refnav.predictive_planner import candidate_for_id
    inputs = smoke_inputs()
    original = inputs['goal']
    auxiliary = original.selection.inputs
    cal = replace(auxiliary.calibration, K=(10000., *auxiliary.calibration.K[1:]))
    auxiliary = replace(auxiliary, calibration=cal)
    proposal, selection = create_reasoner(StrategyConfig('hybrid_information_gain').reasoner()).select(
        original.context, auxiliary, monotonic=lambda: 0.)
    inputs['goal'] = GoalBinding(original.context, proposal, selection)
    result = assess(**inputs)
    assert result.validation_reason == 'eligible_for_offline_review'
    assert result.receipt.plan.ranked_ids == ('inspection-yaw',)
    candidate = candidate_for_id(inputs['state'], inputs['objective'], inputs['alignment'], 'inspection-yaw',
                                 goal=inputs['goal'], snapshot=inputs['snapshot'], now_ns=inputs['now_ns'])
    assert candidate.schedule[0].position_m == inputs['state'].position_m
    assert candidate.schedule[0].yaw_rad != inputs['state'].yaw_rad
    assert candidate_for_id(inputs['state'], inputs['objective'], inputs['alignment'], 'inspection-yaw') is None
    assert candidate_for_id(inputs['state'], inputs['objective'], inputs['alignment'], 'inspection-yaw',
                            goal=inputs['goal'], snapshot=inputs['snapshot'], now_ns=inputs['now_ns'] + 300_000_000) is None
