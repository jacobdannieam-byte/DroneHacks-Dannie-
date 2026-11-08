"""Goal comparison invariants, independent rejection and actual metric effects."""
from copy import deepcopy
from dataclasses import replace
import hashlib
import json

import pytest

from examples.goal_synthetic import InspectionEpisode
from examples.h0_hybrid_smoke import smoke_inputs
from examples.unified_spatial_replay import calibration
from refnav.hybrid_integration import assess
from refnav.reasoning import DeterministicReasoner, build_context, validate_proposal
from refnav.reasoning.inspection import Attempt, candidates, make_inputs
from refnav.reasoning.selection import ReasonerConfig, create_reasoner
from refnav.reasoning.bridge import resolve_inspection
from refnav.reasoning.schema import ReasoningContext, GoalProposal
from refnav.trajectory_evaluation import GoalBinding


def test_default_is_exact_baseline_and_v1_roundtrips():
    context, inputs = InspectionEpisode(201).observe()
    proposal, envelope = create_reasoner().select(context, inputs)
    assert proposal == DeterministicReasoner().propose(context)
    assert ReasoningContext.from_json(context.to_json()) == context
    assert GoalProposal.from_json(proposal.to_json()) == proposal
    assert envelope.inputs.context_reference == context.context_reference


@pytest.mark.parametrize('variant', ['G0', 'G1', 'G2'])
def test_determinism_missing_model_and_permutation(variant):
    context, inputs = InspectionEpisode(205).observe()
    reasoner = create_reasoner(ReasonerConfig(variant))
    assert reasoner.select(context, inputs) == reasoner.select(context, inputs)
    proposal, _ = reasoner.select(context, inputs)
    assert validate_proposal(proposal, context, now=.8, reference=context.reference).valid
    if variant == 'G2':
        assert proposal == DeterministicReasoner().propose(context)


@pytest.mark.parametrize('regime', ['stale', 'contradiction', 'missing_support', 'reset'])
def test_bad_evidence_cannot_get_an_inspection(regime):
    context, inputs = InspectionEpisode(200, regime).observe()
    assert candidates(context, inputs) == ()
    proposal, envelope = create_reasoner(ReasonerConfig('G1')).select(context, inputs)
    assert envelope.selected_entity_id is None
    assert proposal.intent == 'insufficient_information'


def test_failed_attempts_stop_and_history_is_bounded():
    context, inputs = InspectionEpisode(201).observe()
    attempts = tuple(Attempt(c.entity_id, .8, 'failed') for c in candidates(context, inputs) for _ in range(3))
    inputs = replace(inputs, attempts=attempts)
    proposal, _ = create_reasoner(ReasonerConfig('G1')).select(context, inputs)
    assert proposal.intent == 'insufficient_information'
    assert proposal.schema_version == 2
    assert GoalProposal.from_json(proposal.to_json()) == proposal
    with pytest.raises(ValueError):
        replace(proposal, schema_version=1)
    assert validate_proposal(proposal, context, now=.8, reference=context.reference).valid
    assert candidates(context, replace(inputs, attempts=(Attempt('opening_0', 100., 'failed'),))) == ()
    with pytest.raises(ValueError):
        replace(inputs, attempts=(Attempt('opening_0', .8, 'failed'),) * 33)


def test_deadline_falls_back_to_fresh_baseline():
    context, inputs = InspectionEpisode(202).observe()
    ticks = iter((0., .1))
    proposal, envelope = create_reasoner(ReasonerConfig('G1')).select(context, inputs, monotonic=lambda: next(ticks))
    assert proposal == DeterministicReasoner().propose(context)
    assert envelope.reason == 'deadline_fresh_baseline_fallback'


def test_inspection_pose_is_not_opening_center_and_reconstructs():
    inputs = smoke_inputs()
    binding = inputs['goal']
    point, reason = resolve_inspection(binding, inputs['snapshot'], inputs['state'], inputs['alignment'], inputs['now_ns'])
    target = next(e for e in binding.context.entities if e.entity_id == binding.proposal.target_entity_id)
    assert reason is None and point != target.geometry.center
    assert assess(**inputs).receipt.plan.decision == 'candidate_proposed'
    moved = replace(inputs['state'], position_m=(2.2, -.1, -1.3))
    assert resolve_inspection(binding, inputs['snapshot'], moved, inputs['alignment'], inputs['now_ns'])[1] == 'inspection_inputs_changed'
    forged = replace(binding, selection=replace(binding.selection, selected_entity_id='unknown'))
    assert resolve_inspection(forged, inputs['snapshot'], inputs['state'], inputs['alignment'], inputs['now_ns'])[1] == 'inspection_identity_changed'


def test_legacy_observe_more_cannot_mean_move_to_center():
    inputs = smoke_inputs()
    inputs['goal'] = replace(inputs['goal'], selection=None)
    result = assess(**inputs)
    assert result.receipt.plan.decision == 'safe_stop_requested'
    assert all('inspection_contract_required' in c.reasons for c in result.receipt.plan.evaluation.candidates)


def test_reasoner_choice_changes_grounded_downstream_objective():
    inputs = smoke_inputs()
    source = deepcopy(inputs['snapshot'].semantic)
    first = next(e for e in source['entities'] if e['semantic_type'] == 'opening')
    first['geometry']['center'][1] = .7
    first['observations'] = 8
    alternative = deepcopy(first)
    alternative.update(entity_id='alternative', observations=1)
    alternative['geometry']['center'][1] = .1
    source['entities'].append(alternative)
    source['total_entities'] = len(source['entities'])
    snapshot = replace(inputs['snapshot'], semantic=source)
    context = build_context(source, now=inputs['goal'].context.context_created_at,
                            reference=inputs['goal'].context.reference)
    auxiliary = make_inputs(context, snapshot, inputs['state'], inputs['alignment'], calibration())
    selected, positions = [], []
    for variant in ('G0', 'G1'):
        from examples.h1_observation import HybridBagReplay
        from examples.h1_fixture import profile
        replay = HybridBagReplay(replace(profile(), alignment=inputs['alignment'],
                                         reasoner=ReasonerConfig(variant)))
        replay._state = (inputs['state'], inputs['now_ns'], inputs['now_ns'], 100000, ())
        binding = replay.select_goal(snapshot, context.mission_goal)
        proposal, envelope = binding.proposal, binding.selection
        assert validate_proposal(proposal, context, now=context.context_created_at, reference=context.reference).valid
        binding = GoalBinding(context, proposal, envelope)
        selected.append(proposal.target_entity_id)
        positions.append(resolve_inspection(binding, snapshot, inputs['state'], inputs['alignment'], inputs['now_ns'])[0])
        result = assess(**(inputs | {'snapshot': snapshot, 'goal': binding}))
        assert result.receipt.plan.evaluation.goal_proposal_id == proposal.proposal_id
    assert selected[0] != selected[1] and positions[0] != positions[1]


def test_model_checks_hash_schema_and_finite_values(tmp_path):
    from refnav.reasoning.learned import GoalMLP, features
    import numpy as np
    context, inputs = InspectionEpisode(201).observe()
    options = candidates(context, inputs)
    assert np.array_equal(features(context, options[::-1]), features(context, options)[::-1])
    path = tmp_path / 'bad.json'
    path.write_text(json.dumps({'feature_version': 999}))
    with pytest.raises(ValueError):
        GoalMLP.load(path, hashlib.sha256(path.read_bytes()).hexdigest())
    reasoner = create_reasoner(ReasonerConfig('G2', str(path), 'wrong_hash'))
    assert reasoner.select(context, inputs)[0] == DeterministicReasoner().propose(context)


def test_nonfinite_model_falls_back_and_no_yaw_support_abstains():
    from types import SimpleNamespace
    context, inputs = InspectionEpisode(201).observe()
    reasoner = create_reasoner(ReasonerConfig('G2'))
    reasoner.model = SimpleNamespace(identity='bad', score=lambda context, options: (float('nan'),) * len(options))
    assert reasoner.select(context, inputs)[0] == DeterministicReasoner().propose(context)
    backwards = replace(inputs, rotation_map_optical=((0., 0., -1.), (1., 0., 0.), (0., -1., 0.)))
    assert candidates(context, backwards) == ()


def test_observation_goal_cannot_complete_checkpoint():
    from refnav.hybrid_mission import CourseMission, Stage
    from refnav.trajectory_evaluation import MetricGoal
    inputs = smoke_inputs()
    stage = Stage('inspect', 'landing', MetricGoal(inputs['alignment'].map, (2.1, .1, 1.3)),
                  semantic=inputs['goal'].context.mission_goal, landing_site_review='fixture')
    mission = CourseMission((stage,))
    for stamp in (inputs['now_ns'], inputs['now_ns'] + 1):
        mission.select(stamp, inputs, inputs['goal'])
    assert mission.index == 0 and mission.land_reason is None
    assert mission.reason == 'inspection_goal'


def test_h1_exhaustion_history_survives_abstention_and_resets_with_stage():
    from examples.h1_observation import HybridBagReplay
    from examples.h1_fixture import profile
    inputs = smoke_inputs()
    replay = HybridBagReplay(profile())
    context = inputs['goal'].context
    snapshot = inputs['snapshot']
    replay._inspection_history(context, snapshot, context.mission_goal, 'first')
    replay._inspection_attempts = (Attempt('opening_0003', context.context_created_at, 'failed'),) * 3
    replay._inspection_last = None
    replay._inspection_history(context, snapshot, context.mission_goal, 'first')
    assert len(replay._inspection_attempts) == 3
    replay._inspection_history(context, snapshot, context.mission_goal, 'next')
    assert not replay._inspection_attempts
