from examples.semantic_reasoning_demo import reasoning_replay
from refnav.reasoning import ContextConfig, MissionGoal, WorldReference, build_context
from refnav.reasoning import DeterministicReasoner, validate_proposal
from refnav.reasoning.evaluation import evaluate_replay, populated_world


def test_expected_replay_decisions_and_deliberately_weak_comparison():
    result = evaluate_replay()
    assert result['scenario_count'] == 7
    assert result['expected_decisions_matched'] == result['grounded_proposals_validated'] == 7
    assert result['width_only_proposals_rejected'] == 6


def test_demo_is_deterministic_and_contains_no_ground_truth_input():
    first = reasoning_replay()
    assert first == reasoning_replay()
    assert all('depth' not in row['context'] and 'rays' not in row['context'] for row in first)
    assert all(row['validation']['valid'] for row in first)
    assert first[-1]['proposal']['intent'] == 'reject_blocked_candidate'


def test_representative_counts_stay_bounded_and_keep_preferred_mission_target():
    for count in (1,32,128):
        world = populated_world(count)
        now = world.snapshot()['timestamp']
        config = ContextConfig(max_entities=8,max_context_bytes=12000)
        target = f'opening_{999+count:04d}'
        context = build_context(world,MissionGoal(target_entity_id=target),now=now,reference=WorldReference(),config=config)
        assert context.context_valid
        assert len(context.entities) <= 8
        assert context.omitted_entities == count - len(context.entities)
        assert target in {e.entity_id for e in context.entities}
        assert len(context.to_json().encode()) <= 12000
        assert len(world.snapshot()['entities']) == count
        proposal = DeterministicReasoner().propose(context)
        assert validate_proposal(proposal,context,now=now,reference=WorldReference()).valid
        if context.omitted_entities:
            assert 'omitted_context_entities' in proposal.missing_information
