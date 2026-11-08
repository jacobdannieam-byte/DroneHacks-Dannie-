"""Matched D5 decision ablation over one S2-selected grounded inspection goal.

Uses existing positive and failure fixtures. No simulator truth is supplied to
the predictor or selector. These are component results, not course completion.
"""
from dataclasses import asdict
from hashlib import sha256
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from refnav.integration_timing import Timings

from examples.h0_hybrid_smoke import smoke_inputs
from refnav.hybrid_integration import assess
from refnav.reasoning.selection import ReasonerConfig, create_reasoner
from refnav.trajectory_evaluation import GoalBinding
from refnav.dynamics.residual import ResidualMLP, ResidualDynamicsModel


def main():
    root = Path('artifacts/strategy_comparison')
    rows = []
    for model_name in ('d0', 'plain_11', 'physics_11', 'plain_29', 'physics_29', 'plain_47', 'physics_47'):
        predictor, identity = None, None
        if model_name != 'd0':
            path = Path('artifacts/goal_comparison/predictors') / (model_name + '.npz')
            identity = sha256(path.read_bytes()).hexdigest()
            predictor = ResidualDynamicsModel(network=ResidualMLP.load(path))
        for blocked in (False, True):
            inputs = smoke_inputs()
            original = inputs['goal']
            proposal, selection = create_reasoner(ReasonerConfig('G1')).select(original.context, original.selection.inputs,
                                                                             monotonic=lambda: 0.)
            inputs['goal'] = GoalBinding(original.context, proposal, selection)
            if blocked:
                # An unavailable estimator must stop all models before planning.
                inputs['estimator_healthy'] = False
            samples, timings = [], Timings()
            for _ in range(10):
                started = perf_counter()
                result = assess(**inputs, predictor=predictor, model_id=identity, timings=timings)
                samples.append((perf_counter() - started) * 1000)
            plan = result.receipt.plan
            wall_inputs = {key: value for key, value in inputs.items() if key != 'monotonic'}
            wall = assess(**wall_inputs, predictor=predictor, model_id=identity)
            rows.append({'predictor': model_name, 'checkpoint_sha256': identity,
                         'case': 'sensor_loss' if blocked else 'grounded_positive_inspection',
                         'goal_proposal_id': proposal.proposal_id, 'selection_identity': selection.identity,
                         'reason': result.validation_reason, 'ranked_ids': plan.ranked_ids,
                         'timing_ms': dict(zip(('p50', 'p95', 'p99', 'max'),
                                              map(float, np.percentile(samples, (50, 95, 99, 100))))),
                         'timing_samples': len(samples), 'pipeline_timing': timings.summary(),
                         'wall_deadline_reason': wall.validation_reason,
                         'evaluation': asdict(plan.evaluation) if plan.evaluation else None})
    with (root / 'predictor_decisions.json').open('x', encoding='utf-8') as stream:
        json.dump({'scope': 'S2 fixed; D5 fixed; logical deadline for accuracy; measured host timing', 'rows': rows},
                  stream, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main()
