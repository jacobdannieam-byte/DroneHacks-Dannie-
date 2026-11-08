"""Compare D5, classical MPC and learned-dynamics MPC without command authority.

Synthetic plane truth scores returned diagnostics only. These results cannot
qualify a controller, native deadline, or collision-free physical flight.
"""
from dataclasses import asdict
import json
from pathlib import Path

from examples.diagnostic_mpc import optimize
from examples.unified_planner_ablation import cases, _truth
from examples.unified_trajectory_replay import VEHICLE
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP
from refnav.predictive_planner import plan, candidate_for_id


def main():
    output = Path('artifacts/goal_comparison')
    learned = ResidualDynamicsModel(network=ResidualMLP.load(output / 'predictors/plain_11.npz'))
    rows = []
    for case in cases():
        for name, predictor in (('classical_mpc', None), ('neural_model_mpc', learned)):
            result = optimize(case.state, case.snapshot, case.alignment, VEHICLE, case.objective,
                              case.now_ns, predictor=predictor, model_id=None if predictor is None else 'plain_11')
            rows.append({'case': case.name, 'variant': name, 'status': result.status,
                         'elapsed_ms': result.elapsed_seconds * 1000, 'evaluations': result.evaluations,
                         'truth': None if result.candidate is None else _truth(case, result.candidate)})
        fixed = plan(case.state, case.snapshot, now_ns=case.now_ns, alignment=case.alignment,
                     vehicle=VEHICLE, objective=case.objective, goal=None)
        chosen = candidate_for_id(case.state, case.objective, case.alignment, fixed.ranked_ids[0]) if fixed.ranked_ids else None
        rows.append({'case': case.name, 'variant': 'D5', 'status': fixed.reason,
                     'truth': None if chosen is None else _truth(case, chosen)})
    (output / 'mpc.json').write_text(json.dumps({'scope': 'diagnostic only; no gateway artifact', 'rows': rows}, indent=2) + '\n')
    print(json.dumps(rows, indent=2))


if __name__ == '__main__':
    main()
