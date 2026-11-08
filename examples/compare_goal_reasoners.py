"""Paired recorded and interactive synthetic goal comparison, with honest limits.

No collision/completion claims are possible in this abstract observation model.
The separate H0 smoke test exercises actual D0/D3/D5 over rendered depth.
"""
import argparse
import hashlib
import json
from pathlib import Path
import platform
from time import perf_counter
import tracemalloc

import numpy as np

from examples.goal_synthetic import InspectionEpisode, SPLITS, EPISODE_STEPS
from refnav.reasoning.selection import ReasonerConfig, create_reasoner
from refnav.reasoning.validation import validate_proposal
from refnav.reasoning.inspection import candidates

# Fixed before holdout: no promotion without target-device and paired benefit.
THRESHOLDS = {'invalid_action_rate': 0., 'paired_evidence_gain_lower_95ci': 0.,
              'host_reasoner_target_ms': 5., 'native_concurrent_budget': 'unmeasured',
              'default': 'G0', 'promotion_requires_native_evidence': True}
REGIMES = ('nominal', 'dead_end', 'stale', 'contradiction', 'missing_support', 'reset', 'omissions')


def episode_result(layout, regime, reasoner, interactive):
    environment = InspectionEpisode(layout, regime)
    gained = switches = repeats = invalid = abstentions = useful_abstentions = rejected = 0
    seen, previous, latency, context_latency = set(), None, [], []
    for _ in range(EPISODE_STEPS if interactive else 1):
        start = perf_counter()
        context, inputs = environment.observe()
        context_latency.append((perf_counter() - start) * 1000)
        start = perf_counter()
        # Accuracy comparisons use a fixed logical clock. Wall timings and missed
        # allocations are reported separately; H2 and negative tests enforce deadlines.
        proposal, selection = reasoner.select(context, inputs, monotonic=lambda: 0.)
        latency.append((perf_counter() - start) * 1000)
        validation = validate_proposal(proposal, context, now=.8, reference=context.reference)
        rejected += not validation.valid
        invalid += not validation.valid and proposal.target_entity_id is not None
        target = selection.selected_entity_id if validation.valid else None
        options = candidates(context, inputs)
        if target is None:
            abstentions += 1
            useful_abstentions += not any(environment.expected_measurement(c) > 0 for c in options)
            continue
        repeats += target in seen
        switches += previous is not None and target != previous
        seen.add(target)
        previous = target
        if interactive:
            gained += environment.act(target)
    return dict(layout=layout, regime=regime, acquired=gained, switching=switches, repeats=repeats,
                invalid=invalid, rejected=rejected, abstentions=abstentions,
                appropriate_abstentions=useful_abstentions, reasoner_ms=latency, context_ms=context_latency)


def interval(values):
    values = np.asarray(values, dtype=float)
    rng = np.random.default_rng(411)
    means = values[rng.integers(0, len(values), (2000, len(values)))].mean(1)
    return [float(v) for v in np.percentile(means, (2.5, 97.5))]


def summarize(rows):
    result = {key: sum(r[key] for r in rows) for key in
              ('acquired', 'switching', 'repeats', 'invalid', 'rejected', 'abstentions', 'appropriate_abstentions')}
    for key in ('reasoner_ms', 'context_ms'):
        times = [t for row in rows for t in row[key]]
        result[key] = dict(zip(('p50', 'p95', 'p99', 'max'), map(float, np.percentile(times, (50, 95, 99, 100)))))
        result[key]['over_5ms'] = sum(t > 5 for t in times)
    result.update(unsupported_connectivity_claims=0, verified_mission_progress=None,
                  recovery_success=None, collisions=None, completion=None)
    decisions = sum(len(r['reasoner_ms']) for r in rows)
    result.update(decisions=decisions, invalid_targeted_proposal_rate=result['invalid'] / decisions,
                  validator_rejection_rate=result['rejected'] / decisions,
                  unnecessary_abstentions=result['abstentions'] - result['appropriate_abstentions'])
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/goal_comparison'))
    args = parser.parse_args(argv)
    training = json.loads((args.output / 'models/training.json').read_text())
    configs = {'G0': ReasonerConfig(), 'G1': ReasonerConfig('G1')}
    for row in training['seeds']:
        configs[f"G2_seed{row['seed']}"] = ReasonerConfig('G2', row['checkpoint'], row['sha256'])
    results = {}
    for name, config in configs.items():
        reasoner = create_reasoner(config)
        results[name] = [episode_result(layout, regime, reasoner, True)
                         for layout in SPLITS['test'] for regime in REGIMES]
    # Bootstrap paired layout means, never treating repeated seeds/steps as independent layouts.
    paired = {}
    for name, rows in results.items():
        differences = [np.mean([a['acquired'] - b['acquired'] for a, b in zip(rows, results['G0'])
                                if a['layout'] == layout]) for layout in SPLITS['test']]
        paired[name] = {'mean_evidence_difference': float(np.mean(differences)), '95ci': interval(differences)}
        versus_g1 = [np.mean([a['acquired'] - b['acquired'] for a, b in zip(rows, results['G1'])
                              if a['layout'] == layout]) for layout in SPLITS['test']]
        paired[name]['versus_G1_95ci'] = interval(versus_g1)
    tracemalloc.start()
    episode_result(200, 'nominal', create_reasoner(configs['G1']), True)
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    report = {'thresholds': THRESHOLDS, 'host': platform.platform(), 'layout_count': len(SPLITS['test']),
              'regimes': REGIMES, 'episodes_per_variant': len(results['G0']), 'steps_per_episode': EPISODE_STEPS,
              'interactive': {name: summarize(rows) for name, rows in results.items()}, 'paired': paired,
              'recorded_replay': {name: summarize([episode_result(i, 'nominal', create_reasoner(config), False)
                                                  for i in SPLITS['test']]) for name, config in configs.items()},
              'python_peak_bytes_one_G1_episode': peak, 'GPU_CPU_load': 'not measured',
              'scope': 'abstract synthetic evidence acquisition; no flight or route progress',
              'selection': 'G0 retained; native concurrent timing and physical outcomes unavailable'}
    from refnav.reasoning.evaluation import evaluate_replay
    report['legacy_recorded_semantic_replay'] = evaluate_replay()
    (args.output / 'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    (args.output / 'episodes.json').write_text(json.dumps(results, allow_nan=False) + '\n')
    print(json.dumps({'paired': paired, 'acquired': {k: v['acquired'] for k, v in report['interactive'].items()}}, indent=2))


if __name__ == '__main__':
    main()
