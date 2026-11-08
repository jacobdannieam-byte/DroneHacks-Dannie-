"""Matched physics-consistency residual ablation on existing closed-loop datasets.

Reuse D1 training/inference and A1 causal replay. This is a soft D0 response-law
prior, not thrust dynamics or a claim of calibrated aircraft physics.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.d1_synthetic import generate
from examples.train_adaptive_dynamics_d1 import fit, training_samples
from examples.a1_data import fixtures, fingerprints
from examples.evaluate_adaptive_dynamics_a1 import load_models, score_cohort
from refnav.dynamics.residual import ResidualMLP, ResidualDynamicsModel, FEATURE_VERSION

SEEDS = (11, 29, 47)
PHYSICS_STRENGTH = .1  # Our own preregistered soft consistency weight.


def summarize(results, models):
    forecasts = [row for result in results for row in result.forecasts]
    summary = {}
    for name in models:
        summary[name] = {}
        for horizon in (10, 25, 50, 100):
            rows = [r for r in forecasts if r.model == name and len(r.errors['position_m']) >= horizon]
            summary[name][str(horizon * .02)] = {
                key: float(np.sqrt(np.mean([r.errors[key][horizon - 1] ** 2 for r in rows])))
                for key in ('position_m', 'heading_rad')}
    return summary


def main():
    output = Path('artifacts/goal_comparison/predictors')
    output.mkdir(parents=True, exist_ok=True)
    training, validation = generate('train', count=40), generate('validation', count=12)
    train, valid = training_samples(training), training_samples(validation)
    existing, hashes, _, d2_manifest = load_models(Path('artifacts'))
    models = dict(existing)
    from examples.evaluate_adaptive_dynamics_a2 import load_candidate
    from refnav.dynamics.adaptive_features import AdaptiveFeatureDynamics, FeatureBasis
    models['adaptive_features'], hashes['adaptive_features'] = load_candidate(
        Path('artifacts/unified_phase3'), d2_manifest)
    models['gated_linear'] = AdaptiveFeatureDynamics(FeatureBasis.linear_only())
    report = {'physics_weight': PHYSICS_STRENGTH, 'training': {}, 'existing_hashes': hashes,
              'training_fingerprints': [fingerprints(e) for e in training],
              'validation_fingerprints': [fingerprints(e) for e in validation],
              'loss': 'mean((g*r-y)^2) + lambda*mean((g*r)^2); r normalized acceleration residual',
              'parameters_each': 5572, 'epochs_each': 55, 'default': 'd0'}
    report['thresholds'] = {'nominal_regression_fraction_max': .1,
                            'shifted_improvement_fraction_min': .2, 'native_runtime_required': True}
    for seed in SEEDS:
        for strength in (0., PHYSICS_STRENGTH):
            name = f"{'physics' if strength else 'plain'}_{seed}"
            weights, epoch, history = fit(train, valid, seed=seed, physics_strength=strength)
            path = output / (name + '.npz')
            np.savez(path, feature_version=np.int32(FEATURE_VERSION), **weights)
            models[name] = ResidualDynamicsModel(network=ResidualMLP.load(path))
            report['training'][name] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                        'epoch': epoch, 'validation_loss': history[epoch]['validation_loss']}
            print('Trained', name, flush=True)
    cohorts, _ = fixtures(include_legacy=False)
    report['test_fingerprints'] = {}
    report['regimes'] = {}
    for regime in ('matched_nominal', 'in_range', 'shifted', 'switched', 'delayed', 'noisy'):
        rows = cohorts[regime][:4]
        training_ids = {e.response_id() for e in (*training, *validation)}
        if any(row.episode.response_id() in training_ids for row in rows):
            raise ValueError('Training and test dynamics configurations overlap.')
        report['test_fingerprints'][regime] = [fingerprints(r.episode) for r in rows]
        report['regimes'][regime] = summarize(score_cohort(rows, models), models)
        print('Scored', regime, flush=True)
    (output / 'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')


if __name__ == '__main__':
    main()
