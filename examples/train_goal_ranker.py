"""Train three small goal rankers on layout-separated synthetic outcome labels.

This is supervised expected-outcome regression, not G0 policy imitation. Hidden
visibility is used only here and in the evaluator, never in runtime features.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.goal_synthetic import InspectionEpisode, SPLITS
from refnav.reasoning.inspection import candidates
from refnav.reasoning.learned import features, FEATURE_VERSION, FEATURE_COUNT, HIDDEN_WIDTH

SEEDS = (11, 29, 47)
EPOCHS = 300  # Our own fixed tuning budget, identical for all seeds.
LEARNING_RATE = .02


def dataset(split):
    rows, labels = [], []
    for layout in SPLITS[split]:
        episode = InspectionEpisode(layout)
        for step in range(4):
            context, inputs = episode.observe()
            options = candidates(context, inputs)
            rows.extend(features(context, options))
            labels.extend(2 * episode.expected_measurement(c) - .25 * c.movement_cost
                          - .75 * c.risk_proxy - c.failed_attempts / 3 - .2 * c.switching for c in options)
            if options:
                episode.act(options[step % len(options)].entity_id)
    return np.asarray(rows), np.asarray(labels)


def fit(train, validation, seed):
    x, y = train
    mean, scale = x.mean(0), np.maximum(x.std(0), .001)
    x = (x - mean) / scale
    vx, vy = (validation[0] - mean) / scale, validation[1]
    rng = np.random.default_rng(seed)
    w1 = rng.normal(0, .15, (FEATURE_COUNT, HIDDEN_WIDTH))
    b1, w2, b2 = np.zeros(HIDDEN_WIDTH), rng.normal(0, .15, HIDDEN_WIDTH), np.array(0.)
    best, best_loss, best_epoch = None, float('inf'), None
    for epoch in range(EPOCHS):
        hidden = np.tanh(x @ w1 + b1)
        error = 2 * (hidden @ w2 + b2 - y) / len(y)
        hidden_gradient = error[:, None] * w2 * (1 - hidden ** 2)
        gradients = (x.T @ hidden_gradient, hidden_gradient.sum(0), hidden.T @ error, error.sum())
        for parameter, gradient in zip((w1, b1, w2, b2), gradients):
            parameter -= LEARNING_RATE * gradient
        loss = float(np.mean((np.tanh(vx @ w1 + b1) @ w2 + b2 - vy) ** 2))
        if loss < best_loss:
            best_loss, best_epoch = loss, epoch
            best = {name: value.copy().tolist() for name, value in
                    zip(('w1', 'b1', 'w2', 'b2', 'mean', 'scale'), (w1, b1, w2, b2, mean, scale))}
    return dict(feature_version=FEATURE_VERSION, **best), best_loss, best_epoch


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/goal_comparison/models'))
    args = parser.parse_args(argv)
    args.output.mkdir(parents=True, exist_ok=True)
    train, validation = dataset('train'), dataset('validation')
    report = {'label': 'synthetic expected measurement outcome; not policy imitation', 'seeds': [],
              'splits': {k: list(v) for k, v in SPLITS.items()}, 'epochs': EPOCHS,
              'learning_rate': LEARNING_RATE, 'feature_count': FEATURE_COUNT, 'parameter_count': 289,
              'dataset_sha256': hashlib.sha256(b''.join(a.tobytes() for pair in (train, validation) for a in pair)).hexdigest()}
    for seed in SEEDS:
        model, loss, epoch = fit(train, validation, seed)
        path = args.output / f'goal_mlp_{seed}.json'
        path.write_text(json.dumps(model, sort_keys=True, allow_nan=False) + '\n')
        report['seeds'].append({'seed': seed, 'validation_mse': loss, 'epoch': epoch,
                                'checkpoint': path.as_posix(), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    (args.output / 'training.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report['seeds'], indent=2))


if __name__ == '__main__':
    main()
