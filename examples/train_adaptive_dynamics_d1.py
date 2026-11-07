"""Train a D1 checkpoint offline using NumPy Adam and episode-level validation."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from examples.d1_synthetic import RANGES, RESPONSE_NAMES, SEED, generate
from refnav.dynamics import DynamicsModel
from refnav.dynamics.residual import FEATURE_VERSION, RESIDUAL_LIMITS, SHAPES, ResidualMLP, residual_features

EPOCHS = 55
BATCH_SIZE = 256
LEARNING_RATE = 0.001


def training_samples(episodes):
    nominal = DynamicsModel()
    acceleration_limits = np.array((*nominal.parameters.acceleration_limits_mps2,
                                    nominal.parameters.yaw_acceleration_limit_rad_s2))
    residual_limits = np.array(RESIDUAL_LIMITS)
    features, gates, targets = [], [], []
    for episode in episodes:
        for index, command in enumerate(episode.commands):
            state = episode.sample(index)
            tracking = nominal._tracking(state, command, episode.dt)
            feature, gate = residual_features(state, command, episode.dt, tracking)
            delta = episode.states[index + 1] - episode.states[index]
            achieved = np.array((*delta[3:6], delta[7])) / episode.dt
            achievable = np.clip(achieved, -acceleration_limits, acceleration_limits)
            residual = achievable - np.array((*tracking[1], tracking[3]))
            # Train the correction that can pass both the residual bound and the activity gate.
            target = np.clip(residual / residual_limits, -np.asarray(gate), gate)
            features.append(feature)
            gates.append(gate)
            targets.append(target)
    return tuple(np.asarray(values, dtype=np.float32) for values in (features, gates, targets))


def _forward(inputs, weights):
    hidden1 = np.maximum(inputs @ weights["w1"] + weights["b1"], 0)
    hidden2 = np.maximum(hidden1 @ weights["w2"] + weights["b2"], 0)
    output = np.tanh(hidden2 @ weights["w3"] + weights["b3"])
    return hidden1, hidden2, output


def fit(train, validation, *, seed=SEED):
    inputs, activity, targets = train
    valid_inputs, valid_activity, valid_targets = validation
    mean = inputs.mean(axis=0)
    scale = np.maximum(inputs.std(axis=0), 0.001)
    inputs = (inputs - mean) / scale
    valid_inputs = (valid_inputs - mean) / scale
    rng = np.random.default_rng(seed)
    weights = {}
    for layer in range(1, 4):
        name = f"w{layer}"
        shape = SHAPES[name]
        weights[name] = rng.normal(0, np.sqrt(2 / shape[0]), shape).astype(np.float32)
        weights[f"b{layer}"] = np.zeros(SHAPES[f"b{layer}"], dtype=np.float32)
    weights["w3"].fill(0)
    moment = {name: np.zeros_like(weight) for name, weight in weights.items()}
    variance = {name: np.zeros_like(weight) for name, weight in weights.items()}
    best_loss = float(np.mean(valid_targets ** 2))
    best_weights = {name: value.copy() for name, value in weights.items()}
    best_epoch, update = 0, 0
    history = [{"epoch": 0, "validation_loss": best_loss}]
    for epoch in range(1, EPOCHS + 1):
        order = rng.permutation(len(inputs))
        for start in range(0, len(order), BATCH_SIZE):
            batch = order[start:start + BATCH_SIZE]
            x, gate, y = inputs[batch], activity[batch], targets[batch]
            h1, h2, out = _forward(x, weights)
            grad3 = (2.0 / out.size) * (out * gate - y) * gate * (1 - out ** 2)
            grad2 = (grad3 @ weights["w3"].T) * (h2 > 0)
            grad1 = (grad2 @ weights["w2"].T) * (h1 > 0)
            gradients = {"w1": x.T @ grad1, "b1": grad1.sum(axis=0),
                         "w2": h1.T @ grad2, "b2": grad2.sum(axis=0),
                         "w3": h2.T @ grad3, "b3": grad3.sum(axis=0)}
            update += 1
            for name, gradient in gradients.items():
                moment[name] = 0.9 * moment[name] + 0.1 * gradient
                variance[name] = 0.999 * variance[name] + 0.001 * gradient ** 2
                first = moment[name] / (1 - 0.9 ** update)
                second = variance[name] / (1 - 0.999 ** update)
                weights[name] -= LEARNING_RATE * first / (np.sqrt(second) + 1e-8)
        loss = float(np.mean((_forward(valid_inputs, weights)[-1] * valid_activity - valid_targets) ** 2))
        history.append({"epoch": epoch, "validation_loss": loss})
        if loss < best_loss:
            best_loss, best_epoch = loss, epoch
            best_weights = {name: value.copy() for name, value in weights.items()}
    return {**best_weights, "mean": mean, "scale": scale}, best_epoch, history


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("artifacts/adaptive_dynamics_d1"))
    args = parser.parse_args()
    training_episodes, validation_episodes = generate("train"), generate("validation")
    response_ids = {split: [episode.response_id() for episode in episodes]
                    for split, episodes in (("train", training_episodes), ("validation", validation_episodes))}
    unique = set(response_ids["train"] + response_ids["validation"])
    if len(unique) != len(training_episodes) + len(validation_episodes):
        raise ValueError("training and validation response settings must be disjoint")
    train, validation = training_samples(training_episodes), training_samples(validation_episodes)
    weights, best_epoch, history = fit(train, validation)
    args.output.mkdir(parents=True, exist_ok=True)
    checkpoint = args.output / "residual_64x64.npz"
    np.savez(checkpoint, feature_version=np.int32(FEATURE_VERSION), **weights)
    ResidualMLP.load(checkpoint)
    report = {
        "seed": SEED, "epochs": EPOCHS, "batch_size": BATCH_SIZE, "learning_rate": LEARNING_RATE,
        "selection": "Minimum validation normalized residual MSE, including epoch zero",
        "training_episodes": len(training_episodes), "validation_episodes": len(validation_episodes),
        "training_transitions": len(train[0]), "validation_transitions": len(validation[0]),
        "response_ranges": dict(zip(RESPONSE_NAMES, RANGES)),
        "response_ids": response_ids,
        "selected_epoch": best_epoch, "validation_history": history,
        "parameter_count": sum(v.size for name, v in weights.items() if name not in ("mean", "scale")),
        "checkpoint_sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
    }
    (args.output / "training.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Selected epoch {best_epoch}, validation loss {history[best_epoch]['validation_loss']:.6f}")
    print(f"Wrote {checkpoint} and training.json")


if __name__ == "__main__":
    main()
