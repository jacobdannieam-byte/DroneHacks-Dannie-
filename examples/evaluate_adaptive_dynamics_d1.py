"""Score a frozen D1 checkpoint on held-out episodes and benchmark host inference."""

import argparse
import hashlib
import json
import os
import platform
from pathlib import Path

import numpy as np

from examples.adaptive_dynamics_d0_demo import INITIAL, error_series, schedule, state_vector, time_calls
from examples.d0_synthetic import SyntheticResponse, simulate
from examples.d1_synthetic import Episode, SEED, generate
from refnav.dynamics import DynamicsModel, PositionSetpoint
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP


def episode_errors(model, episode):
    truth = episode.states[1:]
    step_predictions = np.array([
        state_vector(model.predict(episode.sample(i), command, dt=episode.dt).state)
        for i, command in enumerate(episode.commands)
    ])
    rollout = model.rollout(episode.sample(0), episode.commands, dt=episode.dt)
    rollout_predictions = np.array([state_vector(step.state) for step in rollout])
    if not np.isfinite(step_predictions).all() or not np.isfinite(rollout_predictions).all():
        raise ValueError(f"nonfinite prediction for {episode.identifier}")
    return {"one_step": error_series(step_predictions, truth),
            "open_loop": error_series(rollout_predictions, truth)}


def score_cohort(episodes, models):
    errors = {name: [episode_errors(model, episode) for episode in episodes] for name, model in models.items()}
    horizons = (0.2, 0.5, 1.0, 2.0)
    summary = {}
    for name, rows in errors.items():
        summary[name] = {}
        for mode in ("one_step", "open_loop"):
            samples = {quantity: np.stack([row[mode][quantity] for row in rows]) for quantity in rows[0][mode]}
            summary[name][mode] = {
                quantity: {"rmse": float(np.sqrt(np.mean(values ** 2))), "max": float(values.max())}
                for quantity, values in samples.items()
            }
            if mode == "open_loop":
                summary[name]["endpoint_rmse_by_horizon"] = [
                    {"horizon_s": horizon, **{
                        quantity: float(np.sqrt(np.mean(values[:, round(horizon / episodes[0].dt) - 1] ** 2)))
                        for quantity, values in samples.items()
                    }} for horizon in horizons if round(horizon / episodes[0].dt) <= len(episodes[0].commands)
                ]
    baseline, learned = (np.array([np.mean(row["open_loop"]["position_m"] ** 2) for row in errors[name]])
                         for name in ("d0", "d1"))
    comparison = {"episodes_with_lower_d1_position_rmse": int(np.sum(learned < baseline))}
    if len(episodes) > 1:
        rng = np.random.default_rng(SEED)
        selection = rng.integers(0, len(episodes), size=(2000, len(episodes)))
        deltas = np.sqrt(learned[selection].mean(axis=1)) - np.sqrt(baseline[selection].mean(axis=1))
        comparison["position_rmse_delta_d1_minus_d0_bootstrap_95pct_m"] = np.quantile(deltas, [0.025, 0.975]).tolist()
    return {"episodes": len(episodes), "steps_per_episode": len(episodes[0].commands), "dt_s": episodes[0].dt,
            "finite_episodes_per_model": len(episodes), "models": summary, "comparison": comparison,
            "settings": [{"episode": episode.identifier, "response": episode.response,
                          "response_id": episode.response_id()} for episode in episodes]}


def nominal_episode(equilibrium=False):
    segments = ((100, INITIAL.position_m, INITIAL.yaw_rad),) if equilibrium else (
        (20, INITIAL.position_m, INITIAL.yaw_rad), (60, (3.2, -1.8, -1.7), -2.5),
        (100, (-1.4, 0.6, -0.7), -0.7))
    commands = schedule(*segments)
    response = SyntheticResponse()
    trace = simulate(state_vector(INITIAL), [(c.position_m, c.yaw_rad) for c in commands], dt=0.02, response=response)
    return Episode("equilibrium" if equilibrium else "matched_nominal", 0.02, INITIAL.reference,
                   trace, tuple(commands), {"fixture": "D0 nominal response"})


def benchmark(models, repeats):
    command = PositionSetpoint(INITIAL.reference, 0, 5, (1, -1, -1.4), -2.5)
    alternatives = tuple(PositionSetpoint(INITIAL.reference, 0, 5, (0.4 + 0.4 * i, -1, -1.4), -2.5) for i in range(8))
    return {name: {
        "single_prediction": time_calls(lambda: model.predict(INITIAL, command, dt=0.02), repeats * 10),
        "one_second_rollout": time_calls(lambda: model.rollout(INITIAL, command, dt=0.02, steps=50), repeats),
        "eight_one_second_rollouts": time_calls(
            lambda: tuple(model.rollout(INITIAL, item, dt=0.02, steps=50) for item in alternatives), repeats),
    } for name, model in models.items()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("artifacts/adaptive_dynamics_d1"))
    parser.add_argument("--output", type=Path)
    parser.add_argument("--repeats", type=int, default=100)
    args = parser.parse_args()
    if not 10 <= args.repeats <= 2000:
        parser.error("--repeats must be between 10 and 2000")
    checkpoint = args.input / "residual_64x64.npz"
    network = ResidualMLP.load(checkpoint)
    digest = hashlib.sha256(checkpoint.read_bytes()).hexdigest()
    training = json.loads((args.input / "training.json").read_text(encoding="utf-8"))
    if digest != training["checkpoint_sha256"] or training["seed"] != SEED:
        raise ValueError("checkpoint and training report must describe the same experiment")
    models = {"d0": DynamicsModel(), "d1": ResidualDynamicsModel(network=network)}
    seen = set(training["response_ids"]["train"] + training["response_ids"]["validation"])
    cohorts = {split: generate(split) for split in ("test", "shifted")}
    for episodes in cohorts.values():
        for episode in episodes:
            identifier = episode.response_id()
            if identifier in seen:
                raise ValueError("held-out response setting overlaps another episode")
            seen.add(identifier)
    cohorts.update(matched_nominal=(nominal_episode(),), equilibrium=(nominal_episode(equilibrium=True),))
    scores = {name: score_cohort(episodes, models) for name, episodes in cohorts.items()}
    report = {
        "seed": SEED, "checkpoint_sha256": digest, "checkpoint_bytes": checkpoint.stat().st_size,
        "parameter_count": training["parameter_count"], "selected_epoch": training["selected_epoch"],
        "cohorts": scores,
        "host": {"platform": platform.platform(), "processor": platform.processor(),
                 "machine": platform.machine(), "python": platform.python_version(), "numpy": np.__version__,
                 "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS")},
        "host_benchmark": benchmark(models, args.repeats),
    }
    output = args.output or args.input
    output.mkdir(parents=True, exist_ok=True)
    (output / "evaluation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for name, cohort in scores.items():
        d0, d1 = (cohort["models"][model]["open_loop"]["position_m"]["rmse"] for model in models)
        print(f"{name:16} position rollout RMSE D0 {d0:.4f} m | D1 {d1:.4f} m")
    print(f"Wrote {output / 'evaluation.json'}")


if __name__ == "__main__":
    main()
