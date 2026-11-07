"""Evaluate frozen D2 encoders on causal replay, response switches and host timing."""

import argparse
import hashlib
import json
import os
import platform
from pathlib import Path

import numpy as np

from examples.adaptive_dynamics_d0_demo import error_series, state_vector, time_calls
from examples.d2_data import CHANGE_STEP, SEED, episodes
from examples.evaluate_adaptive_dynamics_d1 import nominal_episode
from refnav.dynamics import DynamicsModel
from refnav.dynamics.adaptive import ENCODERS, AdaptiveDynamicsModel, AdaptiveNetwork, OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory, STEP_SECONDS
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP

HORIZON_STEPS = 50
RECOVERY_THRESHOLD_MPS = 0.025
RECOVERY_WINDOW = 5


def recovery_time(velocity_errors):
    """First end of five consecutive 100 ms RMS windows within the threshold."""
    if len(velocity_errors) < RECOVERY_WINDOW * 2 - 1:
        return None
    windows = np.lib.stride_tricks.sliding_window_view(velocity_errors, RECOVERY_WINDOW)
    settled = np.sqrt(np.mean(windows ** 2, axis=1)) <= RECOVERY_THRESHOLD_MPS
    for index in range(len(settled) - RECOVERY_WINDOW + 1):
        if settled[index:index + RECOVERY_WINDOW].all():
            return (index + 2 * RECOVERY_WINDOW - 1) * STEP_SECONDS
    return None


def _run_case(episode, models, selected):
    history = MotionHistory()
    predictions = {name: [] for name in (*models, "frozen_at_change")}
    rollouts = {name: {} for name in predictions}
    latents = {name: [] for name in ENCODERS}
    frozen = None
    start = episode.change_step if episode.change_step is not None else CHANGE_STEP
    anchors = {"before_switch": start - HORIZON_STEPS, "at_switch": start,
               "after_0.4s": start + 20, "after_0.7s": start + 35}
    if episode.change_step is None:
        anchors = {"control_0.3s": 15}
    anchor_names = {index: name for name, index in anchors.items() if index + HORIZON_STEPS <= len(episode.commands)}
    before = episode.sample(0)
    for index, command in enumerate(episode.commands):
        bound = {}
        for name, model in models.items():
            if name in (*ENCODERS, "online_fit"):
                predictor, adaptation = model._bound(before, history)
                if name in ENCODERS:
                    latents[name].append(adaptation.context or (0.0,) * 8)
            else:
                predictor = model
            bound[name] = predictor
        if episode.change_step == index:
            frozen = bound[selected]
        # Before the switch the ablation follows D2. At the switch it pins the last pre-switch context.
        bound["frozen_at_change"] = frozen if frozen is not None else bound[selected]
        for name, predictor in bound.items():
            prediction = predictor.predict(before, command, dt=episode.dt)
            predictions[name].append(state_vector(prediction.state))
            if index in anchor_names:
                end = index + HORIZON_STEPS
                trajectory = predictor.rollout(before, episode.commands[index:end], dt=episode.dt)
                vectors = np.array([state_vector(step.state) for step in trajectory])
                rollouts[name][anchor_names[index]] = error_series(vectors, episode.states[index + 1:end + 1])
        after = episode.sample(index + 1)
        history.observe(before, command, after)
        before = after
    one_step = {name: error_series(np.asarray(values), episode.states[1:]) for name, values in predictions.items()}
    for errors in (*one_step.values(), *(error for result in rollouts.values() for error in result.values())):
        if not all(np.isfinite(values).all() for values in errors.values()):
            raise ValueError(f"nonfinite error in {episode.identifier}")
    return one_step, rollouts, latents


def _metrics(series):
    return {key: {"rmse": float(np.sqrt(np.mean(np.square(values)))), "worst": float(np.max(values))}
            for key, values in series.items()}


def score(cases, models, selected):
    records = [_run_case(case, models, selected) for case in cases]
    result = {}
    for name in (*models, "frozen_at_change"):
        errors = {key: np.array([record[0][name][key] for record in records]) for key in records[0][0][name]}
        rollout_summary = {}
        for anchor, values in records[0][1][name].items():
            rollouts = {key: np.array([record[1][name][anchor][key] for record in records]) for key in values}
            rollout_summary[anchor] = {"pooled": _metrics(rollouts), "endpoint": {
                str(step * STEP_SECONDS): _metrics({key: value[:, step - 1] for key, value in rollouts.items()})
                for step in (10, 25, 50)
            }}
        entry = {"one_step": _metrics(errors), "one_second_rollouts": rollout_summary}
        if cases[0].change_step is not None:
            start = cases[0].change_step
            recovery = [recovery_time(row[start:]) for row in errors["velocity_mps"]]
            recovered = [value for value in recovery if value is not None]
            entry["post_switch"] = {
                "rmse": _metrics({key: value[:, start:] for key, value in errors.items()}),
                "recovered_episodes": len(recovered),
                "median_recovery_s_among_recovered": float(np.median(recovered)) if recovered else None,
                "velocity_rmse_by_interval": [
                    {"from_s": lo * STEP_SECONDS, "until_s": hi * STEP_SECONDS,
                     "rmse_mps": float(np.sqrt(np.mean(errors["velocity_mps"][:, start + lo:start + hi] ** 2)))}
                    for lo, hi in ((0, 5), (5, 20), (20, 40), (40, 85))
                ],
            }
            if name in ENCODERS:
                latent = np.array([record[2][name] for record in records])
                pre = latent[:, start - 20:start]
                post = latent[:, -20:]
                entry["latent_diagnostics"] = {
                    "median_within_window_step_l2": float(np.median(np.linalg.norm(np.diff(post, axis=1), axis=2))),
                    "median_pre_post_mean_l2": float(np.median(np.linalg.norm(post.mean(axis=1) - pre.mean(axis=1), axis=1))),
                    "maximum_absolute_coordinate": float(np.max(np.abs(latent))),
                }
        result[name] = entry
    return {"episodes": len(cases), "steps_per_episode": len(cases[0].commands), "models": result,
            "response_ids": [case.response_id() for case in cases]}


def benchmark(models, episode, repeats):
    anchor = CHANGE_STEP + 20
    history = MotionHistory()
    for index in range(anchor - 20, anchor):
        history.observe(episode.sample(index), episode.commands[index], episode.sample(index + 1))
    state = episode.sample(anchor)
    command = episode.commands[anchor]
    schedule = episode.commands[anchor:anchor + HORIZON_STEPS]
    results = {}
    for name, model in models.items():
        kwargs = {"history": history} if name in (*ENCODERS, "online_fit") else {}
        results[name] = {
            "one_prediction": time_calls(lambda: model.predict(state, command, dt=STEP_SECONDS, **kwargs), repeats * 10),
            "one_second_rollout": time_calls(lambda: model.rollout(state, schedule, dt=STEP_SECONDS, **kwargs), repeats),
        }
        if kwargs:
            results[name]["context_update"] = time_calls(lambda: model.adaptation(state, history), repeats * 5)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("artifacts/adaptive_dynamics_d2"))
    parser.add_argument("--d1", type=Path, default=Path("artifacts/adaptive_dynamics_d1/residual_64x64.npz"))
    parser.add_argument("--repeats", type=int, default=60)
    args = parser.parse_args()
    if not 10 <= args.repeats <= 1000:
        parser.error("--repeats must be between 10 and 1000")
    training = json.loads((args.input / "training.json").read_text(encoding="utf-8"))
    if training["seed"] != SEED:
        raise ValueError("training report seed differs from the evaluation design")
    models = {"d0": DynamicsModel(), "d1": ResidualDynamicsModel(network=ResidualMLP.load(args.d1)),
              "online_fit": OnlineFitDynamicsModel()}
    for name in ENCODERS:
        checkpoint = args.input / f"adaptive_{name}.npz"
        if hashlib.sha256(checkpoint.read_bytes()).hexdigest() != training["candidates"][name]["sha256"]:
            raise ValueError("checkpoint differs from the training record")
        models[name] = AdaptiveDynamicsModel(AdaptiveNetwork.load(checkpoint))
    test, shifted = episodes("test"), episodes("shifted")
    fingerprints = training["response_ids"]["train"] + training["response_ids"]["validation"]
    fingerprints += [episode.response_id() for episode in (*test, *shifted)]
    if len(set(fingerprints)) != len(fingerprints):
        raise ValueError("response settings overlap across splits")
    selected = training["selected_encoder"]
    cohorts = {}
    for name, cases in (("test", test), ("shifted", shifted), ("matched_nominal", (nominal_episode(),)),
                        ("equilibrium", (nominal_episode(equilibrium=True),))):
        cohorts[name] = score(cases, models, selected)
        print(f"Scored {name} ({len(cases)} episodes)", flush=True)
    report = {
        "seed": SEED, "selected_encoder": selected, "switch_s": CHANGE_STEP * STEP_SECONDS,
        "recovery_threshold_mps": RECOVERY_THRESHOLD_MPS, "recovery_rms_window_steps": RECOVERY_WINDOW,
        "recovery_required_consecutive_windows": RECOVERY_WINDOW,
        "d1_checkpoint_sha256": hashlib.sha256(args.d1.read_bytes()).hexdigest(),
        "cohorts": cohorts,
        "host": {"platform": platform.platform(), "processor": platform.processor(), "machine": platform.machine(),
                 "python": platform.python_version(), "numpy": np.__version__,
                 "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS")},
        "benchmark": benchmark(models, test[0], args.repeats),
    }
    (args.input / "evaluation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Wrote {args.input / 'evaluation.json'}", flush=True)


if __name__ == "__main__":
    main()
