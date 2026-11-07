"""Compare all shipped dynamics predictors in offline, causal replay."""

import argparse
from collections import Counter
import csv
from dataclasses import dataclass, replace
from hashlib import sha256
import json
import math
import os
from pathlib import Path
import platform
from time import perf_counter_ns

import numpy as np

from examples.a1_data import (
    CALIBRATION_COUNT, CALIBRATION_SEED, FAULT_END, FAULT_STEP, NOISE_STD,
    STEPS, TEST_COUNT, TEST_SEED, fixtures, verify_splits,
)
from examples.a1_metrics import (
    HORIZONS, ONE_STEP_METRICS, ROLLOUT_LIMITS, calibrate, decision, summarize,
)
from examples.adaptive_dynamics_d0_demo import error_series, state_vector, timing_summary
from refnav.dynamics import DynamicsModel, PositionSetpoint
from refnav.dynamics.adaptive import ENCODERS, AdaptiveDynamicsModel, AdaptiveNetwork, OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory, STEP_SECONDS
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP

ADAPTIVE = ("online_fit", *ENCODERS)
D2_BASE_SHA = "ed029018d08ae91c4fb173c6b50f5b076e048bec"


@dataclass
class Forecast:
    episode_id: str
    model: str
    anchor: str
    anchor_s: float
    status: str
    errors: dict


@dataclass
class EpisodeResult:
    forecasts: list
    one_step: dict
    sample_indices: np.ndarray
    change_step: int | None
    statuses: Counter
    reset_events: Counter


def load_models(artifacts):
    """Verify the NPZ hashes before loading the existing inference objects."""
    d1 = artifacts / "adaptive_dynamics_d1"
    d2 = artifacts / "adaptive_dynamics_d2"
    d1_training = json.loads((d1 / "training.json").read_text(encoding="utf-8"))
    d2_training = json.loads((d2 / "training.json").read_text(encoding="utf-8"))
    hashes = {}

    def verified(directory, filename, name, expected):
        path = directory / filename
        digest = sha256(path.read_bytes()).hexdigest()
        if digest != expected:
            raise ValueError(f"checkpoint hash differs from training manifest: {path}")
        hashes[name] = digest
        return path

    d1_path = verified(d1, "residual_64x64.npz", "d1", d1_training["checkpoint_sha256"])
    models = {"d0": DynamicsModel(), "d1": ResidualDynamicsModel(network=ResidualMLP.load(d1_path)),
              "online_fit": OnlineFitDynamicsModel()}
    for name in ENCODERS:
        expected = d2_training["candidates"][name]
        path = verified(d2, f"adaptive_{name}.npz", name, expected["sha256"])
        network = AdaptiveNetwork.load(path)
        if network.encoder != name:
            raise ValueError(f"checkpoint encoder does not match the model name: {path}")
        models[name] = AdaptiveDynamicsModel(network)
    return models, hashes, d1_training, d2_training


def _rebind(command, reference):
    return command if command.reference == reference else replace(command, reference=reference)


def _observe(history, before, command, after):
    """Apply the public history contract, preserving the observed reset behavior."""
    if before.reference == after.reference:
        history.observe(before, _rebind(command, before.reference), after)
        return False
    try:
        history.observe(before, _rebind(command, before.reference), after)
    except ValueError as error:
        if str(error) != "a transition cannot cross a reference reset":
            raise
        return True
    raise RuntimeError("history accepted a transition across a reference reset")


def score_replay(replay, models):
    """Feed only measured transitions ending at or before the current anchor."""
    episode = replay.episode
    measurements = replay.measurements()
    history = MotionHistory()
    last_accepted = None
    indices = []
    one_step = {name: [] for name in models}
    forecasts, statuses, resets = [], Counter(), Counter()
    anchors = {i: label for label, i in replay.anchors.items()}
    max_steps = round(max(HORIZONS) / episode.dt)
    min_steps = round(min(HORIZONS) / episode.dt)

    for index, command in enumerate(episode.commands):
        state = measurements[index]
        if state is None:
            continue
        before = measurements[index - 1] if index else None
        if (before is not None and replay.fault != "missing_history"
                and not (replay.fault == "stale_history" and FAULT_STEP <= index < FAULT_END)):
            if _observe(history, before, episode.commands[index - 1], state):
                resets["cross_epoch_rejected"] += 1
                last_accepted = None
            else:
                last_accepted = state.timestamp_s
        if replay.fault == "late_epoch_transition" and index == FAULT_END:
            if history.window(state).status != "ready":
                raise RuntimeError("late reset probe needs a ready current history")
            if not _observe(history, measurements[FAULT_STEP - 1], episode.commands[FAULT_STEP - 1],
                            measurements[FAULT_STEP]):
                raise RuntimeError("late reset probe did not cross an epoch")
            resets["late_cross_epoch_rejected"] += 1
            if history.window(state).status != "ready":
                resets["late_cross_epoch_cleared_ready_history"] += 1
                last_accepted = None

        # The interval ending at this anchor can enter history. The next interval cannot.
        if last_accepted is not None and last_accepted > state.timestamp_s:
            raise RuntimeError("future observation entered forecast history")
        window = history.window(state)
        status = window.status
        if status == "ready" and last_accepted != state.timestamp_s:
            raise RuntimeError("ready history does not end at its measured anchor")
        statuses[status] += 1
        indices.append(index)
        current = _rebind(command, state.reference)
        end = min(index + max_steps, len(episode.commands))
        anchor = anchors.get(index)
        # Reference resets are metadata-only in these fixtures. Keep the anchor reference
        # through the forecast and give every model the same known future commands.
        schedule = ()
        if anchor is not None and end - index >= min_steps:
            schedule = tuple(_rebind(c, state.reference) for c in episode.commands[index:end])
        nominal_step = nominal_rollout = None
        for name, model in models.items():
            kwargs = {"history": history} if name in ADAPTIVE else {}
            prediction = model.predict(state, current, dt=episode.dt, **kwargs)
            if name == "d0":
                nominal_step = prediction
            elif name in ADAPTIVE and status != "ready" and prediction != nominal_step:
                raise RuntimeError(f"{name} one-step fallback differs from D0")
            one_step[name].append((*prediction.state.velocity_mps, prediction.state.yaw_rate_rad_s,
                                   *prediction.acceleration_mps2, prediction.yaw_acceleration_rad_s2))
            if schedule:
                trajectory = model.rollout(state, schedule, dt=episode.dt, **kwargs)
                if name == "d0":
                    nominal_rollout = trajectory
                elif name in ADAPTIVE and status != "ready" and trajectory != nominal_rollout:
                    raise RuntimeError(f"{name} rollout fallback differs from D0")
                vectors = np.asarray([state_vector(step.state) for step in trajectory])
                truth = episode.states[index + 1:end + 1]
                errors = error_series(vectors, truth)
                forecasts.append(Forecast(replay.source_id, name, anchor, state.timestamp_s,
                                          status if name in ADAPTIVE else "stateless",
                                          {metric: errors[metric] for metric in ROLLOUT_LIMITS}))
        if history.window(state) != window:
            raise RuntimeError("a prediction mutated measured history")

    indices = np.asarray(indices, dtype=int)
    rates = episode.states[:, [3, 4, 5, 7]]
    truth_rates = rates[indices + 1]
    truth_accel = (truth_rates - rates[indices]) / episode.dt
    errors = {}
    for name, rows in one_step.items():
        predicted = np.asarray(rows)
        velocity = predicted[:, :4] - truth_rates
        acceleration = predicted[:, 4:] - truth_accel
        errors[name] = dict(zip(ONE_STEP_METRICS, (
            np.linalg.norm(velocity[:, :3], axis=1), np.linalg.norm(acceleration[:, :3], axis=1),
            np.abs(velocity[:, 3]), np.abs(acceleration[:, 3]),
        )))
    return EpisodeResult(forecasts, errors, indices, episode.change_step, statuses, resets)


def score_cohort(replays, models):
    return [score_replay(replay, models) for replay in replays]


def check_legacy(results, published):
    """Guard the D2 comparison against metric, checkpoint and fixture drift."""
    comparisons = {}
    steps = round(1.0 / STEP_SECONDS)
    for split in ("test", "shifted"):
        rows = [row for result in results[f"legacy_d2_{split}"] for row in result.forecasts]
        comparisons[split] = {}
        for name in published["cohorts"][split]["models"]:
            if name == "frozen_at_change":
                continue
            actual = np.concatenate([row.errors["position_m"][:steps] for row in rows if row.model == name])
            measured = float(np.sqrt(np.mean(actual ** 2)))
            legacy = published["cohorts"][split]["models"][name]
            expected = legacy["one_second_rollouts"]["after_0.4s"]["pooled"]["position_m"]["rmse"]
            if not math.isclose(measured, expected, abs_tol=1e-7, rel_tol=0):
                raise ValueError(f"D2 regression for {split}/{name}: {measured} vs {expected}")
            comparisons[split][name] = {"a1_rmse_m": measured, "published_d2_rmse_m": expected}
    return comparisons


def benchmark(models, replay, repeats):
    """Time six public APIs on the same warm state and eight distinct held candidates."""
    episode = replay.episode
    anchor = FAULT_STEP
    state = episode.sample(anchor)
    history = MotionHistory()
    for i in range(anchor - 20, anchor):
        history.observe(episode.sample(i), episode.commands[i], episode.sample(i + 1))
    if history.window(state).status != "ready":
        raise RuntimeError("runtime comparison requires ready history")
    steps = round(1.0 / STEP_SECONDS)
    command = episode.commands[anchor]
    schedule = episode.commands[anchor:anchor + steps]
    candidates = tuple(
        PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + 1.0,
                         (command.position_m[0] + (i - 3.5) * 0.15, *command.position_m[1:]),
                         command.yaw_rad + (i - 3.5) * 0.05)
        for i in range(8)
    )

    def measure(run, n, budget_ms):
        for _ in range(5):
            run()
        samples = []
        for _ in range(n):
            start = perf_counter_ns()
            run()
            samples.append(perf_counter_ns() - start)
        return {**timing_summary(samples), "p99_ms": sorted(samples)[math.ceil(n * 0.99) - 1] / 1e6,
                "samples": n, "budget_ms": budget_ms,
                "over_budget": sum(sample > budget_ms * 1e6 for sample in samples)}

    times = {}
    for name, model in models.items():
        kwargs = {"history": history} if name in ADAPTIVE else {}
        times[name] = {
            "one_prediction": measure(lambda: model.predict(state, command, dt=STEP_SECONDS, **kwargs),
                                      repeats * 10, 2.0),
            "one_second_rollout": measure(lambda: model.rollout(state, schedule, dt=STEP_SECONDS, **kwargs),
                                          repeats, 10.0),
            "eight_one_second_rollouts": measure(
                lambda: tuple(model.rollout(state, c, dt=STEP_SECONDS, steps=steps, **kwargs) for c in candidates),
                repeats, 10.0),
        }
    return times


def write_rollouts(path, results):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("regime", "episode_id", "model", "anchor", "anchor_s", "history_status", "horizon_s",
                         "endpoint_position_m", "endpoint_heading_rad", "pooled_position_rmse_m",
                         "pooled_heading_rmse_rad"))
        for regime, episodes in results.items():
            for result in episodes:
                for row in result.forecasts:
                    for horizon in HORIZONS:
                        steps = round(horizon / STEP_SECONDS)
                        if len(row.errors["position_m"]) >= steps:
                            series = [row.errors[metric][:steps] for metric in ROLLOUT_LIMITS]
                            writer.writerow((regime, row.episode_id, row.model, row.anchor, row.anchor_s, row.status,
                                             horizon, *(v[-1] for v in series),
                                             *(np.sqrt(np.mean(v ** 2)) for v in series)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifacts", type=Path, default=Path("artifacts"))
    parser.add_argument("--output", type=Path, default=Path("artifacts/adaptive_dynamics_a1"))
    parser.add_argument("--repeats", type=int, default=100)
    parser.add_argument("--skip-timing", action="store_true", help="generate repeatable accuracy artifacts only")
    parser.add_argument("--plot", action="store_true", help="render an SVG summary using optional matplotlib")
    args = parser.parse_args(argv)
    if not 10 <= args.repeats <= 2000:
        parser.error("--repeats must be between 10 and 2000")

    models, hashes, d1_training, d2_training = load_models(args.artifacts)
    cohorts, calibrators = fixtures()
    manifest = verify_splits(cohorts, calibrators, d1_training, d2_training)
    published_path = args.artifacts / "adaptive_dynamics_d2" / "evaluation.json"
    published = json.loads(published_path.read_text(encoding="utf-8"))
    for split in ("test", "shifted"):
        responses = [item["response"] for item in manifest["test"][f"legacy_d2_{split}"]]
        if responses != published["cohorts"][split]["response_ids"]:
            raise ValueError("legacy D2 test fingerprints differ from the published run")
    calibration = calibrate(score_cohort(calibrators, models), models)
    results = {}
    for name, replays in cohorts.items():
        results[name] = score_cohort(replays, models)
        print(f"Scored {name} ({len(replays)} episodes)", flush=True)
    summary = {name: summarize(rows, models, calibration) for name, rows in results.items()}
    report = {
        "schema_version": 1, "d2_base_sha": D2_BASE_SHA, "checkpoints_sha256": hashes,
        "fixtures": {"test_base_seed": TEST_SEED, "calibration_seed": CALIBRATION_SEED,
                     "test_count": TEST_COUNT, "calibration_count": CALIBRATION_COUNT,
                     "long_steps": STEPS, "dt_s": STEP_SECONDS, "noise_std": NOISE_STD,
                     "fault_begin_step": FAULT_STEP, "fault_end_step": FAULT_END},
        "splits": manifest, "calibration": calibration,
        "legacy_position_check": check_legacy(results, published),
        "regimes": summary, "selection": decision(results, summary, models),
    }
    if not args.skip_timing:
        report["host"] = {"platform": platform.platform(), "processor": platform.processor(),
                          "python": platform.python_version(), "numpy": np.__version__,
                          "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS"),
                          "omp_num_threads": os.environ.get("OMP_NUM_THREADS")}
        report["timing"] = benchmark(models, cohorts["switched"][0], args.repeats)

    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "evaluation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                                                 encoding="utf-8")
    write_rollouts(args.output / "rollouts.csv", results)
    if args.plot:
        from examples.a1_metrics import plot_summary
        plot_summary(args.output / "rollout_summary.svg", report)
    print(f"Wrote A1 evidence to {args.output}", flush=True)


if __name__ == "__main__":
    main()
