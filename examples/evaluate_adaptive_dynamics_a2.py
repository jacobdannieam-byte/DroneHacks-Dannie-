"""Compare the frozen A2 experiment with the complete A1 suite and a new holdout."""

import argparse
import gzip
from hashlib import sha256
import json
import os
from pathlib import Path
import platform

import numpy as np

from examples.a1_data import FAULT_STEP, TEST_SEED, digest, fixtures, verify_splits
from examples.a1_metrics import MATERIAL_GAIN, ROLLOUT_LIMITS, _nominal_gate, _paired_delta, calibrate, decision, summarize
from examples.evaluate_adaptive_dynamics_a1 import benchmark, load_models, score_cohort, write_rollouts
from refnav.dynamics.adaptive_features import AdaptiveFeatureDynamics, FeatureBasis
from refnav.dynamics.history import MotionHistory

HOLDOUT_SEED = 202611470
HOLDOUT = ("in_range", "shifted", "switched", "delayed", "noisy", "matched_nominal")
CANDIDATE = "adaptive_features"
ABLATION = "gated_linear"
ANCHOR = "after_0.4s"


def verify_holdout(a1, holdout):
    """Allow the same nominal physics control. Require different measured traces."""
    earlier = a1["test"]
    unseen = holdout["test"]
    traces = {row["trace"] for rows in earlier.values() for row in rows}
    if any(row["trace"] in traces for rows in unseen.values() for row in rows):
        raise ValueError("A2 holdout reuses an A1 trajectory")
    responses = {regime for name, rows in earlier.items() if "nominal" not in name and name != "equilibrium"
                 for row in rows for regime in row["regimes"]}
    if any(regime in responses for name, rows in unseen.items() if name != "matched_nominal"
           for row in rows for regime in row["regimes"]):
        raise ValueError("A2 holdout reuses an A1 response regime")


def load_candidate(directory, d2_manifest):
    record = json.loads((directory / "training.json").read_text(encoding="utf-8"))
    checkpoint = directory / "feature_basis.npz"
    checkpoint_hash = sha256(checkpoint.read_bytes()).hexdigest()
    if (checkpoint_hash != record["checkpoint_sha256"]
            or digest(d2_manifest) != record["d2_training_manifest_sha256"]):
        raise ValueError("A2 checkpoint or D2 training manifest hash mismatch")
    for split in ("train", "validation"):
        if [item["response"] for item in record["split_fingerprints"][split]] != d2_manifest["response_ids"][split]:
            raise ValueError("A2 training uses an unverified split")
    return AdaptiveFeatureDynamics(FeatureBasis.load(checkpoint)), checkpoint_hash


def score(replays, models, calibration):
    results = {}
    for name, episodes in replays.items():
        results[name] = score_cohort(episodes, models)
        print(f"Scored {name} ({len(episodes)} episodes)", flush=True)
    return results, {name: summarize(items, models, calibration) for name, items in results.items()}


def verify_a1_baselines(published, manifest, summary):
    if manifest != published["splits"]:
        raise ValueError("A1 fixture fingerprint drift")
    for cohort, entry in published["regimes"].items():
        for model, metrics in entry["models"].items():
            if metrics != summary[cohort]["models"][model]:
                raise ValueError(f"A1 accuracy changed for {cohort}/{model}")


def _delay_gate(summary, name):
    models = summary["delayed"]["models"]
    def at(model):
        return models[model]["by_anchor"][ANCHOR]["1.0"]["pooled"]
    return all(at(name)[metric]["rmse"] <= at("d0")[metric]["rmse"] + 1e-12 for metric in ROLLOUT_LIMITS)


def selection(a1_decision, primary, holdout_results, holdout, timing):
    pairs = {regime: _paired_delta(holdout_results[regime], CANDIDATE, ANCHOR)
             for regime in ("shifted", "switched")}
    confidence = [entry for regime in ("shifted", "switched", "delayed", "noisy")
                  for entry in holdout[regime]["models"][CANDIDATE]["rollouts"]["1.0"]["confidence"].values()]
    nominal = _nominal_gate(holdout, CANDIDATE, regimes=("matched_nominal",))
    active = timing.get("active_eight_candidate_p95_ms") if timing is not None else None
    gates = {
        "a1_accuracy": a1_decision["learned_vs_ridge"][CANDIDATE]["clears_accuracy"],
        "holdout_accuracy": all(item["relative_improvement"] >= MATERIAL_GAIN and item["paired_delta_95pct_m"][1] < 0
                                for item in pairs.values()),
        "nominal": a1_decision["nominal_gate"][CANDIDATE]["passed"] and nominal["passed"],
        "fixed_delay": _delay_gate(primary, CANDIDATE) and _delay_gate(holdout, CANDIDATE),
        "holdout_confidence": all(item["episode_coverage"] >= .80 and item["false_low_risk_count"] == 0
                                  for item in confidence),
        "host_active_batch_p95": active is not None and active <= 10.,
    }
    return {"gates": gates, "passes_all_offline_screens": all(gates.values()), "default": "d0",
            "adaptive_comparison_baseline": "online_fit", "holdout_pairs_vs_ridge": pairs,
            "holdout_nominal_gate": nominal, "host_batch_limit_ms": 10.,
            "confidence_minimum_episode_coverage": .80, "orin_qualification": "unmeasured"}


def warm_context(model, replay, anchor=FAULT_STEP):
    episode = replay.episode
    history = MotionHistory()
    for i in range(anchor - 20, anchor):
        history.observe(episode.sample(i), episode.commands[i], episode.sample(i + 1))
    return np.asarray(model.adaptation(episode.sample(anchor), history).context).reshape(4, 4)


def activity(models, cohorts):
    """Count enabled axes on clean measured histories at A1's common anchors."""
    return {
        model: {regime: {
            label: [int(np.count_nonzero(np.any(warm_context(models[model], replay, anchor) != 0., axis=1)))
                    for replay in cohorts[regime]]
            for label, anchor in (("at_switch", FAULT_STEP - 20), (ANCHOR, FAULT_STEP),
                                  ("after_0.7s", FAULT_STEP + 15))}
            for regime in ("switched", "shifted", "delayed", "matched_nominal")}
        for model in (CANDIDATE, ABLATION)
    }


def timing_report(models, cohorts, repeats):
    first = cohorts["switched"][0]
    warm = benchmark(models, first, repeats)
    active = next((replay for replay in (*cohorts["switched"], *cohorts["shifted"])
                   if np.any(warm_context(models[CANDIDATE], replay))), None)
    active_times = None
    if active is not None:
        active_times = warm[CANDIDATE] if active is first else benchmark(
            {CANDIDATE: models[CANDIDATE]}, active, repeats)[CANDIDATE]
    return {"host": {"platform": platform.platform(), "processor": platform.processor(),
                     "python": platform.python_version(), "numpy": np.__version__,
                     "openblas_num_threads": os.environ.get("OPENBLAS_NUM_THREADS")},
            "a1_fixture": first.source_id, "shared_fixture_results": warm,
            "active_a2_fixture": None if active is None else active.source_id, "active_a2_results": active_times,
            "active_eight_candidate_p95_ms": None if active_times is None else
            active_times["eight_one_second_rollouts"]["p95_ms"]}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--artifacts", type=Path, default=Path("artifacts"))
    parser.add_argument("--checkpoint-dir", type=Path, default=Path("artifacts/unified_phase3"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=100, choices=range(10, 2001), metavar="10..2000")
    parser.add_argument("--skip-timing", action="store_true")
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output directory already exists")

    models, hashes, d1, d2 = load_models(args.artifacts)
    models[CANDIDATE], hashes[CANDIDATE] = load_candidate(args.checkpoint_dir, d2)
    models[ABLATION] = AdaptiveFeatureDynamics(FeatureBasis.linear_only())
    primary, calibration_replays = fixtures()
    extra, _ = fixtures(test_seed=HOLDOUT_SEED, include_legacy=False)
    holdout = {name: extra[name] for name in HOLDOUT}
    manifest = verify_splits(primary, calibration_replays, d1, d2)
    unseen_manifest = verify_splits(holdout, calibration_replays, d1, d2)
    verify_holdout(manifest, unseen_manifest)
    with gzip.open(args.artifacts / "adaptive_dynamics_a1" / "evaluation.json.gz", "rt", encoding="utf-8") as stream:
        published = json.load(stream)

    calibration = calibrate(score_cohort(calibration_replays, models), models)
    a1_raw, a1_summary = score(primary, models, calibration)
    verify_a1_baselines(published, manifest, a1_summary)
    hold_raw, hold_summary = score(holdout, models, calibration)
    comparison = decision(a1_raw, a1_summary, models)
    timing = None if args.skip_timing else timing_report(models, primary, args.repeats)
    report = {"schema_version": 1, "a1_seed": TEST_SEED, "holdout_seed": HOLDOUT_SEED,
              "checkpoint_hashes": hashes, "a1_baselines_match": True,
              "a1_splits": manifest, "holdout_splits": unseen_manifest,
              "calibration": calibration, "a1_regimes": a1_summary, "holdout_regimes": hold_summary,
              "a1_selection": comparison, "activity": activity(models, primary),
              "selection": selection(comparison, a1_summary, hold_raw, hold_summary, timing)}
    if timing is not None:
        report["timing"] = timing
    args.output.mkdir(parents=True)
    (args.output / "evaluation.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                                                 encoding="utf-8")
    write_rollouts(args.output / "a1_rollouts.csv", a1_raw)
    write_rollouts(args.output / "holdout_rollouts.csv", hold_raw)
    print(f"Wrote A2 evidence to {args.output}", flush=True)


if __name__ == "__main__":
    main()
