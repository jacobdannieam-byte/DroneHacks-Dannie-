"""Deterministic metrics and selection rules for A1 offline replay."""

import math

import numpy as np

from refnav.dynamics.history import STEP_SECONDS

HORIZONS = (0.2, 0.5, 1.0, 2.0)
ONE_STEP_METRICS = ("velocity_mps", "acceleration_mps2", "yaw_rate_rad_s", "yaw_acceleration_rad_s2")
ROLLOUT_LIMITS = {"position_m": 0.5, "heading_rad": math.radians(30)}
COVERAGE = 0.90
NOMINAL_REGRESSION = 0.10
MATERIAL_GAIN = 0.20
BOOTSTRAP_SEED = 202611172


def stats(values, limit=None):
    if not np.isfinite(values).all():
        raise ValueError("nonfinite errors in evaluation")
    summary = {
        "n": len(values), "rmse": float(np.sqrt(np.mean(values ** 2))),
        "mean": float(np.mean(values)), "median": float(np.median(values)),
        "p95": float(np.quantile(values, 0.95)), "worst": float(np.max(values)),
    }
    if limit is not None:
        summary["catastrophic_count"] = int(np.count_nonzero(values > limit))
    return summary


def _for_horizon(rows, horizon):
    steps = round(horizon / STEP_SECONDS)
    return steps, [row for row in rows if len(row.errors["position_m"]) >= steps]


def _episode_maxima(rows, values):
    maxima = {}
    for row, value in zip(rows, values):
        maxima[row.episode_id] = max(maxima.get(row.episode_id, 0.0), value)
    return np.asarray(list(maxima.values()))


def calibrate(results, models):
    """Freeze empirical upper bounds using independent calibration episode maxima."""
    forecasts = [row for result in results for row in result.forecasts]
    bounds = {}
    for model in models:
        bounds[model] = {}
        for horizon in HORIZONS:
            steps, rows = _for_horizon([row for row in forecasts if row.model == model], horizon)
            n = len({row.episode_id for row in rows})
            rank = math.ceil((n + 1) * COVERAGE)
            if rank > n:
                raise ValueError("too few independent calibration episodes for finite bound")
            bounds[model][str(horizon)] = {
                metric: float(np.sort(_episode_maxima(rows, [row.errors[metric][steps - 1] for row in rows]))[rank - 1])
                for metric in ROLLOUT_LIMITS
            }
    return {"target_episode_coverage": COVERAGE, "episode_count": len(results), "bounds": bounds}


def _coverage(rows, values, bound, limit):
    maxima = _episode_maxima(rows, values)
    declares_low_risk = bound <= limit
    return {"bound": bound, "endpoint_coverage": float(np.mean(values <= bound)),
            "episode_coverage": float(np.mean(maxima <= bound)), "episodes": len(maxima),
            "low_risk_count": len(values) if declares_low_risk else 0,
            "false_low_risk_count": int(np.count_nonzero(values > limit)) if declares_low_risk else 0}


def _rollout_summary(rows, bounds):
    output = {}
    for horizon in HORIZONS:
        steps, available = _for_horizon(rows, horizon)
        if not available:
            continue
        key = str(horizon)
        details = {"forecasts": len(available), "episodes": len({row.episode_id for row in available}),
                   "endpoint": {}, "pooled": {}, "confidence": {}}
        for metric, limit in ROLLOUT_LIMITS.items():
            samples = np.stack([row.errors[metric][:steps] for row in available])
            endpoints = samples[:, -1]
            details["endpoint"][metric] = stats(endpoints, limit)
            details["pooled"][metric] = stats(samples.ravel(), limit)
            if bounds is not None:
                details["confidence"][metric] = _coverage(available, endpoints, bounds[key][metric], limit)
        output[key] = details
    return output


def _switch_curve(results, model):
    switched = [result for result in results if result.change_step is not None]
    if not switched:
        return []
    intervals = (0.0, 0.1, 0.2, 0.4, 0.7, 1.0, 1.5, 2.0)
    curve = []
    for start, end in zip(intervals, intervals[1:]):
        metrics = {}
        for metric in ONE_STEP_METRICS[:2]:
            values = []
            for result in switched:
                offset = result.sample_indices - result.change_step
                valid = (offset >= round(start / STEP_SECONDS)) & (offset < round(end / STEP_SECONDS))
                values.extend(result.one_step[model][metric][valid])
            if values:
                metrics[metric] = stats(np.asarray(values))
        if metrics:
            curve.append({"from_s": start, "until_s": end, "metrics": metrics})
    return curve


def summarize(results, models, calibration):
    forecasts = [row for result in results for row in result.forecasts]
    statuses, resets = {}, {}
    for result in results:
        for name, count in result.statuses.items():
            statuses[name] = statuses.get(name, 0) + count
        for name, count in result.reset_events.items():
            resets[name] = resets.get(name, 0) + count
    output = {"episodes": len(results), "observed_anchor_history_status": statuses, "reset_events": resets,
              "forecast_history_status_by_anchor": {}, "models": {}}
    anchors = list(dict.fromkeys(row.anchor for row in forecasts))
    for anchor in anchors:
        # All history-based predictors see the same window. Count each forecast anchor once.
        statuses = [row.status for row in forecasts if row.anchor == anchor and row.model == "online_fit"]
        output["forecast_history_status_by_anchor"][anchor] = {
            status: statuses.count(status) for status in sorted(set(statuses))
        }
    for model in models:
        rows = [row for row in forecasts if row.model == model]
        output["models"][model] = {
            "one_step": {
                metric: stats(np.concatenate([result.one_step[model][metric] for result in results]))
                for metric in ONE_STEP_METRICS
            },
            "rollouts": _rollout_summary(rows, calibration["bounds"][model]),
            "by_anchor": {anchor: _rollout_summary([row for row in rows if row.anchor == anchor], None)
                          for anchor in anchors},
            "post_switch_one_step": _switch_curve(results, model),
        }
    return output


def _nominal_gate(summary, model, *, regimes=("matched_nominal", "legacy_nominal")):
    failures = []
    for regime in regimes:
        rows = summary[regime]["models"]
        for horizon, baseline in rows["d0"]["rollouts"].items():
            for mode in ("endpoint", "pooled"):
                for metric in ROLLOUT_LIMITS:
                    expected = baseline[mode][metric]["rmse"]
                    measured = rows[model]["rollouts"][horizon][mode][metric]["rmse"]
                    if measured > expected * (1 + NOMINAL_REGRESSION) + 1e-12:
                        failures.append({"regime": regime, "horizon_s": float(horizon), "mode": mode,
                                         "metric": metric, "model_rmse": measured, "d0_rmse": expected})
    return {"passed": not failures, "regressions": failures}


def _paired_delta(rows, model, anchor):
    steps = round(1.0 / STEP_SECONDS)
    paired = {}
    for result in rows:
        for row in result.forecasts:
            if (row.anchor == anchor and row.model in (model, "online_fit")
                    and len(row.errors["position_m"]) >= steps):
                value = np.mean(row.errors["position_m"][:steps] ** 2)
                paired.setdefault(row.episode_id, {})[row.model] = value
    if len(paired) != len(rows) or any(len(pair) != 2 for pair in paired.values()):
        raise ValueError("selection requires a paired one-second rollout per episode")
    values = np.asarray([(pair[model], pair["online_fit"]) for pair in paired.values()])
    averages = np.sqrt(values.mean(axis=0))
    rng = np.random.default_rng(BOOTSTRAP_SEED)
    indices = rng.integers(0, len(values), size=(2000, len(values)))
    resampled = np.sqrt(values[indices].mean(axis=1))
    interval = np.quantile(resampled[:, 0] - resampled[:, 1], [0.025, 0.975])
    return {"episodes": len(values), "model_rmse_m": float(averages[0]), "ridge_rmse_m": float(averages[1]),
            "relative_improvement": float(1 - averages[0] / averages[1]),
            "paired_delta_95pct_m": interval.tolist()}


def decision(results, summary, models):
    """Rank at the common post-switch anchor and preserve the nominal safety gate."""
    anchor = "after_0.4s"
    nominal = {model: _nominal_gate(summary, model) for model in models}
    leaders = {}
    for regime in ("switched", "shifted", "in_range", "delayed", "noisy"):
        leaders[regime] = {}
        for horizon in HORIZONS:
            leaders[regime][str(horizon)] = {}
            for mode in ("pooled", "endpoint"):
                leaders[regime][str(horizon)][mode] = {
                    metric: min(models, key=lambda model: summary[regime]["models"][model]["by_anchor"][anchor]
                                [str(horizon)][mode][metric]["rmse"])
                    for metric in ROLLOUT_LIMITS
                }
    comparisons = {}
    for model in models:
        if model in ("d0", "online_fit"):
            continue
        measures = {regime: _paired_delta(results[regime], model, anchor) for regime in ("switched", "shifted")}
        clears_accuracy = all(m["relative_improvement"] >= MATERIAL_GAIN and m["paired_delta_95pct_m"][1] < 0
                              for m in measures.values())
        comparisons[model] = {"regimes": measures, "clears_accuracy": clears_accuracy,
                              "clears_accuracy_and_nominal": clears_accuracy and nominal[model]["passed"]}
    return {"anchor": anchor, "leaders_by_horizon": leaders, "nominal_relative_limit": NOMINAL_REGRESSION,
            "nominal_gate": nominal, "material_relative_gain": MATERIAL_GAIN, "bootstrap_seed": BOOTSTRAP_SEED,
            "catastrophic_thresholds": ROLLOUT_LIMITS, "learned_vs_ridge": comparisons}


def plot_summary(path, report):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams["svg.hashsalt"] = "adaptive-dynamics-a1"
    fig, axes = plt.subplots(1, 2, figsize=(9, 3.6), layout="constrained")
    labels = ("D0", "D1", "Ridge", "History MLP", "GRU-32", "GRU-64")
    models = report["regimes"]["switched"]["models"]
    for ax, metric, scale, label in zip(axes, ROLLOUT_LIMITS, (100, 180 / math.pi),
                                         ("Position RMSE (cm)", "Heading RMSE (degrees)")):
        for display, model in zip(labels, models):
            rows = models[model]["by_anchor"]["after_0.4s"]
            ax.plot(HORIZONS, [rows[str(h)]["endpoint"][metric]["rmse"] * scale for h in HORIZONS],
                    "o-", label=display)
        ax.set(xlabel="Forecast horizon (s)", ylabel=label, xticks=HORIZONS)
        ax.grid(alpha=0.25)
    axes[1].legend(fontsize=8, ncol=2)
    fig.suptitle("Synthetic dynamics switch, forecasts starting 0.4 s after the switch")
    fig.savefig(path, metadata={"Date": None})
    plt.close(fig)
    # Matplotlib leaves trailing spaces in multiline SVG path coordinates.
    path.write_bytes(b"\n".join(line.rstrip() for line in path.read_bytes().splitlines()) + b"\n")
