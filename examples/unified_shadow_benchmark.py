
"""Generate deterministic shadow replay evidence and profile host-only prediction."""

import argparse
from collections import Counter
import json
from dataclasses import asdict, replace
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from refnav.dynamics import DroneState, DynamicsModel, LocalNED, NominalParameters, PositionSetpoint
from refnav.dynamics.adaptive import AdaptiveDynamicsModel, AdaptiveNetwork
from refnav.shadow_replay import STEP_NS, ShadowReplay, StreamContract

CONTRACT = StreamContract("local_ned", "synthetic", "fixture-estimator", "fixture-effective",
                          "fixture-common-clock", 0)


def events(*, steps=320, faults=False):
    ref = LocalNED(CONTRACT.frame_id, 0, CONTRACT.clock_id)
    state = DroneState(ref, 0., (0., 0., 0.), (0., 0., 0.), 0., 0.)
    truth = DynamicsModel(replace(NominalParameters(), velocity_tau_s=.53, yaw_tau_s=.24))
    sequence = 0

    def event(kind, tick, **values):
        nonlocal sequence
        sequence += 1
        return dict(kind=kind, source=CONTRACT.state_source if kind == "state" else CONTRACT.command_source,
                    mapping_id=CONTRACT.mapping_id, frame_id=ref.frame_id, clock_id=ref.clock_id, axes="NED",
                    sequence=sequence, received_ns=tick * STEP_NS + 5_000_000,
                    epoch=int(faults and tick >= 220), **values)

    for tick in range(steps + 1):
        if tick:
            target = (1.4, -.25, -.15) if (tick - 1) // 80 % 2 == 0 else (-.3, .35, 0.)
            cmd = PositionSetpoint(ref, (tick - 1) / 50, tick / 50, target, .4)
            state = replace(truth.rollout(state, cmd, dt=.02, steps=1)[-1].state, timestamp_s=tick / 50)
            applied = event("effective", tick, acceptance="verified_effective", position_m=target, yaw_rad=.4,
                            effective_from_ns=(tick - 1) * STEP_NS, effective_until_ns=tick * STEP_NS)
            if not faults or tick != 80:
                yield applied
        msg = event("state", tick, healthy=not (faults and 141 <= tick <= 143), twist_frame="world",
                    yaw_rate_kind="heading_derivative", capture_ns=tick * STEP_NS, position_m=state.position_m,
                    velocity_mps=state.velocity_mps, yaw_rad=state.yaw_rad, yaw_rate_rad_s=state.yaw_rate_rad_s)
        yield msg
        if faults and tick == 115:
            yield msg


def summarize(values):
    return {key: float(np.percentile(values, percentile))
            for key, percentile in (("p50", 50), ("p95", 95), ("p99", 99), ("worst", 100))}


def exercise(learned=None):
    results = {}
    for faults in (False, True):
        replay = ShadowReplay(CONTRACT, learned=learned, monotonic=lambda: 0.)
        statuses, comparisons, errors = Counter(), Counter(), {}
        for event in events(faults=faults):
            row = replay.feed(event)
            statuses[row["status"]] += 1
            comparison = row.get("comparison")
            if isinstance(comparison, dict):
                comparisons["matched_verified_hold"] += 1
                for model, entry in comparison.items():
                    errors.setdefault(model, []).append(entry)
            elif comparison:
                comparisons[comparison] += 1
        results["fault_injection" if faults else "clean"] = {
            "status_counts": dict(statuses), "comparison_counts": dict(comparisons),
            "endpoint_rmse": {model: {key: float(np.sqrt(np.mean([entry[key] ** 2 for entry in entries])))
                                      for key in entries[0]} for model, entries in errors.items()},
        }
    replay = ShadowReplay(CONTRACT, learned=learned)
    for event in events(steps=30):
        replay.feed(event)
    _, state = replay.before
    durations = {name: [] for name in replay.models}
    for model_id, model in replay.models.items():
        kwargs = {} if model_id == "d0" else {"history": replay.history}
        held = [PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + 1.,
                                 (1. + i * .10, .25, -.15), .4) for i in range(8)]
        for rep in range(105):
            start = perf_counter()
            for command in held:
                model.rollout(state, command, dt=.02, steps=50, **kwargs)
            if rep >= 5:
                durations[model_id].append((perf_counter() - start) * 1000)
    return {"scope": "synthetic_host_only", "host": platform.platform(), "processor": platform.processor(),
            "steps_per_candidate": 50, "batch_size": 8, "replay": results, "timing_ms": {
                model: {**summarize(values), "over_25ms": sum(x > 25 for x in values),
                        "over_50ms": sum(x > 50 for x in values), "batches": len(values)}
                for model, values in durations.items()}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fixture-directory", type=Path)
    parser.add_argument('--checkpoint', type=Path)
    args = parser.parse_args(argv)
    if args.fixture_directory:
        args.fixture_directory.mkdir()
        (args.fixture_directory / "manifest.json").write_text(json.dumps({"stream": asdict(CONTRACT)}, indent=2),
                                                               encoding="utf-8")
        (args.fixture_directory / "events.jsonl").write_text(
            "".join(json.dumps(event) + "\n" for event in events()), encoding="utf-8")
    learned = AdaptiveDynamicsModel(AdaptiveNetwork.load(args.checkpoint)) if args.checkpoint else None
    report = exercise(learned)
    with args.output.open('x', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")


if __name__ == "__main__":
    main()
