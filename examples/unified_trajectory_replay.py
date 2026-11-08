"""Exercise Phase 4 on Phase 2 synthetic depth and report host-only timing."""

import argparse
from dataclasses import asdict, replace
import json
import math
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from examples.semantic_world_model_demo import percentiles, render, scene
from examples.unified_spatial_replay import BODY, REFERENCE, calibration, observe
from refnav.dynamics import DroneState, LocalNED, PositionSetpoint
from refnav.dynamics.history import STEP_SECONDS
from refnav.reasoning import DeterministicReasoner, WorldReference, build_context
from refnav.trajectory_evaluation import Candidate, FrameAlignment, GoalBinding, VehicleBounds, evaluate_candidates
from refnav.world_model.spatial_pipeline import SpatialWorld

STAMP = 100_000_000
NED = LocalNED("local_ned", REFERENCE.epoch, REFERENCE.clock_id)
VEHICLE = VehicleBounds(.07, .01, .015)


def fixture(*, blocked=False):
    surfaces = scene(blocked=blocked, rear=False) + [(0, 5.4, (-5., 5.), (-1., 4.))]
    world = SpatialWorld(REFERENCE, calibration())
    if not observe(world, render(surfaces), STAMP).accepted:
        raise RuntimeError("Phase 2 fixture rejected")
    snapshot = world.snapshot(now_ns=STAMP)
    # Independent synthetic evaluation anchor within the observed corridor.
    state = DroneState(NED, STAMP / 1e9, (2.1, -.10, -(BODY[2] + .10)), (0., 0., 0.), 0., 0.)
    alignment = FrameAlignment(NED, REFERENCE, STAMP, 0, -math.pi / 2,
                               (0., 0., 0.), .005, .002, "synthetic-frame-calibration")
    return state, snapshot, alignment


def candidate(state, name, position, *, steps=50):
    held = PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s + steps * STEP_SECONDS,
                           position, state.yaw_rad)
    return Candidate(name, (held,) * steps)


def _binding(snapshot):
    reference = WorldReference(REFERENCE.frame_id, REFERENCE.epoch, REFERENCE.clock_id)
    context = build_context(snapshot.semantic, now=snapshot.semantic["query_timestamp"], reference=reference)
    proposal = DeterministicReasoner().propose(context)
    if proposal.intent != "observe_more":
        raise RuntimeError("fixture did not produce a semantic observation hint")
    from refnav.reasoning.inspection import make_inputs
    from refnav.reasoning.selection import create_reasoner
    state, _, alignment = fixture()
    auxiliary = make_inputs(context, snapshot, state, alignment, calibration())
    proposal, selection = create_reasoner().select(context, auxiliary)
    return GoalBinding(context, proposal, selection)


def exercise():
    state, snapshot, alignment = fixture()
    goal = _binding(snapshot)
    candidates = (
        candidate(state, "hold", state.position_m),
        candidate(state, "advance", (2.25, -.10, -1.3)),
        candidate(state, "high_climb", (3.25, -.10, -3.0)),
    )

    def assess(items, world=snapshot, target=goal, bounds=VEHICLE, now=STAMP):
        return evaluate_candidates(state, items, world, now_ns=now,
                                   alignment=alignment, vehicle=bounds, goal=target)

    open_result = assess(candidates)
    blocked = replace(snapshot, metric=fixture(blocked=True)[1].metric)
    cross = candidate(state, "through_wall", (4.8, -.10, -1.3), steps=100)
    wall_result = assess((cross,), blocked)
    stale_result = assess(candidates, now=STAMP + 400_000_000)
    if ("advance" not in open_result.ranked_candidate_ids
            or open_result.ranked_candidate_ids[0] != "advance"
            or wall_result.candidates[0].status != "rejected"
            or wall_result.fallback_reason != "no_verified_candidate"
            or stale_result.ranked_candidate_ids):
        raise RuntimeError("Phase 4 fixture decision changed")
    rows = {"observed_with_goal": asdict(open_result), "wall_with_opening_hint": asdict(wall_result),
            "stale_geometry": asdict(stale_result)}
    batch = tuple(candidate(state, f"option-{i}", (3.15, -.1 + .03 * (i - 4), -1.3)) for i in range(8))
    return rows, lambda: assess(batch)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=100, choices=range(10, 1001), metavar="10..1000")
    parser.add_argument("--skip-timing", action="store_true")
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output directory already exists")
    scenarios, run = exercise()
    args.output.mkdir(parents=True)
    (args.output / "replay.json").write_text(json.dumps(
        {"evidence": "synthetic_static_geometry", "scenarios": scenarios}, indent=2, allow_nan=False) + "\n",
        encoding="utf-8", newline="\n")
    if not args.skip_timing:
        for _ in range(5):
            run()
        times = []
        for _ in range(args.repeats):
            start = perf_counter()
            run()
            times.append((perf_counter() - start) * 1000)
        timing = {"platform": platform.platform(), "processor": platform.processor(),
                  "python": platform.python_version(), "repeats": args.repeats,
                  "candidate_count": 8, "steps_per_candidate": 50,
                  "includes": "D0 prediction, semantic goal validation, swept-volume scoring and ranking",
                  "latency": {**percentiles(times), "p99_ms": float(np.percentile(times, 99))},
                  "over_25ms": sum(t > 25 for t in times)}
        (args.output / "benchmark.json").write_text(json.dumps(timing, indent=2) + "\n",
                                                    encoding="utf-8", newline="\n")
    print(f"Wrote Phase 4 diagnostics to {args.output}")


if __name__ == "__main__":
    main()
