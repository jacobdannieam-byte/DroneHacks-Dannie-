
"""Compare one-second simulation proposals against separate analytic scene truth."""

import argparse
from dataclasses import asdict, dataclass, replace
import json
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from examples.unified_shadow_benchmark import summarize
from examples.unified_trajectory_replay import STAMP, VEHICLE, _binding, fixture
from onboard.adapters.geometry import R_ENU_NED, rot_z
from refnav.dynamics import DroneState, DynamicsModel, NominalParameters, PositionSetpoint, PredictedState, Prediction
from refnav.dynamics.adaptive import AdaptiveDynamicsModel, AdaptiveNetwork, OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory, STEP_SECONDS
from refnav.predictive_planner import _generate, plan
from refnav.trajectory_evaluation import MetricGoal
from refnav.world_model.spatial_volume import Cell


class StraightLine:
    """Geometric baseline without vehicle dynamics or initial-velocity response."""

    def rollout(self, state, schedule, *, dt):
        initial = np.asarray(state.position_m)
        shift = np.asarray(schedule[-1].position_m) - initial
        return tuple(Prediction(
            PredictedState(state.reference, state.timestamp_s + i * dt, initial + i / len(schedule) * shift,
                           (0., 0., 0.), state.yaw_rad, 0.), (0., 0., 0.), (0., 0., 0.), 0., 0.)
                     for i in range(1, len(schedule) + 1))


@dataclass(frozen=True)
class Case:
    name: str
    state: DroneState
    snapshot: object
    alignment: object
    goal: object
    objective: MetricGoal
    now_ns: int
    plane: tuple
    response: DynamicsModel
    history: MotionHistory


def _room_metric(metric, axis, position, *, unknown=False):
    """Fixture-only voxel observations of a planar wall and optional unobserved region."""
    resolution = metric.policy.resolution_m
    coords = (metric.origin_index[axis] + np.arange(metric.policy.shape[axis])) * resolution
    wall = np.flatnonzero((coords <= position) & (coords + resolution >= position))
    cells = np.full(metric.policy.shape, Cell.FREE, dtype=np.uint8)
    if unknown:
        cells.fill(Cell.UNKNOWN)
    face = [slice(None)] * 3
    face[axis] = wall
    cells[tuple(face)] = Cell.OCCUPIED
    stamps = np.full(cells.shape, STAMP, dtype=np.int64)
    cells.setflags(write=False)
    stamps.setflags(write=False)
    return replace(metric, _cells=cells, _stamps=stamps, observation_id="analytic-room")


def _history(state, truth):
    # A fixed completed prehistory ending at the observed anchor. The earlier
    # synthetic positions and rates vary causally across 20 ms samples.
    command_time = state.timestamp_s
    start = command_time - .4
    velocity = np.asarray(state.velocity_mps)
    position = np.asarray(state.position_m)
    earlier = replace(state, timestamp_s=start, position_m=position - .4 * velocity)
    previous = PositionSetpoint(state.reference, start, command_time,
                                position + .3 * velocity, state.yaw_rad)
    predictions = truth.rollout(earlier, previous, dt=STEP_SECONDS, steps=20)
    # Translate the whole completed trajectory to the fixture anchor. Position
    # control is translation invariant. The anchor keeps the resulting velocity.
    endpoint = predictions[-1].state
    translation = position - endpoint.position_m
    shifted = replace(previous, position_m=np.asarray(previous.position_m) + translation)
    history = MotionHistory()
    before = replace(earlier, position_m=np.asarray(earlier.position_m) + translation)
    for prediction in predictions:
        sample = prediction.state
        after = DroneState(sample.reference, sample.timestamp_s, np.asarray(sample.position_m) + translation,
                           sample.velocity_mps, sample.yaw_rad, sample.yaw_rate_rad_s)
        history.observe(before, shifted, after)
        before = after
    return before, history


def cases():
    base, observed, alignment = fixture()
    semantic = _binding(observed)
    objective = MetricGoal(alignment.map, (4., .10, 1.3))
    nominal = DynamicsModel()
    state, history = _history(base, nominal)
    yield Case("rendered_corridor", state, observed, alignment, semantic, objective, STAMP,
               (0, 5.4), nominal, history)
    wall_state, wall_history = _history(replace(base, position_m=(3.50, -.10, -1.3),
                                               velocity_mps=(.55, 0., 0.)), nominal)
    wall = replace(observed, metric=fixture(blocked=True)[1].metric)
    yield Case("rendered_wall_with_opening_hint", wall_state, wall, alignment, semantic, objective, STAMP,
               (0, 4.), nominal, wall_history)
    for name, axis, plane, velocity, tau, unknown, stale in (
        ("clear", 0, 4.9, 0., .35, False, False),
        ("crosswind_moderate", 1, .51, -1.10, .47, False, False),
        ("crosswind_shifted", 1, .51, -1.28, .59, False, False),
        ("narrow_clearance", 1, .29, 0., .47, False, False),
        ("unobserved", 0, 4.9, 0., .35, True, False),
        ("stale", 0, 4.9, 0., .35, False, True),
    ):
        truth = DynamicsModel(replace(NominalParameters(), velocity_tau_s=tau))
        anchor = replace(base, position_m=(2.1, -.06, -1.3)) if name == "narrow_clearance" else base
        state, history = _history(replace(anchor, velocity_mps=(0., velocity, 0.)), truth)
        snapshot = replace(observed, metric=_room_metric(observed.metric, axis, plane, unknown=unknown))
        yield Case(name, state, snapshot, alignment, semantic, objective,
                   STAMP + (400_000_000 if stale else 0), (axis, plane), truth, history)


def _truth(case, candidate):
    """Exact distance to the fixture's infinite plane along the executed polyline."""
    trajectory = case.response.rollout(case.state, candidate.schedule, dt=STEP_SECONDS)
    points = np.asarray([case.state.position_m, *(p.state.position_m for p in trajectory)])
    transform = rot_z(case.alignment.map_from_enu_yaw_rad) @ R_ENU_NED
    mapped = points @ transform.T + case.alignment.ned_origin_in_map_m
    axis, coordinate = case.plane
    signed = mapped[:, axis] - coordinate
    distance = 0. if signed.min() <= 0 <= signed.max() else np.abs(signed).min()
    clearance = float(distance - VEHICLE.radius_m)
    goal = case.objective.position_m
    progress = np.linalg.norm(mapped[0] - goal) - np.linalg.norm(mapped[-1] - goal)
    return {"truth_min_clearance_m": clearance, "truth_progress_m": float(progress),
            "truth_collision": clearance <= 0., "truth_near_miss": 0. < clearance < .10}


def exercise(repeats, checkpoint):
    geometric = StraightLine()
    variants = (
        ("reactive_geometry", geometric, False, False),
        ("worldstate_only", geometric, True, False),
        ("selected_d0", None, True, False),
    )
    if checkpoint is not None:
        learned = AdaptiveDynamicsModel(AdaptiveNetwork.load(checkpoint))
        variants += (("ridge", OnlineFitDynamicsModel(), True, True), ("gru64", learned, True, True))
    results = {}
    for case in cases():
        candidates = {candidate.candidate_id: candidate for candidate in _generate(case.state, case.objective,
                                                                                  case.alignment)}
        reports = {}
        for name, model, use_goal, adaptive in variants:
            def run(**kwargs):
                return plan(case.state, case.snapshot, now_ns=case.now_ns, alignment=case.alignment,
                            vehicle=VEHICLE, objective=case.objective, goal=case.goal if use_goal else None,
                            predictor=model, model_id=name if model is not None else None,
                            history=case.history if adaptive else None, **kwargs)

            decision = run(monotonic=lambda: 0.)
            checks = {key: _truth(case, candidates[key]) for key in decision.ranked_ids}
            durations, timeouts = [], 0
            for _ in range(repeats):
                start = perf_counter()
                timed = run()
                durations.append((perf_counter() - start) * 1000)
                timeouts += timed.reason == "deadline_exceeded"
            reports[name] = {
                "proposal": decision.ranked_ids[0] if decision.ranked_ids else None,
                "reason": decision.reason,
                "selected_truth": checks.get(decision.ranked_ids[0]) if decision.ranked_ids else None,
                "false_feasible_candidates": sum(check["truth_collision"] for check in checks.values()),
                "near_miss_candidates": sum(check["truth_near_miss"] for check in checks.values()),
                "minimum_truth_clearance_m": min((c["truth_min_clearance_m"] for c in checks.values()), default=None),
                "checks": checks, "timing_ms": summarize(durations), "timeouts": timeouts,
                "candidate_diagnostics": [asdict(entry) for entry in decision.evaluation.candidates]
                if decision.evaluation is not None else [],
            }
        results[case.name] = reports
    return {"scope": "synthetic_one_second_open_loop", "host": platform.platform(), "repeats_per_case": repeats,
            "budget_ms": 25, "learned_checkpoint": checkpoint.name if checkpoint is not None else None, "cases": results}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path,
                        default=Path("artifacts/adaptive_dynamics_d2/adaptive_gru64.npz"))
    parser.add_argument("--repeats", type=int, default=25, choices=range(1, 501), metavar="1..500")
    args = parser.parse_args(argv)
    results = exercise(args.repeats, args.checkpoint)
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(results, handle, indent=2, allow_nan=False)
        handle.write("\n")


if __name__ == "__main__":
    main()
