"""Deterministic D0 closed-loop diagnostic; no ROS, sensor evidence or flight."""
import argparse
from collections import Counter
from dataclasses import replace
import json
from pathlib import Path

import numpy as np

from examples.s0_course_experiment import experimental_assess, load_experiment
from refnav.dynamics import DroneState, DynamicsModel, PositionSetpoint
from refnav.predictive_planner import candidate_for_id
from refnav.reasoning.baseline import DeterministicReasoner
from refnav.reasoning.context import build_context
from refnav.reasoning.schema import MissionGoal, WorldReference
from refnav.world_model.spatial_pipeline import SpatialWorld


def run(profile_path, map_path, *, max_steps=2000):
    runtime = load_experiment(Path(profile_path), Path(map_path), verify_native=False)
    profile, course = runtime.loop.replay.hybrid, runtime.course
    state = DroneState(profile.alignment.ned, 1., (0., 0., -.76), (0., 0., 0.), 0., 0.)
    world = SpatialWorld(profile.spatial.reference, profile.spatial.calibration)
    model, counts, changes = DynamicsModel(), Counter(), []
    try:
        for step in range(max_steps):
            now = round(state.timestamp_s * 1e9)
            snapshot = world.snapshot(now_ns=now)
            # Explicit synthetic unknown sensor layer: never plant free space.
            metric = replace(snapshot.metric, query_ns=now, capture_ns=now, received_ns=now,
                             available=True, observation_id=f'synthetic-unknown-{step}')
            snapshot = replace(snapshot, metric=metric)
            alignment = replace(profile.alignment, capture_ns=now)
            inputs = dict(state=state, snapshot=snapshot, alignment=alignment, vehicle=profile.vehicle,
                          state_received_ns=now, estimator_reset_counter=alignment.estimator_reset_counter,
                          estimator_healthy=True)
            # G0 receives an honestly empty synthetic semantic context; it cannot
            # generate a sensor observation. This diagnostic tests route fallback.
            context = build_context(snapshot.semantic, now=state.timestamp_s,
                                    reference=WorldReference(metric.reference.frame_id, metric.reference.epoch,
                                                             metric.reference.clock_id), mission_goal=MissionGoal())
            counts['g0:'+DeterministicReasoner().propose(context).intent] += 1
            before = course.index
            goal = course.select(now, inputs, None, wall_ns=now)
            if before != course.index:
                changes.append(dict(step=step, reached_stage=before, position_ned_m=state.position_m))
            if course.land_reason:
                break
            command = PositionSetpoint(state.reference, state.timestamp_s, state.timestamp_s+1.,
                                       state.position_m, state.yaw_rad)
            if goal is not None:
                assessment = experimental_assess(runtime.geometry, runtime.identity, now_ns=now,
                    objective=goal, monotonic=lambda: 0., **inputs)
                plan = assessment.receipt.plan
                counts['decision:'+plan.reason] += 1
                if plan.ranked_ids:
                    selected = candidate_for_id(state, goal, alignment, plan.ranked_ids[0])
                    command = selected.schedule[0]
                    counts['candidate:'+selected.candidate_id] += 1
                else:
                    counts.update('refusal:'+reason for row in (plan.evaluation.candidates if plan.evaluation else ())
                                  for reason in row.reasons)
            predicted = model.rollout(state, command, dt=.02, steps=10)[-1].state
            state = DroneState(predicted.reference, predicted.timestamp_s, predicted.position_m,
                               predicted.velocity_mps, predicted.yaw_rad, predicted.yaw_rate_rad_s)
        return dict(scope='synthetic D0 closed-loop route fallback; no native depth, timing qualification or flight',
                    sensor_layer='all UNKNOWN; no live obstacles supplied in this positive model diagnostic',
                    g0_scope='actual reasoner on empty synthetic semantic context; route fallback only',
                    result=course.land_reason or 'step_budget_exhausted', reached_stages=changes,
                    final_stage=course.index, stage_count=len(course.stages), steps=step+1,
                    elapsed_model_s=state.timestamp_s-1., final_position_ned_m=state.position_m,
                    counts=dict(counts), observed_free_voxels=0)
    finally:
        runtime.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, required=True)
    parser.add_argument('--map', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = run(args.profile, args.map)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2)
    print(json.dumps({key:report[key] for key in ('result','final_stage','steps','elapsed_model_s')}))


if __name__ == '__main__':
    main()
