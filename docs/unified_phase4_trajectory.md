# Unified Phase 4 — diagnostic trajectory evaluation

Phase 4 ranks supplied candidate position/yaw schedules using D0 by default,
the Phase 2 metric map, and an optional validated semantic goal. The result
contains diagnostic IDs, reasons, scores and rank. The evaluator does not
generate candidates or produce executable actions. Phase 3's offline decision
keeps ridge as a comparison baseline. Live Phase 1 and 2 acceptance remains open.
[Unified roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) ·
[Phase 2 geometry](unified_phase2_spatial.md) · [Phase 3 decision](unified_phase3_adaptation.md)

## Branch and contract

The review base `integration/unified-phase4-base` joins Phase 2 at
`67b70ed` and Phase 3 at `5816a04` in merge `d6c363c`. This keeps the
Phase 4 review diff focused. The independent Phase 1 diagnostics and the
flight command path are outside this branch's changes.

- Input is a measured `DroneState` in named local NED and one to eight
  unique candidates of up to 100 explicit effective setpoint intervals.
  Prediction uses the existing 20 ms rollout contract. The scorer verifies
  interval coverage and prediction timestamps/references. A separately
  selected predictor can receive a `MotionHistory`; the default is D0.
- `FrameAlignment` explicitly binds the NED and gravity-aligned z-up map
  references, a capture stamp, the ns origin for D0's seconds, NED origin
  translation, map yaw offset from ENU and error bounds. The alignment
  applies the existing NED-to-ENU rotation and then map yaw. Clocks and
  epochs must agree. Provisional freshness limits are 100 ms for state and
  alignment, 250 ms for the map capture, and 30 ms for alignment-to-state
  skew. A clock rewind or reference change requires a new compatible
  reference and map.
- `VehicleBounds` declares an enclosing vehicle sphere, state position
  error and tracking error over the forecast. Expand the radius by those
  errors and the alignment translation and rotation bounds. The rotation
  contribution scales with distance from the NED origin. The tracking
  bound must account for the predicted polyline approximation. These
  inputs require provenance and qualification from the caller.
- An optional `GoalBinding` uses the existing `ReasoningContext` and
  `GoalProposal`. Rebuild and match the context from the supplied detached
  semantic snapshot, then revalidate the proposal on the same map clock.
  A blocked target rejects the objective. A stale, unknown or mismatched
  goal prevents ranking. A valid goal's center only contributes progress.
  Its opening width, height or connectivity claims cannot clear a voxel.

## Clearance and ranking

The scorer takes one detached metric grid at evaluation time and creates
summed-volume tables for occupied and unknown cells. It reuses the Phase 2
swept box calculation for each continuous line segment, including the
measured starting position. Bounding boxes over-approximate the enclosing
sphere. Any occupied voxel rejects the candidate. Any unknown, expired or
out-of-map voxel makes it unverified. Missing or stale transforms, clocks,
state or geometry are also unverified. Freshness is checked at evaluation
time. This is a static-evidence diagnostic, so it cannot predict changes in
the scene or carry a clearance result into a later decision cycle.

For a feasible candidate, the score is progress in meters minus 0.02 times
a dimension-mixed effort index, plus 0.10 times the conservative extra
clearance. The effort index integrates squared linear acceleration and yaw
acceleration. The extra clearance is 0.10 m if the entire sweep is still
observed free when expanded by that amount. Otherwise it is 0 m. This is a
coarse supported lower bound relative to the inflated envelope and current
evidence. Sort feasible results by decreasing score and then ID. No verified
candidate yields `no_verified_candidate`, a diagnostic reason that carries
no command authority.

## Reproduce

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m pytest -q tests
python -m examples.unified_trajectory_replay --output ../phase4-evidence --repeats 100
python -m examples.unified_trajectory_replay --output ../phase4-repeat --skip-timing
```

The replay uses Phase 2's synthetic depth renderer and a separate synthetic
state anchor inside its observed corridor. It exercises a valid opening
hint, a contradictory metric wall despite the hint, and an expired map.
Tests cover frame rotation, yaw wrap, time and epoch mismatch, stale cells,
an overhead obstacle inside the uncertainty-inflated envelope, continuous
checks between samples, missing coverage and stable ranking. A targeted
cross-check compares the summed-volume verdicts to Phase 2's sweep API.

The benchmark measures eight 50-step D0 candidates, proposal validation,
geometry checks and ranking on a Windows host. It excludes mapping, ROS
transport and Orin load. Native replay and live Phase 5 remain separate gates.

**Offline gate passed on the tested fixtures.** The branch passed 291 tests
and repeated replay JSON was identical. The observed corridor ranked
`advance` first. A conflicting metric wall rejected the path despite the
semantic hint. Stale geometry yielded no ranked candidates. The synthetic
eight-candidate run measured 14.09 ms p95 and 15.88 ms p99. No sample
exceeded the provisional 25 ms total allocation.
[Replay](../artifacts/unified_phase4/replay.json) ·
[Benchmark](../artifacts/unified_phase4/benchmark.json) ·
[Test record](../artifacts/unified_phase4/test_results.txt)
