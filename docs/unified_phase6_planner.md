
# Phase 6 simulation-only predictive planner

This branch stacks on Phase 5 at `5ce2595`. The planner generates up to eight
candidates with one-second position/yaw schedules at 20 ms steps. A map-frame
objective steers the candidate fan. A validated semantic proposal may supply the
progress target. Each candidate uses Phase 4's frame, epoch and freshness
checks and conservative inflated swept-volume tests. Unknown and occupied
segments are ineligible. The planner also requires the extra 0.10 m clearance
probe and at least 0.015 m of predicted progress. Hold is evaluated for
diagnostics and excluded from the progress ranking.

The score is predicted progress in meters minus 0.02 times integrated squared
acceleration effort, plus 0.10 times the clearance lower bound in meters,
minus 0.10 times the uncertainty inflation in meters. Candidate ID breaks
ties. The result contains diagnostic IDs and evidence or a safe-stop request.
A request is a simulation outcome. It neither sends an action nor certifies
that the vehicle can brake in the current volume.

Cancellation and a provisional 25 ms wall-clock budget are checked before
rollouts, between candidates and before returning a decision. An interrupted
batch discards partial ranks. A single synchronous rollout can run past the
deadline. Geometry is treated as static at the evaluation timestamp and
would need fresh revalidation before a later consumer used a proposal.

## Reproduce

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m examples.unified_planner_ablation \
  --output phase6.json --repeats 25
```

Eight deterministic cases use the Phase 2 rendered depth fixtures and
analytic planar rooms. The decision inputs are estimated state, supplied
geometry and the same optional WorldState hint. A separate synthetic
closed-loop response executes proposed schedules over a one-second polyline.
An analytic wall-distance oracle checks physical body clearance afterward.
The oracle alone receives the synthetic truth plane and response parameters.
In the open rendered corridor, the numeric clearance is to the rear plane.
All proposed one-second sweeps also end before the opening wall plane.
The shifted response and synthetic measured history are fixed, with no
random draws. `artifacts/unified_phase6/ablation.json` retains per-candidate
diagnostics, executed clearance and progress, and runtime percentiles.

The reactive baseline ranks straight geometric paths against a metric
objective. WorldState-only adds the semantic progress hint to the same
geometric checks. The predictive variants use the same candidates and
semantic hint with D0, D2 ridge or the existing GRU-64 checkpoint. D0 is the
selected default, so it also represents the selected-model arm.

| Variant | False-feasible candidates | Near-miss candidates | Highest per-case p95 | Budget misses / 200 |
| --- | ---: | ---: | ---: | ---: |
| Reactive geometry | 2 | 3 | 11.76 ms | 0 |
| WorldState-only | 2 | 3 | 12.59 ms | 0 |
| D0 | 0 | 0 | 14.86 ms | 0 |
| Ridge | 0 | 0 | 26.57 ms | 8 |
| GRU-64 | 0 | 0 | 27.67 ms | 175 |

Accuracy decisions use an injected fixed clock to separate geometry/model
behavior from deadline behavior. A second run times each variant end to
end with the real monotonic clock and the 25 ms budget. The timings are from
a Windows Intel CPU without perception workloads. Budget overruns terminate
the remaining candidate batch. They measure time to the stop result on
those runs. The stale case rejected before rollouts in all arms.

In the shifted lateral-drift fixture, both straight-line baselines selected a
schedule with about 0.95 cm of synthetic body overlap. All three predictive
variants requested a stop. In the moderate case, the same predictors also
requested a stop where the geometric selection had about 10.08 cm of physical
clearance, giving up progress. No variant proposed motion into the rendered
wall despite the opening hint. These tests support retaining D0 for offline
replay. The fixtures do not measure mission completion, recovery, dynamic
obstacle tracking or Orin performance. Phase 7 stays gated on reviewed
Phase 1–6 qualification and explicit team approval for isolated SITL.
