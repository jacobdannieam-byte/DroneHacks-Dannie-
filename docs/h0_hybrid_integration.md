# H0 hybrid integration contract

H0 composes the existing perception, reasoning, prediction and review APIs
for a single diagnostic planning tick. The baseline is
`integration/unified-sitl-readiness@78ef9131b2ac539042b3bb7d63ea0666724a7fbd`.
Its existing offline suite passed all 333 tests before the H0 changes.

The [readiness runbook](unified_sitl_readiness.md) defines the native capture
and replay requirements. The [Phase 7 review](unified_phase7_review.md)
defines the separate command owner gates. H0 consumes normalized inputs
in process. `refnav.node` and `refnav.core.Mission` keep the existing
execution path.

## Shared API and data flow

| Boundary | H0 contract |
| --- | --- |
| State and alignment | `DroneState` supplies position, velocity, yaw and yaw rate in a named `LocalNED` frame. `FrameAlignment` supplies the map transform, clock origin, capture time, epoch, and uncertainty. The caller supplies the state receive time and a health decision for that sample. H0 compares the estimator reset counter with the alignment's counter. An upstream reset, including counter wrap, must advance the epoch and invalidate earlier evidence. |
| Map | `SpatialWorld` aligns rectified `DepthCapture` and healthy `PoseSample` evidence at capture time. It produces a detached `SpatialSnapshot` with a `MetricSnapshot` and optional semantic evidence. The metric snapshot retains depth capture, receipt and query times, calibration, observation ID and reference. Unknown cells stay ineligible for motion. |
| Goal | A required `MetricGoal` identifies the bounded map-frame objective. An optional `GoalBinding` pairs a `ReasoningContext` containing a `MissionGoal` with a `GoalProposal`. D5 uses the metric objective to generate candidates. When a binding is supplied, D3 rebuilds the context from the supplied snapshot, validates the proposal independently, and uses its grounded target to measure progress. |
| Plan | `assess` calls D5 with the existing D0 predictor and `VehicleBounds`. D3 checks the continuous sweep against observed space with the vehicle envelope and state, alignment and tracking uncertainty. D5 returns a `Plan` with ranked IDs and per-candidate evidence or an explicit stop request. |
| Receipt | `HybridAssessment` returns the existing `ProposalReceipt`, the objective, optional goal binding, vehicle bounds and an independent `validation_reason`. The receipt binds the diagnostic plan to the original state, metric snapshot, alignment, state receipt time and reset counter. `decision_ns` is the logical tick time. `validate_receipt` checks the ranked evidence without owner inputs. The existing Phase 7 `review` also requires owner readiness and authority. |

`assess` returns a stop with a stable reason code for unavailable, stale
or inconsistent input. Incorrect API types raise `ValueError`. D3 retains
candidate rejection codes when an evaluated batch yields a stop. The D5
deadline uses the supplied monotonic clock, separate from simulation time.
H0 checks cancellation and the deadline again after reviewing a proposed
candidate. Late proposals lose their ranking. The returned receipt is
same-tick diagnostic evidence. A later consumer needs current inputs and
its own validation at use.

## Non-flying smoke test

From the repository root with the existing Python test dependencies:

```sh
python -m examples.h0_hybrid_smoke
python -m pytest -q tests/test_hybrid_integration.py tests/test_sitl_review.py tests/test_predictive_planner.py tests/test_spatial_world.py tests/test_trajectory_evaluation.py
python -m pytest -q tests
```

The smoke test renders the existing corridor fixture, processes depth and
pose through WorldState, constructs a mission goal and passes its validated
binding to D5. It prints JSON with an eligible proposal, model and candidate
IDs, timings and rejection reasons. It returns a nonzero exit code if the
proposal fails review or the semantic goal ID fails to reach D5. Its fixed
monotonic clock makes the run deterministic. Timing limits are tested with
injected clocks. Native runtime latency requires its own measurements.

## H0 acceptance checklist

- [x] Verify the exact readiness SHA and run the pre-change regression suite.
- [x] Exercise the semantic and metric-only paths through D5 and independent receipt validation.
- [x] Cover frame, clock, epoch, reset, state/geometry freshness, receive times, health, absent goals and mismatched semantic context.
- [x] Cover deadline expiry during planning and after validation, occupied trajectories and a tampered candidate ID.
- [x] Run the non-flying smoke and the full offline regression suite.
- [ ] Collect native evidence under H1 before considering a live handoff.

## Native evidence and next boundaries

- **H1 — Native read-only inputs and replay.** Confirm ROS 2 Jazzy, matching
  `px4_msgs`, XRCE, live topics and QoS, Workbench `/clock`, and disarmed graph
  ownership. Measure source-to-ROS clock mapping, capture versus receipt time,
  estimator health and reset markers. The current `px4_odom` adapter stamps
  its ROS output at receipt, so it cannot establish sample time by itself.
  Validate native depth encoding and calibration, CameraInfo and TF, time
  alignment and uncertainty with cuVSLAM running. Save the ROS bag and PX4
  evidence. Establish effective applied-setpoint timing and measure deadline
  behavior with perception active, including target-device profiling.
- **H2 — Live shadow planning.** Consume the reviewed H1 stream and exercise
  the diagnostic path under resets, stale sensors, estimator loss, moving
  geometry and deadline misses. Validate at use against current state,
  geometry, semantics and owner inputs. Record comparisons and failure
  evidence before connecting the planner to commands.
- **H3 — Isolated simulator actuation.** With a reviewed native handoff and
  team approval, test bounded proposal consumption inside the existing
  command owner. Verify takeover, watchdog, loss-of-data and landing
  behavior before the qualification sequence.
