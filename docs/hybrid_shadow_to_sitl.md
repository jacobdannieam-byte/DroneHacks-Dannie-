# Hybrid shadow and isolated SITL readiness

**Go for offline review. No-go for native qualification or Hybrid command
activation.** The branch implements H2 and a dormant H3 owner hook. No
launcher or ROS parameter enables H3. The Workbench challenge URL
returned a sign-in screen; no native capture, command trace or flight
was run. Host results below are synthetic.

The audited stack was [H0 draft #19](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/19)
at `d49cbb3e2b8a8517d8bc18e06c1c4206ed3b3178` on readiness
`78ef9131b2ac539042b3bb7d63ea0666724a7fbd`, and
[H1 draft #20](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/20)
at `1cb9c80dd6868a2d160769a0a919d32d75fb6c55`. This branch starts at
that exact H1 head. The 371-test baseline, H0 smoke
(`eligible_for_offline_review`) and H1 fixture
(`no_verified_progress`) reproduced.

## Data path and command boundary

1. The H1 adapter normalizes the reviewed rectified depth, TF, VIO,
   processing status, PX4 state and health streams. Pose alignment,
   the NED-to-map transform, covariance, capture/receipt stamps and reset
   counters retain their existing validation. A changed reference fences the
   epoch; recovery requires a clean session with a newly reviewed profile.
2. Continuous mode keeps one pending depth and one accepted metric and
   semantic snapshot. It discards superseded depth and skips missed ticks.
   Metric evidence is read-only, semantic data is recursively frozen in
   continuous mode, and a semantic goal is cached per accepted snapshot.
   Missing pose brackets, revoked free evidence, stale inputs and a stalled
   or rewound clock produce reason-only ticks. Bounded DDS queues and a
   bounded in-process history limit queued work. The native executor
   serializes ingestion and planning; the configured rate is a ceiling.
3. H2 defaults to 5 Hz (configurable 1–20 Hz). D5 generates bounded
   candidates, D0 predicts, D3 checks swept volume, and H0 independently
   validates the Phase 7 receipt. Cancellation or a missed 25 ms deadline
   removes the proposal. Reports retain frames, clock mapping evidence,
   epoch, resets, capture/receive ages, calibration, observation and goal
   IDs, candidate IDs, deadlines and rejection reasons. An explicit stop
   requests no motion; it supplies no braking trajectory.
4. H3 can take one receipt in the existing owner's `go` tick, with
   generation and deadline checks. It checks fresh owner authority, health,
   position and heading, and independently repeats D3 with current map
   and uncertainty. A second sweep covers the full displacement to the
   held NED position because the D0 horizon may end earlier. Unknown or
   occupied evidence, changed inputs or a rejected receipt returns the
   existing mission output. The adapter sends no arm, mode or land
   commands. The existing owner remains the sole PX4 publisher.

See [H0 contracts](h0_hybrid_integration.md), [H1 observation and profile](h1_native_observation.md),
[Phase 7 review](unified_phase7_review.md) and [SITL readiness](unified_sitl_readiness.md).

## Reproduce offline

Run from the repository root. Each CLI output must be a new file.

```sh
python -m pytest -q tests/test_hybrid_shadow_gateway.py
python -m pytest -q
python -m examples.hybrid_sitl offline --enforce-deadline --repeats 30 --output stock.json
python -m examples.hybrid_sitl offline --offset-rig --enforce-deadline --repeats 30 --output offset.json
python -m examples.hybrid_sitl compare --repeats 10 --output compare.json
python -m examples.hybrid_sitl guarded-review --output guard.json
```

The stock fixture keeps the starting envelope unknown. The offset rig is
a synthetic control case with visible starting clearance and a metric
goal. Its fake sink exercises the exact setpoint selector. The two modes
send nothing to PX4. The matched ablation adds semantic reasoning first,
then replaces the straight-line rollout with D0. It uses fixed synthetic
scenes and a separate fixture response model as a truth proxy.

On the native ROS2 host, use the same offline mode to replay a reviewed
bag. The bag reader needs ROS2 for message deserialization; the synthetic
fixture does not. The review checklist mode reports `gateway_enabled: false`
and cannot launch a mission.

```sh
python3 -m examples.hybrid_sitl offline --bag h1-native-bag \
  --profile reviewed-h1.json --output h2-bag.json
```

## Windows host benchmark

**399 full regression tests and 28 focused tests passed.** The focused
suite covers stale inputs, overload, rewinds, reset and estimator loss,
semantic contradiction, shutdown, deadlines, and occupied/unknown
sweeps, including an obstacle beyond the D0 horizon.

The run used an Intel Core i9-14900HX, Windows build 26200 AMD64,
Python 3.11.9, NumPy 2.3.5, `OPENBLAS_NUM_THREADS=1` and
`OMP_NUM_THREADS=1`. Each fixture supplied 90 decisions. The stock rig
stopped 90/90 for unverified progress. The offset rig had 90/90 eligible
proposals and 90/90 selections by the enabled fake owner. The disabled
fake owner retained its baseline 180/180 times. No skipped ticks,
application depth drops or planner deadline misses occurred in either
timed replay.

Wall time in milliseconds (p50 / p95 / p99; nested stages are not additive):

| Stage | Stock | Offset rig |
| --- | ---: | ---: |
| Depth normalization | 0.040 / 0.063 / 0.086 | 0.050 / 0.077 / 0.081 |
| Pose alignment | 0.012 / 0.018 / 0.023 | 0.012 / 0.015 / 0.018 |
| Metric preparation | 12.076 / 14.054 / 17.305 | 18.753 / 20.995 / 23.152 |
| Metric update apply | 0.385 / 0.510 / 0.637 | 0.409 / 0.492 / 0.640 |
| Semantic WorldState update | 5.949 / 8.467 / 11.932 | 8.105 / 9.759 / 14.240 |
| Semantic goal | 1.972 / 2.341 / 2.543 | — (metric goal) |
| D0 rollout per candidate | 0.839 / 0.982 / 1.348 | 0.837 / 0.949 / 1.058 |
| D3 evaluation | 13.198 / 15.714 / 17.007 | 11.745 / 12.976 / 14.176 |
| D5 planning | 13.403 / 15.932 / 17.197 | 11.912 / 13.180 / 14.399 |
| Receipt validation | 0.016 / 0.024 / 0.042 | 0.020 / 0.028 / 0.034 |
| H0 total | 13.449 / 15.982 / 17.245 | 11.960 / 13.247 / 14.471 |
| Fake owner handoff | 0.013 / 0.017 / 0.021 | 2.802 / 3.529 / 3.953 |
| Depth receipt to decision | 34.741 / 40.890 / 45.683 | 39.890 / 44.101 / 49.567 |
| State receipt to decision | 37.745 / 44.710 / 50.045 | 42.760 / 47.230 / 53.027 |

Synthetic ROS capture-to-tick age was 3 ms for state and depth in both
fixtures. The wall receipt-to-decision measure includes Python ingestion
and work between callbacks; the fixed fixture schedule does not emulate DDS
or native sensor arrival. Metric preparation is the largest measured map
stage. The 25 ms D5 deadline starts inside planning; the full observation
path is longer. No algorithm changes were made for performance.

The matched ablation used eight cases, ten timing repeats per policy per
case. Counts below are for ranked candidates on the fixed cases, while
proposals count cases with a selected candidate. A false feasible
candidate collided under the fixture response; a near miss had positive
clearance below 10 cm.

| Policy | Proposals / 8 | False feasible | Near misses | Slowest case p95 | Deadline misses / 80 |
| --- | ---: | ---: | ---: | ---: | ---: |
| Geometry only | 4 | 2 | 3 | 13.85 ms | 0 |
| WorldState plus straight-line rollout | 4 | 2 | 3 | 14.26 ms | 0 |
| WorldState plus D0 | 2 | 0 | 0 | 18.70 ms | 0 |

D0 stopped in both crosswind cases; the straight-line policies selected
`forward` in both and their selected trajectory collided in the shifted
case. All three stopped with stale, unobserved, narrow, or blocked
opening evidence. These runs contain no aircraft, Workbench course
validation, DDS timing, Orin utilization or effective PX4 command timing.

## Native acceptance

Use the signed-in Workbench and the disarmed procedure in the
[readiness runbook](unified_sitl_readiness.md). Keep flight programs
stopped. Confirm the simulator build, course, exact checkout SHA, package
versions, RMW, ROS domain, PX4/XRCE time config, and supported message
versions. First capture Phase 1 plus a bounded single-epoch H1 bag using
the [H1 capture procedure](h1_native_observation.md). Review the profile
against that evidence before running native shadow.

```sh
python3 -m onboard.diagnostics.sitl_graph --discovery-seconds 4
python3 -m examples.hybrid_sitl native-shadow --profile reviewed-h1.json \
  --seconds 30 --rate 5 --output h2-native.json
```

The shadow reader creates subscriptions only. Its exit 0 indicates a
diagnostic assessment and a clean command graph, not native acceptance.
An explicit planner stop can be a successful diagnostic result. Compare
the live outcomes with replay of the same time window, allowing for the
intentional difference between live queue coalescing and bag delivery.

- [ ] **H1:** Keep offered/requested topic types and QoS, stereo/depth
  intrinsics and rectification, optical-to-body TF, VIO tracking and
  covariance, PX4 estimator flags and reset evidence. Verify clock mapping
  and uncertainty across the capture; verify the NED-to-map alignment and
  vehicle envelope independently. A profile guess is insufficient.
- [ ] **H1:** Replay the reviewed native bag; confirm the aligned state,
  metric map and semantic goal can reach D5, and that stale, unavailable
  and unobserved inputs produce the expected stop. Review the registered
  geometry against the course.
- [ ] **H2:** Collect the native decision trace, per-stage p50/p95/p99,
  callback rates, clock progress/RTF, loss events, application coalescing,
  skipped ticks and the command publisher graph. Account for stalls and
  faults. Check that recovery from an estimator reset starts a new epoch.
  Use replay or a disarmed test for invalid inputs and deadline injection.
- [ ] **H2:** Measure wall receipt-to-decision latency and jitter and keep
  it separate from ROS capture-to-tick age. Use the raw callback wall
  times, RMW source/receipt times and sequence evidence to establish queue
  age and loss only where the pinned RMW supplies comparable timestamps.
  Unsupported events, zero timestamps, and sequence gaps are not proof of
  zero drops. Keep report coverage and collection windows with the data.
- [ ] **H2:** Collect representative process CPU, memory and RTF. Sample
  GPU, memory and temperature throughout the window with host telemetry;
  the harness's optional `nvidia-smi` result is one post-window sample.
  Collect Orin resource and thermal evidence separately if it is a target.
  Establish an acceptable budget and sustained cadence before a motion trial.
- [ ] Verify ROS2 Jazzy `rclpy`, `rosbag2_py` and storage plugin,
  `rosidl_runtime_py`, native message schemas, PX4/XRCE, simulated clock,
  OAK-D/SITL stereo and cuVSLAM/VIO, and the Workbench host and GPU setup.
  These runtime dependencies remain unverified here.

The JSON reports keep ROS acquisition and callback stamps separate from
monotonic processing times. Publication, PX4 acceptance and effective
controller-consumption time have no value in these runs. The
[Phase 5 evidence contract](unified_phase5_shadow.md) requires authoritative
effective intervals before applied-command analysis. For native transport
details see the [Jazzy event callbacks](https://github.com/ros2/rclpy/blob/jazzy/rclpy/rclpy/event_handler.py),
[Jazzy subscription metadata](https://github.com/ros2/rclpy/blob/jazzy/rclpy/src/rclpy/subscription.cpp)
and [NVIDIA SMI documentation](https://docs.nvidia.com/deploy/nvidia-smi/index.html).

## H3 activation and rollback for a later isolated SITL trial

For the current owner composition, consult the [Hybrid Gazebo runtime runbook](hybrid_gazebo_runtime.md).
The steps below describe the earlier H3 draft boundary.

1. Record acceptance of native H1 and continuous H2, the H3 offline
   gateway/fallback results, and team approval of the exact trial.
   Confirm the tested operator takeover and existing watchdog/landing
   procedure. Assign one pilot/operator and one log reviewer.
2. In a separate reviewed launch change, use an isolated SITL transport
   and ROS domain with no aircraft, hardware bridge or competing command
   source. While disarmed, verify the expected owner with
   `python3 -m onboard.diagnostics.sitl_graph --expected-owner /refnav`
   (substitute the actual full node name). Sim time by itself is not
   isolation. Inspect all command endpoints, including alternate names.
3. Review the in-process binding of the owner hooks. An input provider
   must return current H1 inputs; a producer may offer only the just
   reviewed H0 receipt using the current owner generation. Serialize
   those interactions in the owner's execution context. Verify offline
   again that expired, reset, stale, blocked and unknown paths preserve
   the baseline output. Enable only in the approved isolated
   composition; this branch supplies no activation launcher or flag.
4. Use the existing mission to take off, hold and land first. Review that
   evidence before one mapped waypoint in observed space. Review it
   before obstacle avoidance and recovery. If keyboard RC was used, follow
   the repository procedure: land, release RC, and reset the simulated
   world before running an automated program. At each step record the graph,
   operator/mission transitions, gateway result and receipt, ROS
   publication and PX4 ULog; use Phase 5 for effective intervals. Stop
   escalation after any unexplained failure or authority loss.
5. For an in-trial abort use the rehearsed operator takeover or existing
   mission landing/watchdog path; a planner stop is not a certified brake.
   Once disarmed, stop the trial launcher and restart with the original
   managed mission and no H3 hooks. Recheck publisher ownership and the
   original mission's disarmed state. Keep the traces, review why the
   trial stopped, and require a new review before restarting.

**Current gate:** Offline gateway and fallback tests can be reviewed now.
Native H1/H2 acceptance, native command timing, the isolated launch review,
operator takeover review and trial approval remain open.
