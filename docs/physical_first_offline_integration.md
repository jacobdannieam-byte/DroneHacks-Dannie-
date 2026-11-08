# Physical-first offline integration evidence

## Revisions and preservation

Branch: codex/physical-p2-integration. Fetched origin before edits. Started at
ad31ae5, retaining 33dbeeb and ad31ae5 above current remote unified bfb09c8.
Common ancestor with P2 was 66de6f3. Applied only the four P2 commits:
e242580 -> 07dd1f8; 90505e2 -> 67a35ea; 4aac246 -> 2e102c7;
e427467 -> d78cf5b. No conflicts. Existing reference importer, localization,
metric queries, topology and newer runtime remain present.

fc7afda selects refnav/depth.py and test_depth_clearances.py from soham-depth
7781aff and adapts soham-stages 3351c87 free_distance_seen semantics into a pure
first-segment proposal. No donor mission owner or publisher was imported.
507d144 adds bounded registration, optional AsyncDiagnosticSidecar and a disabled
planner contract. 00901b3 records tracking evidence. Classical height work remains
separately committed as e99c41b on codex/physical-height-regressions; it was not
blindly transplanted into this older classical runtime.

## Commands and outcomes

Run from this checkout:

- python -m pytest tests -q: 464 passed in 42.08s immediately after P2 reconciliation.
- python -m pytest tests/test_depth_clearances.py tests/test_refnav_core.py -q:
  57 passed in 11.41s.
- python -m pytest tests/test_first_segment.py -q: 8 passed in 0.21s.
- python -m pytest tests -q: 501 passed in 41.27s after implementation.
- After adding worker failure coverage and replacing a timing wait with an explicit
  completion condition: python -m pytest tests/test_registration_worker.py
  tests/test_planner_map_input.py -q: 3 passed in 1.10s. The full suite was not
  repeated after this focused change.
- git diff --check: passed.

Tracking reproduction (from this checkout):

```powershell
python examples/physical_tracking_audit.py ../classical-audit-source/telemetry/skybeta-16/flight_records/20261108_014841_practice_course_run ../classical-audit-source/telemetry/skybeta-16/flight_records/20261108_010909_practice_practice_out_and_back
```

Output is preserved in physical_tracking_results.json. Course: 608 samples,
arrival-age p95 0.26305 s, max 0.383 s, 89 above 0.2 s. Out-and-back: 656 samples,
p95 0.2353 s, max 0.326 s, 62 above 0.2 s. Neither has sampled ages above 0.5 s.
There are 262/236 identical adjacent sampled poses respectively; these are not
proof of frozen tracking. Sample time never decreases; maximum sampled gaps
are 0.1107/0.1123 s. These are logger gaps, not callback-arrival gaps.
Capture-time anomalies, capture age, raw resets and transport causes remain
unverified. The 0.5-second threshold remains a candidate, not a qualified limit.

## Contract and remaining limitations

Registration has one worker and one replaceable pending capture, copied at
submission. The worker owns its localizer. Stale/future results, wrong map
versions and wrong VIO references are withheld. Close clears pending/results;
a running native registration cannot be forcibly interrupted by this thread
wrapper. Bounded ICP iterations bound work, but hard wall-clock deadline and
Orin concurrency acceptance remain unverified. Copy cost is still on submission.

AsyncDiagnosticSidecar is an optional read-only adapter, not the default replay
path. Its end-to-end replay timing and native integration remain unverified.
The planner contract tests schema, map version, frame, epoch, age, unavailable
state and finite uncertainty, and composes the correction with current odometry.
H2 does not yet call this helper: qualified planner adoption and paired decision
improvement remain outstanding. Do not claim this is completed P2-S.

First-segment proposals bound distance by observed range minus stopping distance
and hold for missing/stale/future depth or unhealthy tracking. This is offline
proposal logic, not full swept-body passage verification or a mission hookup.
Existing fresh collision checks remain authoritative. No new command publisher.

## Review gate

Before physical promotion, review deployed source/adapter hashes, anchoring and
clamping configuration, timestamp domains and reset counters, and collect raw
laser/estimator evidence for the separate height-fusion stop. Run ROS-dependent
classical regressions in the supported environment. Measure body envelope,
calibration and stopping distance; qualify capture-age limits against speed and
clearance. Obtain an authorized reference and real synchronized depth/VIO before
P2 localization acceptance. Native subscriber-only shadow and Orin latency are
unverified. No deployment, arming, flight or safety-parameter changes occurred.

## Follow-up: H2 consumer and swept-body validation

This section supersedes the earlier outstanding offline-hookup items while
preserving their historical status.

- 2fafeb3 connects the opt-in ShadowMapConsumer through ShadowLoop.poll into
  HybridAssessment.map_reasoning. It pins reference/map version/epoch and uses
  the current H2 state with its reviewed NED-to-spatial alignment. The pose is
  gravity-aligned because H2 does not model roll/pitch. Invalid estimator input
  does not reach the consumer. The command receipt and local collision geometry
  are unchanged; default map_consumer is None and the consumer defaults disabled.
- An actual H1 fixture now passes depth through the background MapLocalizer and
  H2 polling loop. It verifies a map result is consumed, the planning receipt
  remains identical to the baseline, and tracking loss fences the worker.
- 4ab00f5 adds swept_first_segment using the existing conservative metric sweep.
  A sphere of radius hypot(horizontal radius, half height) encloses the vehicle
  cylinder at every attitude. It may reject feasible tight passages but does not
  omit body height. Checks include start/end and continuous segment interiors.
  Regressions cover clear space, jamb, unknown, expired, floor and map boundary.

Validation from this checkout:

```powershell
python -m pytest tests -q
python -m pytest tests/test_p2_shadow_replay.py tests/test_hybrid_integration.py tests/test_first_segment.py -q
```

Full run: 505 passed in 43.15s. After strengthening the async test to use
ShadowLoop.poll instead of calling assess directly, focused run: 32 passed in
6.39s. git diff --check passed. No failed tests remain in these offline runs.

Scope limit: this is tested diagnostic H2 adoption, not map-driven command
selection or demonstrated route improvement. No runtime enables the consumer.
The first-segment function validates geometry only; existing mission ownership,
health and command gates remain required. Physical body dimensions, calibration,
real synchronized replay and concurrent Orin timing remain acceptance gates.

## Additional offline qualification and remaining simulation gate

Existing Ubuntu Docker images were discovered without starting a simulator.
Focused classical ROS unit tests now pass: 63 in 2.52s, using the existing
network-isolated, read-only drone-simulator image
sha256:a94b59df2e4aa268a107a56d9087c4645f6819680e4c17ab4db74cbc81c66795.
The exact command is preserved on the classical branch in
 docs/PHYSICAL_INTEGRATION_EVIDENCE.md. This supersedes missing-rclpy as a blocker
for those tests, not native or flight qualification.

Added worker regressions prove caller mutation cannot alter submitted captures
and shutdown during registration discards late results and pending work.

Commands from this checkout:

```powershell
python -m pytest tests/test_registration_worker.py tests/test_gazebo_hybrid_integration.py tests/test_unified_qualification.py -q
python -m examples.p2_registration_benchmark --repeats 40 --output artifacts/physical_offline_registration_review
python -m examples.p2_shadow_replay fixture --repeats 10 --output artifacts/physical_offline_shadow_review
```

18 focused tests passed in 5.53s. The output files were subsequently renamed
with .json extensions and are committed. Registration benchmark: 20/20 positive
fixtures within tolerance, all 20 degenerate/wrong-map negatives rejected;
p95 273.33 ms, maximum 276.66 ms on this Windows laptop. Concurrent host load was
not controlled, so this is not a performance regression diagnosis or Orin budget.

Four-arm replay: all 160 decisions requested safe stop; map arms each accepted
40 fixes and reduced synthetic position RMSE from 0.0675 m to 0.01625 m.
This is the existing synchronous diagnostic replay benchmark. Asynchronous H2
integration is covered separately by behavioral tests; these timing figures must
not be presented as asynchronous or native throughput evidence. H3 stayed false.

Next simulation gate: reviewed isolated Gazebo run with the exact intended
revision and adapter configuration, demonstrating nonzero launch origins,
climb/first-segment behavior and fault response. Existing historical Gazebo
success does not qualify this revision. No flight program was run in this work.
Map-based control remains disabled. Physical identity, anchoring, calibration,
body dimensions, raw capture/reset evidence and Orin timing remain unverified.
