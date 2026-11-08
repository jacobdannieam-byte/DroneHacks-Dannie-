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
