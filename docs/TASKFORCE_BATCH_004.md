# Batch 004: native spatial shadow and bounded trial preparation

Delivery branch: `code-integration-hybrid`. No arming, flight, parameter change,
map control, background registration or persistent autonomous loop occurred.
Classic/S1 remains the protected baseline. Existing S0/S2/S3/S4 were evaluated
before introducing any new strategy.

## Native shadow result

All four curated strategies completed separate 15-second subscriber-only native
windows against Gazebo sensor streams on source `497dacd9602b74442f63f0e5223e1af29cb40d26`.
Each produced 116 decision records, no fatal ingestion fault and no command-graph
issues. The finalized raw recording passed integrity checks over 100.78 seconds.

| Strategy | Full spatial assessments | Fresh free voxels | Occupied voxels |
| --- | ---: | ---: | ---: |
| S0 existing | 4 | 22 | 462–463 |
| S2 information gain | 5 | 22 | 462–463 |
| S3 learned ranker | 6 | 22 | 463 |
| S4 physics informed | 3 | 22 | 462 |

These are diagnostic input/consumer results, not paired policy-performance scores.
The remaining records include waiting for synchronized inputs, stale tracking or
geometry, and retained no-progress state. Full spatial assessments refused motion
because observed whole-body clearance was insufficient. S4 also recorded one
planning deadline miss. There is no demonstrated flight superiority or continuous
fresh planning at the requested rate.

The actual input path is native depth + calibrated optical transform + capture-time
estimated pose into SpatialWorld's local RollingVolume and semantic WorldState,
then existing reasoning and D0/D3/D5 consumers. The P2 global reference-localization
and topology sidecars require visual tracking and remain inactive on this
simulated-estimator path. Neither reference geometry nor simulator truth was
inserted as observed free space. Profile uncertainty values are explicitly
diagnostic assumptions, not measured flight bounds.

The forward camera's near/rear blind volume remains a structural bootstrap gate.
Seeing 22 free cells does not establish the complete stationary or stopping body
volume. Policy-native owner timing, reviewed uncertainty, measured stopping,
recovery and deployment identity binding remain separate gates.

## Repairs and preserved failures

- Pinned Humble supplies message-only callbacks. The runner now supports optional
  message metadata and leaves unavailable RMW timestamps/sequence fields null.
- First live attempts encountered fatal clock-fence refusals. Their logs and raw
  evidence are preserved. A first harness invocation expanded shell variables in
  PowerShell; a second overwrote ROS Python paths. Both invocation failures are
  retained separately from the actual runtime failures.
- Source `8a3d3f3` records the exact rejected stream, source/sample times, callback
  clocks, pending queue and rejection subtype. Native failures showed 56–60 ms
  source-to-callback-clock discrepancies, after depth/map callbacks took roughly
  50–80 ms. They were not queue-capacity overloads.
- Source `497dacd` gives the subscriber-only shadow runner a dedicated ROS clock
  receiver/executor. It samples delivered time rather than sensor timestamps;
  H1/mission mutation stays on one thread. Clock exceptions fail closed and the
  clock thread has bounded shutdown. The original 50 ms wait and 32-message limits
  remain. Native final runs also explicitly bound BLAS/OpenMP to one thread.
- This is a shadow-runner repair. It does not qualify the H3 command owner's timing.

The two earlier disarmed recordings passed integrity/duration checks over 126.28
and 99.976 seconds; their failed shadows are not relabeled as successful. The
initial focused pytest invocation failed a Windows sandbox temporary-file rename;
rerunning with a new workspace-local temporary directory passed. A separate test
invocation named a nonexistent test file and ran no tests; corrected selection
passed. No production checks were weakened to address either harness issue.

## C1 and R1 preparation

[The concrete trial package](C1_BRAKING_RECOVERY_PLAN.md) distinguishes two exact
candidates and requests separate approvals. No approval artifacts were created.

C1 is one northbound 0.20 m/s translation, brake at 1.50 m, settle at ≤0.05 m/s for
0.60 seconds, then native LAND. The braking deadline is four seconds on either
clock. The 2.50 m beyond the brake trigger is design recovery allocation, not a
measured stopping bound. Tested candidate `8a3d3f3ed0df1408e1665d45cb0d553b9a162ec5`,
archive `32083fbeb699b8b74687ca1ffb8f6889fd6e5c63d82bf71c65085d1bdb594106`, domain 54.
Its native disarmed composition passed: 43.86 seconds raw recording, 3,232 strict
position samples and 65 statuses all disarmed, zero faults/pending samples, and
idle command graph. Effective landing speed remained 0.7 m/s; a moving approval
would permit the same ephemeral 0.3 m/s request used in C0.

R1 is a separate stationary test: settle, publish one final zero target/heartbeat,
expire a 250 ms steady-clock lease without renewal, observe configured native
Position fallback, then permit the named operator's ordinary native console LAND
only after owner exit and fresh healthy in-lane/idle-graph checks. It does not
assume automatic LAND. Source `fde3265ee151ce41ebf3fada16a20cc49b2b169e`, archive
`55bf99fc2fcbb4a539995e9ef92ab69721709d0431c4a4c2db1f655c913b036d`, domain 55.
The raw audit checks mode/target fields, no post-expiry outputs, native transitions,
both clock deadlines, bounds, landing/disarm and independently retained operator
receipts. Hash binding is not proof that a manual observation was correct.

The pinned-image private ROS R1 rehearsal passed: final zero intent, no later
outputs after expiry or clock advance, no actuator endpoints, no console command,
valid synthetic receipt schema and rejection of a mismatched console digest.
Synthetic receipts are rejected by the real R1 audit. Native R1 disarmed
composition also passed: 44.808 seconds of raw recording, 3,334 strict positions,
66 statuses all disarmed, no faults or pending samples, and an idle command graph.
The real operator capture tool is a separate read-only overlay from commit
`f992f008ac20a7ab05555539ab60c42316b061cf`, raw SHA256
`3b35d95ec380c756d23bd169c2f83c1b31e374de1cb75a6dce22b81999e1986f`.
It binds explicit operator permission, helper identity, named operator and trial;
requires the previous owner to exit, fresh healthy armed Position telemetry and
an idle graph; resamples both clocks immediately before one fixed console LAND.
Its 30-second issue cutoff reserves 30 seconds for landing within the 60-second
recovery window. Final pinned-image synthetic capture and missing-approval refusal
passed. The armed handoff and native recovery remain future trial evidence.

## Validation and evidence

Frozen driver source `fde3265`: **742 tests passed in 42.94 seconds**. Final
repository source including the operator helper `f992f00`: **751 tests passed in
35.91 seconds**. Relevant
focused checks and independent reviews are retained under
`artifacts/taskforce_batch_004/`. The archived-probe dry profile build in that
directory is a fixture, separate from the same-run profiles used natively.

Raw bags, immutable source archives and complete run logs remain in the sibling
workspace `taskforce-native-004/`. Manifests hash original bytes; Git-normalized
text copies can differ in line endings. No report is a reusable stopping envelope.
An additional read-only raw audit decoded 823 native statuses across all five
recordings, all disarmed, and found no recorded flight-control messages. This
complements the sampled idle command graphs; it does not claim authentication of
every possible unrecorded publisher. All simulator and audit containers stopped.

Full policy flights remain gated on genuine whole-body clearance and deployment
qualification. The next approval request is C1 only; R1 is reviewed separately
after C1, with no automatic retry or policy promotion.
