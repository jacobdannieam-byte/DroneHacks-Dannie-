# C1 braking and R1 recovery: refreshed approval packet

**Prepared for separate user approvals; no arming is authorized.** C1 and R1 are deterministic calibration experiments, not S0, waypoint scans or full-course flights. This packet proposes the same bounded motions as [the historical packet](C1_BRAKING_RECOVERY_PLAN.md), using one newly frozen source with the operator helper included, RTX rendering, and fresh independent disarmed checks in domains 57 and 58. The frozen source and both fresh disarmed checks are now bound in the release review manifest; the independent raw disarmed audit passed; final packet review and separate explicit movement approval remain required. The historical deployment packet remains preserved. Historical failures and successful checks remain unchanged.

## Source decision and audit

Use one final immutable candidate for both experiments rather than mixing old drivers and a helper overlay. This exercises the current native launch composition and the GPU rendering path already demonstrated disarmed in [batch 007](TASKFORCE_S0_007.md). It does not run spatial-policy or scan code as a command owner. The final source/archive and fresh evidence identities belong in [the release manifest](../artifacts/taskforce_batch_008/release-manifest.json); unresolved fields block release and are not zero-valued approvals.

The final candidate is `2ecbee3281727cff8c3791068f3503e66edf2bcf`, archive SHA256 `87e8781e0e26492b0f77e733a0d54cdd6bfc17c7e3016947c9538f5cff350e88` at `taskforce-flight-prep-008/candidate.tar`. Independent byte checks matched all 403 archive members against its manifest, including both profile/recorder pairs. Against prior R1 source `fde3265ee151ce41ebf3fada16a20cc49b2b169e`, the calibration owner, state machine, raw braking auditor, raw recovery auditor, stopping analyzer and `run_hybrid.sh` have identical Git bytes. The in-tree operator capture helper's archive SHA256 is `3b35d95ec380c756d23bd169c2f83c1b31e374de1cb75a6dce22b81999e1986f`, identical to the previously reviewed overlay. No separate helper overlay is needed.

Imported H1/depth modules now propagate the actual camera near clip and include additional diagnostics; these changes do not change the imported clock-mapping implementation or calibration's velocity-command state machine. The final `core.py` change adds four optional Vehicle timing/reset fields; calibration imports its navigation constants and does not instantiate the Hybrid planner or contact guard. Added map/scan/launch-clearance functionality is not a braking measurement. Repeat disarmed checks rather than assuming source equivalence proves deployment equivalence. [Per-file source audit](../artifacts/taskforce_batch_008/calibration-source-audit.json) distinguishes archive, working and Git-normalized bytes.

Focused calibration tests passed **117 in 1.09 s** before freeze. The final frozen-source tracked suite passed **817 in 44.01 s**, with final focused startup/runtime/telemetry **46 in 4.36 s** and independent launch/binder/stopping **34 in 1.04 s**. These are offline evidence; native checks are separately described below.

## Common deployment and recovery envelope

| Binding | Concrete review input |
| --- | --- |
| Image | `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1` |
| Runtime | ROS Humble; Gazebo 8.15.0; PX4 `54f0455ffcd755534539a7cf33a09a20bf71d29d`; px4_msgs `392e831c1f659429ca83902e66820d7094591410` |
| World | Isolated empty calibration lane; SHA256 `d64be312bb5479c529acf2dd6ba559f50d4bf2dd224fbb31cd69be1b5f93dd07` |
| Generated vehicle | SHA256 `f8cc362929337eb37e8649632ce3426d86a029a80932cee06a1a80f2d1e169f3`; verify regeneration on final source |
| Geometry review | SHA256 `6582616dd24cb11488196bfc59bcd4476c685508a8866b410b91d334728fdaf3` and all its source dependencies must match |
| Isolation | Network disabled, localhost DDS, no hardware devices, one `/calibration_trial_owner`, immutable source and separate new evidence directory per attempt |
| Rendering | Preserve batch 007 WSLg/D3D12 RTX setup; verify actual OGRE renderer in each new run. A software fallback is not GPU-qualified |
| Parameters | Disarmed checks make no parameter changes. Only after explicit approval: ephemeral `MPC_LAND_SPEED=0.3` m/s, read back before command endpoints. Preserve `COM_OF_LOSS_T=1 s`, `COM_OBL_RC_ACT=0` (Position) |
| Time | Owner <=60 s ROS and <=120 s wall; landing <=30 s on both clocks; supervisor recovery observation <=60 s after owner exit |
| Global speed | Estimated and independently observed actual 3D speed <=0.50 m/s, including climb and native LAND; exceeding it fails the trial |
| Origin | Native NED origin `[0,0,0]`, start within 0.10 m, fresh landed/disarmed/healthy telemetry and stable frame/reset identity; source transform must agree with fresh observed origin |
| Operator | Proposed: Codex supervised by the requesting user in this chat, isolated simulator only; name explicitly in each future approval |

The 6 x 8 m lane and roof 3.048 m above the floor are authored simulator geometry, not a physical survey. The model's historical all-attitude radius is 0.303078 m; lane review uses a 0.35 m sphere plus a 0.25 m design positional reserve. Design reserve is not measured stopping performance. [Geometry and frame review](calibration_lane_v1.md).

## C1: request one braking attempt first

Trial `calibration-c1-020-001`, domain **57**, profile `scenarios/calibration/c1_stop_020.json`, recorder `scenarios/calibration/c1_recorder.json`. Their actual byte hashes are in the release manifest. No waypoint scan or policy selector controls this trial.

1. Stream zero velocity, acquire native Offboard and arm only under the exact approval. Climb vertically at <=0.20 m/s toward 1.0 m above launch. Confirm 0.8–1.2 m and actual 3D speed <=0.05 m/s for 0.60 s of advancing samples.
2. Command NED velocity `(0.20,0,0)` m/s. At the first eligible decision with north offset >=1.50 m, publish zero velocity and retain actual ROS/PX4 publication receipts. The trigger is sampled, not guaranteed to occur at exactly 1.50 m. Transit timeout is 12 s on either clock.
3. Require 3D speed <=0.05 m/s continuously for 0.60 s; settling deadline is 4 s on either clock. Then issue one native LAND, cease setpoint publication and confirm native AUTO_LAND, landed and ordered disarm within 30 s.
4. Center bounds relative to the reviewed origin: north [-0.50,4.00] m, east [-0.50,0.50] m, down [-1.25,0.02] m; transit/settling height 0.8–1.2 m. The 2.50 m after the brake trigger is geometric recovery allocation, not a proven stopping distance.

The driver requests LAND for transit/settling timeout, speed or navigation-height failures while still holding valid Offboard authority. Lane violation, missing authority, invalid/stale input, clock failure, total deadline or publication-fence failure stops command publication and invokes the reviewed external recovery procedure. It does not promise LAND in every fault. Any recovery makes C1 a failed measurement; preserve it without an automatic retry.

## R1: separate approval after independent C1 review

Trial `calibration-r1-expiry-001`, domain **58**, profile `scenarios/calibration/r1_stationary_expiry.json`, recorder `scenarios/calibration/r1_recorder.json`. Do not automatically proceed from C1.

Climb with the same 0.20 m/s command, 1.0 m target and 0.60 s settling requirement. Keep north/east [-0.50,0.50] m and down [-1.25,0.02] m. Publish one final zero target/heartbeat and intentionally stop renewing its **250 ms steady-clock lease**. There must be no later control publications, including operator requests through that owner. Expected owner result is `planned_authority_expiry` with nonzero process exit, not successful mission completion.

Require native Position fallback within 2 s ROS and 4 s wall of the last heartbeat while clocks advance. After owner process/node exit, fresh armed/healthy/in-lane Position telemetry and an idle command graph, the separately approved named operator uses the in-tree capture helper for **one** native console LAND. The helper must run before its 30 s post-expiry cutoff, leaving 30 s for landing inside the 60 s recovery window. Its approval must bind the exact helper hash and `operator_console_land_authorized=true`. Require native AUTO_LAND, landed and ordered disarm within 30 s on both clocks after LAND. Neither Position mode nor missing setpoints implies automatic landing.

Use the command in the historical packet with `/reviewed_operator_capture.py` replaced by `/repository/examples/calibration_operator_capture.py`. Retain the helper's actual bytes and hash from the same frozen archive. A missing/expired approval, unexpected mode, competing publisher or stale telemetry must refuse the console command. No forced disarm or second ROS owner is included. Unconfirmed recovery permits only the explicitly approved cancellation of this isolated failed simulator, with failure evidence retained.

## Fresh disarmed checks: parent-operated only

Inside each independently isolated, frozen-source RTX container, set its exact reviewed revision and domain. These commands use `calibration_check`, no approval argument and no motion owner:

```bash
export CALIBRATION_IMAGE_SHA256=sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1
# C1 container: ROS_DOMAIN_ID=57, new evidence directory
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_check \
  scenarios/calibration/straight_lane_v1.json 30 "$FROZEN_REVISION" \
  scenarios/calibration/c1_stop_020.json ''
# R1: a separate fresh container, ROS_DOMAIN_ID=58, separate evidence directory
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_check \
  scenarios/calibration/straight_lane_v1.json 30 "$FROZEN_REVISION" \
  scenarios/calibration/r1_stationary_expiry.json ''
```

Both fresh checks completed with exit code 0 on the frozen source. C1: 44.032 s, 3,236 strict positions, 64 statuses. R1: 43.944 s, 3,270 positions, 65 statuses. Both had zero faults, pending samples and trial events, and actual RTX renderer logs. Independent full-rate raw audit decoded 87 statuses in each bag, all disarmed, and found no recorded control messages; both audits passed. Raw bag and audit hashes are bound in the release manifest. The initial R1 external-runner preflight failed for a missing dry-run output directory; its failure is preserved separately and the corrected runner did not change frozen flight source. [Native summary](../artifacts/taskforce_batch_008/native-summary.json).

For release require generated world/vehicle/source hashes, healthy disarmed/landed state, advancing clocks, original native timestamps, stable publishers, zero command publications, no recorder faults/pending samples and finalized immutable raw recording. Verify effective parameters and actual GPU renderer. Disarmed `recording_complete=false` and no brake/LAND events are expected, not flight success. Recheck each moving deployment immediately before any later approved attempt.

## Evidence and independent acceptance

Preserve full raw `observations/`, metadata/hash/integrity audit, strict recorder, owner output, publication events, command acknowledgments, source archive, manifest, parameter readbacks, independent observer, operator receipts and native console log. A JSON summary is not the raw record. Correlate unique zero-velocity and LAND commands with recorded event stamps and identities. Decode full-rate native positions, velocities, status, landed flags and resets; assess speed/lane/height throughout all phases, sampled trigger overshoot, input continuity, elapsed clocks and ordered disarm. Independently examine observer geometry; there is no measured Gazebo contact stream in this calibration composition.

The C1 raw command audit is `python3 -m examples.calibration_bag_audit calibration_recording.json observations calibration-bag-audit.json`. Analyze the strict record with `python3 -m examples.stopping_trial_analysis calibration_recording.json stopping-analysis.json`, then independently reconcile the result with the immutable raw bag. Do not promote an analyzer output directly into a reusable envelope: add reviewed clock, tracking, latency, repeatability and speed-coverage bounds tied to the intended deployment.

R1 audit is `python3 -m examples.calibration_recovery_audit calibration_owner.json calibration_recording.json operator-land.json observations recovery-audit.json --operator-preflight operator-preflight.json --console-log operator-console.log`. It requires real operator capture and raw native transitions; synthetic rehearsal receipts are rejected. Any missing/ambiguous evidence, bound breach or unconfirmed recovery fails the attempt.

The available prior evidence remains C0 flight, disarmed C1/R1 checks and synthetic recovery rehearsal. The latest S0 native run was also disarmed and adds no braking or armed-recovery measurement. C1 and R1 success would support only their exact measured conditions; S0 still needs its own launch/contact transition, landing/recovery geometry, reachable-speed bound, live prior lease and owner timing review. **This packet grants no S0, full-course, scan or hardware approval.**
