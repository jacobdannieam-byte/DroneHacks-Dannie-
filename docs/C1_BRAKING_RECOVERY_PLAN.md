# C1 braking and R1 recovery review package

**Disarmed preparation only.** C1 is one proposed 0.20 m/s northbound stop in the isolated calibration lane. R1 is a separate proposed stationary owner-expiry test. Each requires its own explicit approval, fresh run and evidence directory. The prior approval covered the single completed C0 hover and native landing. Its independent review is the prerequisite for this proposal. [C0 result](C0_FLIGHT_RESULT.md) · [Independent review](../artifacts/calibration_c0_001/independent-review.md)

## C1 deployment to review

| Item | Pinned input or review condition |
| --- | --- |
| Trial and owner | `calibration-c1-020-001`, `/calibration_trial_owner`, one attempt |
| Proposed recovery operator | Codex supervised by the requesting user in this chat; isolated simulator only |
| Image | `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1` |
| Runtime | Gazebo 8.15.0, ROS Humble, PX4 `54f0455ffcd755534539a7cf33a09a20bf71d29d`, px4_msgs `392e831c1f659429ca83902e66820d7094591410` |
| Isolation | Network disabled, localhost DDS domain 54, no hardware devices, immutable archive and separate empty evidence mount |
| Frozen candidate | `8a3d3f3ed0df1408e1665d45cb0d553b9a162ec5`, source archive SHA256 `32083fbeb699b8b74687ca1ffb8f6889fd6e5c63d82bf71c65085d1bdb594106` |
| Lane | `scenarios/calibration/straight_lane_v1.json`, generated world `d64be312bb5479c529acf2dd6ba559f50d4bf2dd224fbb31cd69be1b5f93dd07` |
| Vehicle and geometry | Generated model `f8cc362929337eb37e8649632ce3426d86a029a80932cee06a1a80f2d1e169f3`, geometry report `6582616dd24cb11488196bfc59bcd4476c685508a8866b410b91d334728fdaf3` |
| Profile | [`c1_stop_020.json`](../scenarios/calibration/c1_stop_020.json), SHA256 `cb0f11b36c8591eacfbdc832b8d080d70c4e6858931d2a7637764be6255ea76c` |
| Recorder | [`c1_recorder.json`](../scenarios/calibration/c1_recorder.json), SHA256 `4a3176d59aaba5901a4e835fcf3951d156e0104ccde5791a6d01f1af703dbbc9` |
| Release manifest | Include the current disarmed bag, source/model dependencies and named supervising recovery operator before requesting movement |

Use immutable archive bytes for the hashes. Compare the generated vehicle and world against the archived review. Map control, background registration and experimental policy selection stay off for this deterministic calibration. [Lane review](calibration_lane_v1.md) · [Generated geometry report](../artifacts/taskforce_batch_003/native-lane-reviewed-v2/geometry-review.json)

### Frame and fresh disarmed check

The authored Gazebo ENU transform maps `(east, north, up)` to PX4 NED `(north, east, -up)`. The simulated tracking path rotates its north/left/up representation into ENU and sends PX4 NED external-vision odometry. Review those source and launch bindings, including the generated launch origin, against the frozen image and candidate. Verify fresh native position near `(0,0,0)` against the launch mark and stable reset identity. C0 supplies an observed origin and vertical motion. Its small horizontal drift leaves the full horizontal transform source-derived. Heading and stationary samples alone cannot determine sign or scale. [PX4 bridge convention](../simulator/gazebo_trial/inside_the_box/run_trial.sh) · [Simulated EV conversion](../examples/gazebo_sensor_contract.py) · [ENU to NED conversion](../onboard/vio/frames.py)

Use the exact profile and contract in `calibration_check`. Require continuous fresh landed/disarmed and healthy data, an advancing clock, idle motion endpoints, stable native publishers, valid clock mapping, and a finalized raw bag and strict recorder with no faults. The start must be within 0.10 m of the origin and inside the specified box. The disarmed check uses no motion publisher and leaves effective `MPC_LAND_SPEED` at 0.7 m/s. An unexpectedly slow or unstable runtime is a reason to withhold release rather than extend a deadline during the attempt.

```bash
export CALIBRATION_IMAGE_SHA256=sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_check \
  scenarios/calibration/straight_lane_v1.json 30 8a3d3f3ed0df1408e1665d45cb0d553b9a162ec5 \
  scenarios/calibration/c1_stop_020.json ''
```

The native C1 disarmed composition passed on this candidate in domain 54. The finalized raw bag covers 43.86 s and the strict recorder contains 3,232 position samples with no faults or pending clock samples. Raw evidence is retained in the sibling workspace at `taskforce-native-004/c1-check`.

The future moving entry uses `calibration_trial` in the same invocation and supplies a new read-only `/approval.json` as the final argument. The artifact must bind the exact trial, candidate, image, world, vehicle, profile, geometry, recorder and domain, name the reviewer and recovery operator, set a short expiry, and explicitly authorize failed-test simulator termination. It must be created only after the user approves this attempt. The launcher and owner verify readback of the approved `MPC_LAND_SPEED=0.3 m/s` before command publication. Retain `COM_OF_LOSS_T=1 s` and `COM_OBL_RC_ACT=0` (Position). The landing-speed change is confined to this ephemeral simulator.

### C1 motion and acceptance

| Phase | Bound and expected evidence |
| --- | --- |
| Prime and climb | One reviewed owner from disarmed start. Vertical velocity command at most 0.20 m/s with zero horizontal command. Hover target 1.0 m above the native launch origin. Confirm 0.8–1.2 m and 3D speed ≤0.05 m/s for 0.60 s from advancing native samples. |
| Translate | Command `(0.20, 0, 0)` m/s in native NED. At the first eligible owner decision using north offset ≥1.50 m, publish zero velocity and record the actual ROS and PX4 issue stamps. If no trigger within 12 s on either clock, request ordinary LAND while still authorized and fail C1. |
| Brake and settle | Hold zero velocity and require measured 3D speed ≤0.05 m/s continuously for 0.60 s. The independent timing check requires at least 0.50 s after accounting for clock error and sample coverage. If settling exceeds 4 s on either clock, request ordinary LAND while still authorized and fail C1. |
| Land | One normal native LAND in place and stop setpoint publication. Require fresh native `AUTO_LAND`, landed and ordered disarm within 30 s of LAND. The existing watchdog can yield authority at any phase. |
| Whole attempt | Owner deadline 60 s ROS and 120 s wall. Estimated and independently observed 3D speed ≤0.50 m/s throughout. Native center north [-0.50, 4.00] m, east [-0.50, 0.50] m and down [-1.25, 0.02] m. The airborne navigation band is 0.8–1.2 m above launch. |

The 2.50 m between the trigger and north limit is a geometric recovery allocation. The trial records actual trigger overshoot, command-to-settle time and maximum three-dimensional excursion from the last pre-brake sample through LAND. Those measurements, reviewed clock and tracking error, and coverage across the intended speeds are needed before deriving any reusable stopping envelope. One successful 0.20 m/s command supports only its measured speed and conditions. [Stopping analyzer](../examples/stopping_trial_analysis.py)

Save immutable `observations/`, strict recorder JSON, owner JSON, native command acknowledgements, timestamped brake/LAND receipts, effective parameter readbacks, independent truth JSONL, startup and recovery reports. Independently correlate the receipts with a unique raw zero-velocity setpoint and LAND command. Decode full-rate native position, velocity, status and landed messages and compare them against the recorder. Check the northward velocity at the trigger, actual lane bounds and speed across all airborne phases, missing/future data, estimator resets, command order, settle interval and disarm. Check truth geometry for side or roof intrusion. This observer supplies sampled geometry. There is no measured Gazebo contact stream. [Raw command correlation](../examples/calibration_bag_audit.py)

Any limit breach, ambiguity, missing evidence, recorder failure or unconfirmed landing fails the attempt. Archive failures separately. A C1 result cannot authorize a retry, a higher speed, a policy flight or hardware movement.

## Recovery while running C1

An operator may request normal LAND through `/calibration_trial/control` using `{"trial_id":"calibration-c1-020-001","request":"land"}`. The current owner checks its authority and fresh telemetry before sending LAND. This is a failed or interrupted C1 measurement and is recorded as such.

On loss of authority, stale native data or a clock fault, the owner stops issuing commands. The native configuration requests Position after Offboard loss. Automatic landing is not the recovery assumption. After the owner exits, verify the idle motion graph, advancing fresh armed/healthy native state, vehicle inside the reviewed lane and no competing controller. The named and separately approved operator may issue one ordinary native console LAND. [Existing native console transport](../simulator/gazebo_trial/inside_the_box/run_trial.sh)

```bash
cd /opt/PX4-Autopilot/build/px4_sitl_default/rootfs
../bin/px4-commander land
```

Keep the bag and observer active through recovery. Look for native `AUTO_LAND` followed by landed/disarmed within 30 s of LAND. The supervisor observes for up to 60 s after owner exit. If recovery remains unconfirmed, terminate only this isolated simulator under the explicit failed-test cancellation provision. Preserve the failure, including native mode and last trustworthy position. No forced disarm or second ROS motion owner is part of this procedure.

## R1 stationary authority-expiry proposal

Review R1 separately after C1. Use the same image, world and vehicle with trial `calibration-r1-expiry-001`, localhost DDS domain 55, frozen source `fde3265ee151ce41ebf3fada16a20cc49b2b169e` and archive SHA256 `55bf99fc2fcbb4a539995e9ef92ab69721709d0431c4a4c2db1f655c913b036d`. The profile SHA256 is `d7f17360b5c6e437fd25a406aece54ab3d09534c350f8a66facd266b368cf4cd` and the recorder contract SHA256 is `2e8211d0c8a498149dd5af4bee8ab026e5174fdfd4fe228cde81fb22e4b57bbb`. Use an independent evidence directory. [R1 profile](../scenarios/calibration/r1_stationary_expiry.json) · [R1 recorder](../scenarios/calibration/r1_recorder.json)

Keep C0's tight center box north/east [-0.50, 0.50] m and down [-1.25, 0.02] m. Climb at up to 0.20 m/s and command zero horizontal velocity. At 0.8–1.2 m, fresh armed Offboard, no resets and native 3D speed ≤0.05 m/s for 0.60 s, the owner starts a 250 ms steady-clock lease with one final zero-velocity target. It stops renewal, records the final publication and deadline, then exits on expiry. Check the raw command graph and bag for zero control publications after the final pair. The fault leaves `/clock` advancing.

```bash
export CALIBRATION_IMAGE_SHA256=sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_check \
  scenarios/calibration/straight_lane_v1.json 30 fde3265ee151ce41ebf3fada16a20cc49b2b169e \
  scenarios/calibration/r1_stationary_expiry.json ''
```

Require the native Position transition after the local expiry and within both 2 s ROS and 4 s wall of the last heartbeat. Maintain the same 0.50 m/s actual speed and tight lane limits. The named recovery operator takes the manual console path above only after the owner has exited and fresh in-lane telemetry and idle publishers are verified. Require native landed/disarmed within 30 s on both clocks after LAND and apply the same 60 s post-exit recovery window. R1 qualifies only that exact stationary fault/recovery sequence.

The R1 owner exits with `planned_authority_expiry` and a nonzero status by design. The harness keeps recording while it waits for native recovery. The independent offline audit requires the owner JSON, strict recorder JSON, finalized raw bag, operator handoff JSON, original preflight probe and captured native console log. Record `trial_id`, `operator`, `command` (`px4-commander land`), `exit_code`, `owner_exited`, `idle_motion_graph`, `fresh_healthy_in_lane`, native `status_px4_us`, and `land_issue_ros_ns` and `land_issue_wall_ns` sampled in the same container. Include SHA256 fields `operator_preflight_sha256` and `console_log_sha256`. Keep the unmodified artifacts for an independent reviewer to verify the operator's declarations against their contents. The console uses PX4's native internal command path, so the audit binds the retained console receipt and independently observes the subsequent native mode and disarm. [Recovery auditor](../examples/calibration_recovery_audit.py)

### Operator handoff capture

Keep the flight driver on the frozen R1 archive above. Mount the separately reviewed capture helper read-only at `/reviewed_operator_capture.py` in the same isolated container. Its SHA256 is `3b35d95ec380c756d23bd169c2f83c1b31e374de1cb75a6dce22b81999e1986f`. A future approval artifact must include this exact `operator_capture_sha256` and `operator_console_land_authorized=true`, and name the operator who will make the supervised handoff. The helper compares that approval with the authorization artifact produced before R1 and the owner exit report. [Capture helper](../examples/calibration_operator_capture.py)

For the separately approved attempt, the named operator runs the following inside the same container and only after the owner exits. Use the exact name from the approval for `APPROVED_RECOVERY_OPERATOR`.

```bash
cd /repository
source simulator/inside_the_box/load_ros_environment.sh
export PYTHONPATH="/repository${PYTHONPATH:+:$PYTHONPATH}"
python3 /reviewed_operator_capture.py --execute-reviewed --output-dir /evidence \
  --owner /evidence/calibration_owner.json --authorization /evidence/calibration_authorization.json \
  --profile /evidence/calibration_profile.json --recording-contract /evidence/recorder_contract.json \
  --approval /approval.json --operator "$APPROVED_RECOVERY_OPERATOR"
```

The helper reserves the three evidence filenames exclusively. It verifies that the owner process and node are gone and the motion graph is idle. Fresh native telemetry must show armed Position, a healthy estimator, position and speed inside the R1 box, and an advancing ROS clock. Issue LAND before 30 s wall time from the recorded expiry, leaving 30 s for landing within the supervisor window. Just before the one console invocation the helper rechecks approval expiry, native report freshness and the paired ROS and steady samples. It retains the paired timestamps, native status stamp, preflight evidence, helper hash and combined console output. A refusal, timeout or ambiguous result ends this attempt with its evidence retained.

To rehearse the receipt file handling without a ROS session or console invocation, run `python3 -m examples.calibration_operator_capture --dry-run --output-dir` with a new empty directory. The files are marked synthetic. The native auditor rejects the synthetic receipt. The local rehearsal is retained in `taskforce-native-004/r1-operator-capture-dry-run` in the sibling workspace.

```bash
python3 -m examples.calibration_recovery_audit \
  calibration_owner.json calibration_recording.json operator-land.json observations recovery-audit.json \
  --operator-preflight operator-preflight.json --console-log operator-console.log
```

The pinned-image private rehearsal `python3 -m examples.calibration_recovery_probe` passed. It recorded one final zero-velocity diagnostic intent, no later intents and zero actuator publishers. The steady callback detected expiry 299.73 ms after the final intent, 49.73 ms after the 250 ms deadline. The synthetic operator receipt passed schema validation and a mismatched console digest was refused. No console command was executed. The original report is retained in `taskforce-native-004/r1-private/private-probe.txt` in the sibling workspace. The frozen driver suite passed 742 tests; final repository source including the capture helper passed 751 tests in 35.91 seconds. The final helper's pinned-image synthetic dry run passed and missing approval was refused before live setup.

The native subscriber-only owner check passed in domain 55 with an idle motion graph and no input fault. It observed a NED start of approximately `(0.00068, 0.00446, -0.01459)` m. Effective `MPC_LAND_SPEED` remained 0.7 m/s. The finalized bag passed SQLite integrity and required-stream checks with 44.808 s of coverage. The strict recorder retained 3,334 position samples and 66 disarmed status reports, with zero faults, pending samples or trial events. Its `recording_complete=false` is expected for this disarmed check, which did not exercise an armed-to-disarmed transition. The raw evidence is retained in `taskforce-native-004/r1-check` in the sibling workspace.

The capture helper and operator sequence are prepared and the dry-run file binding passed. The armed Position handoff and native console LAND remain unexercised future evidence. R1 requires its own approval before those steps. The C1 candidate does not include the intentional expiry trigger.
