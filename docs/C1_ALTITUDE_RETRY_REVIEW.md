# C1 altitude-maintenance retry: approval packet

2026-11-08. **Ready for approval of one C1 retry; no moving retry has run.** Registration remains off. This packet does not authorize R1 or a waypoint-policy flight.

## Exact candidate

- Runtime source: `38a9a36195c5bdc8c3531e0d31908fb89e92a69f` on `codex/simple-waypoint-qualification`, in the isolated `simple-waypoint-integration` worktree.
- Image: `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`.
- Retry ID: `calibration-c1-020-002`; ROS domain 61.
- Profile remains `scenarios/calibration/c1_stop_020.json`, with its original limits and profile hash. The recorder contract changes only its trial ID so the retry is distinct from the failed attempt.
- Frozen archive, hashes, finalized native recording, independent audit and an unexecuted approval-only runner are in the sibling `physical-gazebo-evidence/c1-altitude-retry-001/` directory. Use that exact archive, not a later branch HEAD. No retry approval JSON has been created.

## Failure and correction

The failed C1 accepted height near the 0.80 m lower boundary, then commanded zero vertical velocity during transit. Estimated height crossed below 0.80 m at roughly 0.97 m north, before the 1.50 m brake trigger. The owner issued normal LAND and native disarm followed 9.58 s later. That attempt produced no braking qualification and remains preserved in `simple-waypoint-c1-moving-001/`.

For the C1 stopping mode only, the corrected driver requires position within **0.03 m of the 1.00 m target** (or one quarter of a smaller configured height tolerance) before starting the existing measured settling interval. Leaving that inner window resets settling. It continues bounded height correction while settling and translating: NED vertical velocity is the negative height error with a 1/s proportional gain, clipped to +/- the existing 0.20 m/s climb limit. Horizontal transit stays 0.20 m/s north. Combined commanded speed remains below the unchanged 0.50 m/s actual-speed abort limit; commanded speed does not guarantee actual speed.

The outer 0.80–1.20 m navigation band is unchanged and still triggers ordinary LAND on violation. The 1.50 m brake trigger, all-axis zero-velocity brake, sustained settling, 4 s brake timeout and 30 s landing timeout are unchanged. Altitude feedback stops at the brake event, preserving the measurement's three-axis zero-command contract. C0 and stationary R1 behavior are unchanged. This is a control correction, not an enlarged tolerance or a change to sensor/clock admission.

## Validation

| Evidence | Result |
| --- | --- |
| Full repository suite | 843 passed, 38.84 s |
| Follow-up checks after assigning new recorder trial ID | 52 passed |
| Paired synthetic first-order response rehearsal | Old driver: 7/12 complete, 5 height-band aborts. Corrected: 12/12 complete |
| Native disarmed exact-source check | PASS; healthy disarmed admission and idle command graph |
| Strict recorder | 3,291 position samples, 65 status reports; zero faults and pending samples |
| Finalized native bag | Integrity/duration PASS; 44.256 s advancing clock coverage |
| Independent raw-command audit | Zero flight-command publications; all recorded authority reports disarmed |
| Preserved artifacts | Failed C1 evidence manifest and all eight protected Classical artifact hashes unchanged |

The synthetic rehearsal uses response time constants 0.20/0.35/0.60 s and airborne vertical tracking biases -0.02/0/+0.01/+0.02 m/s. It is a regression model, not a physical uncertainty or stopping bound. Initial rehearsal attempts are retained: the first supplied NumPy scalars to an intentionally strict native-value contract; the second applied airborne bias while still idling on the ground. Both harness defects were corrected without changing the controller's guards. Their failed outputs remain separate from `offline-rehearsal-003.json`.

Regression tests cover centering, settling reset, upward/downward transit correction, the unchanged outer-band aborts, unchanged zero-axis braking and complete synthetic sequences. Disarmed tests validate composition and recording, not airborne performance. The moving retry is still necessary to establish C1 braking evidence.

## Proposed retry: same limits, one attempt

1. Fresh isolated 6 x 8 m calibration lane, exact frozen source/image and no background registration. Require healthy landed/disarmed native telemetry, idle command graph and fresh recorder readiness before arming.
2. Climb toward 1.00 m at commanded vertical speed <=0.20 m/s. Center within 0.03 m and sustain measured 3D speed <=0.05 m/s for the existing 0.60 s settling period before translating.
3. Translate north at 0.20 m/s with bounded altitude maintenance. Brake at the first eligible decision with north offset >=1.50 m. Preserve the 12 s translation timeout and 4 s brake/settle timeout.
4. Native NED bounds remain north [-0.50,4.00] m, east [-0.50,0.50] m and down [-1.25,0.02] m. Actual 3D speed <=0.50 m/s throughout. No widened height band, geofence, clock limits or deadline.
5. Ordinary LAND, native landed/disarm within 30 s on both clocks. Overall budgets remain 60 s ROS and 120 s wall. The approved ephemeral `MPC_LAND_SPEED=0.3` behavior and unchanged Offboard-loss behavior remain as in the original packet.
6. Stop on the first failure. Preserve every raw bag, command/event record, PX4 log and failure. No automatic retry or R1. Use the existing reviewed recovery composition; no forced airborne disarm. Any unconfirmed recovery ends as a failed simulator attempt under the separately approved bounded termination procedure.

The named reviewer/recovery operator, current expiry, exact source/image/world/vehicle/profile/recorder identities and one-attempt scope must be supplied in the new approval. `run_approved_c1_retry.ps1` verifies archive identity and the retry's key identities; the native owner independently validates the complete approval and fresh inputs before any command. The prior approval binds the failed source and trial ID and cannot authorize this retry.
