# Simpler waypoint integration: bounded trial review

> **Update, 2026-11-08:** Historical bounded-gate snapshot. Later C1 and full-course PINN diagnostic runs passed. The latter used separate explicit authorization and non-blocking experimental guards; it does not close the normal clearance-gated policy requirements below. [Full result and Classical comparison](HYBRID_FULL_COURSE_RESULT.md).

November 8, 2026. **C1 is ready for team approval; no motion is authorized.** R1's disarmed package passes but its moving release depends on C1 and independent review. Full waypoint-policy and PINN flights remain unreleased.

## Frozen candidate and evidence

Worktree: `simple-waypoint-integration`; branch: `codex/simple-waypoint-qualification`. Native candidate: **`afc0eb63b0a9be1b7ba67ec18db9cbc25a56b80c`**. Image: **`sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`**. Use this exact source archive, not whatever HEAD happens to become after documentation or further edits.

The sibling `physical-gazebo-evidence/simple-waypoint-qualification-004/` contains immutable `source.tar`, exact archive verification, finalized native C1/R1 bags and recorder reports, `full-tests.txt`, and `disarmed-audit.json`. The independent audit verifies identities, source/profile hashes, idle motion graph, zero raw flight-command publications, disarmed status, raw bag integrity/duration and clean recorder completion without pending samples. Flight-event completeness correctly remains false because no flight occurred.

| Check | Result |
| --- | --- |
| Full repository suite | 821 passed, 43.55 s |
| C1 native disarmed | PASS; 3,283 position samples, 65 status reports, 44.564 s bag clock coverage |
| R1 native disarmed | PASS; 3,279 position samples, 65 status reports, 44.384 s bag clock coverage |
| Both strict recorders | Zero faults, zero pending samples |
| Both command graphs and raw bags | No flight-command publishers or messages |
| Classical preservation | All eight protected baseline artifact hashes unchanged |
| Registration | Off; no registration component launched |

Earlier evidence remains in `simple-waypoint-qualification-001`, `-002`, `-003` and the earlier `classical-map-comparison-*` folders. Do not overwrite, merge or relabel failures as passes.

## Integration completed

- Exact archived Classical participant map, 5 cm height-slab grid, waypoint planner/follower and mission are shared by the isolated diagnostic comparison. Prior occupancy remains separate from observed FREE. Classical, D0 and three pinned PINN checkpoints each find a diagnostic candidate in 44 matched cases; this is not a closed-loop success rate.
- `scan_mode: partial` is integrated through the mission loader, course scheduler and real read-only scan runtime. It uses measured -20, +20, 0 degree headings relative to the stabilized anchor, <=0.25 rad/s, 0.30 s settled dwell, fresh capture-pose-matched depth at each heading and <=15 s total scheduling budget including stabilization. Total nominal travel is 80 degrees. Reset, stale depth/state, missing clearance, excessive drift/rate, wrong direction and excess travel refuse. The 7 s preview describes ideal motion/dwell only; runtime allowance includes stabilization and sampling. Scans remain shadow-only; native moving scan configurations still refuse.
- The archived Classical route remains a source of waypoint goals, not a grant of native clearance. No semantic reasoner, background registration or full-turn scan is required by the simpler experimental design.
- Fixed the deployment archive mismatch: historical calibration contracts pin a mixture of LF and CRLF files. Git attributes now preserve their exact reviewed bytes. No expected hash or review contract was weakened. Added an archive verifier and host calibration launch modes; a moving mode without approval refuses before launch.
- Fixed recorder startup by waiting for initialized, advancing ROS time before telemetry subscriptions. Fixed shutdown by closing telemetry subscriptions and draining already-received samples against the real clock within the existing 50 ms bound. No fabricated timestamps, larger clock fences or relaxed stale-data checks.

## Approval request: C1 only, one attempt

The exact existing profile is `scenarios/calibration/c1_stop_020.json`, trial ID `calibration-c1-020-001`, ROS domain 61. Its source/world/vehicle/profile/recorder/geometry identities are pinned in `disarmed-audit.json` and the retained native check. The world is the **separate 6 x 8 m calibration lane**, not a claim of phone-course performance. No map-assisted policy or neural owner runs in C1.

1. Fresh isolated simulator, native healthy landed/disarmed reports, idle command endpoints and strict recorder readiness. No retry after a failed condition.
2. Climb with <=0.20 m/s commanded vertical velocity to 1.00 m above the native launch origin. Require height 0.8–1.2 m and measured 3D speed <=0.05 m/s for 0.60 s.
3. Command 0.20 m/s north. At the first eligible owner decision at north offset >=1.50 m, publish zero velocity and timestamp braking. Translation timeout 12 s; brake/settle timeout 4 s.
4. Native NED bounds: north [-0.50,4.00] m, east [-0.50,0.50] m, down [-1.25,0.02] m. Actual 3D speed must remain <=0.50 m/s. Stop the batch on the first failure.
5. Ordinary LAND; ordered native landed/disarm within 30 s on ROS and wall clocks. Overall trial budgets: 60 s ROS and 120 s wall. The existing reviewed ephemeral landing parameter is `MPC_LAND_SPEED=0.3`; disarmed checks leave the deployed value unchanged, and moving mode verifies readback only after exact approval. Offboard-loss behavior remains unchanged.
6. On a fault, use the existing reviewed owner/recovery composition while authority and telemetry permit. On authority loss, stop publication and yield to PX4/operator recovery. Never force airborne disarm. The approval must name the recovery operator and explicitly permit bounded failed-simulator termination if landing cannot be confirmed; termination is a failed attempt, never a landing pass.

The packet includes `run_approved_c1.ps1`, an unexecuted runner that requires an externally supplied current exact-candidate approval and a new evidence directory. It verifies the source archive and key approval identities; the native owner independently verifies the complete approval, current telemetry, geometry, recorder and command graph before publishing. No approval JSON has been created. After approval, use only the frozen candidate and archive referenced above.

## R1 and policy release dependencies

R1 is prepared on the same frozen candidate in domain 62, using `r1_stationary_expiry.json`. After reviewed C1 evidence and separate approval, stabilize within the tighter north/east +/-0.50 m box, expire the stationary 250 ms owner lease, verify no further commands and native Position fallback, then execute the separately approved named-operator LAND with its captured native receipt. Preserve the 0.50 m/s actual speed ceiling and 30 s LAND-to-disarm limit. [Full recovery procedure](C1_BRAKING_RECOVERY_PLAN.md). The helper hash for this candidate is in `disarmed-audit.json`; do not reuse an old helper hash or approval.

The bounded waypoint-policy trial cannot honestly be marked ready yet. It depends on measured C1/R1 evidence, justified directional/attitude bounds through contact-to-airborne transition, qualified landing geometry, a reviewed prior/transform/exclusion lease, and owner timing under load. PINN motion additionally requires model-aware owner revalidation; existing D0-only execution restrictions remain. These requirements cannot be completed by inventing measurements in a disarmed run. The immediate approval gate is the C1 trial above, followed sequentially by R1, then the bounded waypoint segment. [Policy segment proposal](S0_BOUNDED_FLIGHT_PLAN.md).

The successful Classical checkout, historical corridor evidence and candidate rehearsals remain untouched. The separate integration worktree avoids overwriting concurrent edits on `code-integration-hybrid`.
