# S0 map-assisted integration and disarmed validation

Runtime source `e072f4538cea058523444624af3bc5b51da7e764`, branch `code-integration-hybrid`. Archive SHA256 `cce1b57647ea2231dceb7f112e85ca50a7d9476ce45666d73477d030d5b50d50`. No moving trial or approval artifact was created. All containers stopped after validation.

## Delivered

- Separate participant-prior clearance, finite ROS/wall exclusion lease, source/frame/reset and stopping-evidence binding, occupied vetoes and unchanged observed voxel evidence.
- Shared forecast and full-target stopping checks, including held targets and receipt identities. Predicted speed outside the measured envelope refuses; prediction is not a certified controller speed governor.
- Final-output revalidation with post-verification clock, state/depth, lease and target-expiry checks.
- Incoming launch fixes hardened: exact startup output and live alignment, complete conservative collision envelope, explicit deployment launch authorization. Collision resolver now refuses unsupported joint frames and articulated parent motion. Incoming diagnostic covariance provenance and candidate inflation counts retained.
- RTX WSLg/D3D12 option integrated into the existing simulator launcher; default EGL behavior preserved.

## Evidence

All **799 tracked repository tests passed in 37.49 seconds** on the frozen source. Independent and focused reviews covered map binding, occupied/unknown distinctions, reference mismatch, revoked leases, complete stopping tubes, startup output, stale publication inputs and collision resolution. An earlier working-tree run passed 807 tests, including untracked incoming tests; it is not the frozen-source result.

Fresh network-isolated, subscriber-only S0 run in domain 56 used the pinned image `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`. Actual OGRE2 renderer was `D3D12 (NVIDIA GeForce RTX 5070 Laptop GPU)`. Observed real-time factor was 1.000018. One GPU sample reported 2% utilization and 201 MiB; this is not an average or a measured speedup.

The native observed-only negative control had 96 records and four complete spatial assessments; all 32 candidate checks rejected occupied swept volume. The 25 `no_verified_progress` records include repeated mission refusal and are not 25 assessments. Other records included 41 stale-geometry, 16 awaiting-pose, 10 stale/unconfirmed-tracking and four missing-depth results. No fatal ingestion error or predictor fallback occurred. This is diagnostic sensing/refusal evidence, not continuous owner timing qualification or a successful map-assisted native motion decision.

Recording verification passed over 45.468 seconds. Read-only raw audit decoded 90 native status reports, all disarmed, and found no recorded command messages. All 13 sampled command graphs were empty. Raw bag SHA256 `b67dadbbf04b1cf8029b43872ae514d7364a84ce3ad9d8e502ffa89fc960679c`. Topic absence and sampled graphs have their usual observation limits; no command owner was attached.

The prior-backed positive path was validated with synthetic survey, depth/state and stopping fixtures. No deployment lease or invented stopping measurement was inserted into the native run. Evidence is under `artifacts/taskforce_s0_007/`; large raw data and preserved incoming snapshots are under sibling `taskforce-s0-007/`.

## Preserved failures and limitations

The initial suite caught a changed refusal reason: first-target stopping now rejects earlier in the inflated forecast. The regression was corrected to assert this actual refusal plus absence of a go setpoint, rather than require the old current-position diagnostic. The first raw audit invocation used a nonexistent ROS overlay path; the corrected `/opt/px4_messages/install/setup.bash` audit passed without altering raw data. Native build emitted a future-file-time warning but completed; native stream and recording checks subsequently passed. No failed flight is hidden; none was attempted.

S0 is **not ready for moving approval**. The launch regression demonstrates that the conservative body sphere still intersects the floor immediately after the landed flag clears. A justified attitude/contact transition remains necessary. Native landing/recovery admission, actual C1 braking and R1 recovery evidence, reachable-speed bounds, deployment survey/lease and owner timing also remain unresolved. These are explicitly retained gates, not inferred from unit tests or disarmed operation.

The [bounded flight packet](S0_BOUNDED_FLIGHT_PLAN.md) proposes one 0.50 m northward S0 G0/D0 segment from a stabilized 1 m hover in the wider lane, then native LAND. Scans remain disabled. Its proposed speed, position and time bounds require measured feasibility and enforcement before release. C1 and R1 each require separate approval; this packet does not combine their approvals or substitute the new source into their frozen configurations.

Additional incoming comparison/preview files appeared during this task and remain untouched/uncommitted by this batch. Protected Classic remains unchanged.

