# S0 first bounded segment: unreleased review packet

Status: proposed isolated-SITL experiment, 2026-11-08. **Not executable approval and not a full-course run.** New launch geometry checks and shadow waypoint scans do not close measured stopping, owner recovery, route-clearance or runtime timing gates. No approval artifact is created by this document. Protected Classic is unchanged.

**Concrete launch feasibility blocker:** the new conservative all-attitude body sphere is approximately 0.30 m in radius, while the resting body center is approximately 0.04 m above the floor. The contact exception requires a fresh native landed report and applies only at the reviewed resting contact height. Once `landed=false` during early climb, the sphere still intersects the floor and the guard correctly refuses startup output until the center is above the sphere radius plus applicable uncertainty. A clear hover endpoint does not resolve this intervening unsupported interval. The regression `test_unmeasured_attitude_envelope_blocks_early_airborne_floor_transition` preserves that refusal. S0 launch is therefore not ready. Closing this gate needs an independently reviewed contact-to-flight swept-body contract with justified attitude and tracking bounds, or a differently qualified launch composition; this packet supplies neither and does not enlarge the contact exception. Native LAND/recovery also needs separate qualified geometry and authority and is not authorized by the launch guard.

## Evidence audit and release order

| Evidence inspected | What it supports | What remains missing |
| --- | --- | --- |
| [C0 result](C0_FLIGHT_RESULT.md), `artifacts/calibration_c0_001/calibration_owner.json` | One approved native simulated takeoff, stationary settling, LAND and disarm | Horizontal braking, intentional expiry recovery and policy navigation |
| `artifacts/taskforce_batch_004/c1-disarmed/recorder-summary.json` | 3,232 samples, zero recorder faults/pending samples | `events=[]`, `recording_complete=false`: no brake/LAND/disarm measurement sequence |
| `artifacts/taskforce_batch_004/r1-disarmed/recorder-summary.json` | 3,334 samples, zero recorder faults/pending samples | `events=[]`, `recording_complete=false`: no armed expiry/Position/operator-LAND sequence |
| [Batch 004 native shadow](TASKFORCE_BATCH_004.md) | Real native depth reaches all four existing strategies; final ingestion run has no fatal clock fault | Positive whole-body motion clearance, owner timing, flight performance |
| [Waypoint scan](WAYPOINT_SCAN.md), commit `9e18401` | Bounded shadow scheduling and offline regression coverage | No motion authority; native `hybrid_sitl` rejects scan-enabled missions |

The inspected sibling `taskforce-native-004/` contains C1/R1 **checks**, shadow runs and private/synthetic recovery rehearsals; the only located native calibration owner flight artifact is C0. Historical landing measurements elsewhere are not a matched C1 braking envelope. This is a bounded evidence audit, not a claim that no other team-held artifact exists.

Keep the release sequence: **separately approved C1 -> independent raw braking review -> separately approved R1 -> independent raw recovery review -> S0 disarmed composition -> separate S0 approval**. Use [the C1/R1 packet](C1_BRAKING_RECOVERY_PLAN.md) for their exact frozen sources and bounds. A new source must be repinned and rechecked, not substituted into an old approval. C1 success at one speed is not a general stopping model; add measurements if S0's reachable speeds, delays, vehicle or controller differ. R1 supports only its tested stationary fault, so additionally verify the final S0 owner's failure composition and applicable recovery scope.

## Proposed first segment

Use a fresh isolated instance of the existing 6 x 8 m calibration enclosure, never silently substitute it for the phone course. Author a separate participant prior with explicit permission and completeness scope; the calibration world JSON's existing description expressly denies policy-free-space status. Its mere reuse is not permission to use it as clearance. The flight policy must not read live truth pose/contact/geometry. An external observer may independently grade those quantities.

All bounds below are **design choices awaiting measured feasibility**, expressed in native NED displacement from a freshly validated launch origin. Source-derived world axes alone do not prove the live local transform.

| Quantity | Proposed limit |
| --- | --- |
| Trial | `s0-bounded-segment-001`, one attempt, existing S0 G0/D0 only |
| Start | Landed/disarmed, horizontal position within 0.10 m of the reviewed launch mark; stable reset/frame identity |
| Takeoff | Vertical target 1.00 m above resting center; commanded climb at most 0.20 m/s; reviewed contact/launch path only |
| Stabilization | Height 0.8–1.2 m; measured 3D speed <=0.05 m/s and angular speed <=0.10 rad/s for at least 0.60 s of advancing fresh samples; timeout 5 s ROS and wall after entering the height band |
| Route | One goal 0.50 m north, zero east displacement, unchanged hover height; maximum requested horizontal speed 0.10 m/s; no other stage or checkpoint traversal |
| Transit deadline | 12 s on either clock; reaching the nominal goal does not bypass arrival/clearance checks |
| Airborne center limits | North [-0.25, 1.00] m, east [-0.25, 0.25] m, height [0.80, 1.20] m |
| Contact-phase monitoring | Height [-0.02, 1.25] m, with floor contact only under the separately reviewed takeoff/landing contract |
| Speed | Hard trial-failure ceiling 0.50 m/s actual 3D speed throughout, including ascent/descent; commanded speed is not a guarantee |
| Arrival | Fresh position including reviewed error inside a 0.10 m goal tolerance and 3D speed <=0.05 m/s for 0.60 s; timeout remains active |
| Finish | Ordinary native LAND in place; confirmed native AUTO_LAND, landed and ordered disarm within 30 s on both clocks |
| Attempt budget | 60 s ROS, 120 s wall; no retry, replanning to another goal, or automatic strategy promotion |

The 0.50 m/s failure ceiling is **not** a stopping qualification at 0.50 m/s. Release requires measured coverage of the entire reachable speed envelope before the speed monitor can react, including latency and tracking error, or a reviewed controller limit and smaller bound that the measured data support. A single commanded-0.20 m/s C1 trial cannot be assumed to provide that coverage. If the current S0 position-target owner cannot enforce the proposed command/trajectory bounds, this packet remains blocked; do not infer a velocity guarantee from short target distance.

The trial keeps scans disabled. A scan-enabled motion trial is separate because the current scan API is shadow-only. If later promoted, scan startup must follow stabilized hover and each yaw/hold must pass geometry, stopping and owner checks; 24 views still do not prove fresh whole-body clearance.

## Geometry and input requirements

The enclosure reference is course east [-3,3] m, north [-4,4] m, floor 0 and roof 3.048 m; launch mark (0,-2). With the existing authored transform, the resting center is 0.04 m above the floor. Those are design/source facts, not a physical survey. Revalidate the exact generated model: historical C0 radius was 0.303078 m; model changes can invalidate it.

For the proposed airborne center box the raw distance to side/rear/front boundaries is at least 2.75/1.75/5.00 m respectively, floor at least 0.84 m and roof at least 1.808 m, **before** body, uncertainty, stopping and initial-position allowances. These geometric distances are not stopping margins. The independently reviewed certificate must cover the full candidate path, tracking deviation and all-direction stopping tube, plus recovery, not just the center line. Subtract validated launch-offset uncertainty or incorporate it in the transform bound exactly once. Refuse if any inflated volume exceeds the certificate, intersects floor/roof or lacks current occupancy-control permission.

The new `SimulatorLaunchReference`/`bind_simulator_launch` work is scoped to isolated calibration startup and does not authorize route navigation or landing. Its current source is being integrated and must be frozen before review. Participant prior authority must use the agreed typed API with explicit identity, map/frame/reset binding, completeness bounds, transform errors and revocable occupancy-control lease. Never seed FREE voxels or manufacture sensor timestamps. Fresh observed obstacles veto any prior. Retain depth/estimator/clock/authority requirements and refuse stale or contradictory evidence. A default diagnostic uncertainty profile is not a reviewed flight bound.

[The schema example](../scenarios/fixtures/s0_participant_prior.schema-example.json) records the agreed certificate and separate lease fields inside a deliberately non-runtime synthetic wrapper. It supplies no real map, alignment, stopping or certificate hash and no occupancy verification. Its coordinates and uncertainty values are illustrative, not the released S0 geometry. The runtime must independently validate exact certificate bytes, participant source, alignment identity and the current external occupancy assertion; parsing this example is not an admission test.

## Exact release manifest: unresolved fields block approval

Freeze and independently verify all of the following together before requesting S0 movement:

- Final commit and immutable source-archive SHA256, image digest, ROS/PX4/px4_msgs versions, vehicle/controller/sensor identities and effective parameter readbacks.
- Generated world and resolved vehicle hashes; reviewed body/attitude bounds; participant-prior file/hash, approval scope, lease issuer and current lease; launch/landing contracts and transform/reference epoch.
- S0 G0/D0 strategy/model hashes, H1 profile, mission profile, native owner identity, owner timing bound, exact stopping report and its independently audited raw C1 evidence. Include R1 raw audit and applicability review.
- Exact recorder/observer configuration, fresh disarmed native probe and bag, clock mapping, exclusive command graph, recovery operator and reviewed handoff procedure, trial ID and short approval expiry.
- Bound implementation and tests for every limit above, including the source-to-final-publication path. Empty, stale or placeholder fields must refuse release.

Use a new evidence directory and an unused localhost DDS domain (proposed 56, subject to a fresh exclusivity check), network-disabled isolated container and no hardware devices. The old image digest in the C1/R1 packet is only a candidate until the final S0 composition confirms it. No executable S0 command or numeric hash is guessed while these inputs are unresolved. Do not generate an approval file from this design document.

## Acceptance and recovery

Before motion, require an offline positive typed-prior fixture plus stale/mismatched/occupied/out-of-domain refusals, unchanged observed-only behavior, no post-revocation target replay, and first/held/startup/publication checks. Then repeat subscriber-only native composition on the frozen candidate. Preserve failures separately.

During the attempt, any violated bound, stale/missing input, geometry disagreement, lease loss, reset, excessive latency, recorder failure or ambiguous completion fails the attempt. While still authorized and with reviewed landing geometry, the sole owner requests ordinary LAND. On authority loss it stops publication; configured Position fallback is not automatic LAND. Use only the separately reviewed recovery handoff after owner exit, fresh in-domain telemetry and an idle command graph. No forced disarm or concurrent replacement ROS owner. Failure cancellation applies only to this isolated simulator if explicitly approved.

Independent acceptance correlates raw commands, receipt/source clocks, target/lease identities, full-rate position/velocity/mode/landed/reset reports and native disarm with recorder and observer. Report observed-free versus prior-authorized volume separately. Passing this single segment would establish only that bounded S0 composition under its pinned controlled-SITL assumptions, not a full course, other strategies, real-aircraft readiness or observed-only navigation.
