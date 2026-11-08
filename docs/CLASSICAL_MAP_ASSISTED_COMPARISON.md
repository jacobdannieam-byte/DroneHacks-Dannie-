# Isolated Classical-map comparison

> **Update, 2026-11-08:** Later executed experiment: a separate PINN simulator diagnostic completed the known course. The offline comparison and normal moving-release requirements below retain their historical scope; no generic D3/S4 qualification is implied. [Full result and Classical comparison](HYBRID_FULL_COURSE_RESULT.md).

Prepared 2026-11-08. Offline comparison implemented; **native moving release remains blocked**. This is a separate experiment, not a change to the observed-only default, protected Classical checkout, or native PINN eligibility.

## Decision and Notion context

The user selected simpler spatial mapping, waypoints and small context-gathering rotations. The [original roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) prioritizes a lightweight closed-loop predictor, geometry-aware scoring and one existing PX4 owner. It explicitly keeps DATT-inspired preview tracking and disturbance estimation in an optional research lane. The [strategy tracker](https://app.notion.com/p/411f393aa768816abd1bd8651fbb4a90) separates S4 prediction from MPC and says native S4 is not qualified. The [architecture synthesis](https://app.notion.com/p/411f393aa76881fa9a0eee681393ffb2) calls for a geometry-only baseline before added semantics. These sources support simplifying the experimental route layer without replacing PX4 or granting a predictor command authority.

The comparison default is now: permitted participant map -> Classical height-slab grid and waypoints -> bounded candidate forecasts -> independent checks -> existing owner only after release. Semantic reasoning and background registration are unnecessary for this comparison. Map assistance applies only to this isolated experiment. A participant prior is never labeled observed FREE. Partial yaw looks improve observations; they cannot expose the blind body origin or certify unseen space.

## Implemented scope

`python -m examples.classical_map_comparison --output <new-directory>` runs from the repository root. It verifies every protected artifact hash in `artifacts/strategy_comparison/classical_baseline.json`, copies regular participant files from the pinned successful run's `source.tar`, verifies map/mission bytes, then imports that isolated copy. Existing output directories are refused. Protected files are rehashed afterward.

The successful baseline is `ad31ae55ebf7cd3b2eaf57917537242831e3c752`, run `run-AdMalPxA`. The archive, not a potentially changed checkout, supplies the planner, follower, 5 cm grid, phone map and side-gap-first mission. It produces 22 legs, four checkpoint crossings and a 17.319 m planned path. The narrowest planned center clearance is 0.352446 m beside M3; that is only 0.002446 m beyond the Classical 0.35 m design requirement and is not a native uncertainty/stopping certificate.

Three comparison families use identical route/start/goal inputs:

1. Archived Classical follower, with a D0 forecast used only for diagnostic scoring.
2. Existing Hybrid D5 candidate generator with D0 prediction.
3. The same D5 generator with physics-informed residual checkpoints 11, 29 and 47, each verified against its training manifest. No retraining or silent fallback.

Each leg is sampled at its start with zero and 0.20 m/s synthetic initial speed: 44 matched cases per variant, 220 rows total. Each candidate is forecast for one second in 20 ms steps. The shared diagnostic scorer uses the participant-map geometry, Classical 0.35 m horizontal and 0.20 m vertical envelope, and rejects predicted speed above 0.50 m/s. It selects minimum terminal goal error among eligible candidates. This scorer is explicitly **not native D3**; it omits native uncertainty/reaction/stopping certification and emits no executable receipt. These are open-loop component results, not completion rates or forecast accuracy measured against flight data.

Separate compressed grid artifacts contain prior occupancy, distances and origin. The point/segment checks retain the Classical continuous geometry query rather than treating a free grid center as whole-volume clearance. No grid is inserted into H1. The evaluator uses batched distance queries; all diagnostic results match the previous unbatched run exactly.

`examples/waypoint_look_preview.py` proposes -20, +20, 0 degree offsets, at no more than 0.25 rad/s with 0.30 s dwell. Total scheduled yaw travel is 80 degrees, not a full revolution. Each look requires stationary state, clearance, fresh depth and permission to stop; a new captured frame is required at each heading. Previews omit pass-through points and the Classical planner's boulder-proximity stop. This is offline scheduling, not a live scan state machine or evidence that those prerequisites hold. The existing full-scan runtime is unchanged.

## Evidence and results

Evidence is in the sibling `physical-gazebo-evidence/` directory:

- `classical-map-comparison-001`: retained failed first attempt, incorrect positional frame constructor; corrected to keyword arguments.
- `classical-map-comparison-002`: initial completed candidate comparison.
- `classical-map-comparison-003`: grid and look-preview addition, preserved independently.
- `classical-map-comparison-004`: final vectorized geometry comparison and artifacts.
- `classical-map-tests-001.log`: complete repository suite, 797 passed before the look addition.
- `classical-map-tests-002.log`: 30 focused comparison/map-binding tests passed after the look addition.

All five variants had at least one diagnostically eligible candidate in all 44 matched cases. This does not mean every candidate passed, or that any full flight is qualified. Host p95 scoring was Classical 2.32 ms, Hybrid D0 17.79 ms, PINN 26.69–28.00 ms. PINN single-rollout p95 was 2.12–2.17 ms. These measurements include synthetic map scoring on Windows, not ROS/VIO load or Orin. No predictor superiority is established, and PINN's full scoring p95 exceeds the historical provisional 25 ms target.

## Bounded simulator trial packet — conditional proposal

Preserve the successful complete course as an archival reference. Do not replay it under an old approval or use its narrow M3 margin to justify a new owner. Use an independently reviewed wide segment of the same participant map, vehicle, sensors and launch configuration; if no such segment supports the complete envelope, use the separate calibration lane and clearly label the changed course. Never silently substitute a wider world into a matched-course claim.

Release order:

1. Finish the already documented C1 braking and R1 recovery measurements, or locate and independently verify exact matching evidence. The inspected disarmed recorder files do not contain those moving measurements.
2. Resolve contact-to-airborne floor handling with the full vehicle attitude envelope and justified directional errors. Preserve the early-climb refusal regression. Qualify the corresponding landing/recovery path.
3. Bind the exact participant-map permission, transform/epoch, surveyed route and stopping tube, source/image/vehicle/model hashes and live exclusion lease. Fresh observed obstacles veto the prior. Complete exact-source disarmed composition, watchdog, owner timing and recording checks.
4. Separately approve one Classical bounded reference segment, then one matched D0 segment. Start nominal translation at 0.20 m/s, absolute measured 3D speed <=0.50 m/s; at most 0.50 m displacement from the reviewed hover anchor, followed by settle, hold <=2 s, then ordinary LAND. Landing must reach disarm within 30 s. These are proposed bounds; absolute start, geofence, source and recovery operator must be populated from the released geometry package before approval.
5. Run PINN shadow on the matched streams first. PINN moving tests additionally require model-aware proposal, held-command and publication rechecks; the current D0-only gateway must not be bypassed. Freeze the chosen checkpoint before testing, never choose the best seed after viewing moving results.
6. After translation passes, separately propose one stationary partial look (-20,+20,0 degrees, <=0.25 rad/s, <=7 s total). Do not combine its first qualification with a first translational trial. Verify real capture-time depth for each settled heading, stable pose, current rotation envelope and new observations. No stop/look at pass-through or boulder-sensitive waypoints.

Every attempt starts with a fresh simulator reset and fresh disarmed telemetry, one command owner and registration off. Preserve raw bags, PX4 logs, configuration and source hashes, commands, measured state, model timings, gate decisions and all failed attempts. Stop the batch on the first failed condition. On rejection or expiry, use the already reviewed recovery composition while authority remains valid; if authority is lost, stop publishing and follow the qualified operator/PX4 recovery procedure. Do not force airborne disarm, invent a hold in uncleared space, extend the deadline, or change PX4 safety parameters to pass. Paired closed-loop comparisons must match seeds, starting pose, permitted map, mission, sensors, termination and scoring; missing identities remain missing.

No moving approval artifact is generated. The packet is concrete enough to review the experimental design, but not ready to authorize arming until the listed release inputs and native gates are complete.
