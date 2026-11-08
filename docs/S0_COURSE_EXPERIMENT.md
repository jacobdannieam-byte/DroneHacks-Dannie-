# S0 full-course simulation experiment

The user requested an end-to-end simulator run with qualification and calibration prerequisites waived, while retaining obstacle avoidance. This is an **unqualified simulation experiment**, not a measured-stopping demonstration, hardware authorization, or promotion of the reviewed Hybrid owner. Final experiment source and native outcome are pending; no successful full-course result is claimed here.

## What selects motion

The participant phone map and side-gap-first mission supply 23 ordered route goals at course height 0.8 m. The existing participant route planner supplies geometry only; the Classic `RouteFollower` does not control this experiment. The route is approximately 17.319 m and passes four referee checkpoint lines: first corridor, Gate A, tunnel exit and Gate B. Its final goal is the participant landing pad. Local waypoint arrival does not itself earn referee credit.

Existing S0 uses the actual G0 deterministic reasoner for suggestions derived from live semantic observations, existing D5 local candidate generation and D0 dynamics prediction for motion selection. G0 is an evidence-seeking reasoner: it can request observation, reject a blocked candidate or report insufficient information. It does not independently invent the full route. Logs must distinguish G0 influence, route fallback and no semantic event; no participant-map feature is relabeled as a fresh observed semantic entity. S2/S3/S4 and the Classic follower are not substituted to fill a failed S0 result.

## What the experiment changes

It waives prior flight qualification, measured braking-envelope admission and the calibration campaign as prerequisites for this specific simulator experiment. No fabricated stopping report, covariance or reviewed certificate is supplied. Retain native state/clock validity and command authority, ordinary PX4 stabilization, a single command owner, and explicit cleanup/termination of the isolated attempt.

Obstacle avoidance remains active: participant static obstacle geometry plus fresh native occupied-depth evidence veto candidates. Unknown sensor volume may rely on the explicitly labeled participant static map assumption; it is never written into the observed grid as FREE. Model dimensions and the obstacle reserve are nominal/source-derived assumptions, not measured tracking or braking bounds. Predicted and requested-target geometry must both be checked; takeoff and landing contact are distinct from ordinary airborne floor clearance. Intrusion coverage, sensor blind spots and map errors remain limitations of this experiment.

## Frames and profile

The native visual pose stays in `odom`, north/left/up relative to the authored spawn. H1 retains its native NED-to-odom yaw of -pi/2. Its translation is derived from paired native PX4 and visual-estimator poses after settling, rather than assuming both estimated origins coincide. Static participant geometry uses a separate transform:

`course_point = Rz(pi/2) * odom_point + [launch_x, launch_y, 0.04]`.

For the current map, the translation is `[-2.5146, 0.9144, 0.04]` m. Thus local `[0,0,0]` is the resting center 0.04 m above the course floor; local hover `[0,0,0.76]` is course height 0.80 m. Native +north advances course +Y; native +east advances course +X. Relabeling raw odometry as course coordinates without this transform is incorrect.

Build inputs from the fresh native disarmed probe in the experiment container:

```bash
python3 -m examples.build_s0_course_ablation \
  --probe /evidence/disarmed-1.json --output /evidence/s0-profile
```

The resulting launcher is `/evidence/s0-profile/s0-experiment.json`, using the explicit scope `s0_participant_map_unqualified_sitl`. It preserves real native sensor calibration and recorded PX4 covariance. It includes the participant map hash, route/mission source identity, static transform, G0 semantic requests with route fallback, and a nominal 0.02 m reserve. Scans are disabled. The base launcher remains `hybrid_shadow`; only the separate isolated experimental entry attaches command authority. It is not an accidental bypass of the ordinary `hybrid_sitl` loader.

## Evidence to retain

Retain frozen source/profile/map/mission identities, raw native recording, actual renderer, policy decisions and selected D5 candidate IDs, D0 predictions, G0 suggestions and fallback labels, obstacle rejection reasons, state/clock/authority failures, native landing/disarm, and independent referee checkpoint/contact results. A failed run is preserved as a failed experiment, not retried silently or renamed a success. A deterministic D0 rollout is an offline model diagnostic; it does not establish native sensor or flight performance. Final source, offline rollout outcome and native result must be appended after they exist.

Offline route/profile checks passed **5 tests in 8.96 s**. The deterministic diagnostic uses the actual experimental assessment, `ExperimentCourse` arrival logic, D5 candidate reconstruction and D0 physics model. It completed all 23 route stages in 202 control iterations / 40.2 model seconds, selecting 155 forward candidates and one drift-check candidate. G0 ran 202 times on an explicitly empty synthetic semantic context and returned insufficient information; route fallback supplied the objectives. The metric layer remained UNKNOWN with zero observed-free voxels. This demonstrates model-level route/arrival continuity under the participant map; it does not exercise live occupied-depth returns, real scheduling, takeoff, landing or native referee scoring. The independently retained initial report is `taskforce-s0-rollout-009-initial.json` in the sibling workspace.

```bash
python3 -m examples.s0_course_rollout \
  --profile /evidence/s0-profile/s0-experiment.json \
  --map drone_code/team_nav/course_maps/phone_scan_2026-11-06.json \
  --output /evidence/synthetic-route-rollout.json
```


## Attempt 001 frame correction

Attempt 001 refused before arming because the initial PX4 estimate was still settling; its down coordinate placed the nominal mapped center below the authored floor. This was preserved as a refusal, not repaired by moving the floor or widening its tolerance. The new builder fits translation only: `visual_position_odom - R_ned_to_odom * native_position_ned`, at the visual capture timestamp using an exact native sample or a bracket no wider than 20 ms. It records both original positions, source timestamps, reset counter and probe hash. Native calibration and covariance remain unchanged.

The calibration window requires at least 0.4 s of advancing native samples within the latest 0.5 s, position-range norm at most 0.02 m, fresh local speed at most 0.05 m/s, and coherent native frame/reset identity. These are stationary-pair acceptance criteria, not measured flight-error bounds. The runner waits for settling and captures a fresh probe; a late measurement is not applied retroactively to the earlier drifting estimate. Runtime additionally checks the live initial consistency of the bound pair.

Offline use of attempt 001's late probe at visual capture 45.900 s produced translation approximately `[0.0032840484, 0.0037049508, 0.043725514]` m in odom. The authored course transform stayed unchanged. Six route/profile/frame tests passed in 9.13 s, including rejection of moving/unpaired probes and a full D0 route rollout with nonzero alignment. The paired late-probe diagnostic still completed 23 stages in 202 iterations / 40.2 model seconds; this remains synthetic route-fallback evidence, not a new flight.

Attempt 002 stopped in the builder before owner attachment: three PX4 source samples were 4 ms ahead of their callback's delivered ROS clock. The offline pairing check now uses the existing 50 ms ingestion bound and additionally requires the final captured probe clock to have caught up to every retained source sample. Original source/receipt timestamps are never changed. The preserved native regression rejects a 50 ms + 1 ns discrepancy and an unreconciled future source. Seven tests passed in 8.52 s. The settled attempt-002 probe yields an exact pair at 47.000 s and translation `[0.00103218894, -0.00033024829, -0.00309065654]` m; this is a new observation for that run, not reuse of attempt 001's correction.

The experimental owner now buffers the complete original PX4 readiness callback before changing vehicle state when the source is slightly ahead of delivered ROS time. It uses the existing 50 ms/32-message limits, releases only after source publication plus clock uncertainty is covered, and preserves original source/receipt stamps in its audit. Changed authority, contact or reset identity revokes the old admission immediately; conflicting payloads at the same source time, rewind, overflow and expiry fail closed. During an expected Offboard acquisition transition in the arm phase, it suppresses outputs until the pending status is accepted. This exception does not retain airborne authority after a mode loss. The normal `RefNav` owner remains unchanged; only the explicit experiment subclass uses this mixin. Fifteen focused owner-clock/runtime tests passed in 4.89 s; that result is offline evidence, not proof of a successful native run.

Attempt 004 then stopped before arming with a callback wait expiry. Retained successful releases show 4 ms source leads and roughly 7–24 ms waits, but the original report omitted the offending queued row; those entries cannot establish the failed row's exact timing. Expiry reports now retain the original receipt, source-ready and current ROS/steady timestamps. The experimental owner can use the existing dedicated ROS TimeSource service so slow spatial work cannot freeze its receipt clock. It attaches that actual clock only after node construction, leaving the node's private TimeSource separate; mission and spatial mutation remain on the main executor. Service exceptions block final publication and fail closed. Sixteen focused clock-service/owner tests passed in 0.52 s. The 50 ms queue limit remains unchanged: a genuinely late callback or prolonged main-executor stall still fails, rather than being restamped or silently admitted.
