# Hybrid full-course Gazebo result and Classical comparison

Reviewed 2026-11-08. **PASS for the isolated map-assisted simulator diagnostic.**
The aircraft completed 22 post-launch waypoints, all four referee checkpoints,
and landed inside the pad with zero contacts. Native disarm was confirmed.
This does not qualify the normal depth/clearance-gated policy or hardware.

Branch: `codex/hybrid-model-flight`. Flown source:
`cfdffc79b0cb79dab6abd81d130f1e0f78715067`.
Later documentation commits are not the flown source. The separate
`hybrid-run-success` branch preserves the earlier deterministic C1 success;
it is not this PINN-controlled full-course result.

## What actually controlled the flight

1. **Fixed route:** the archived Classical phone map, side-gap-first mission,
   height-slab geometry and route planner supply the same 23 route points:
   launch plus 22 goals. The runtime checks the profile goals against the
   planner output. The PINN does not choose sections, reorder waypoints,
   discover a tunnel or invent a route.
2. **Native lifecycle:** deterministic code admits fresh native state, enters
   Offboard, arms, climbs toward 0.8 m, settles, and eventually brakes and
   requests native LAND. PX4 controls attitude, rates, thrust and motors.
3. **Model-controlled transit:** each planning cycle reads actual PX4 position,
   velocity and heading. Heading-rate estimation uses captured headings.
   Four position targets are generated: horizontal lead 0, 0.08, 0.14 or
   0.20 m toward the current fixed goal, clipped at the goal. Each uses up
   to 0.04 m of vertical correction toward the fixed height and holds the
   observed yaw. A horizontal-hold choice can still command vertical correction.
4. **Physics plus learned residual:** `physics_11.npz` adds learned acceleration
   corrections to nominal closed-loop dynamics. Each target is rolled out for
   30 steps of 0.02 s, a 0.6 s prediction horizon. The score is terminal
   distance to the goal plus 0.15 times terminal speed. The lowest PINN score
   determines the actual position/yaw command published to PX4.
5. **Fixed progression:** each goal requires distance <=0.06 m and speed
   <=0.05 m/s, sustained for 0.6 s. This applies to every goal, unlike the
   historical Classical follower's corner-only stopping behavior.
6. **D0 comparison only:** the same four targets are also scored using nominal
   dynamics without the residual network. Those scores are logged; D0 never
   takes over and the Classical follower never supplies a fallback command.

The network is a frozen 17 -> 64 -> 64 -> 4 MLP with 5,572 parameters. Inputs
describe state, command error, nominal response, yaw demand and timestep;
outputs correct three accelerations and yaw acceleration. Its checkpoint SHA256
is `e935d56dfd0a9cf898c15ab2611afe9525fa9507f859f813121a9e181ed23375`.
It was trained on synthetic dynamics, not on the newly exported real telemetry.
The real-data preparation still has only two overlapping candidate windows and
no held-out set. This is a local predictive command selector, not an LLM pilot
or a depth-perception network. No online learning occurred.

Source: [selector and lifecycle](../refnav/model_flight.py),
[full-course integration](../refnav/full_course_model.py),
[residual predictor](../refnav/dynamics/residual.py), and
[native owner](../examples/calibration_trial_owner.py).

## Which decisions differed

There were 1,822 computed decisions: **1,617 published and 205 discarded**.
Every discarded decision had reason `model_state_expired`; it was not executed.
All decisions have a verified hash chain and an explicit publication/discard
outcome. Independent replay matched model selections and raw PX4 commands.

| Published choice | Nominal horizontal lead | Count |
|---|---:|---:|
| 0 | Hold current horizontal position | 119 |
| 1 | Up to 0.08 m | 284 |
| 2 | Up to 0.14 m | 99 |
| 3 | Up to 0.20 m | 1,115 |

**11 published choices (0.680%) differed from D0. All chose horizontal hold
where D0 chose the 0.08 m candidate.** Because these were close to the goal,
the actual alternative target was only 0.49–3.62 cm farther forward. These
are small differences in braking/settling commands, not different corridors
or obstacle-avoidance routes. They do not establish that the PINN was better.

The audit's count of 14 differing selections includes three discarded
decisions. The table below includes only commands actually published. ROS
seconds are simulation timestamps, not seconds since takeoff; waypoints are
1-based post-launch goals.

| Decision | ROS seconds | Goal | PINN / D0 choice | Target difference, cm |
|---:|---:|---:|---|---:|
| 188 | 60.504 | 2 | 0 / 1 | 2.658 |
| 258 | 65.884 | 3 | 0 / 1 | 2.789 |
| 428 | 79.084 | 5 | 0 / 1 | 3.139 |
| 429 | 79.100 | 5 | 0 / 1 | 2.909 |
| 506 | 84.364 | 6 | 0 / 1 | 0.590 |
| 759 | 103.024 | 9 | 0 / 1 | 1.036 |
| 874 | 112.052 | 11 | 0 / 1 | 3.210 |
| 975 | 119.552 | 12 | 0 / 1 | 3.621 |
| 1293 | 144.864 | 15 | 0 / 1 | 2.863 |
| 1331 | 147.784 | 16 | 0 / 1 | 1.734 |
| 1749 | 178.308 | 21 | 0 / 1 | 0.486 |

For example, decision 188 gave hold a PINN score of 0.011145 and advance
0.011715. D0 reversed that ranking: hold 0.011384, advance 0.010600.
The model therefore changed the command. No alternate flight was executed
from that state, so this is not evidence that its choice improved the outcome.
Some score differences are very small.

[All differences with exact scores and targets](../artifacts/hybrid_full_course_success/published-pinn-d0-differences.json)
and [the full decision table](../artifacts/hybrid_full_course_success/decision-review.csv)
make this comparison inspectable. **D0 is not the historical Classical route
follower.** We cannot identify same-state Classical-vs-PINN executed decisions
from these two independent trajectories. A paired closed-loop comparison has
not been run.

## Comparison with the successful Classical run

Reference: protected `run-AdMalPxA`, source
`ad31ae55ebf7cd3b2eaf57917537242831e3c752`. All eight protected artifact hashes
were reverified unchanged. The fetched current `classical` branch is
`f4264a2`; it is not the revision used for these historical performance numbers.

| Measure | Historical Classical | Hybrid diagnostic |
|---|---:|---:|
| Checkpoints | 4/4 | 4/4 |
| First corridor, s | 10.66 | 14.90 |
| Maze, s | 37.30 | 65.97 |
| Tunnel, s | 47.73 | 90.44 |
| Chicane, s | 69.13 | 133.37 |
| Motors off / course time, s | 84.30 | 153.57 |
| Landed inside pad | Yes | Yes |
| Contacts | 0 | 0 |
| Recorded horizontal path, m | 17.974 | 17.481 |
| Peak truth height, m | 0.9084 | 0.8968 |
| Largest height-estimate error, m | 0.078 | 0.091 |

Hybrid took **69.27 s longer, about 82.2% slower**. The historical Classical
follower moves a target along the fixed route at 0.5 m/s, with a 0.65 m target
lead and corner tolerances of 0.05 m horizontally / 0.10 m vertically. It
passes through straight-on waypoints. Hybrid uses a much shorter maximum
lead of 0.20 m, settles at every goal, and pays inference/audit scheduling
cost. These controller differences plausibly explain much of the slowdown;
the two runs do not isolate the effect of the learned residual.

Both use PX4 stabilization and a known participant map. Both are simulator
runs with simulated tracking, not successful real stereo VIO flights.
Launch adapters, timing policy and stopping rules differ; this is a historical
comparison, not a controlled randomized performance benchmark. Horizontal
path length was recomputed using the same consecutive-XY-sample sum on both
published referee CSVs. A shorter sampled path does not establish greater safety.

## What the successful diagnostic did not test

- Fixed waypoint geometry, order, 0.8 m target height, candidate family,
  cost function, settle thresholds, and takeoff/landing sequence were authored.
- Depth was recorded but the depth-section adapter and assessments were
  excluded from the command loop. No depth-guided corridor/tunnel recognition,
  partial-look behavior, exploration, semantic reasoner or learned goal ranking
  controlled this flight. Referee checkpoint labels are evaluation output.
- Registration stayed off. Participant-map candidate checks were computed
  for diagnostics but could not reject a candidate. The geometric route still
  depended on the map; this was not a map-free flight.
- The normal clearance, navigation-box, height/speed refusal and inference
  timing limits were non-blocking in the separate diagnostic profile. Native
  health, reset/authority checks, fresh model input, recording, finite run
  deadlines and recovery remained. Normal non-diagnostic profiles retain
  their original restrictions.

## Log review and remaining timing/clearance work

The final owner result, model/raw-command replay, recording, bag integrity and
independent full-course referee all passed. Maximum observed native speed was
0.339191 m/s. Registration/fallback were false throughout. The video contains
199 actual overhead frames at 1 fps, with no dropped capture-queue frames.
The score/telemetry panel is an added visualization of recorded data.

Residual limitations in the successful logs:

- 205/1,822 decisions expired before publication and were discarded.
- 1,134/1,617 publications exceeded the normal 50 ms timing target. Maximum
  decision-to-publication delay was 100 ms; the diagnostic audit requires
  command input age <=100 ms. This is not a pass of the normal 50 ms gate.
- Decision computation: median 36.50 ms, p95 48.06 ms, p99 56.20 ms,
  maximum 106.29 ms, including decisions subsequently discarded.
- Five scheduler delays exceeded 300 ms; the largest was 581.90 ms. ROS time
  advanced during them. Two deferred clock samples were coalesced without
  extending the original wait's expiry.
- Ten published predictions would have failed the normal map-margin guard.
  The smallest predicted reserved margin was -0.001931 m. This is a violation
  of the design reservation, not a measured collision; referee contacts were
  zero. It must not be relabeled as clearance qualification.

The fixes separate native reception/clock progress from inference, distinguish
clock stalls from scheduler delay, replan expired commands, and coalesce
monotonic deferred samples. The separate diagnostic receiver permits a 300 ms
clock wait; normal ingestion retains 50 ms. Map-independent native health and
freshness were not replaced by invented state. No new flight was run for this
publication review. Full repository verification: **879 tests passed in 64.40 s**.

## Evidence and continuation

[Portable evidence package](../artifacts/hybrid_full_course_success/README.md)
includes the actual video, compressed complete decision log, audit, exact
differences, both referee trajectories, candidate/profile identity, final
review and small owner/audit reports for all 11 earlier attempts. Original
failed evidence is not overwritten or relabeled. The 1.27 GB final ROS bag,
ULogs, source archives and earlier videos remain in the separate workspace
`physical-gazebo-evidence/hybrid-diagnostic-full-course-001` through `-012`.
Their provenance and final raw-file hashes are retained in the package.

Next work is a paired Classical / Hybrid-D0 / Hybrid-PINN experiment with
matched route-following rules, followed by depth-guided section qualification,
timing improvements and justified clearance bounds. The original frozen
approval applies only to its recorded experiment; documentation and publication
do not authorize a new flight. Real-telemetry training and hardware transfer
remain separate gates.
