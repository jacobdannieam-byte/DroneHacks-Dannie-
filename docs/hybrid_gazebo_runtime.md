# Hybrid runtime and Gazebo integration

The [offline goal comparison](goal_reasoning_comparison.md) adds an H1 `reasoner`
configuration (G0 default; G1/G2 shadow only) and inspection-specific metric
objectives. It does not close any native acceptance gap recorded here. An
`observe_more` goal no longer implicitly targets the opening center.

For the imported Gazebo assets and the integrated container launcher, use
[the current Gazebo trial runbook](gazebo_hybrid_trial.md). The earlier
runtime design and Windows-only evidence are retained below.

**Ready for offline review. Gazebo/PX4 flight remains unverified and disabled
in the shipped launch configuration.** The default mission is still classical.
No standalone Hybrid command publisher was added.

## Canonical history

The canonical branch is `integration/unified-sitl-readiness`. Its baseline
`78ef9131b2ac539042b3bb7d63ea0666724a7fbd` received the single H0–H3 squash
`675ed13f00b0e599153614b8acfbfcf58c0d9f41`. Its Git tree
`7ed1413ec7ac3817e1baa4207c6e0badfdfc84f0` matches the combined source at
`aa215e26947d5463318e1860c70eaf7c2b291e02` exactly. Historical sources:

| Scope | GitHub | Original head |
| --- | --- | --- |
| H0 contract | [#19](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/19) | `d49cbb3e2b8a8517d8bc18e06c1c4206ed3b3178` |
| H1 observations | [#20](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/20) | `1cb9c80dd6868a2d160769a0a919d32d75fb6c55` |
| H2 shadow and H3 gateway | [#21](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/21) | `aa215e26947d5463318e1860c70eaf7c2b291e02` |

[H0](h0_hybrid_integration.md), [H1](h1_native_observation.md), and the
[H2/H3 benchmark](hybrid_shadow_to_sitl.md) retain their original evidence.
This runbook supersedes the older instructions about attaching private H3
hooks: the new owner composition calls the guarded executor directly.

## Runtime contracts

```text
ROS message bindings / H1 → normalized state + immutable metric and semantic snapshot
     → reviewed ordered course / checkpoint → optional WorldState goal binding
     → bounded local metric objective → D5 / D0 / D3 → H0 review
     → short held target / independent recheck → existing RefNav PX4 owner
     → estimated vehicle state and fresh depth feed back into the next cycle
```

- **Classical** is the default. It uses the existing `main.py` and
  `refnav.core.Mission` route logic.
- **Hybrid shadow** runs the course policy and D5 over observations, recording
  diagnostic proposals only. It does not construct the mission ROS owner
  or send commands. Flight phases cannot be qualified in shadow.
- **Hybrid SITL** uses the same policy and one RefNav owner. The deployment
  entry point `run_reviewed_sitl` requires a reviewed profile, landing site,
  isolated nondefault ROS domain, a deployment-owned live isolation probe,
  an idle graph before construction, and exactly the expected owner after
  construction. A CLI choice, config field or sim clock alone cannot
  enable it. This branch ships no live isolation probe or enabled launch.

Observation ingestion follows sensor arrivals and retains only bounded H1
evidence. The planning ceiling is 8 Hz by default. The semantic decision
ceiling is 5 Hz, with a cache bound to the checkpoint and exact accepted
map. If the semantic binding is unavailable for a changed map, a held
semantic target is invalidated while the system waits for a new binding.
The owner timer remains at 20 Hz in simulation time. Scheduling or DDS
delays can reduce these rates; native cadence is still to be measured.

Configuration is split across the [simulator profile](../scenarios/hybrid_gazebo.template.json),
the existing [H1 profile](h1_native_observation.md) (calibration, clock,
reference, covariance and vehicle envelope), and the
[course profile](../scenarios/hybrid_course.template.json) (goals, stages,
arrival and recovery). `runtime` contains planner/semantic rates, wall
budget, and execution lease; `logging` contains the record limit. Map
resolution/age, WorldState/reasoning algorithms, D0 dynamics and D5
candidate/horizon policy retain the existing reviewed defaults in
`spatial_volume.py`, `trajectory_evaluation.py`, `refnav/reasoning/`,
`refnav/dynamics/` and `predictive_planner.py`. The launcher does not
provide overrides that loosen those checks.

## Mission and recovery

```text
INITIALIZE → PREFLIGHT → READY → TAKEOFF → STABILIZE_OBSERVE
        → PLAN_AND_NAVIGATE ⇄ VERIFY_GOAL_COMPLETION → SELECT_NEXT_GOAL
                        ↑                                  │
                        └──────────────────────────────────┘
        → MISSION_COMPLETE → LAND → DISARMED

OBSERVATION_REQUIRED / BLOCKED_PATH / NO_PROGRESS / ESTIMATOR_UNHEALTHY
        → REPLAN or verified stationary hold → MISSION_ABORT → LAND
        → AUTHORITY_LOST: yield through the existing owner, publish nothing
```

The existing owner keeps its takeoff, arming, mode, operator request,
watchdog and PX4 landing lifecycle. At a configured checkpoint the Hybrid
policy uses a reviewed map-frame point. A grounded semantic opening can
guide local ranking; D3 must still verify observed swept volume. A blocked
semantic target stops that path. If configured and reviewed, metric-only
fallback can cover unavailable semantic evidence; unknown clearance cannot
be bypassed. No automatic traversal through a dark tunnel is provided.
The current policy uses staged positions and cannot infer an unpublished
course layout or identify a landing pad from pixels. A landing checkpoint
without a reviewed site remains unknown, and an abort landing is logged as
an abort rather than successful course completion.

Two new state and depth observations must confirm arrival inside the
tolerance with uncertainty and the current vehicle envelope checked.
A reviewed detour is a bounded subgoal, after which the original
checkpoint is retried. Failed detours are not repeated. New observation
failures, elapsed goal/observation time and insufficient progress bound
retrying. On a reference/reset fault, invalidate motion and require a
reviewed new epoch before restart. After the last confirmed goal, request
the existing landing; only observed landed and disarmed status counts as
completion. The program cannot certify a vertical approach to an unknown
pad. Emergency landing and Offboard loss remain PX4 responsibilities.

## Held target and logs

Design A is implemented. The local goal is at most 0.50 m from the aligned
estimate. The executor rejects a target over 0.65 m from the current state,
vertical change over 0.25 m, yaw change over 0.45 rad, or estimated speed
over 1 m/s. It only accepts the existing position/yaw interface; no
velocity or acceleration feed-forward and no arbitrary time-indexed
schedule are sent. D0 retains its own forecast limits and PX4 retains
its existing controller settings. The H0 proposal must be consumed
inside its deadline. The resulting target expires on either wall or
simulation time (default 250 ms) and is independently checked against
current state, metric evidence, semantic binding, epoch, calibration
and owner generation on every use. No classical route target is used
during Hybrid navigation.

On a rejected/expired plan, only a freshly verified near-stationary
position hold can stream. A diagnostic STOP supplies no braking path.
If a safe hold is unavailable with confirmed authority, request the
existing landing and stop setpoint publication; PX4's existing watchdog
remains the fallback if the landing command is not accepted. With stale
authority, graph conflict or operator takeover, stop all publications
to PX4, including mode commands. The simulator must verify these
transitions before obstacle or course trials.

Structured records include strategy, course and owner phase, checkpoint,
semantic proposal/evidence, map clock/frame/epoch and ages, candidate
rank, D0 endpoint, swept-volume result, owner decision, PX4 state and
reset evidence, landing/abort causes and timestamps. Logs use a bounded
in-memory tail on the control executor and are written on orderly exit.
Use an independent ROS bag and PX4 ULog for full capture. A ROS send
timestamp and the outgoing PX4 timestamp field are not evidence of
controller consumption; see the [Phase 5 timing contract](unified_phase5_shadow.md).

## What exists in the teammate repositories

- [`classical` at `ef22d00`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/tree/ef22d001036eb30821bd65443ffa40e4c26f527d/simulator/gazebo_trial)
  has a Docker build recipe for **Gazebo Harmonic** and recompiles the
  PX4 1.16.2 bridge on its ROS 2 Humble base. It has no Gazebo course
  world/vehicle launch, sensor bridge, complete clock/TF graph, or
  executable Gazebo mission in that revision. Image build and launch
  were not run here.
- [`soham-stages` at `8ad7003`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/tree/8ad700332ccb22be77c98ba86f9c6b6714f8367f)
  and the inspected team simulator launch use PX4 **SIH** with
  generated point-cloud depth, truth-derived simulated tracking, an
  organizer flight interface and a referee. This is useful for
  comparison; it does not supply H1's calibrated depth image, visual
  diagnostic chain or Gazebo course. Do not run the organizer flight
  interface or the simulated RC owner alongside RefNav.
- The [checked-in teammate mission specification](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/blob/ef22d001036eb30821bd65443ffa40e4c26f527d/drone_code/team_nav/missions/dronehacks_2026.json)
  orders the first corridor, maze/Gate A, tunnel exit, chicane
  via Gate B or the window, and landing pad. The Hybrid template
  reflects this order, with an approach stage for the tunnel. It has
  no coordinates: course points must be registered to the estimator's
  reviewed map, and the referee remains independent. Consult the
  simulator owner for final course, crossing criteria and permitted
  prior information; checkpoint arrival does not prove referee credit.

The source adapter accepts native ROS messages of the H1 types and only
remaps topic names. It never translates frames, fabricates health,
remaps clock time or estimates pose from truth. It deliberately refuses
PointCloud2 as a depth image. It does not support a different flight
interface or PX4 command topic set: those require a separately reviewed
owner adapter.

## Run now, without ROS or flight

From the repository root:

```sh
python -m examples.hybrid_launch --check
python -m examples.hybrid_mission_smoke --output mission.json
python -m examples.hybrid_mission_smoke --unavailable --output blocked.json
python -m pytest -q
```

Development verification on the Windows host: 414 regression tests passed,
including 15 focused runtime tests. Compile and profile checks passed;
the incomplete template was rejected as expected.

The smoke tests feed rendered depth and synthetic ROS-shaped estimates
through the actual course, D5 and guarded owner logic. They use a fake
sink and scripted vehicle states, so they do not measure closed-loop
tracking. New output files are required. This template command should
exit 2 until real inputs are supplied:

```sh
python -m examples.hybrid_launch --config scenarios/hybrid_gazebo.template.json --check
```

## First approved Gazebo run

1. Obtain the exact simulator revision and Gazebo launch/container,
   PX4/SITL airframe, controller settings, ROS and message versions,
   bridge/transport, /clock source and reset procedure. Confirm camera
   stereo/IMU suitability for cuVSLAM and depth outputs. If the
   simulator uses simulated sensor generation, keep any raw pose truth
   and referee feed in the independent evaluator.
2. Launch that simulator **without another command owner**. Verify
   disarmed/landed, health, topic types/QoS, optical TF and CameraInfo,
   depth encoding, VIO/PX4 estimator state, uncertainty and reset and
   clock mapping. Build the H1 file from this evidence; register a
   single observed waypoint and a verified landing site before the
   full course. Fill copies of the templates. There is no verified
   Gazebo/PX4 launch command in the available source.
3. Run the read-only graph check and a bounded native shadow window.
   On the pinned native host, with reviewed files and the correct domain:

   ```sh
   python3 -m onboard.diagnostics.sitl_graph --discovery-seconds 4
   python3 -m examples.hybrid_launch --config reviewed-gazebo-shadow.json --check
   python3 -m examples.hybrid_launch --config reviewed-gazebo-shadow.json --seconds 30 --output shadow.json
   ```

   Retain the bag, timing and profiles. Validate that H1/H2 keep
   current observations and reject missing/unknown/expired evidence,
   including the actual speed, jitter and message cadence. No native
   ROS2, Gazebo, Docker or PX4 executable was available in the local
   host/WSL tool checks.
4. Review command ownership, verified hold, landing and operator
   takeover. Build a deployment-owned isolation check and single
   executor binding for `run_reviewed_sitl`. Approve the exact
   isolated test (including fallback and transport), and check fresh
   authority and both clocks. Do not invoke a competing simulator
   controller. The shipped CLI refuses actuation.
5. Execute and review takeoff/hold/land, then one observed waypoint,
   one obstacle detour, one verified corridor/opening, then the
   ordered course and controlled landing/disarm. Compare ROS and
   PX4 ULog with referee data. Stop the sequence on an unexplained
   failure. Repeat with altered starts and injected observation,
   clock, watchdog and authority failures only after the initial
   successful run. Do not claim a full course without referee and
   landed/disarmed evidence.
6. For abort, use the rehearsed landing or operator takeover through
   the existing mission/PX4. Interrupting Python is not a landing
   command. Once disarmed, stop the composition, retain the logs and
   revert to shadow or the original classical launch with no Hybrid
   binding. Reset the simulator and create a new reviewed reference
   epoch before resuming; verify the command graph again.
