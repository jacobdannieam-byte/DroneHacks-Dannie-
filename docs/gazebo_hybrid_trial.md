# Local Gazebo trial

**Gazebo and PX4 ran disarmed on Windows with Docker Desktop and Ubuntu
WSL 2.** See the [native validation report](gazebo_hybrid_validation.md)
for results, open readiness gates and evidence locations. The
original classical results are a separate baseline.

The import [`f0497ad`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/f0497ad145441eeab4e30fd3d686417fa491225c)
preserves the 130 files in `simulator/`, `drone_code/team_nav/` and
`drone_code/README.md` byte-for-byte from
[`classical@690892f`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/tree/690892fc535350b00295f32042dc12e9358dc9ca).
The simulator assets include the course OBJ, generated-world sources,
OAK-D S2 model, drone-16 model generator, ROS packages, organizer code
and provenance, route driver, maps, missions and referee. There are no
submodules or generated simulator outputs in the import. The OBJ is a
regular Git blob. The integration changes are on
`integration/unified-sitl-readiness`; each run writes its exact checkout
SHA to `revision.txt`.

## Build the pinned environment

From a Linux or WSL shell at the repository root, with a working Docker
daemon (Docker Desktop WSL integration or a Linux Docker engine), run:

```sh
bash simulator/build_simulator_image.sh
bash simulator/gazebo_trial/build_gazebo_image.sh
docker build -f simulator/gazebo_trial/Dockerfile.hybrid \
  -t drone-simulator-gazebo-hybrid simulator/gazebo_trial
```

The upstream recipes pin PX4 **1.16.2**, Micro XRCE-DDS Agent **2.4.3**
and [`px4_msgs@392e831c`](https://github.com/PX4/px4_msgs/tree/392e831c1f659429ca83902e66820d7094591410).
The base is Ubuntu 22.04 with ROS 2 Humble. The overlay uses Gazebo
Harmonic and adds Humble rosbag tools. The imported recipe was originally
measured on an M1 Mac. The native validation report records the x86-64
build and stationary rendering rate.
Use Linux containers, the default native CPU architecture and the
software renderer first. Allow disk space for three image layers and
bag data. The trial launcher passes no hardware or GPU devices and
does not require host networking or a global ROS installation.

## Inspect first, with no flight owner

```sh
bash simulator/gazebo_trial/run_hybrid.sh --check
bash simulator/gazebo_trial/run_hybrid.sh --mode observe --seconds 30
```

The second command starts a fresh container with loopback only, a
nondefault ROS domain (37 by default), no device mounts or privileges,
and a unique evidence directory. It launches the imported Gazebo world,
drone 16 model, PX4 SITL, MicroXRCEAgent, sensors and independent referee.
It waits for an advancing clock, complete observation graph, fresh
landed/disarmed PX4 status and estimated health. No mission runs in
`observe`. The PX4 DDS domain is set to the ROS domain and the Agent
address stays on loopback. The graph also rejects manual, rate,
attitude, actuator and alternate command publishers.

| Input | Source and evidence |
| --- | --- |
| Depth | `/simulator_only/oak/depth/image` and `…/camera_info`. The imported Gazebo optical depth is little-endian `32FC1` in metres, 512 × 320, nominal 10 Hz, clips 0.35–10 m. The bridge verifies float format and layout, retains capture stamps and uses the actual CameraInfo. No-return values stay invalid. The Gazebo bridge offers reliable depth/CameraInfo; H1 requests best effort with a bounded queue. |
| Optical TF | `base_link ← oak_depth_optical_frame` on reliable transient-local `/tf_static`. The mounted lens is 0.131 m forward: 0.12 m camera mount plus 0.011 m sensor pose in the checked-in SDF. Optical forward/right/down map to FLU forward/right/down using the explicit rotation in the bridge. Verify it from the native TF and CameraInfo. |
| Clock | `/world/dronehacks_course/clock` Gazebo simulation time becomes ROS `/clock`. The report records ROS receipt time, source sample and publish stamps separately. PX4 XRCE sync is disabled by the imported SITL settings; H1 needs a measured source clock offset, error and epoch. Do not assume zero offset. Only this Gazebo owner path inverts the reviewed mapping for outgoing PX4 timestamps. |
| Simulated estimator | The existing delayed, jittered, drifting tracking emulator is remapped under `/gazebo_hybrid/simulated_estimator/`. Its status and capture-matched diagnostics explicitly identify a simulated estimator; health also requires fresh PX4 estimator flags. The existing organizer EV bridge feeds PX4. This is **not cuVSLAM** or a stereo/IMU evaluation. Raw truth and scoring go to the referee, not H1 or the planner. |
| PX4 | The launcher derives topic suffixes from the pinned message classes and records the actual graph. The Humble local-position message is unversioned in that image; do not use the managed Jazzy `_v1` local-position default. Sensor data is best effort; command ownership is checked at startup and each owner tick. |

The base settings use external vision for position and yaw, a simulated
downward range sensor for height, and disabled GPS fusion. The imported
Gazebo launch and referee record the known height issues. Review actual
active settings, landed reports, reset counters, frame registration and
covariance from `disarmed-*.json`, the bag and PX4 logs. H1 rejects stale,
unhealthy, mismatched or unreviewed inputs. Forward-only depth may leave
the starting vehicle envelope unobserved; a refusal is a valid finding.
Review legitimate additional observations before attempting translation.

## Load a reviewed profile and run shadow

Use [H1's profile schema](h1_native_observation.md) with
`tracking_source: "simulated_estimator"`, `odom` / `base_link` and the
observed optical frame. Keep the measured intrinsics, reviewed rigid
extrinsics, reset markers, map registration and uncertainty in that
file. Never guess them from a desired acceptance result. Use an
independently reviewed course file in the same directory.

To draft the *full course* profile offline from a map whose use is
permitted, substitute the clock and epoch of the H1 profile:

```sh
python3 simulator/gazebo_trial/draft_hybrid_course.py \
  --map drone_code/team_nav/course_maps/phone_scan_2026-11-06.json \
  --mission drone_code/team_nav/missions/dronehacks_2026_side_gap_first.json \
  --clock-id gazebo_sim --epoch 1 --output course-draft.json
```

This draft was reproduced with 22 stages: first corridor, maze/Gate A,
tunnel, chicane/Gate B, then the pad, plus connecting waypoints. It
remains unreviewed. Use a small separately reviewed profile first.
Check launch pose, heading, vertical origin, arrivals, tunnel and
landing approach. The alternate window mission is present, but no
Hybrid crossing has been attempted. D5 receives reviewed stage goals
and onboard observations; the portable runtime does not read the
simulator world, raw truth, or referee outputs.

After reviewing the source and course, generate the bindings and
run shadow:

```sh
python3 -m simulator.gazebo_trial.make_hybrid_profile \
  --h1 reviewed-h1.json --course reviewed-course.json \
  --profile-id gazebo-humble-shadow --output reviewed-shadow.json
bash simulator/gazebo_trial/run_hybrid.sh --mode hybrid_shadow \
  --profile reviewed-shadow.json --seconds 30
```

Review H1 freshness, semantic evidence, D0/D3/D5 decisions, rejected
candidates, deadline misses, RTF, CPU and RSS. The observer records ROS
capture age separately from wall receipt-to-decision latency. The
depth-camera image has no dedicated GPU passed through in this launcher.
Reports and bag data are under
`simulator/gazebo_trial/generated/hybrid_runs/run-*/`. PX4 ULog and
active settings are under `gazebo_generated/last_route_run/`. Referee CSV
and summary are separate from the autonomy bag. ROS issue stamps do
not establish controller consumption; use [Phase 5](unified_phase5_shadow.md).

## Flight modes, only after team review

The team reviews the actual isolated graph, shadow and fallback evidence,
and the exact trial. For `hybrid_sitl`, use the same profile builder with
`--mode hybrid_sitl`. The approval JSON must include:
`trial_id`, `reviewed_by`, `mode`, `revision`, `ros_domain_id`,
`world_sha256`, and a future UTC `expires_utc`. For Hybrid it also binds
the three SHA-256 digests in `profiles` (`launcher`, `h1`, `course`) and
requires `native_shadow_accepted`, `fallback_reviewed`,
`isolation_reviewed` and `course_prior_permitted`. These fields record a
team review; they do not substitute for it. The launcher checks network
isolation, fresh disarmed observations and the idle graph even with an
approval file. No approval is shipped.

```sh
# Historical classical route comparison, in a separate isolated run.
bash simulator/gazebo_trial/run_hybrid.sh --mode classical \
  --approval classical-trial-approval.json

# Exact approved Hybrid-only trial.
bash simulator/gazebo_trial/run_hybrid.sh --mode hybrid_sitl \
  --profile reviewed-sitl.json --approval hybrid-trial-approval.json
```

The classical mode runs the imported classical launcher and route
program. The Hybrid mode excludes that program, the organizer command
interface, and pretend RC. RefNav is the only PX4 command publisher.
The gateway's held-target, envelope, unknown-space and landing/yield
behavior is unchanged. During a trial, a reviewed operator can request
landing through the existing mission control topic inside the container:

```sh
docker exec RUN_CONTAINER bash -lc 'source /repository/simulator/inside_the_box/load_ros_environment.sh &&
  cd /repository && python3 -m examples.gazebo_container land --trial-id "$1"' _ APPROVED_TRIAL_ID
```

Use the printed container name and exact approved trial ID. Do not treat
stopping the container as landing. If the owner yields, the existing
PX4 failsafe handles the loss of Offboard. Before restarting, finish or
observe the abort, keep the logs, and use a fresh container and reviewed
epoch. To revert, launch `observe` or `hybrid_shadow`; neither starts a
command owner.

Progress only after reviewing each run: takeoff/hover/land, an observed
waypoint, a known obstacle and alternate or safe refusal, a verified
opening, then the full ordered course. Stop on unexplained failure.
The original classical Gazebo runs recorded four credited checkpoints
and pad landing, but failed the referee height check. The integrated
runner preserves height, touch, and course-boundary failures and
reports full-course credit separately from a partial trial.

## Verification from the original integration pass

- The original 414 tests reproduced. The final Hybrid suite passed 421
  tests and the imported classical pure suite passed 84. The new tests exercise the optical
  rotation and depth layout, simulation/command clock, explicit tracking
  source, delayed pose health, strict ownership and exact trial approval.
- All three imported map-to-world geometry checks passed. Native
  `gz sdf -k`, Docker builds, live topic/QoS, PX4 readiness and the
  classical and Hybrid flights could not run on this host.
- WSL Bash syntax and launcher checks ran. Without Docker the launch
  refused before creating any simulator or issuing commands.
- Native compatibility uses [Humble QoS event callbacks](https://github.com/ros2/rclpy/blob/humble/rclpy/rclpy/qos_event.py),
  [Humble bag recording](https://github.com/ros2/rosbag2/blob/humble/ros2bag/ros2bag/verb/record.py),
  and the pinned [PX4 DDS settings](https://github.com/PX4/PX4-Autopilot/blob/v1.16.2/src/modules/uxrce_dds_client/module.yaml).
