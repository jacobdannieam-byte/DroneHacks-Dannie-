# Gazebo Hybrid validation

Validation began on November 7, 2026. The canonical branch and GitHub both reported
`66de6f390ff9c854a2ebb46ad99b63d7fe7e5de6` at the start of this pass. The
working tree was clean. The reference simulator and the flight procedure are
documented in the [Gazebo runbook](gazebo_hybrid_trial.md).

| Stage | Status | Evidence |
| --- | --- | --- |
| Host | WSL 2 available | Windows build 26200.9711, WSL 2.6.3, Ubuntu 24.04.4 x86-64, 32 logical CPUs, 15 GiB visible WSL memory and about 253 GiB free on the Windows drive. WSL exposes an NVIDIA GeForce RTX 5070 Laptop GPU with 8,151 MiB of VRAM, driver 592.15. |
| Container runtime | Ready | Installed Docker Desktop 4.98.0 per user with WSL 2 after the user selected local setup. Verified the installer signature, installer exit 0, and Linux x86-64 Docker Engine 29.9.2. Enabled Docker Desktop integration for the default Ubuntu distribution and confirmed `docker info` from Ubuntu. The initial audit found no competing Gazebo, PX4, ROS 2 or XRCE processes and no ROS domain override. |
| Offline and native world geometry | Passed | All three checked-in maps passed the geometry checks and the isolated image's native `gz sdf -k` check. |
| Offline Hybrid and owner gates | Passed | The baseline passed 421 tests. The final suite passed 423, including the new native JSON and Gazebo pose conversion regressions. The focused integration suite passed 9. |
| Imported classical suite | Passed | 84 tests passed. The historical classical referee height failure remains open. |
| Gazebo and PX4 startup | Running | Gazebo 8.15.0 loaded the phone-scan course and drone model, advanced the clock and produced depth. PX4 1.16.2 attached and exchanged telemetry with Micro XRCE-DDS. |
| Observation contract and WorldState | Native capture recorded | The first decodable capture was consistently landed and disarmed. It exposed a PX4 yaw frame mismatch and a clock/health cadence constraint. WorldState live acceptance requires a qualified observation profile. |
| Native shadow | Pending observation qualification and profile review | Decision rate, map and planner latency and stop reasons remain unmeasured. |
| Initial flight and full course | Pending native gates and trial approval | No takeoff, waypoint, avoidance, corridor or full course result was produced. Team approval applies to the exact isolated flight trial after live checks. |

Offline commands executed from the repository root:

```text
python -m pytest -q tests
python -m pytest -q drone_code/team_nav/test
python simulator/gazebo_trial/check_worlds.py
```

The classical suite used `drone_code/team_nav` on `PYTHONPATH`. Each
command exited 0. The console logs and initial revision record are in
`simulator/gazebo_trial/generated/hybrid_runs/run-validation-20261107T221253Z-9051fceb/`.
The host report and test logs are separate from native run artifacts.

The three launcher scripts passed `bash -n`. The following commands were
executed in Ubuntu WSL from the repository root before Docker setup:

```text
bash simulator/gazebo_trial/run_hybrid.sh --check
bash simulator/gazebo_trial/run_hybrid.sh --mode observe --seconds 30
```

The check exited 0 and printed `launch=false` with ROS domain 37 and the
phone-scan world. The observation invocation exited 127 with
`Docker is required; no simulator was started`. It exited before the
launcher created a native evidence directory. The attempt is in
`wsl-checks.txt` in the validation evidence directory.

The Docker install and engine checks are recorded in `docker-install.txt` and
`docker-engine.txt`. The first engine check passed on Windows and returned 127
from Ubuntu because WSL integration was off. Enabling the default Ubuntu
integration resolved that failure.

## Images and startup attempts

The three pinned recipes ran in Ubuntu WSL. The simulator image built with
exit 0. The first Gazebo build failed in the package install step with apt
exit 100 during an Ubuntu mirror DNS failure. The unchanged recipe built
successfully on retry, including the PX4 Gazebo bridge. The Hybrid overlay
built with exit 0 and imported `rclpy`, `rosbag2_py`, `px4_msgs` and
Gazebo messages and transport. Build output is in `simulator-build.txt`,
`gazebo-build.txt`, `gazebo-build-retry.txt` and `hybrid-build.txt`.
The Hybrid image manifest digest was
`sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`.

```sh
bash simulator/build_simulator_image.sh
bash simulator/gazebo_trial/build_gazebo_image.sh
docker build -f simulator/gazebo_trial/Dockerfile.hybrid \
  -t drone-simulator-gazebo-hybrid simulator/gazebo_trial
GIT_CONFIG_COUNT=1 GIT_CONFIG_KEY_0=core.autocrlf GIT_CONFIG_VALUE_0=true \
  bash simulator/gazebo_trial/run_hybrid.sh --mode observe --seconds 30
```

The temporary Git setting matches this Windows checkout's line-ending policy.
WSL Git otherwise flagged CRLF files as modified and the launcher returned
exit 2. The override kept the dirty-source check active and left global Git
settings alone. Every native observation ran with domain 37, loopback only,
no hardware device mounts and all container capabilities dropped.

| Evidence directory | Result and fix |
| --- | --- |
| `run-zfEKQAiG` | Exit 2 during source extraction. GNU tar tried to restore the checkout UID/GID in the restricted container. `1bdec0f8c80ff8362b3c174a59b7bb5d87953e70` extracts without restoring host ownership. |
| `run-wHtF57fP` | Colcon built four packages. The ROS CLI failed to import `examples` from the launch file. `72d7fca31301a9cfe3e9cbc0bfdf41c63f606183` puts the checkout on the launch process's Python path. |
| `run-dA1lPMEl` | Gazebo, PX4, XRCE and depth started. The observer could not write native PX4 `float32` fields as strict JSON. `c004ba0a439d9baa68ddd0b7a454b70422f892a2` converts NumPy scalars before serialization and encodes nonfinite readings as null. |
| `run-WMnqe6sw` | Exit 1 at the bounded disarmed preflight. The bag recorded 93.5 s of advancing simulation time. The estimator reported unhealthy throughout. |
| `run-4MOJNtS1` | Exit 1 at the bounded disarmed preflight. At `5406a6e8c9fd43e268ca6efb5215dd61c2685a6c`, PX4 yaw aligned and the local heading became usable. Land and estimator evidence never satisfied the combined freshness gate at the probe endpoints. |

## Native observation evidence

`run-WMnqe6sw/versions.txt` identifies Gazebo 8.15.0, ROS 2 Humble, PX4
`54f0455ffcd755534539a7cf33a09a20bf71d29d` (1.16.2) and `px4_msgs`
`392e831c1f659429ca83902e66820d7094591410`. The native startup logs report
the PX4 connection, XRCE data writers and a ULog. The initial PX4 log
reported missing EKF data and no GCS connection before startup completed.
Gazebo reported Ogre material and deprecated motor parameter warnings. The
probe measured about 1.00 sim seconds per wall second while disarmed. One
Docker sample reported 285% CPU (about 2.85 cores) and 919 MiB resident
container memory. The test used software rendering. These measurements are
from this stationary Windows/WSL run.

Rates below come from the native ROS bag, over each topic's own simulation
time window. Publisher QoS is from `disarmed-4.json`. Each listed topic
had one publisher. The probe requested best effort with depth 1, except
reliable transient-local static TF with depth 16. The bag and probe
received the expected streams. They do not establish DDS drop counts.

| ROS topic | Message type | Offered reliability and durability | Rate |
| --- | --- | --- | --- |
| `/clock` | `rosgraph_msgs/msg/Clock` | Best effort, volatile | 250 Hz |
| `/simulator_only/oak/depth/image` and `…/camera_info` | `sensor_msgs/msg/Image` and `CameraInfo` | Reliable, volatile | 10 Hz each |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Reliable, transient local | One transform |
| `/gazebo_hybrid/simulated_estimator/odometry` | `nav_msgs/msg/Odometry` | Reliable, volatile | 30 Hz |
| `/gazebo_hybrid/simulated_estimator/processing_status` | `diagnostic_msgs/msg/DiagnosticArray` | Best effort, volatile | 30 Hz |
| `/gazebo_hybrid/simulated_estimator/healthy` | `std_msgs/msg/Bool` | Reliable, volatile | 20 Hz |
| `/fmu/in/vehicle_visual_odometry` | `px4_msgs/msg/VehicleOdometry` | Best effort, volatile | 30 Hz |
| `/fmu/out/vehicle_odometry`, `…/vehicle_local_position`, `…/vehicle_attitude` | `px4_msgs/msg/VehicleOdometry`, `VehicleLocalPosition`, `VehicleAttitude` | Best effort, transient local | About 100 Hz each |
| `/fmu/out/estimator_status_flags`, `…/vehicle_land_detected`, `…/vehicle_status_v1` | `px4_msgs/msg/EstimatorStatusFlags`, `VehicleLandDetected`, `VehicleStatus` | Best effort, transient local | 1.05, 1.00 and 2.00 Hz |

The bag had 936 depth images. Native CameraInfo reported 512 × 320,
`fx=fy=317.2533`, `cx=256`, `cy=160` and zero distortion. Depth was
`32FC1` with 2,048 bytes per row. The latched transform was
`base_link ← oak_depth_optical_frame`, offset 0.131 m forward with quaternion
`(0.5, -0.5, 0.5, -0.5)` in XYZW order. It matches the authored 0.12 m
mount and 0.011 m lens offset and the optical-forward to body-forward axis
mapping. The estimator emulator published `odom` and `base_link`. Its position
covariance diagonal was 0.0004 m² and its orientation covariance diagonal
was 0.0001 rad². This evidence covers the simulated source. Map registration
and uncertainty still need review.

All 88 landed samples and all 176 arming samples indicated landed and
disarmed. The strict graph found zero command publishers. Some probe
windows reported no fresh landed sample despite that state because the
1 Hz land stream exceeds the probe's 300 ms freshness window. Native PX4
odometry changed reset counters during startup (2, 4, then 6), and the local
counters settled at `[2, 1, 1, 1, 1]`. The external vision reset counter was
0. Each future profile must bind the settled epoch it actually observes.

The decoded flags showed active external yaw and position fusion in 92 of
93 messages, no yaw fault and `cs_yaw_align=false` throughout. Local position
never reported a heading usable for control. The old bridge sent
`POSE_FRAME_FRD`, with a 90° external yaw at the authored north-facing
launch. PX4 1.16.2 deliberately leaves `yaw_align` false for local-FRD
vision yaw. The corrected adapter derives the change of basis from the
simulator's declared axes. H1 still uses the unchanged ROS odometry and
requires its own independently reviewed NED-to-map transform.
[Pinned PX4 yaw fusion](https://github.com/PX4/PX4-Autopilot/blob/v1.16.2/src/modules/ekf2/EKF/aid_sources/external_vision/ev_yaw_control.cpp)

The corrected run recorded 92.3 sim seconds and 924 depth frames. Every
complete preflight report showed `yaw_aligned=true` and zero flight-command
publishers. After startup, 8,694 consecutive local-position messages
reported a heading usable for control. All 174 vehicle status and all 87
land messages showed disarmed and landed. The latest
estimator flag sample alternated between 320 ms before and 680 ms after
the latest land sample. The simulated health bridge requires flags at
most 200 ms old and the preflight requires land evidence within 300 ms.
Those observed phases leave no overlap. Simulated health was true in
361 of 1,846 samples. A valid landed indication at
83.64 s was 804 ms old when health was true, so the probe correctly
refused it as stale. The fix addressed the frame defect. Completing the
startup gate still needs a validated evidence source with compatible
cadence. The acceptance thresholds are unchanged.

XRCE time sync was disabled in the active settings. Across 8,833 native PX4
odometry samples, bag receipt minus PX4 publication was zero for 6,326
and ranged from −8 to +100 ms. This measures the recorder's ROS time
relative to the PX4 publication stamp, with clock delivery and scheduling
included. It does not establish a reviewed offset or error bound. PX4
estimator flags also had a 1,008 ms maximum observed gap, beyond the
existing continuous H1 health-age requirement. Preserve the captures and
review native cadence and capture/receive alignment before shadow.

The per-run directories retain `revision.txt`, `run.txt`, `versions.txt`,
`colcon.txt`, `stack.txt`, `rosbag.txt`, `observations/metadata.yaml`, the
SQLite bag, `disarmed-*.json`, and the Gazebo/PX4 logs, active settings and
ULog under `gazebo_generated/last_route_run/` where available. The decoded
bag analyses and their reader are in the validation evidence directory as
`native-bag-retry.txt`, `native-bag-ev-aligned.txt` and `audit_rosbag.py`.

## Hybrid, authority and flight acceptance

Offline verified. Live ROS observation capture verified. Native H1
qualification, continuous WorldState/D0/D3/D5 shadow, and live command
handoff remain open. The existing offline tests cover expired proposals,
rejection, authority conflict and fallback. The observed empty flight
command graph verifies isolated observation behavior. Native ownership
with RefNav enabled, the watchdog in flight and operator landing have
no results from this run.

Initial closed-loop SITL and full-course SITL are unverified. The
takeoff/hover/land, waypoint, avoidance, opening and full-course trials
were not executed. No referee score or airborne clearance result exists
for Hybrid. The historical classical referee height failure stays open.

Next, qualify the clock evidence, native estimator cadence and the
landed/disarmed evidence window. Independently review the H1 calibration,
frame registration and a small course profile. Run native shadow and
record map health, unknown starting clearance and planning timings.
Team review of the exact isolated trial follows shadow acceptance.
