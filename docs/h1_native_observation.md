# H1 native observation and replay

**Offline adapter verified. Native H1 qualification is incomplete.**

H0 PR #19 was open and unmerged at
`d49cbb3e2b8a8517d8bc18e06c1c4206ed3b3178`. H1 branches from that exact
head. Its parent is the managed readiness revision
`78ef9131b2ac539042b3bb7d63ea0666724a7fbd`. Before H1, all 351 tests
passed and `python -m examples.h0_hybrid_smoke` returned an eligible
offline proposal. The local environment is Windows build 26200, AMD64,
Python 3.11.9 and NumPy 2.3.5, with no `rclpy`, `rosbag2_py` or
`px4_msgs`. Workbench inspection stopped because Windows Computer Use
could not verify the browser URL. No native capture, flight or rosbag was
available for this run. [H0 contract](h0_hybrid_integration.md)

## Source inventory and native verification

These are the configured source names and reader expectations. Confirm the
actual types and offered QoS in Workbench before recording. For PX4 outputs,
the existing probe derives the suffix from the installed message version.
H1 takes the three PX4 output names from the reviewed capture profile.

| Configured topics | ROS message types | Role and source QoS |
| --- | --- | --- |
| `/stereo/{left,right}/image`, `/stereo/{left,right}/camera_info`, `/stereo/imu` | `sensor_msgs/msg/{Image,CameraInfo,Imu}` | Phase 1 evidence and cuVSLAM inputs. cuVSLAM uses sensor QoS for images and CameraInfo and best effort depth 512 for IMU. |
| `/camera/depth/image_raw`, `/camera/depth/camera_info` | `sensor_msgs/msg/{Image,CameraInfo}` | Phase 2/H1 inputs. The existing managed mission subscribes with sensor QoS. H1 requires a reviewed rectified optical stream with matching intrinsics and TF. |
| `/visual_slam/tracking/odometry`, `/visual_slam/processing_status` | `nav_msgs/msg/Odometry`, `diagnostic_msgs/msg/DiagnosticArray` | cuVSLAM publishes both with depth 10 and the default reliable QoS. H1 checks the visual result and its matching pose capture stamp. |
| `/estimator/healthy`, `/fmu/in/vehicle_visual_odometry` | `std_msgs/msg/Bool`, `px4_msgs/msg/VehicleOdometry` | Existing EV bridge health and estimator input. The health publisher uses depth 10. The EV publisher uses the bridge's PX4 QoS. H1 reads these for freshness and the EV reset marker. |
| `/fmu/out/vehicle_odometry`, `vehicle_local_position`, `estimator_status_flags` in the same namespace | `px4_msgs/msg/{VehicleOdometry,VehicleLocalPosition,EstimatorStatusFlags}` | H1 uses native NED odometry, validity and heading flags, and odometry and local-position reset counters. `/refnav/odom` uses the ROS receipt stamp and cannot establish PX4 sample time. |
| `/clock`, `/tf_static`, `/tf` | `rosgraph_msgs/msg/Clock`, `tf2_msgs/msg/TFMessage` | Simulation clock, static rig and dynamic TF evidence. The Phase 1 probe uses best effort depth 64 for its stream subscriptions and reliable transient-local for static TF. H1 validates the recorded static optical-to-body chain. |
| `/fmu/out/vehicle_status`, `vehicle_land_detected` in the same namespace, and the PX4 command topics | `px4_msgs/msg/{VehicleStatus,VehicleLandDetected,OffboardControlMode,TrajectorySetpoint,VehicleCommand}` | The observer checks landed, armed and navigation state. It discovers command publishers from the graph. The existing mission owns command publication. |

Use the installed PX4 message schema to verify NED pose and velocity, the
body FRD angular velocity and the `timestamp_sample`/reset fields. Record
the actual XRCE timing configuration because bridged PX4 timestamps may
already have an offset. A fixed offset in this replay is valid only over
its reviewed time window. [PX4 odometry fields](https://docs.px4.io/main/en/msg_docs/VehicleOdometry) ·
[PX4 local position fields](https://docs.px4.io/main/en/msg_docs/VehicleLocalPosition) ·
[PX4 ROS time synchronization](https://docs.px4.io/main/en/ros2/user_guide)

## Adapter and fault behavior

`HybridBagReplay.feed(topic, message, received_ns)` extends the existing
Phase 2 message reader. It uses the same depth decoding, CameraInfo check,
VIO covariance validation, bounded pose buffer and metric/semantic
WorldState. It verifies the optical-to-body extrinsics against recorded
static TF and matches the map's pose bracket to recent visual-tracking
diagnostics. A nonvisual or invalid tracking result revokes free evidence.

The adapter maps the native PX4 sample and publication stamps using a
reviewed clock offset and bounded error. The source sample time, mapped
capture, recorder receipt and assessment tick remain distinct in the
report. It preserves PX4 variances and rejects unsuitable covariance.
Reviewed pose and alignment uncertainty and the vehicle envelope reach
H0 through their existing contracts. The alignment profile describes one
registered, stable NED-to-map transform for the selected epoch. H1 uses it
at the mapped state capture within that epoch. Review registration and
its uncertainty from independent evidence before using a native profile.

The adapter keeps one pending depth capture, the latest PX4 state, a
bounded static TF tree, and at most 64 confirmed visual capture stamps.
Phase 2 also bounds pose history and depth report rows. An overloaded depth
slot, reference rewind, incompatible TF or changed reset counter fences the
selected epoch. Restart replay with a new reviewed profile and a clean
WorldState after such a fault. Missing brackets, invalid depth, unhealthy
or stale state, unavailable or stale geometry, and D5 deadline misses
produce explicit diagnostic reasons. H1 uses the existing 100 ms state
and 250 ms map limits and a provisional 300 ms health/visual limit.
Verify stream cadence against those limits in the native recording. The output contains diagnostic
candidate IDs and evidence. The adapter creates no ROS node or publisher and calls no
flight-control entry point.

## Native capture and replay

The local Gazebo test path has an explicit simulated-estimator profile,
separate from the native visual-tracking configuration below. Use the
[Gazebo trial runbook](gazebo_hybrid_trial.md) for its clock, bindings,
provenance and launch commands.

Use the signed-in, approved Workbench with the aircraft landed and disarmed
and flight programs stopped. Keep the SHA, simulator build and course,
ROS domain and package versions, camera configuration, reset evidence and
topic inventory beside the capture. The [readiness runbook](unified_sitl_readiness.md)
has the setup and disarmed procedure.

```bash
python3 -m onboard.diagnostics.phase1_probe capture \
  --require-idle --seconds 30 --window settled-disarmed --output phase1-settled

mapfile -t topics < <(python3 -c 'import json; print(*sorted(set(json.load(open("phase1-settled/capture.json"))["topics"].values()) | {"/tf"}), sep="\n")')
for topic in "${topics[@]}"; do ros2 topic info -v "$topic"; done > topic-qos.txt
ros2 bag record --use-sim-time -o h1-native-bag --topics "${topics[@]}"
# Stop the recorder after the reviewed bounded window.
ros2 bag info h1-native-bag > h1-bag-info.txt
python3 -m examples.h1_replay bag h1-native-bag --profile reviewed-h1.json --output h1-native.json
```

Use one epoch with advancing `/clock`, CameraInfo, static TF and EV/health
evidence before the first accepted depth. Keep raw bags and ULogs in the
team's evidence storage. The H1 reader requires the canonical Phase 2
names plus the recorded processing status, static TF and reviewed PX4
outputs. If a deployment uses different Phase 2 names, review the bag
mapping before replay.

The JSON profile uses these fields. Populate them from reviewed evidence.

- `spatial` is the existing Phase 2 `BagProfile` JSON including
  `record_clock: "ros_sim"`, `rectified: true`, reference and calibration,
  and the reviewed positive pose error bounds.
- `clock` contains `source_clock_id`, `ros_clock_id`, signed `offset_ns`,
  `max_error_ns` (at most 10 ms), `valid_from_ns`, `valid_until_ns` and
  `evidence_id`. Derive the mapping from clock evidence before replay and
  retain the evidence with the bag.
- `ned_frame`, plus `alignment` with `time_origin_ns`,
  `map_from_enu_yaw_rad`, `ned_origin_in_map_m`, `position_error_m`,
  `angle_error_rad`, `provenance` and `estimator_reset_counter`.
  The map frame, clock and epoch come from `spatial.reference`.
- `vehicle` uses `VehicleBounds` fields and `objective_m` is a bounded
  three-component map goal. Optional `mission_goal` is an existing
  `MissionGoal` object with `intent` and `target_entity_id`.
- `px4_topics` maps `vehicle_odometry`, `vehicle_local_position` and
  `estimator_status_flags` to their observed absolute topic names.
  `ev_reset_counter` and `local_reset_counters` bind the reviewed epoch.
  Local counter order is XY, Z, VXY, VZ, heading.

A replay exit of 0 means a diagnostic assessment was produced, including
an explicit D5 stop. Exit 1 means the last inputs could not reach H0.
Exit 2 records an invalid epoch, input, or unavailable dependency.

## Offline reproduction and performance

```sh
python -m examples.h1_replay fixture --output h1-fixture.json
python -m examples.h1_replay fixture --offset-rig --output h1-offset.json
python -m examples.h1_replay fixture --repeats 30 --enforce-deadline --output h1-host-timing.json
python -m pytest -q tests/test_h1_observation.py tests/test_spatial_bag_input.py tests/test_phase1_localization.py tests/test_hybrid_integration.py
python -m pytest -q tests
```

The H1 focused suite passed 63 tests and the full suite passed 371.

The first fixture sends ROS-shaped messages with rendered corridor depth and
co-located VIO and mapped PX4 estimates. It builds both maps, binds a semantic
goal and reaches D5. Unknown space near the body yields
`no_verified_progress`. The offset test rig supplies observed starting
clearance and a metric objective, producing an independently reviewed
`forward` proposal. The offset exists only in the synthetic fixture.
Default fixture runs use a fixed deadline clock. The timing command uses
the real monotonic clock to enforce the existing 25 ms planner deadline.

Host measurement on the Intel Core i9-14900HX, Windows build 26200 AMD64,
Python 3.11.9, NumPy 2.3.5, one BLAS thread. Thirty repetitions supplied
90 depth frames and 90 assessments.

| Host stage | p50 | p95 | Max |
| --- | ---: | ---: | ---: |
| Phase 2 depth processing and map update | 18.82 ms | 20.77 ms | 54.74 ms |
| H0 assessment including D5 and receipt validation | 13.11 ms | 14.03 ms | 16.04 ms |

All 90 assessments retained the conservative stop, with zero planner
deadline misses in this run. The measurement excludes DDS transport,
decoding from CDR, real sensor processing and concurrent Orin workloads.

Native stereo/IMU rates, depth/VIO age, DDS queue/drop counters, end-to-end
latency and jitter, moving RTF and Orin resource/thermal data remain
unmeasured for H1. The existing Phase 1 report measures observed rate,
age, gaps, pairing and disarmed RTF. Pair it with recorder loss events and
middleware or source sequence evidence before attributing gaps to DDS.
Moving RTF near 0.53 remains unresolved. Collect the moving and Orin data
only under the applicable approved procedure.

## Applied-command evidence and go/no-go

Keep publication, PX4 acceptance and controller consumption as separate
events. In an approved later run, correlate exact PX4/ULog versions and
logged `vehicle_status`, `vehicle_control_mode` and `offboard_control_mode`
with the ROS record. Compare the onboard `trajectory_setpoint` input and
the controller's `vehicle_local_position_setpoint` telemetry. Use them to
corroborate the selected controller path. Verify each effective
interval's start, end and owner against a controller-consumption trace
and the reviewed clock mapping.
If the pinned log cannot resolve those events, the interval remains
unverified and Phase 5 skips the comparison. H1 does not export effective
commands. [Phase 5 contract](unified_phase5_shadow.md) ·
[PX4 controller input](https://docs.px4.io/main/en/msg_docs/TrajectorySetpoint) ·
[PX4 controller telemetry](https://docs.px4.io/main/en/msg_docs/VehicleLocalPositionSetpoint)

- [x] Verify H0 ancestry, offline regression and the H0 smoke test.
- [x] Validate synchronized synthetic messages, the read-only H0 handoff,
  frame and clock faults, resets, freshness, tracking loss and overload.
- [ ] Capture a landed/disarmed Workbench window with native topic/QoS evidence.
- [ ] Replay a reviewed single-epoch native depth/VIO/PX4 bag through H0 and
  verify clock alignment, reset provenance, calibration and observed clearance.
- [ ] Record native timing and identify authoritative applied-command evidence.

**H1 go for offline review. No-go for native qualification.** After native
H1 acceptance, H2 can attach a bounded subscriber-only stream to this
normalization boundary, discard stale work, and run H0 in shadow at measured
rates. Compare decisions and failure outcomes against the recorded replay.
Revalidate any receipt against current evidence and record results for
resets, tracking loss, unknown geometry and budget misses. H2 and command
handoff are separate reviews.
