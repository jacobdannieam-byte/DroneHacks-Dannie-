# Unified Phase 1 — simulator and localization foundation

**Gate: NO-GO for live acceptance.** The offline observer and replay checks are
ready for review. ROS 2 Jazzy, PX4 SITL and cuVSLAM were unavailable together in
the accessible Windows and Ubuntu WSL environment. The earlier isolated
cuVSLAM maneuver passed its flight objective and failed timing at moving RTF
0.53. That is repository-reported evidence. [Unified roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) ·
[Native run record](../onboard/README.md)

## Branch inventory and decision

| Accessible heads at kickoff | Verified status |
| --- | --- |
| `main`, `Dannie-experimental` | Both `feb55b6`, the native companion baseline. |
| `baseline-original` | `c3c5df3`, the original import. |
| D0 → D1 → D2 → A1 | `e99e617`, `8f695cd`, `ed02901`, `c36918e`. PRs #3–#6 are open stacked drafts. D0 stays default and fallback. Ridge is the adaptive comparison baseline. |
| WorldState P1 → reasoning I0–I4 | `8f71328`, `b026465`. PRs #1–#2 are open drafts. The P1 fork point is `a254248`, one notebook commit behind its target. I4 is design evidence. |
| `soham-depth` → `soham-mission` | `7781aff`, `dd09b79`. Depth clearance probes and reactive mission stages. |
| `classical` | `1f60fb5`, separate course simulator and flight scaffold with emulated camera tracking. |

The remote exposed twelve heads and six PRs. The local checkout exposed seven
branches and one registered worktree. The six draft source branches matched
their remote heads. Local `experimental` is the earlier `a254248`. The
untracked WorldState checkpoint predated this work. There was no stash. Other
machines' unpushed work is outside the audit scope.
[Remote branches](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/branches) ·
[Draft PRs](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pulls)

**Proposed decision.** Base `feature/unified-phase1-localization` on
`Dannie-experimental@feb55b6`. Review Phase 1 independently because its
observer needs only the native interfaces. Keep the `classical` simulator as
a separate comparison fixture. Its camera-tracking emulation cannot establish
cuVSLAM behavior. Review depth-probe reuse separately from reactive mission
hooks and any command changes.

Land the dynamics and semantic stacks in order through normal review. Use the
semantic stack for Phase 2 and PR #6 for Phase 3 once shared contracts are
approved. Use a reviewed joint base for Phase 4 after the spatial and
predictor dependencies are ready. Keep the command-gateway review separate.

## Native architecture and audit

```text
Stereo + CameraInfo + IMU + rig TF
  -> cuVSLAM -> VIO odometry + per-capture diagnostics
  -> EV bridge -> PX4 external vision + estimator health topic
  -> PX4 EKF -> native NED local position/odometry/flags
  -> PX4 odometry adapter -> /refnav/odom + odom-to-base_link TF

Depth + CameraInfo + extrinsics -> existing RefNav depth/mission checks
Existing approved command owner -> PX4 Offboard and position/yaw setpoints
```

- The `vio` service profile starts cuVSLAM, EV bridge and PX4 odometry.
  PX4 SITL and the existing XRCE transport are outside the companion.
  Runtime configuration overrides hardware defaults such as
  `use_sim_time: false` and nominal 500 Hz IMU. G4 specifies 480 Hz IMU and
  15 Hz stereo/depth targets. [Services](../onboard/config/services.json) ·
  [Lab configuration](../AI_CONTEXT.md)
- cuVSLAM pairs stereo by exact capture stamp and initializes using CameraInfo
  and camera/IMU TF. The body FLU frame is the SDK rig. A returned pose can be
  inertial-only. Per-frame diagnostics distinguish visual, inertial,
  withheld and failed processing. [Tracker](../onboard/slam/cuvslam_lib_node.py)
- The EV bridge checks freshness, pose, ownership and health. It preserves
  `timestamp_sample` separately from publication. It uses FRD-world when
  startup heading is arbitrary and needs a reviewed heading reference for NED.
  EV is an estimator input. RefNav and the native example are alternative
  command owners for their respective runs. [EV bridge](../onboard/vio/ev_bridge.py) ·
  [RefNav](../refnav/node.py)
- The PX4 adapter outputs ENU pose and body FLU twist at ROS receipt time.
  It omits the original PX4 capture time and reset counters. The RefNav depth
  check uses latest attitude. Phase 2 needs a synchronized capture-time pose
  from an explicitly chosen estimated source.
  [PX4 adapter](../onboard/adapters/px4_odom.py)

## Read-only runbook

Use the reviewed native simulation companion. Leave the flight program
stopped for the settled smoke window. After the clock advances and VIO settles,
run these commands from the checkout while landed and disarmed.

```bash
python3 onboard/services.py status
ros2 node list
ros2 topic list -t
ros2 param get /visual_slam_node use_sim_time
ros2 param get /visual_slam_node imu_frequency
ros2 param get /visual_slam_node left_frame
ros2 param get /visual_slam_node right_frame
ros2 param get /visual_slam_node imu_frame
ros2 param get /px4_vision_bridge use_sim_time
ros2 topic info -v /fmu/in/vehicle_visual_odometry
ros2 topic info -v /fmu/in/trajectory_setpoint
python3 -m onboard.diagnostics.phase1_probe capture --seconds 30 --window settled-disarmed --output phase1-settled
python3 -m onboard.diagnostics.phase1_probe replay phase1-settled/capture.json
```

The observer uses installed PX4 message version suffixes and accepts repeatable
`--topic STREAM=/actual/topic` overrides. It uses bounded subscriptions,
records graph snapshots and writes `capture.json` and `report.json`. The
static TF subscription uses transient-local durability. The tool creates no
application publishers. DDS discovery samples only the visible domain.
[ROS 2 Jazzy QoS](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Concepts/Intermediate/About-Quality-of-Service-Settings.rst)

The report measures wall/stamped rates, gaps, clock RTF and stalls, exact stereo
pairs, static rig connectivity and baseline, processing p50/p95/p99/max,
EV-to-VIO sample-stamp matches, PX4 validity and resets, and observed publisher
ownership. ROS capture age uses the latest observed clock and includes callback
skew. Native PX4 stamps stay in their boot domain. The PX4 `cs_ev_pos` flag
reports intended fusion. Confirm accepted fusion and innovations with the ULog.
[PX4 flag definitions](https://github.com/PX4/PX4-Autopilot/blob/main/msg/EstimatorStatusFlags.msg)

Exit 0 is a metadata smoke pass, 1 is observed no-go, and 2 is invalid input
or unavailable runtime. The provisional screens are 30 s, RTF ≥0.95, fast
stream gaps ≤0.5 s, status/graph gaps ≤1 s, estimator/landing gaps ≤1.5 s,
at least 90% stereo pairs, visual tracking and EV stamp matches, and
300/50 ms capture age/future-skew screens. Record the rationale for any
profile-specific thresholds. A smoke pass alone cannot qualify a maneuver.

Keep exact launch/config hashes, PX4 and cuVSLAM build IDs, host CPU/GPU, bag and
ULog from the same reset-qualified run. Use the discovered topic map for
`ros2 bag record`, including raw stereo/IMU/depth, CameraInfo, VIO/EV,
`/clock`, `/tf`, `/tf_static` and native PX4 status/odometry/flags. Repeat
with lighter capture to measure observation overhead. Keep simulator truth
separate from onboard inputs.

Reproduce the existing isolated maneuver only with explicit approval, fresh
landed/disarmed telemetry and one command owner. Follow Reset world after
keyboard RC. Preserve the safe-refusal evidence. Compare settled and moving
intervals against camera rates, cuVSLAM frame times and host CPU/GPU load.
The historical RTF alone cannot identify a bottleneck.
[Native maneuver procedure](../onboard/README.md) · [Lab safety](../AGENTS.md)

## Shared interface proposals for review

| Boundary | Contract |
| --- | --- |
| Time | Carry integer capture and receipt stamps with source and clock ID. Distinguish ROS simulation, native PX4 boot and monotonic wall time. Cross-clock comparisons require a measured mapping with uncertainty and a valid interval. |
| Reference | Carry named frame, axes, clock and reference epoch, plus transform provenance and validity time. Increment the consumer epoch on a confirmed clock rewind, estimator reset, origin or source change. Observe PX4 counter wraps. Reject older epochs and clear only affected joins, maps and history when a valid new epoch begins. |
| Estimated state | D0-compatible local NED position (m), world velocity (m/s), wrapped yaw (rad) and heading rate (rad/s), with health, uncertainty and capture provenance. Rotate FLU twist into world. For verified north-aligned ENU use `wrap(pi/2 - yaw_enu)`. Body FRD z rate alone cannot give heading rate off level. Missing quantities remain unknown. |
| Effective setpoint | Absolute NED position and yaw over a half-open effective interval in the same reference and clock. Preserve publication, receipt and effective time separately. The existing RefNav publication stamp uses system time. Withhold adaptive updates if accepted-command coverage or response-delay semantics are unresolved. |

| Phase | Proposed interface and requirements |
| --- | --- |
| **2 — Live spatial and semantic** | Input depth with encoding, CameraInfo, calibrated optical-to-body extrinsics, capture stamp and a healthy pose interpolated at capture. Express metric depth and clearance in meters. Preserve map-frame transform uncertainty and epoch. Output bounded occupied/free/unknown volume and read-only WorldState. Invalid pose/time, stale or occluded evidence and unobserved height/connectivity remain unknown. Verify the full swept envelope and margins. At the G4 target of 15 Hz depth, keep the work queue bounded and measure p95 processing and input age against the capture period. |
| **3 — Adaptive prediction** | Input D0-compatible estimated state and effective setpoints with causal completed transitions. Output a trajectory, uncertainty and validity reason. Fall back to D0 for invalid history, reference, excitation or confidence. Use identical A1 comparisons and an untouched holdout. Gate on delay, nominal heading and tails across 0.2–2 s, with a provisional 10 ms eight-candidate allocation and subsequent concurrent Orin measurement. |
| **4 — Diagnostic evaluator** | Input bounded candidate schedules, aligned forecasts, 3D geometry/transforms and non-executable goal hints. Output feasible/unknown/rejected diagnostics, reasons, supported clearance lower bound, progress and uncertainty. Reject stale/cross-epoch inputs and preserve unknown geometry. Provisional geometry budget is 10 ms and total local planning budget 25 ms. |

## Acceptance

The tests replay synthetic observations and injected failures. Live Phase 1
requires a repeatable native graph and bag, verified calibration and clock
mapping, PX4 fusion/reset evidence, and an approved maneuver with fresh
telemetry and moving timing evidence under declared load. See the test record
for what was executed here. Phases 2–4 are interface proposals only.
[Verification record](../artifacts/unified_phase1/test_results.txt)
