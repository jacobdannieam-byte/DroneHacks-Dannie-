
# Phase 8 qualification protocol

Status is **protocol and evidence review ready, qualification open**.
This draft stacks on the Phase 7 rehearsal. Repeatable, independently
reviewed native Phase 7 acceptance is the prerequisite for scored runs.

## Trial design

The machine-readable protocol fixes five seeds, three layouts, five
navigation arms and five full-planner fault conditions. That is 75
nominal trials and 25 fault trials. The original obstacle course is
the reference layout. Archive its actual revision and calibrations
together with the derived offset-gate and narrow-corridor variants
before starting. Confirm feasible starting positions and physical
clearance using an evaluator independent of the autonomy stack.
The supplied course session and native run traces were unavailable
for this draft.

Use a seeded Python `random.Random(seed)` to draw, in order, initial
map X/Y offsets, heading offset and one transport-delay value uniformly
from the ranges in the protocol. Keep the drawn scenario, asset IDs,
mission scoring rules, fault injection time or waypoint, and native
device settings fixed for each matched layout/seed/fault key. Archive
the complete scenario configuration and use its digest across the arms.
The geometry and semantic baselines use the same geometric candidate
producer. D0 and ridge comparisons keep that producer and use the same
history and evidence rules. The full-planner arm uses the Phase 6 fan
and D0 as the selected model. Give each policy a fixed configuration
digest. Hold calibration, vehicle bounds, depth/estimator checks,
watchdog, landing path and operator takeover constant across arms.
Archive the first declared attempt in the scored matrix and keep retries
as separate evidence.

The proposed acceptance thresholds are frozen in the protocol for team
review before running. The full planner needs at least 14 of 15 nominal
completions, zero collisions, at least 0.10 m physical swept-body
clearance and moving RTF of at least 0.90. Score physical obstacle clearance during navigation and recovery. Grade intentional touchdown separately with the landing evidence. Nominal ages at motion are
bounded to 100 ms for state and 250 ms for geometry. Decision timing
requires at most 25 ms p95 and 50 ms p99, with at most 1% of decisions
over the 25 ms budget and zero motion based on late results. The dedicated
deadline injection deliberately exceeds the budget and is graded on the
absence of late motion and the verified watchdog outcome. VIO fault
timing starts when an explicit unhealthy report reaches the owner.
Other fault timing starts at the registered observation or missing-result
boundary. Score the safe-stop or operator-yield response separately from
the final confirmed landed/disarmed state.

Report mission scores, physical minimum clearance, collision counts,
native PX4 estimator validity and ROS/PX4 deadline counts, scene-graph
validity, and aligned dynamics error. Use physical scene truth only in
the independent simulator grader. Record p50, p95, p99 and worst on the
native graph and with stereo, cuVSLAM, mapping and PX4/ROS running on the
Orin. Archive device power mode, temperatures and power, CPU/GPU load,
peak RAM, capture delay and the raw profiling trace.

## Normalized evidence and review

```sh
# Empty preflight writes the open-gate report and returns exit code 1.
python -m examples.unified_qualification --output preflight.json
python -m examples.unified_qualification --evidence bundle/manifest.json --output review.json
```

The evidence manifest is a JSON object. Set `evidence_kind` to
`native_isolated_sitl` for independently checked native data, include the
report's `protocol_sha256` and the 40-character `build_sha`, then supply
`phase7_review`, `orin_profile` and `runs`. Artifact references are
`{"path": "...", "sha256": "..."}` relative to the manifest directory.
Archive actual bytes for the following:

- `phase7_review` carries `status="reviewed_pass"`, `build_sha` and
  `record` with signed or linked team review and successful native trials.
- `orin_profile` carries `device="Jetson Orin Nano 8GB"`, `build_sha`,
  `workloads=["stereo", "cuvslam", "mapping", "px4_ros"]`,
  `profile_trace`, `peak_ram_mib`, `mean_power_w`, `cpu_p95_pct`,
  `gpu_p95_pct`, `late_result_motion_ticks` and timing fields.
- Each run carries `layout`, `seed`, `arm`, `fault` (use `nominal` for
  routine trials), `build_sha`, `scenario_sha256`, `policy_sha256`,
  `rosbag`, `ulog`, and `grading_trace`. Preserve failed flights and
  the reasons in the native trace. Include `result` (`completed`,
  `safe_stop`, `yielded` or `failed`), `landed_disarmed`, `mission_score`,
  `collisions`, `min_clearance_m`, `moving_rtf`, `max_state_age_ms`,
  `max_geometry_age_ms`, `command_owner_violations`, `unsafe_motion_ticks`,
  `late_result_motion_ticks` and `invalid_scene_graph_uses`. Include
  `fault_response_ms` for injected faults and `dynamics_rmse_m` for
  predictive arms. Age values measure samples used for motion. Use zero
  only when no motion was authorized and preserve that fact in the trace.
- Timing fields are `p95_ms`, `p99_ms`, `worst_ms`, `decision_ticks` and
  `deadline_misses`. Count a miss when elapsed decision time exceeds
  the registered budget. Preserve the source timing series, p50 and
  watchdog events with the grading or profiling trace.

The reviewer verifies manifest consistency, artifact hashes, paired
scenarios, policy separation and thresholds. The metrics come from a
reviewed exporter and independent grade of raw logs. Artifact hashes check
integrity. Independent review establishes exporter correctness and trial provenance. A passing report opens independent review.
The checked-in preflight records zero native trials. The existing Phase 6
synthetic ablation supplies development evidence only.

## Release and hardware transition

For an accepted simulation release, archive the reviewed combined SHA,
native dependency versions, manifests, full scored matrix, misses and
failed runs. Link a demo from an accepted run alongside the raw evidence.
Keep the Orin workload receipt with the same build. Before separate
aircraft approval, verify a matching ARM dependency build, OAK-D
intrinsics and extrinsics, camera/IMU time calibration, PX4 message and
XRCE versions, frame alignment, physical body envelope and RC takeover.
Review power, thermals, failsafes and landing as independent flight
requirements. Retain the simulated release evidence separately from any
later aircraft qualification.
