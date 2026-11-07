# Unified Phase 2 — spatial and semantic evidence

**Offline implementation ready for review. Live Phase 2 acceptance remains
NO-GO.** The new path joins calibrated depth with estimated pose in replay,
feeds the existing WorldState, and builds a separate bounded 3D metric map.
The Phase 1 live gate is open. This branch has synthetic and recorded-message
fixture evidence. A native rosbag and target-device timing remain required.
[Unified roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) ·
[Phase 1 draft](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/7)

## Branch and interface

Base `feature/unified-phase2-spatial` on `feature/semantic-reasoning-i0-i1@b026465`
(PR #2, including PR #1). Reuse WorldState's extraction, association, aging,
contradiction and detached snapshot APIs. Keep Phase 1's native ROS diagnostics
and Phase 3's dynamics stack separate. Their shared integration point is a
reviewed capture/clock/frame contract and later a joint Phase 4 base.

| Boundary | Contract |
| --- | --- |
| Depth | One rectified pinhole optical stream, with a named camera frame, a named FLU body frame, immutable calibration ID, intrinsics, optical-to-body transform, calibrated dimensions and depth error bound. 32FC1 is meters. 16UC1 is millimeters. Zero, 65535 uint16, nonfinite and out-of-range returns have no free-space value. |
| Estimated pose | Gravity-aligned local z-up world, body FLU, position in meters and normalized `wxyz` quaternion. Capture and receipt are integer ns in a caller-bound clock. Exact healthy pose or a healthy pair already received by processing time. Position is linearly interpolated and rotation uses shortest-arc interpolation. Explicit pose error bounds and motion limits enlarge the bracket uncertainty. |
| Frame and clock | The same named map/body frames, clock and epoch apply to depth and pose. A new epoch clears both geometry and semantic memory. Wrong-reference packets are rejected. Detected clock rewind or implausible pose motion requires explicit reset. Delayed old-epoch packets cannot clear a newer map. |
| Metric output | Detached `MetricSnapshot` with fixed lattice origin, resolution, reference, query/capture times, calibration and observation provenance, and fresh occupied/free/unknown cells. Current consumers must supply the active reference and use a fresh snapshot. |
| Semantic output | Detached `WorldState.semantic_snapshot`. It retains stable semantic IDs, evidence and contradictions. P1 times are seconds relative to `semantic_time_origin_ns` in the current epoch. Reasoning consumes this detached view as before. Opening height and region connectivity stay unknown where the source model lacks evidence. |

The pipeline uses a 64-pose buffer, at most one pending depth frame in recorded
message replay, 80 ms maximum interpolation bracket and 250 ms capture-age
limit by default. No extrapolation. A health-loss marker invalidates existing
free evidence and fences future interpolation. A late bracket can recover a
previously rejected capture while it remains fresh. A reset discards prior
metric evidence and P1 beams/entities while preserving P1's ID allocation
semantics. The consumer-owned epoch may start on a new clock or map frame.

## Conservative geometry

The rolling default grid is 40×40×24 at 0.20 m. Origin changes preserve only
overlapping cells on the same global lattice. New cells start unknown.
Each map voxel is inflated for pose and extrinsic uncertainty and enclosed
in a camera-aligned box. The entire projected footprint must be inside the
camera frustum to qualify for free evidence. A one-pixel guard and outward
8×8 tile rounding enlarge the required footprint. All those pixels must be
valid and their minimum depth, reduced by the depth error bound, must exceed
the far side of the enlarged box.

**Free evidence is opt-in.** The depth calibration must declare
`validated_pixel_footprint: true` and explicit position and angular
extrinsic error bounds before any voxel can become free. This declaration
means the depth profile has evidence that the guarded pixel footprints
provide a depth lower bound under the reviewed conditions. CameraInfo,
depth encoding and a covariance matrix alone do not establish that.
The default leaves free space unknown. The synthetic fixtures exercise the
opt-in with known geometry. Thin or transparent obstacles, occlusion,
unvalidated pose or depth bounds, and real sensor coverage require separate
qualification. Point-depth samples do not establish a physical guarantee.

Occupied evidence uses the conservative range overlap between valid depth
returns and the inflated voxel. It can over-mark occupancy at discontinuities.
Current foreground occlusion or invalid returns revoke old free cells in the
affected view. Fresh occupied evidence wins conflicts until expiry and fresh
reobservation. Defaults are 0.5 s free age and 2 s occupied age, in the source
clock. Partial views and cells beyond the observed volume remain unknown.

The diagnostic sweep encloses the full vehicle in an explicit radius plus
margin and tests each continuous segment's expanded bounding box. The caller
must supply a radius enclosing the vehicle and propeller envelope and margin
for tracking and localization. Rotational extent is covered by the enclosing
sphere. Occupied evidence wins and unknown, stale or out-of-volume cells prevent
a free result. This deliberately rejects some physically feasible paths.
The camera's near blind region is not cleared by the drone's presence. A
verified path starting at the drone needs sufficient prior observed coverage.
Semantic opening width cannot override this query. Outputs carry no execution
authority.

## Reproduce offline

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m pytest -q tests
python -m examples.unified_spatial_replay synthetic --repeats 40 --output ../phase2-replay
python -m examples.unified_spatial_replay synthetic --skip-timing --output ../phase2-repeat
```

The fixture checks an observed corridor, overhead and floor interference,
a narrow gap, the shadow behind a wall and the camera's near blind region.
Additional tests exercise depth holes, sticky occupied evidence, rolling
lattice overlap, stale and malformed inputs, interpolation, health loss,
clock/epoch rejection and the unchanged reasoning adapter. The diagnostic SVG
shows two slices of the metric voxel map. The timing output is offline Windows
processing and includes pose ingestion and both map updates. The 640×400
fixture scales the existing synthetic raster. It excludes ROS transport,
camera processing, queue age and concurrent target load.

The semantic baseline passed 160 tests. Phase 2 passed 180, or 286 with the
Phase 1 and A1 source overlays. Two runs produced identical replay JSON and
SVG on this host. At 640×400, 40 frames measured 30.85 ms p95 and 33.69 ms
worst. [Replay](../artifacts/unified_phase2/replay.json) ·
[Map slices](../artifacts/unified_phase2/map.svg) ·
[Benchmark](../artifacts/unified_phase2/benchmark.json) ·
[Test evidence](../artifacts/unified_phase2/test_results.txt)

## Recorded rosbag path

A Jazzy environment with `rosbag2_py`, `rclpy`, and the pinned message packages
can run this read-only path. Use a trusted, single-epoch bag recorded with
`ros2 bag record --use-sim-time` and the actual native topics. The required
topic/type list is in the reader. Record `/clock`, depth image/CameraInfo,
cuVSLAM odometry, bridge health, and the EV output. Retain the full Phase 1
bag and ULog separately for source validation. Start the replay span with
clock, CameraInfo, EV epoch evidence and health before consuming depth.
A recorder clock disagreement, rewind, changed calibration or changed EV
reset counter rejects the selected bag span. Split across confirmed resets
and assign a new reviewed reference epoch.

```bash
python3 -m examples.unified_spatial_replay bag /path/to/recording \
  --profile /path/to/reviewed-profile.json --output phase2-bag
```

The profile contains `record_clock: "ros_sim"`, `rectified: true`, a
`reference` object matching `SpatialReference`, a `calibration` object
matching `DepthCalibration`, and explicit `position_error_m` and
`angle_error_rad` bounds. Record the calibration ID and use measured
optical-to-body TF and rectified intrinsics. Covariance and health sanity
checks can reject a sample. Reviewed bounds must come from outside those
checks. Leave `validated_pixel_footprint` false until supported by the depth
qualification. An optional `--sweep` JSON file contains `points_m`,
`vehicle_radius_m` and `margin_m` in the selected map. Output is a replay
report and map SVG. Exit 1 preserves the report for an unavailable final
observation. Exit 2 identifies invalid input or unavailable dependencies.
[Native topic reader](../examples/spatial_bag_input.py) ·
[Jazzy sequential reader](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_py/rosbag2_py/_reader.pyi)

## Acceptance gate

Require a reviewed native recording and reproducible map replay with real
CameraInfo, measured extrinsics and bounded capture/pose error. Verify that
unknown cells remain unknown under invalid depth, clock faults, stale
observations, occlusions and reset. Qualify the pixel-footprint model before
enabling free evidence on the native depth stream. Measure p95 processing and
input age under the declared 15 Hz depth load, including DDS and co-running
VIO, with a bounded queue. This session did not run a native bag, live graph,
SITL or Jetson. No Phase 3 code or flight-control path was changed.
