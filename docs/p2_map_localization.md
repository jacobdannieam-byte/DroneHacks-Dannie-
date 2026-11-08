# P2-B offline map localization

P2-B consumes an optional AlignedDepth from an accepted SpatialWorld update.
The detached depth and pose rotation are backed by immutable bytes. The
handoff retains calibration, observation ID, the source clock and epoch,
capture and receipt times, and the exact healthy pose bracket. Existing
callers get the original update behavior by default.

## Coordinate contract

Write T_A_from_B for a rigid transform taking coordinates in B to A.
For column vectors p_A = R_A_from_B p_B + t_A_from_B. R is orthonormal
with determinant +1. t is in meters along A's axes. The code composes
left to right as T_A_from_B.compose(T_B_from_C). Array rows are multiplied
by R transpose. The pose snapshot uses a rotation matrix, so no new
quaternion order enters the interface.

| Frame | Direction, scale and evidence |
| --- | --- |
| Reference scan R | Explicit frame ID and right-handed basis documented in the manifest. Source scale converts to meters. Independent scale evidence and a reviewed span are required before registration. |
| VIO odometry V | H1 SpatialReference, including gravity-aligned z-up axes, named body frame, clock and epoch. It need not share the scan's axes or origin. |
| Body B | H1 FLU body frame. H1 PoseSample stores q_wxyz and the position of B in V. PoseBuffer resolves T_V_from_B at the depth capture. |
| Camera C | Rectified optical x right, y down, z forward. The recorded CameraInfo and static TF contract define K and T_B_from_C. z is optical-axis depth in meters. |
| PX4 local NED | Separate H1 FrameAlignment with NED-to-ENU conversion, map yaw/origin, time evidence and reset counter. P2-B neither consumes nor produces that transform. |

The reviewed anchor binds the exact reference artifact hash, scan epoch,
VIO reference and validity start to T_R_from_V and a provenance ID.
The initial guess is T_R_from_V composed with T_V_from_B. ICP returns
T_R_from_B. A successful keyframe updates only the sidecar's map-relative
transform T_R_from_V = T_R_from_B composed with inverse(T_V_from_B).
Between keyframes, VIO propagates that diagnostic estimate with growing
age and movement error. Current and previous pose-error indicators and
sensor errors contribute to the reported heuristic. The position and
rotation fields are neither calibrated covariance nor certified margins.

## Quality and failure behavior

The baseline uses point-to-point ICP with proper rigid SVD updates, a
bounded local spherical crop, an upper correspondence distance, and a
finite set of translation and rotation starts. The fit gates on sample
count, distinct nearest neighbors, overlap, 30 cm spatial-bin coverage,
residual and convergence. A point-to-plane information check estimates
stable local tangent normals and rejects weakly constrained directions
such as a translation along an unbroken corridor. Similar fits from
separated starts reject as ambiguous. These are local tests. Geometrically
indistinguishable places outside the search, unsuitable scale evidence,
and some map changes within the accepted tolerances remain undetectable.

A rejected fit emits no pose and retains the previous transform only as a
bounded initialization prior. Later keyframes can recover inside that
prior's validity window. Tracking loss, a new epoch, calibration change,
clock rewind, an excessive observation gap or a prior timeout fences the
localizer. A new session needs a reviewed anchor. An older epoch or an
out-of-order capture cannot overwrite a good newer fix. Querying past
the hold interval returns an unavailable snapshot. MapLocalization v1
contains both references, pose, uncertainty indicators, fit diagnostics,
capture, last-success time and reason. Its status distinguishes accepted,
VIO-propagated, rejected and unavailable output.

## Evidence

Files include world_model/map_registration.py, map_localization.py,
spatial_input.py and spatial_pipeline.py, the deterministic tests and
examples/p2_registration_benchmark.py. The P2-B focused suite is
reproduced below.

~~~sh
python -m pytest -q tests/test_map_localization.py
python -m examples.p2_registration_benchmark --repeats 40 --output registration.json
~~~

An initial test found that normals estimated across corridor seams could
create spurious longitudinal observability. Tightening the planarity
gate rejected this case. The focused suite covers rotation/translation
recovery, partial dynamic overlap, wrong map, bad initial pose, sparse
data, planar and corridor degeneracy, repeated features, capture-time
bracketing, map version, epochs, ordering, gap, tracking loss, expiration,
drift and reacquisition.

The seeded evaluator perturbs initial poses and adds 3 mm Gaussian
point noise. It alone holds the truth transform. On the same Windows
i9-14900HX host as P2-A, with one BLAS thread, all 20 positive fits met the
5 cm / 3 degree test tolerance. The worst errors were 0.69 mm and
0.048 degrees. All 20 negative cases rejected. The latency
p50/p95/p99/max was 87.82/142.43/148.89/149.22 ms for 40 attempts.
This benchmark uses matched synthetic samples and an idealized local scan.
Independent rendered-depth tests exercise a separate point sampling.
The timings motivate a multi-rate diagnostic path outside H2 planning.
P2-B is ready for offline review. A reviewed course scan and native
recording, broader alias and mismatch data, and Orin latency and
resource measurements remain open.

Final P2-B run with one BLAS thread passed all 455 tests, including 16
focused localization tests and 18 P2-A tests.
