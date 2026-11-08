# P2 spatial intelligence

## Architecture decision

**Status:** Proposed for offline review. **Date:** 2026-11-07.

PRs [#1](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/1)
and [#2](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/2)
introduced persistent semantic identity and bounded, independently validated
reasoner inputs. PRs [#19](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/19),
[#20](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/20), and
[#21](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/21)
established H0–H3 and were superseded by consolidation
[675ed13](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/675ed13f00b0e599153614b8acfbfcf58c0d9f41).
The isolated P2 stack targets verified remote integration head
66de6f390ff9c854a2ebb46ad99b63d7fe7e5de6. The original local checkout
c004ba0a439d9baa68ddd0b7a454b70422f892a2 contains two additional Gazebo
commits and a separate untracked note. The historical PR #21 reports
399 passing tests. The remote-base suite passed 421 and the original
local checkout passed 422 with Python 3.11.9.

Keep static reference samples, live occupied/free evidence, and semantic
hypotheses in separate representations. Use NumPy and the CPU SciPy
nearest-neighbor index in opt-in offline tools. Open3D would supply general
mesh tools and more registration algorithms, with a larger distribution
and dependency footprint. A NumPy-only search would avoid SciPy but would
require a spatial index implementation or repeated linear scans. The
vertex-only reader and SciPy index cover the point-cloud path with one
optional runtime dependency and no ROS or GPU requirements.

| Capability | Reuse or extension | Evidence and remaining gate |
| --- | --- | --- |
| H1 synchronization | PoseBuffer, DepthCalibration, DepthCapture and SpatialWorld own capture timestamps, health, TF-derived calibration and reset handling | Existing H1 replay and regression. Native profile and ROS bag still required. |
| Voxel evidence | RollingVolume rolls a bounded local grid and projects entire uncertainty-inflated voxels into calibrated depth. Missing returns revoke visible free space and fresh occupied wins over contradictory free. | Existing spatial tests cover aging, uncertainty, motion, contradictions, clipping and unknowns. Floor and ceiling follow the same 3D voxel rule. |
| Metric snapshots and trajectory checks | MetricSnapshot freezes classes and timestamps. D3 reuses conservative continuous segment boxes, sphere inflation and prefix counts. | Existing D3 tests, including unknown and out-of-volume rejection. Point and local surface queries can reuse the same evidence. |
| Semantic model | WorldState already retains IDs, evidence, uncertainty, age, bounded_by relationships, geometry and restart JSON. The reasoner consumes a bounded projection. | Verified map anchors and persistent traversal connectivity need additional evidence. Width or adjacency alone cannot advance traversal status. |
| Reference geometry | P2-A adds independent offline PLY preprocessing, immutable point arrays, reproducible artifact and bounded local crops. | Synthetic preprocessing passes. Course scan, scale, axes, coverage and defects await review. |
| Map-relative localization | P2-B adds bounded local ICP, capture-matched diagnostic poses and fail-closed quality and epoch gates. | Seeded geometry, rendered depth and H1 replay pass. Native alignment and target timing remain open. |
| Geometry queries | P2-C adds point occupancy, occupied proximity, vertical swept clearance and observed-free/unknown counts on MetricSnapshot. | Aging, inflation, thin obstacles and map boundaries are covered offline. |
| Semantic topology | P2-C anchors current entity geometry and observed boundary links in a bounded map-epoch view. | The graph preserves unknown connectivity. Full aperture and verified region endpoints are still required. |
| Shadow diagnostics | P2-D reads H1 capture evidence after H2 decisions and publishes a separate offline snapshot/report. | Four matched arms maintain the same H2 decisions. Target scheduling, native replay and ROS timing are open. |
| Hybrid authority | H0 validates proposals, H2 runs shadow and H3 belongs to the existing PX4 owner. | Existing fallback and gateway regression. P2 has no command binding. |

See [P2-B localization](p2_map_localization.md) and
[P2-C/D spatial shadow](p2_spatial_shadow.md) for frame contracts,
commands, test counts and benchmark evidence.

## P2-A reference artifact

The supplied dronehacks-course-scan-2026-11-06.ply was unavailable in the
workspace and attachments. Pass the actual PLY as the first argument when
available. Use a course map only under an approved configuration. Install
NumPy and SciPy in the offline environment. The live map-free entry points
do not import these new modules. Tested versions are NumPy 2.3.5 and SciPy
1.17.0.

The PLY reader accepts ASCII, little-endian and big-endian PLY 1.0 point
clouds with scalar vertex attributes, XYZ and optional integer RGB in
[0, 255]. Empty trailing elements are supported. Nonempty faces and
vertex-list properties are rejected. The input cap is 256 MiB with a
default one-million-vertex budget and a hard two-million-vertex ceiling.
Nonfinite XYZ is dropped. Metric conversion uses a supplied positive scale.
When independent scale evidence is available, supply its ID and an
expected longest-axis span in meters. Preprocessing checks this interval.
The check can detect unit-order errors and leaves verification of the
scale evidence to the reviewer. The axes description records the source
basis and is never applied as an implicit rotation or handedness change.

The deterministic pipeline deduplicates XYZ, retains the first associated
RGB and picks an actual measured point per voxel. Radius and statistical
filters are optional. A bounded CPU index supplies spherical crops and
rejects oversized crops. The artifact directory contains little-endian
XYZ, optional RGB, a canonical manifest with input and output hashes and
processing parameters, a three-projection SVG, and diagnostics. The loader
checks array content and rebuilds the index. The manifest hash identifies
the artifact version. Artifact integrity hashes do not authenticate the
scan supplier. Coordinates in the artifact are meters.

An example for an independently measured synthetic scene follows. Supply
the actual scan's reviewed unit, frame, axes and span for a course run.

~~~sh
python -m examples.reference_map synthetic.ply reference-output \
  --map-id synthetic-surfaces --frame-id synthetic_world \
  --axes 'right-handed XYZ, Z up' --source-unit m --meters-per-unit 1 \
  --scale-evidence 'synthetic fixture design' --expected-span-m 23.9 24.1
python -m pytest -q tests/test_reference_map.py
~~~

Use this fixture generator to reproduce the scale-size benchmark.

~~~python
from pathlib import Path
import numpy as np

rng = np.random.default_rng(26)
n = 596349
xyz = rng.uniform([-12, -7, -2], [12, 7, 4], size=(n, 3))
for i in range(3):
    xyz[i::3, i] = np.take([-1., 1.], rng.integers(2, size=len(xyz[i::3]))) * (12, 7, 2)[i]
rgb = rng.integers(32, 255, (n, 3), dtype=np.uint8)
vertices = np.empty(n, dtype=[("xyz", "<f4", (3,)), ("rgb", "u1", (3,))])
vertices["xyz"], vertices["rgb"] = xyz, rgb
header = (
    f"ply\nformat binary_little_endian 1.0\ncomment deterministic synthetic surfaces, seed 26\n"
    f"element vertex {n}\nproperty float x\nproperty float y\nproperty float z\n"
    "property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n"
)
Path("synthetic.ply").write_bytes(header.encode() + vertices.tobytes())
~~~

On Windows AMD64 build 26200, Intel Core i9-14900HX, Python 3.11.9,
the traced run took 1.076 s with a 75.65 MiB Python/NumPy allocation peak
and 7.68 MiB retained point/color arrays. Add --trace-memory to instrument
allocations. These values exclude native allocation coverage and should
not be treated as process RSS or Orin results. The run retained
298,249 of 596,349 points after 5 cm voxel sampling. XYZ spans were
24.000 × 14.000 × 6.000 m. XY/XZ/YZ sampled 100% of their coarse 32×32
projected bins, at about 888/2,071/3,551 points per projected square meter.
Projected density and coverage are inspection aids. Sparse or empty
areas may reflect occlusions and supply zero verified free-space evidence.

P2-A files are refnav/world_model/reference_ply.py,
refnav/world_model/reference_map.py, examples/reference_map.py,
tests/test_reference_map.py and this audit. Focused suite result was
18 passed. P2-A is ready for offline review using synthetic data.
Course-specific diagnostics and measurement accuracy remain open.
