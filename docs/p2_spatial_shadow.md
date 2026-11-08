# P2-C and P2-D offline spatial shadow evidence

## Metric and semantic interfaces

P2-C reuses RollingVolume, its calibrated projective evidence integration,
and D3's existing collision geometry. Cell.OBSERVED_FREE aliases the
legacy Cell.FREE numeric value. MetricSnapshot.cell_at samples a single
fresh voxel. nearest_occupied_m gives the distance to an occupied voxel
box within a bounded radius. An absent hit conveys no clearance.
vertical_clearance checks upward and downward swept body envelopes in
the z-up VIO frame using the existing sweep. check_sweep retains its
point, line and polyline behavior and reports observed-free and unknown
fractions for the in-volume footprint. Its separate outside-map flag
retains the unknown classification beyond the grid. The caller supplies
an enclosing vehicle radius and position/tracking margin. Projection
already inflates voxel coverage for H1 pose, depth and extrinsic error.
The D3 policy and H0 proposal checks still use their existing bounds.

The sidecar's TopologyMemory uses WorldState IDs, provenance, width and
observed height. A capture-matched accepted map fix transforms entity
centers, normals and observed width segments. The coarse map-region
index links the anchor to a location without turning the scan into voxel
evidence. The graph retains at most 128 entities. Entity and neighbor
queries return at most 32 immutable records and omit dangling or stale
links. Observed bounded_by links and blocking evidence come from
WorldState. Opening height, verified connectivity, complete bounding
volumes and traversability remain unknown where the extractor lacks the
required evidence. No route edges, junctions, visit records or mission
checkpoints are inferred from proximity. The current graph is suitable
for bounded diagnostic reads. Feeding it to the semantic reasoner needs
a separately versioned evidence contract and verified region endpoints.

P2-C changes refnav/world_model/spatial_volume.py, adds
refnav/world_model/spatial_topology.py and tests/test_p2_spatial_evidence.py,
and updates the audit. Four focused tests cover vertical obstacles, thin
jambs, inflation, unknowns, evidence expiration, bounded identity/links
and coordinate resets. Existing spatial tests cover the depth update and
contradiction paths. See [the capability matrix](p2_spatial_intelligence.md).

## Offline integration

P2-D optionally retains one validated AlignedDepth in H1 replay. A
diagnostic consumer runs after the H2 tick and checks matching visual
brackets, fresh H1 depth and the owner's pose-loss and epoch state.
A tracking loss clears the topology view and fences the reviewed anchor.
P2Snapshot v1 wraps the existing immutable SpatialSnapshot with optional
MapLocalization v1 and TopologyView v1. H2 still consumes its existing
SpatialSnapshot. Neither H2 nor the existing command owner uses the
diagnostic transform, topology or registration uncertainty.

Files changed are examples/h1_fixture.py, examples/h1_observation.py,
examples/hybrid_shadow_loop.py, examples/spatial_bag_input.py,
refnav/world_model/spatial_pipeline.py, examples/p2_shadow_replay.py and
tests/test_p2_shadow_replay.py. Three focused tests cover matched H2
assessments and candidate evidence, no-anchor fallback, and tracking-loss
fencing. The cumulative suite passed **462 tests** on the verified
remote base of 421. ROS nodes and flight entry points were never started.
The default H3 state and existing owner and fallback code are preserved.

~~~sh
python -m pytest -q tests/test_p2_spatial_evidence.py tests/test_p2_shadow_replay.py
python -m pytest -q tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  python -m examples.p2_shadow_replay fixture --repeats 10 --output paired.json
~~~

For a recorded H1 bag, run the offline reader in an environment with the
installed message definitions and ROS bag deserializer. With only the
H1 profile, replay is map-free. Supply a processed reference directory
and reviewed anchor together to add the sidecar. A missing or malformed
reference input is reported in the output while replay continues map-free.

~~~sh
python -m examples.p2_shadow_replay bag /path/to/recording \
  --profile reviewed-h1.json --reference /path/to/reference-output \
  --anchor reviewed-reference-anchor.json --output p2-replay.json
~~~

The anchor JSON contains reference_version (artifact SHA-256),
reference_epoch, valid_from_ns (on the VIO clock), evidence_id,
reference_from_vio (rotation as three matrix rows and translation_m as
three components), and vio (frame_id, body_frame, clock_id and epoch).
The VIO fields must match the reviewed H1 profile. A different artifact
hash or VIO epoch leaves the map fix unavailable. Reference usage needs
the applicable course approval and independent scale and frame evidence.

## Matched replay measurements

Each arm received the same four ROS-shaped captures ten times. The
evaluator rendered a room with independently sampled scan surfaces,
a partial opening, a floor and a ceiling. It injected up to 11 cm of VIO
drift and used the physical render pose only for scoring. The scan has
15,400 points and 369,600 bytes of XYZ storage. The metric snapshot's
class and timestamp arrays use 345,600 bytes. The largest bounded graph
view had eight entities and serialized to 6,299 bytes. These figures
describe the measured arrays and encoded graph, excluding tree and
interpreter overhead.

| Replay arm | H2 stops | Accepted map fixes | Map pose RMSE | Floor/ceiling occupied |
| --- | ---: | ---: | ---: | ---: |
| H1 metric baseline | 40/40 | — | — | — |
| H1 with geometry queries | 40/40 | — | — | 80/80 |
| H1 with reference localization | 40/40 | 40/40 | 1.62 cm | — |
| Combined geometry, localization and topology | 40/40 | 40/40 | 1.62 cm | 80/80 |

The matched VIO position RMSE was 6.75 cm, with worst map rotation error
0.47 degrees. All H2 stops were for no verified progress. Neither geometry
arm classified a vertical collision probe as free. A stop-only scene
cannot measure reduction in H2 false feasibility or redundant exploration.
The sidecar reduced synthetic localization error and provided local
clearance and anchored geometry evidence. H2 decision counts were identical.
There were zero depth drops, skipped ticks or planner deadline misses.

Windows AMD64 build 26200, Intel Core i9-14900HX, Python 3.11.9, NumPy
2.3.5, SciPy 1.17.0, one BLAS thread. Times are ms p50 / p95 / p99 / max.

| Stage | Baseline | Combined |
| --- | --- | --- |
| Metric preparation | 11.80 / 12.38 / 12.85 / 13.04 | 11.36 / 12.87 / 13.61 / 13.84 |
| Metric apply | 0.36 / 0.42 / 0.47 / 0.49 | 0.38 / 0.46 / 0.51 / 0.53 |
| WorldState update | 8.89 / 9.47 / 9.55 / 9.57 | 9.13 / 10.54 / 11.96 / 12.50 |
| Metric queries | — | 1.18 / 1.31 / 1.67 / 1.89 |
| Registration plus snapshot update | — | 219.25 / 243.16 / 250.64 / 250.90 |
| Topology update | — | 0.21 / 0.25 / 0.26 / 0.26 |
| Topology view | — | 0.057 / 0.061 / 0.069 / 0.072 |
| H0 total within H2 | 13.78 / 15.44 / 19.96 / 21.90 | 13.58 / 16.04 / 16.78 / 17.00 |
| Depth receipt to H2 decision | 38.12 / 40.47 / 45.33 / 48.26 | 37.61 / 42.10 / 44.28 / 45.27 |

Capture-to-map and H2 tick age was 3 ms on the fixture clock. Wall
receipt-to-decision includes ingestion and H0 but ends before the
sidecar begins. The 25 ms D5 deadline applies inside planning.
The combined run used 10.56 s process CPU in 10.59 s wall time for
40 ticks. Baseline was 1.69 s CPU in 1.71 s wall time. The benchmark
does not emulate concurrent DDS, VIO, depth or GPU load. Registration
at this cost is an offline diagnostic. A live integration would need
a measured independent worker, a single-slot queue and a deadline
and freshness policy tested under target load.

**Review gate:** P2-C's local queries and evidence anchors and P2-D's
offline replay are ready for offline review. Native ROS 2/Workbench and
Jetson acceptance, actual course scan and alignment, richer motion and
scene benchmarks, and verified topology and reasoner adoption are open.
