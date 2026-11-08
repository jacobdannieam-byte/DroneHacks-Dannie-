# Attempt 008 launch obstruction replay

The isolated S0 experiment at source `02949ac` entered climb at 57.760 s. Its last issued target was approximately `(0.00133, 0.00157, −0.76426)` m NED at 61.108 s. The publication guard then refused `live_occupied_voxel_intersection`. It had reached stage `course_00` and issued no D0 navigation target. The independent referee recorded zero touches and zero checkpoint crossings. The final native probe found the vehicle disarmed and landed.

## Raw-bag reconstruction

The read-only decoder used the retained SQLite bag, exact experiment profile, current occupied-cell retention, captured depth, and capture-aligned native simulated visual odometry. It applied the same H1 voxel classifier and experimental swept-volume veto to recorded PX4 states near the refusal. Recorder and owner callbacks can select different captures. This replay reproduces the refusal at plausible owner ticks while leaving the exact live callback-selected cell unverified.

At the 61.208 s check, the reconstructed 0.20 m grid rejected the 0.334 m inflated climb sweep on four cells centered at `(0.50, ±0.10, ±0.10)` m in native `odom`. The nearest voxel boxes came within 0.317 m of the sampled sweep. The aligned depth frame captured at 61.100 s supplied 4,480–5,152 qualifying returns per cell. Their endpoints were 3.1–3.4 mm below the modeled floor plane in the recorded pose alignment. Neither cell at `odom z=+0.10 m` contained a nominal endpoint. The closest qualifying endpoint was 0.459 m from the swept centerline. Pose inflation was approximately 0.043 m in these cells. These returns are consistent with the floor, and the raw endpoints do not match an above-floor obstruction. The static participant geometry passed the same sweep.

| Replayed resolution | 61.108 s | 61.156 s | 61.208 s |
| --- | --- | --- | --- |
| 0.20 m, executed profile | Pass | 4 intersecting cells | 4 intersecting cells |
| 0.10 m, offline sensitivity | Pass | 4 intersecting cells | 6 intersecting cells |
| 0.05 m, offline sensitivity | Pass | 5 intersecting cells | 14 intersecting cells |

The finer grids retained the same body radius, source-pose uncertainty, occupied-cell lifetime, and obstacle veto. Both reproduced the refusal. The closest nominal cell face encroached as the estimated body moved forward during climb. Projected pixel tiles and pose inflation can support occupied neighbors above the floor even when all qualifying returns lie on it.

Local reproduction files are under the retained run directory `taskforce-s0-experiment-009/attempt-008`. Run `offline-voxel-analysis.py` against that directory's original bag and profile. `offline-voxel-020.json` records the executed resolution and `offline-voxel-010.json` and `offline-voxel-005.json` record sensitivity cases. The fine-resolution variants are offline comparisons with a smaller spatial grid, so they do not establish usable course coverage.

## Evidence required for refinement

A follow-up can refine a coarse veto using the contributing source returns and conservative pixel, depth and pose uncertainty over the actual swept path. Bind that evidence to the exact capture, pose bracket, voxel identity, calibration and expiry. Retain the occupied veto for missing, contradictory or ambiguous source evidence and for any return that can intersect the swept body outside the modeled floor support. Test the raw replay alongside genuine above-floor obstacle and mixed floor/obstacle cases. Extending the existing floor-cell mask or changing resolution alone would not establish that distinction.

Future experiment logs capture the exact refused sweep and up to four nearest vetoing cells with their observation timestamps and voxel distances. This instrumentation preserves the same clearance result. Attempt 008 remains a failed full-course attempt, and no moving retry used this change.
