# Hybrid clearance bootstrap plan

**Scope is design and disarmed validation.** The next usable mode is a separately approved, map-assisted policy run inside a controlled SITL course. Keep observed-only clearance as the default. A controlled-course authorization can cover the whole surveyed route, including takeoff and recovery. It need not hand off to an observed-only mode where the forward sensor cannot support that handoff. Preserve Classic/S1 and evaluate the four existing Hybrid strategies through one common clearance boundary. No motion was run for this plan.

## What the current owner can establish

H1 joins calibrated depth with a healthy capture-time pose and updates `SpatialWorld`'s 0.20 m rolling grid. It marks a voxel free only when its entire uncertainty-inflated box is in the frustum and all required depth pixels clear its far face. Invalid or occluded pixels can revoke prior free evidence. Default per-cell ages are 0.50 s free and 2.0 s occupied. H1 additionally requires a recent depth snapshot and coherent state and estimator reports. A grid having some fresh free cells says nothing about the swept volume at the vehicle origin. [Spatial volume](../refnav/world_model/spatial_volume.py) · [H1 input checks](../examples/h1_observation.py) · [Native disarmed results](TASKFORCE_BATCH_004.md)

D3's forecast sweep and the gateway's full displacement start at the body position. D3 uses expanded segment voxel boxes and the vehicle, tracking, state and alignment margins. The gateway requires another 0.10 m clearance and independently checks the whole displacement. A held or standby target also passes the current-state stopping envelope, the D3 forecast and the full displacement. Any occupied cell vetoes motion and any unknown, expired or out-of-grid cell prevents an observed-free result. The fallback hold has to prove its own drift sweep and can fail at the same blind starting volume. [D3](../refnav/trajectory_evaluation.py) · [Gateway](../refnav/hybrid_gateway.py) · [Held executor](../refnav/hybrid_execution.py)

### Structural blind volume and a corrected bug

The optical origin is 0.131 m forward of the body and the camera clips depths below 0.35 m. In a level, aligned view the foremost surface of a 0.30062 m body sphere is still before the first observable forward location at 0.481 m from the body origin. The origin itself sits behind the optical origin at every yaw. A smaller voxel, faster planner or longer observation at the same pose cannot expose it. Rotating in place cannot observe the body origin either.

The bounded offline fixture uses the pinned native intrinsics and a synthetic 5 m plane. It keeps the diagnostic pose and alignment error assumptions and is deliberately more favorable than the native scene. The counts below are expanded-box query cells, including conservative corners.

| Query at body `(0,0,0.8)` m | Radius including diagnostic margins | Free cells | Unknown cells |
| --- | ---: | ---: | ---: |
| Source-derived body sphere only | 0.30062 m | 0 | 64 |
| Held displacement check, zero displacement | 1.19865 m | 16 | 1,712 |
| Stopping check at 0.20 m/s with 0.75 s reaction and no braking term | 1.24865 m | 40 | 2,704 |

All three refuse. A small probe in the forward field is free and becomes unknown once its evidence expires. These are synthetic isolation checks. They provide no native stopping or uncertainty measurements.

The cell counts can be reproduced from the repository root without ROS.

```python
import json
import math
from pathlib import Path

import numpy as np

from examples.gazebo_sensor_contract import native_depth_policy
from examples.h1_observation import HybridBagProfile
from refnav.world_model.spatial_input import Alignment
from refnav.world_model.spatial_volume import RollingVolume

path = Path('artifacts/taskforce_batch_004/diagnostic-shadow-dry-build/hybrid_existing-h1.json')
profile = HybridBagProfile.from_dict(json.loads(path.read_text()))
cal = profile.spatial.calibration
body = (0., 0., .8)
stamp = 100_000_000
pose = Alignment(body, np.eye(3), .03, .02, (stamp, stamp))
grid = RollingVolume(native_depth_policy())
depth = np.full((cal.height, cal.width), 5., dtype=np.float32)
grid.apply(grid.prepare(depth, cal, pose), stamp)
snapshot = grid.snapshot(profile.spatial.reference, stamp, stamp, 'synthetic-plane', cal.calibration_id, True)
margin = (profile.vehicle.state_position_error_m + profile.vehicle.tracking_error_m
          + profile.alignment.position_error_m + 2 * math.sin(profile.alignment.angle_error_rad / 2) * .8)
for radius, inflation in ((.3006195965640447, 0.), (.35 + .10, margin), (.35 + .20 * .75, margin)):
    print(snapshot.check_sweep((body,), reference=snapshot.reference, now_ns=stamp,
                               vehicle_radius_m=radius, margin_m=inflation))
```

Separately, Gazebo H1 used the generic 0.30 m minimum even though the authored sensor clips at 0.35 m. In the regression the body is at `(0.115,0.1,0.9)` m and voxel `(3,0,4)` is centered at `(0.7,0.1,0.9)` m. With the diagnostic pose errors of 0.03 m and 0.02 rad, its inflated near face is about 0.309 m from the camera. A valid far raster made it `FREE` with the generic minimum. The corrected native policy intersects the authored `[0.35,10]` m range with the existing map `[0.30,6]` m range to use `[0.35,6]` m. It leaves that voxel `UNKNOWN`, preserves the forward positive control and still refuses the body origin. Legacy native H1 profiles receive the restriction without a profile migration. [Regression](../tests/test_gazebo_hybrid_integration.py)

```text
python -m pytest tests/test_gazebo_hybrid_integration.py tests/test_h1_observation.py tests/test_spatial_bag_input.py tests/test_spatial_world.py tests/test_disarmed_shadow_profiles.py tests/test_hybrid_runtime.py -q -p no:cacheprovider
```

The focused command passed 78 tests. The clip correction adds no visibility.

### Physical fit is a separate check

At the current phone-scan course hover point, the M1 divider is about 0.489 m from the body center. The 0.35 m design body radius plus 0.20 m/s times the minimum 0.75 s authority reaction allowance already requires 0.50 m before braking, tracking, pose error and map quantization. The smaller source-derived body sphere leaves only 0.0384 m before those additions. Survey a wider, explicitly identified start and route for the first map-assisted experiment. Keep the original narrow start as a negative control. The existing C1 and R1 calibration proposals measure and validate different parts of that envelope. They do not authorize policy navigation. [Geometry review](TASKFORCE_GEOMETRY_REVIEW_002.md) · [C1 and R1 package](C1_BRAKING_RECOVERY_PLAN.md)

## Phase 1. Prepare the controlled-course evidence

**Source boundary:** only an independently permitted participant map/survey may supply prior geometry to the navigation algorithm. Generated simulator collision geometry and live truth remain external harness inputs for consistency and scope validation, never algorithm inputs. A world file's availability does not grant permission to use it as a prior. If no complete permitted prior exists, this mode cannot be enabled until that input is supplied and reviewed.

Create an immutable clearance certificate outside the live sensor grid from the permitted participant-facing survey map. Require positive surveyed coverage for the declared route. Missing point-cloud samples supply no clearance. An independent reviewer checks survey completeness against the generated collision world, which stays outside the flight algorithm along with simulator truth. The first certificate covers a surveyed launch column, a bounded airborne route and a reviewed in-place or designated recovery corridor in the wider isolated world. Pin the exact source, image, world, generated vehicle, participant map and certificate bytes, route, body envelope, speed, braking inputs, time budgets and clock epoch. Assess the full conservative tube with a reviewed transform and error bound. H1 reuses an authored alignment with a current state capture stamp. That does not independently refresh the certificate's map alignment.

The certificate requires explicit sterile-zone authority. For a pinned isolated SITL course, the supervisor must establish exact immutable collision geometry, controlled access and no unmonitored dynamic actors. It maintains a separate, bounded, revocable occupancy-control lease. Every region the live sensors cannot observe must have that exclusion or independent fresh surveillance. A stale monitor, unexplained geometry change or intrusion revokes permission. Route maps and P2 registration remain diagnostic inputs until their transform, completeness and uncertainty have separate review. Neither a static surface map nor an absence of live hits is evidence of an empty blind region. [Existing P2 consumer](../refnav/world_model/planner_map_input.py) · [Reference-map contract](../refnav/world_model/reference_map.py)

Keep the launch contact phase explicit. A free inflated sphere cannot intersect the floor at rest. Specify a bounded takeoff/contact footprint and vertical path under the same owner. At the airborne transition, require the entire stopping volume and route tube inside the authorized domain. Use a separate reviewed landing/contact envelope. The floor remains an obstacle for all airborne queries.

## Phase 2. Implement one shared clearance boundary

1. Add a typed, optional clearance view next to `SpatialSnapshot.metric`. Keep the observed metric grid, timestamps and semantic state intact. Default callers use observed-only behavior. The controlled-course mode may authorize an unobserved part of a query only inside its certified volume under a current exclusion or surveillance lease. Keep live occupied evidence as a veto and reject stale sensor/health inputs, invalid transforms, expired permissions and out-of-domain volume. Report which parts were observed versus statically authorized. The existing approval boolean `course_prior_permitted` is not a certificate or a live lease.
2. Make D3 `_Geometry` and the shared displacement query use the same clearance contract, preserving all current body, tracking and extra-clearance checks. Bind certificate, transform and lease identities to `HybridAssessment` and `ProposalReceipt`. `sitl_review` must independently validate those identities and freshness. Revalidate the same authority at `HybridGateway` and every `HeldExecutor` tick. Drop cached targets when the certificate, source, frame or owner generation changes. Apply the same semantics to `CourseMission` arrival and any `first_segment` or inspection consumer whose result can advance the mission. Keep semantic observations labeled as observations.
3. Extend the stopping check beyond the current-position sphere. Bound the entire possible path to the requested target, including forecast/tracking deviation and the subsequent all-direction stopping tube. Bound the reachable speed over that path by the measured envelope. Use the full detection, scheduling and transport budget. Preserve independent checks on first proposal, held target and standby drift. In `HybridRuntime.owner_step` and the final `RefNav` publication fence, gate priming, climb and hover with the explicit launch permission and airborne geometry. The current pre-`go` path can stream from `Mission.step` without a 3D swept check. Do not leave a launch exception that can leak into ordinary navigation.
4. Bind enablement to an exact, separately approved controlled SITL profile. Revoke before arm if inputs are incomplete. In flight, use only the reviewed owner landing path while authority and recovery geometry remain valid, or yield to the separately reviewed operator procedure. Position fallback is not automatic LAND. Measure this composition's executor timing under load. The read-only shadow clock repair is not owner timing qualification. [Owner composition](../examples/hybrid_runtime.py) · [Final ROS owner](../refnav/node.py)

## Phase 3. Evidence required to release the first segment

- Keep observed-only fixture results identical except for the near-clip correction. A single perfect forward view, all in-place yaw headings and expired prior views must still refuse the blind starting volume. Show a positive visible control and separate negative occlusion, invalid depth, stale, occupied and outside-volume cases.
- Add a certified-corridor fixture with distinct `authorized_static` evidence. Show a positive takeoff/hold/first-target/held-target sequence within the exact tube and current lease. Assert zero new free cells in the observed grid. Reject a narrow or truncated certificate, stale/mismatched transform or clock epoch, false or missing intrusion proof, a fresh obstacle, floor or roof interference, and a target whose body fits but whose stopping tube does not. Demonstrate no post-revocation target replay.
- Replay one immutable native recording through S0, S2, S3 and S4 with the same geometry, input stamps, uncertainty and pinned models. Require a genuinely supported positive controlled-course case and adverse refusals under the same policy. Record full timing and no model fallback. Keep the evidence label separate from a native observed-only positive. If the goal is observed-only navigation outside a controlled course, add independently calibrated rear/side/downward or external sensing and plan acquisition before the vehicle enters the queried volume. Verify coverage under body occlusion and the actual freshness window.
- Before a policy owner can move, require fresh disarmed composition on the exact candidate, measured stopping and recovery evidence, complete timing bounds, one command owner and separate movement approval. Start with a bounded S0 segment. The current native owner admits G0 and reconstructs D0; S2/G1, S3/G2 and the S4 predictor need explicit model-aware owner rechecks before promotion. Compare their shadow results under the same clearance contract first. Do not manufacture observed clearance to activate a model.
