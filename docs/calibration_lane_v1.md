# Proposed isolated SITL calibration lane

This is a **new simulator design for approval**, separate from the phone-scan
course and protected Classical S1. It is not a physical survey, policy-positive
clearance evidence, measured stopping qualification, or flight authorization.
The calibration owner must use the reviewed lane manifest; Hybrid map control
and background registration remain off.

The map is `scenarios/calibration/straight_lane_v1.json`. The existing world
generator produces a 6 m wide by 8 m long empty enclosure, course x = [-3, 3]
and y = [-4, 4]. There are no interior walls, tubes, boulders, or checkpoints.
The existing solid safety-net generator places its interior roof at 3.048 m
above the floor and its 0.10 m side thickness outside those edges. Floor
collision is an infinite plane; its visual size is not a collision boundary.
The launch mark is course (0, -2); the landing-pad metadata is not a requirement
to return. The approved recovery is vertical landing in place inside the lane.

The full-simulator transform is course origin (0, 2, -0.04) in Gazebo ENU:
Gazebo (0, 0, 0) is the resting vehicle center, with floor at z = -0.04 and roof
at z = 3.008. Launch yaw is the existing +90 degrees, facing +course Y.
The generated world is named `dronehacks_course` because existing clock and PX4
bindings require that name; isolate this run by its explicit map hash, source
archive, DDS domain, run directory and container, not by its world name alone.

## Proposed bounded trajectory

After fresh disarmed/landed preflight, validate the NED-to-lane transform and
native starting position. Do not assume native local-position zero equals the
generated Gazebo origin. Require launch XY within 0.10 m of the reviewed mark;
this allowance is included in the total 0.25 m design positional reserve below.
The intended +NED north direction is +Gazebo Y / +course Y only after this frame
mapping is verified. Heading alone does not establish that transform.

- Take off vertically at a commanded climb no faster than 0.20 m/s to a center
  height 1.00 m above the resting center (1.04 m above the modeled floor).
- First calibration trial: command 0.20 m/s forward, issue brake by north 1.50 m
  from the validated start, record the actual braking response, then land in place.
  Higher-speed trials need separately reviewed results and approval.
- Proposed navigation center bounds relative to validated start: east [-0.50,
  0.50] m, north [-0.50, 4.00] m, height [0.75, 1.25] m. The 2.50 m between the
  brake trigger and north limit is a design recovery allocation, not a measured
  stopping distance. There remains 1.40 m of inflated geometric margin beyond
  the north bound; this is not a guarantee that a breached bound can be recovered.
- The observed **3D speed cap is 0.50 m/s in every airborne phase**, including
  climb, braking and AUTO_LAND. Nominal commands cannot guarantee this cap;
  exceedance must fail the trial and trigger the reviewed recovery procedure.
- Takeoff and landing necessarily pass below the navigation band. Their
  proposed center-height monitoring range is [-0.02, 1.25] m relative to the
  validated resting center, with intentional ground contact only during landing.
  The navigation floor-clearance calculation does not cover contact phases.
- Driver phase and whole-run wall/ROS deadlines, authority/expiry fallback,
  independent recording, and <=30 s landing timeout must be pinned in the
  approval manifest before any arming. Geometric bounds alone do not supply them.

## Vehicle and geometric evidence

The existing model generator uses a 0.20 x 0.20 x 0.08 m body, motor XY positions
(0.08,-0.16), (-0.15,0.14), (0.08,0.16), (-0.15,-0.14) m in forward/left axes,
propeller center height +0.06 m and diameter 0.1778 m. The attached camera collision
box is 0.02 x 0.10 x 0.03 m at +0.12 m forward. These are simulator-authored
dimensions; mass/thrust and hardware camera placement remain estimates.

The archived `policy-gates-qualification/restart-observe` generated drone SDF
has SHA256 `f8cc362929337eb37e8649632ce3426d86a029a80932cee06a1a80f2d1e169f3`.
Its rotor collision width is 0.016923076923076923 m and thickness
0.0008461538461538462 m. Summing each link-center distance, collision-center
distance and box half-diagonal conservatively covers all body attitudes and
propeller rotations. The largest bound is **0.303078363889 m**. This is a
collision-model bound, not a measurement of real protrusions or the visual mesh.

Use a proposed 0.35 m enclosing body sphere plus a 0.25 m total positional
reserve = 0.60 m inflated radius. Read-back of generated enclosure faces gives
the following minimum geometric navigation margins over the entire center box:

| Surface | Margin (m) |
| --- | ---: |
| Left / right | 1.900 |
| Rear | 0.900 |
| Forward | 1.400 |
| Floor | 0.190 |
| Roof | 1.158 |

Both envelope and reserve are conservative **design choices**, not measured
tracking uncertainty or stopping performance. Rerun the offline reviewer against
the newly generated model from the exact candidate image before approval.

## Sensor and bootstrap boundary

Keep the existing authored camera: 512 x 320 float depth, 10 Hz, horizontal FOV
1.3579 rad, near clip 0.35 m, far clip 10 m. The optical center is 0.131 m forward
(0.12 mount + 0.011 lens); TF and CameraInfo must match this rig. These settings
come from the Gazebo model, not the separate cloud-lab preset in AI_CONTEXT.md.
The bridge's simulated-estimator stream must remain labeled as simulated, not
physical VIO. Preserve PX4 status/position/landed health and clock checks.

At launch, a forward-only depth camera cannot observe its own near, rear, and
full stopping volume. A wider empty world does not fix that policy bootstrap
gap. This lane supports a separately approved calibration controller constrained
by reviewed geometry and independent telemetry. Never inject this ground-truth
geometry into Hybrid's depth map or label it a positive native policy decision.

## Offline reproduction

```text
python -m simulator.gazebo_trial.calibration_lane --generated-drone <exact-generated-model.sdf> --output-dir <new-review-directory>
python -m pytest tests/test_calibration_lane.py -q -p no:cacheprovider
```

The first command only reads sources, generates SDF text and writes a hashed
geometry review. It does not start Gazebo, ROS, PX4, or a command owner. It rejects
unfamiliar collision geometry and changed lane faces. Gazebo's own `gz sdf -k`,
fresh sensor preflight and the final driver/recorder/watchdog composition remain
separate checks; this pure-Python review is not their substitute.

The lane intentionally has no course checkpoints. Its launch must explicitly set
`CALIBRATION_LANE_RUN=1`; `hybrid_environment.launch.py` then validates the exact
lane path and bytes before selecting `calibration_referee`. All ordinary worlds
retain the original course referee. No checkpoint is fabricated and the course
grader is unchanged. The calibration observer records native simulator truth,
source/receipt timestamps, status, 3D speed and conservative body-sphere margins
in separate `calibration_*` files, with no course score or flight-pass claim.
The current world exposes no measured contact stream. Side/roof envelope
intrusions may publish explicitly labeled geometric diagnostics on
`/simulator_only/touches`; these are not measured contacts. Floor envelope
overlap is recorded separately because ground contact is intentional at launch
and landing. Simulator-truth data must stay outside policy sensing and planning.
