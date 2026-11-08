# Batch 002 independent geometry and evidence review

Date: 2026-11-08. Scope: static source/XML inspection and arithmetic only.
No simulation, ROS session, arming, or motion was run for this review. These
calculations are not reviewed uncertainty margins or an executable flight approval.

## Exact inspected artifacts

The batch001 raw root is `../taskforce-native-001/observe/gazebo_generated/`.
Both generated files are byte-identical to their counterparts under
`../physical-gazebo-evidence/policy-gates-qualification/restart-observe/gazebo_generated/`.

| Artifact | SHA256 |
| --- | --- |
| `models/drone_16_like/model.sdf` | `f8cc362929337eb37e8649632ce3426d86a029a80932cee06a1a80f2d1e169f3` |
| `worlds/phone_scan_2026-11-06_full_simulator.sdf` | `1bc7121fb721bcffe460055c90446f3aff72c31ba0946e86f76d87278182b00d` |
| Repository `simulator/gazebo_trial/models/drone_16_with_oak_d_s2/model.sdf` | `12d2b065f4467456f58666c8d05733491859565445937a41603fe02b07e493de` |
| Repository `simulator/gazebo_trial/models/oak_d_s2_camera/model.sdf` | `abcfc9977bdc8fe29ec1f7a319d97470ea407a1ce2abf46796022dcffa2cb61c` |
| Repository `drone_code/team_nav/course_maps/phone_scan_2026-11-06.json` | `74a3b67ed942292d880c79cc43eb2d0740466d0c27bfa42cb5045210e05f9ccf` |

The camera rows identify inspected repository source, not independently extracted
camera files from the running image. Bind these dependencies and the image/PX4
identity in the eventual deployment manifest. The organizer drawing is not the
selected world for the following calculations.

## Source-derived vehicle bound

The generated vehicle has one body collision box `0.20 x 0.20 x 0.08 m`.
Its motor centers are `(0.08, +/-0.16, 0.06)` and `(-0.15, +/-0.14, 0.06)`
in body forward/left/up coordinates. Each rotor collision box is
`0.1778 x 0.016923076923076923 x 0.0008461538461538462 m`.
The generator preserves upstream rotor width/thickness; diameter alone therefore
does not describe the full rotating collision footprint.

For all rotor angles, a conservative horizontal bound about the body origin is:

```text
r = hypot(0.15, 0.14) + hypot(0.1778/2, 0.016923076923076923/2)
  = 0.2944846237301311 m
h = max(0.08/2, 0.06 + 0.0008461538461538462/2)
  = 0.06042307692307692 m
body sphere = hypot(r, h) = 0.3006195965640447 m
```

The inspected camera collision box is `0.02 x 0.10 x 0.03 m`, centered
`0.12 m` forward; it fits inside these source-derived bounds. These are
simulation collision bounds, not measured hardware dimensions. See
`simulator/gazebo_trial/make_drone_16_model.py:59`, `:79`, and `:146`;
the generated SDF supplies the retained upstream rotor dimensions.

## Selected-world constraint at the proposed hover point

The generated world declares course-frame translation `(2.5146, -0.9144, -0.04)`
at line 6. World ENU `(0, 0, 0.8)` therefore corresponds to course coordinates
`(-2.5146, 0.9144, 0.84)`. This is 0.8 m above the model's initial body origin,
and 0.84 m above the modeled floor. A package saying merely "0.8 m altitude"
must resolve that distinction.

Using each collision box's emitted pose, yaw and half-size, transform this point
into box coordinates and compute `norm(max(abs(local_point)-half_size, 0))`.
The nearest relevant box distances are:

| Surface | Center-to-surface distance |
| --- | ---: |
| M1 maze divider | 0.48903021870804914 m |
| Long barrier | 0.5297500371894965 m |
| West safety net | 0.5333999999999997 m |

Generated-world references: west net collision line 66, Long barrier model line
113, M1 model line 128. These are static geometry distances, not sensed free-space
evidence or usable travel allowances.

`refnav/stopping_envelope.py` checks an all-direction sphere with radius
`max(vehicle.radius_m, body_radius_m) + max_speed_mps*reaction_s + braking_distance_m`,
then adds position/tracking/alignment margin. `refnav/owner_timing.py` currently
requires at least 0.75 s authority reaction allowance for simulated-estimator use.
Even using only the source-derived body sphere:

| Speed cap | Body + 0.75 s reaction distance | Remaining before M1 |
| --- | ---: | ---: |
| 0.20 m/s | 0.4506195965640447 m | 0.0384106221440044 m |
| 0.50 m/s | 0.6756195965640447 m | negative |

The 0.20 m/s remainder must still cover braking, all uncertainty, and voxel-query
conservatism; it is not an approved allowance. A 0.50 m/s stopping envelope cannot
fit at this starting location even with zero braking distance and zero uncertainty.
The check uses the envelope speed cap even when the current measured speed is zero.
Do not lower margins, inject free cells, or reinterpret this as approval to move
to another starting location.

## Positive-clearance and evidence gates

The inspected forward camera has optical origin 0.131 m ahead of the body center,
77.8-degree horizontal field of view and 0.35 m near clip. A stationary view cannot
observe the complete sphere around its own body. `spatial_volume.py:227` only
classifies forward boxes, and `:252` requires the entire box beyond minimum depth
and inside the image; unknown cells correctly remain unknown. Fresh full-body
clearance needs legitimate observations covering that volume, not just a physically
empty lane. The default free-space lifetime is 0.5 s (`spatial_volume.py:33`), so
old observations from a slow sweep are not automatically usable either.

`stopping_measurement.py:13` checks provenance strings/digest syntax, and
`hybrid_launch.py:60` hashes the report file. Neither binds the report's candidate,
vehicle configuration or raw-evidence digest to the active deployment or raw bag.
The new observation-bundle auditor can establish byte/SQLite agreement only;
it does not decode telemetry or confer clearance/braking qualification.

Useful disarmed work remains: archive/hash the complete resolved model dependency
tree and deployment settings; decode existing raw bag timestamps, CameraInfo/TF,
state validity and clearance refusal cells; establish an independently reviewed
uncertainty profile; and test the calibration/fault driver's exact event provenance
with synthetic messages before any approved moving trial. The moving package still
needs a geometrically feasible surveyed lane, exact altitude/frame/travel bounds,
native recovery evidence and reviewed stopping measurements. No such values are
invented by this review.
