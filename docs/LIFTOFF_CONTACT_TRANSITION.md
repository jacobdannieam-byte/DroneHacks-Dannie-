# Bounded liftoff contact transition

The Hybrid SITL launch guard has an optional contact transition for the pinned isolated calibration enclosure. It handles intended pad contact during initial lift, then hands off irreversibly to the ordinary all-attitude airborne envelope. A deployment must provide separately reviewed bounds and exact source identities. The repository supplies no enabled transition profile or operating bounds.

## What C0 established

The previously approved C0 run used source `32dd89c21e95b45c497856b1c52f453947e7dac3`, image `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`, world `d64be312bb5479c529acf2dd6ba559f50d4bf2dd224fbb31cd69be1b5f93dd07` and generated vehicle `f8cc362929337eb37e8649632ce3426d86a029a80932cee06a1a80f2d1e169f3`. Its calibration owner used velocity control during climb. The Hybrid launch path uses a ramped position setpoint. [C0 result](C0_FLIGHT_RESULT.md)

The retained external observer log, `taskforce-c0-001/evidence/flight_records/calibration_1794130738417851527_truth.jsonl`, has SHA256 `81b39c5bffdd531465b7ab4b840018b6f5516ef7aef8c08b0a21712c4b238f13`. It pairs sampled simulator position and attitude with the native landed report. The first armed sample carrying `landed=false` was at source time 24.524 s, with center world z approximately zero and speed 0 m/s. The first sampled center z above 0.01 m was at 32.656 s and above 0.30 m at 35.128 s. The largest sampled tilt under center z 0.40 m was 1.292° at 29.656 s, at center z 0.00224 m. This ordering rules out using `landed=false` as physical separation. The trace contains sampled kinematics, while actual contact force, continuous attitude reach, estimator differential error and controller tracking bounds remain unmeasured. [Evidence manifest](../artifacts/calibration_c0_001/evidence-manifest.json) · [Independent C0 review](../artifacts/calibration_c0_001/independent-review.md)

Resolving the retained generated collision model with the pinned wrapper and camera yields collision-only SDF SHA256 `6f207a9d49a310c87ddce2e31d9570a4e8808d9f231fb73cd7bc40f291c198e0`. Its base box is centered at the origin and measures 0.20 × 0.20 × 0.08 m. Its lower face at z = −0.04 m matches the modeled floor at spawn. The resolved support identity is `drone_16_like__base_link__base_link_collision_0/collision`. The camera and swept rotor boxes remain in the collision set. The runtime computes the enclosing sphere from the actual resolved bounds. [Geometry review](../artifacts/taskforce_batch_003/native-lane-reviewed-v2/geometry-review.json) · [Collision resolver](../examples/resolve_launch_vehicle.py)

## Source binding

In the reviewed launch profile, `simulator_launch.contact_transition` contains `file` and `sha256`. The launch-only `deployment_authorization` must repeat the exact `contact_transition_sha256`. The separate file has these fields.

| Field | Required meaning |
| --- | --- |
| `scope` | `reviewed_model_conditioned_isolated_sitl_contact` |
| `world_sha256`, `resolved_vehicle_sha256` | Exact approved enclosure and resolved collision bytes |
| `support_collisions` | Exact `resolved_link/collision` names whose bottom vertices form the modeled support plane at spawn |
| `bounds` | All explicit `ContactTransition` fields, including model evidence identity, pad footprint, state/command bounds and both clock budgets |

The binder checks the same-frame world/ENU alignment, reviewed spawn and floor geometry, and bounds-file hash. It rereads pinned source identities during use. It rejects an unlisted collision sharing the lowest support plane and any declared support face above it. The files specify conditional launch geometry. Existing owner admission, clock and watchdog checks still govern publication. [Binder](../examples/bind_simulator_launch.py) · [Contract](../refnav/launch_clearance.py)

## Model assumptions and handoff

The guard requires advancing, fresh and stationary disarmed/landed telemetry within the reviewed pad neighborhood before it anchors once. The first exact output check binds the anchor to the mission, home, frame and reset identities. During the transition it checks source-timed native attitude, H1 position and velocity, sample gaps, displacement and speed, command height and climb rate, pad footprint, and ROS and monotonic deadlines. A failed active transition latches. Initial priming and the same tick's output check do not renew the anchor or timers.

The floor test rotates the collision vertices into the bound world frame and uses a lower altitude bound relative to the fixed support anchor. `relative_height_error_m` covers differential estimation from that anchor. `max_downward_tracking_excursion_m` separately covers continuous future downward motion below the lowest checked center or command, including estimator sample age, retained PX4 setpoints, and the reviewed response after a missed update or owner loss. `control_horizon_ns` must cover that response on the exact deployment. Attitude reach includes measurement uncertainty, the oldest permitted sample, and the full control horizon. Sampled motion checks can expose contradictions in those bounds. Establishing the bounds requires independent evidence.

Only the named pad-support vertices may enter the explicitly reviewed numerical contact tolerance. This permission applies within the low support band and command-lead bound, including a detector transition before physical lift. Every other collision vertex must clear the floor throughout the modeled horizon. The full all-attitude sweep checks the sides, roof and obstacles throughout. Once the oriented lower bound proves separation, pad-contact permission cannot return. Once the full airborne sphere clears the floor, all subsequent startup output uses that envelope and retains the freshness and reset fences. The guard cannot authorize a return to the pad, route motion or LAND.

## Qualification still required

The synthetic replay exercises zero-rise `landed=false`, a bounded ramp, sphere handoff, and rejection of stale, reset, clock, geometry and motion faults. C0 establishes the detector ordering for the earlier velocity-controlled setup. The actual Hybrid position ramp, command lead, differential height error, continuous downward and angular reach, and effective command-retention and loss-response horizon still require evidence on the exact isolated composition. Preserve a failed outcome if the real vehicle cannot gain height within the reviewed pad and command bounds. A moving validation also requires the existing separate authorization, measured stopping and recovery evidence, and an exact disarmed composition. No motion was run for this change.
