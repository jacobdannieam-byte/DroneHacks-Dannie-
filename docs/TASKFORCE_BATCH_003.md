# Batch 003: preparation for first bounded calibration flight

Flight-source candidate `32dd89c21e95b45c497856b1c52f453947e7dac3`, delivery
`code-integration-hybrid`. Supervised taskforce: Sol xhigh implementation,
Astra low geometry/research and independent review. No arming, movement,
parameter override, hardware flight, map control, background registration or
persistent unattended lab loop was activated.

## Outcome

The concrete approval request is **C0 only: isolated simulated climb, stationary
hover and native LAND**, no horizontal segment. See `C0_CALIBRATION_FLIGHT_PLAN.md`.
There is no approval artifact. The proposed one-off landing speed tightening
0.7 to 0.3 m/s is explicitly part of that future approval; observed 3D speed must
still stay <=0.5 m/s. It was not applied during disarmed preparation.

The separate open calibration lane solves the calibration experiment's geometry
constraint without substituting for the phone course or injecting free space
into policy sensing. C0 cannot produce a measured stopping envelope. Only after
independent C0 acceptance may a separate C1 translating-stop trial be proposed.
Full policy flights remain later, with legitimate native whole-body observations,
reviewed stopping and recovery. Classic/S1 is unchanged and protected; S2/G1
remains the existing candidate for future shadow/policy comparison.

## Implemented and reviewed

- Separate 6 x 8 m calibration world and collision-envelope reviewer, binding
  exact generated SDF/model plus generator and camera source hashes. Positive
  geometry is design evidence, not policy sensor clearance or hardware bounds.
- Explicit calibration observer records independent truth/bounds without
  pretending the empty lane completes course checkpoints. Original course
  referee remains unchanged. Geometric intrusion is not measured contact.
- Single-owner C0/C1 sequencer, dual-clock freshness, native reset/authority
  fences, ordered command acknowledgement, actual speed/bounds/landing limits.
  CLI creates no command publishers in disarmed-check mode. Movement requires
  exact expiring user approval, geometry/profile/recorder binding, ready and live
  recording processes, idle command graph and the approved landing speed.
- Strict recorder retains native source/publish, mapped, callback and ingestion
  timestamps. Bounded 50 ms deferral handles observed 4 ms clock delivery skew;
  original times remain visible. Reset, silence, stall and identity failures
  fail closed. C0 readiness/completeness differs from moving-stop completeness.
- Humble event identity uses a stable singleton publisher graph, explicitly
  labeled graph-only. Raw bag correlation checks actual LAND or velocity-mode
  zero-target brake messages before measurement review. Analysis requires strict
  clock/reset/status provenance, includes clock uncertainty, and cannot promote
  synthetic fixtures or C0 to stopping qualification.
- Failure recovery keeps recording during a bounded native observation window.
  Explicit future approval includes emergency isolated-simulator cancellation
  if landing cannot be confirmed; this is failure, not recovery success or retry.

## Validation and preserved corrections

Final repository suite: **710 passed in 68.20 s**. Earlier 702-test suite is kept
separately. Gazebo's SDF checker and private ROS driver/event transport passed on
the pinned image; no actuator endpoints were created by that private probe.

Initial lane startup failed because the stock referee expected course checkpoints.
It was preserved and fixed with explicit calibration observer selection, not fake
checkpoints or a weakened course grader. The next lane observation passed with
40.94 s raw coverage and confirmed actual unchanged native recovery parameters.

First composed owner/recorder rehearsal refused the generated-world hash:
reviewer emitted +0.0000 while actual generator emitted -0.0000 in the launch
transform. The correction reproduces exact generator bytes and has a regression.
That failed attempt and its readiness/recording data remain separate. Initial
offline SDF validation omitted required model paths; it failed, was retained,
and passed after supplying the generated model and explicit SDF resource paths.

Final native composition passed: disarmed, landed, healthy, valid native start,
empty motion command graph, 44.36 s raw bag coverage and SQLite integrity.
Strict recorder: 3,271 samples, 65 status reports, zero faults/pending samples.
No command events occurred, so flight recording_complete=false is expected.
Shutdown was interrupted intentionally while disarmed and handled cleanly.

Raw archives and bags remain in `../taskforce-native-003/`. Portable reports,
failure excerpts, source/profile hashes and raw paths are in
`artifacts/taskforce_batch_003/release-manifest.json` and adjacent folders.
Disposable pytest directories are not experiment evidence and are not published.

## Honest limits

C0 has not flown. No moving recovery or braking result exists. Native landing
speed must be measured; changing its request to 0.3 does not guarantee actual
speed. Recorder graph provenance is not cryptographic message authentication.
Raw evidence plus independent full-rate review is required, not just the owner's
20 Hz `complete` flag. The current simulator supplies no measured contact stream.
The forward-only camera's near/rear blind region remains a policy bootstrap gap.
