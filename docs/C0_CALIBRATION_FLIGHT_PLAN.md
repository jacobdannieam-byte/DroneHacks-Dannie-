# C0: isolated simulated takeoff, hover and native landing

**Ready for user review; not authorized or executed.** Final disarmed composition
evidence is attached below. One trial, `calibration-c0-001`, in the explicitly separate empty
calibration lane. This is not a policy flight, stopping measurement, course test,
or physical flight. Classic/S1, the phone-scan course, map control and background
registration remain unchanged/off. No autonomous retry or C1 authorization.

## Candidate and configuration

Flight-source candidate: `32dd89c21e95b45c497856b1c52f453947e7dac3` on
`code-integration-hybrid`. Deployment uses the byte-pinned source archive in the
batch manifest, preserving source line endings used by the geometry hashes.
Image: `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`.
ROS Humble; PX4 `54f0455ffcd755534539a7cf33a09a20bf71d29d`;
px4_msgs `392e831c1f659429ca83902e66820d7094591410`; Gazebo 8.15.0.

World: `scenarios/calibration/straight_lane_v1.json`; generated SDF SHA256
`d64be312bb5479c529acf2dd6ba559f50d4bf2dd224fbb31cd69be1b5f93dd07`.
Vehicle generated SDF SHA256
`f8cc362929337eb37e8649632ce3426d86a029a80932cee06a1a80f2d1e169f3`.
The exact profile, recorder contract, geometry report and source archive hashes
are in `artifacts/taskforce_batch_003/release-manifest.json`.

The only proposed parameter change is **MPC_LAND_SPEED: 0.7 to 0.3 m/s**, confined
to this ephemeral calibration simulator after approval. It has not been applied
during preparation. Both launcher and owner read back the effective value before
command publication. Existing offboard-loss settings remain unchanged:
COM_OF_LOSS_T=1 second, COM_OBL_RC_ACT=0 (Position mode). No failsafe relaxation,
forced airborne disarm, hardware settings or baseline changes.

## Flight and geometric bounds

- Empty enclosed design: 6 x 8 m, roof 3.048 m above floor. Launch course
  coordinates (0,-2); world origin is the resting body center, 0.04 m above floor.
- Fresh native PX4 NED start must be within 0.10 m of (0,0,0), stationary,
  landed/disarmed, with valid estimator and advancing clocks.
- C0 commands **zero horizontal velocity throughout**. Climb toward 1.0 m above
  the resting body origin (nominal 1.04 m above floor), commanded vertical speed
  at most 0.20 m/s. At 0.8–1.2 m height and speed at most 0.05 m/s for 0.5 seconds,
  request native LAND once; no northbound segment and no brake event.
- Hard vehicle-center limits relative to reviewed NED origin: north and east
  each -0.50 to +0.50 m; down -1.25 to +0.02 m. Navigation geometry was reviewed
  with a 0.35 m all-attitude body sphere plus 0.25 m positional design reserve.
  These are simulator design choices, not measured hardware uncertainty bounds.
- Actual **three-dimensional speed must stay at or below 0.50 m/s**, including
  climb and native landing. Requested speed does not guarantee measured speed.
- Owner limits: 60 seconds ROS time / 120 seconds wall time; native LAND to
  reported disarm at most 30 seconds. No target or authority is renewed by stale
  data, duplicated reports, a stalled clock or resumed old state.

## Before any command publisher exists

Run one fresh isolated container, network disabled, ROS domain 52, localhost DDS,
no hardware mounts, and immutable source/image identities. Start the lane with
its separate calibration observer. Build packages and obtain fresh healthy,
landed/disarmed preflight with idle motion graph. Preserve raw rosbag from startup.

The strict subscriber-only recorder must report ready with the exact
trial/owner/profile/contract/PID identity and actual data. Owner attachment checks
the recorder and bag process identities and native stream publishers. Recorder
faults or silence terminate it; owner admission then closes. Recorder and raw bag
remain available through the recovery window (recorder budget 240 seconds).

Only after explicit user approval may a new expiring approval artifact be written.
It binds this candidate, image, world, vehicle, profile, recorder, geometry, domain,
trial ID, named reviewing person and recovery operator, exact parameter override,
and permission for failed-simulation cancellation. No pre-approved artifact exists.

## Recovery and cancellation included in the requested scope

Proposed recovery operator: Codex supervised in this chat by the requesting user.
The approval must name this role explicitly; no unattended recovery is authorized.

While the owner remains healthy, one operator LAND request uses
`/calibration_trial/control` with the exact trial ID; the same owner handles it.
Authority loss, stale inputs or clock failure stops publication. **Position mode
after offboard loss is not automatic landing.**

If the owner has exited, a separately approved recovery operator may issue one
native PX4 console LAND only after verifying an idle command graph, fresh healthy
armed state and position within the lane. This is a manual recovery action, not
an automatic second owner. No force-disarm command is allowed.

After owner exit, the harness observes fresh native landed/disarmed recovery for
up to 60 seconds. If unconfirmed, the requested approval permits **emergency
termination of this isolated simulator**, saving the raw bag and failure report.
This is failed-test cancellation, not landing/recovery success, not a reset, and
never authorization to retry. This provision does not apply to hardware.

## Evidence and acceptance

Keep source/profile/parameter identities, raw bag, strict recorder JSON, owner
events, native command acknowledgements/status/landed reports, independent truth
log and all startup/recovery reports. Recorder events are bound to a stable
singleton publisher graph in Humble, not cryptographically authenticated. Correlate
them with raw command messages using `examples.calibration_bag_audit`.

Independent review must check the full-rate raw position stream (not just the
owner's 20 Hz summary): finite valid data, no reset or missing interval, max 3D
speed <=0.50 m/s, start/altitude/drift limits, native AUTO_LAND acknowledgement
after the LAND command, and fresh landed/disarmed confirmation within 30 seconds.
Check the separate observer's geometric intrusion evidence. There is no measured
Gazebo contact stream in this setup; geometric diagnostics are not contact proof.
The ground-contact phase is distinct from airborne wall/roof intrusion.

Any breach, recorder failure, unconfirmed landing, ambiguity or emergency
termination fails C0. A driver `complete` flag alone does not pass the trial.
C0 deliberately cannot pass the moving-stop analyzer and produces no stopping
envelope. Only independently reviewed C0 success permits **planning a separate
C1 approval request** for 0.20 m/s northbound stopping calibration.

## Exact entry point after approval

Final preparation evidence on the pinned candidate: 710 tests passed; Gazebo SDF
validation and private ROS event rehearsal passed; native C0 disarmed check passed
with no motion publishers; raw bag coverage 44.36 s and integrity passed. Strict
recorder retained 3,271 position samples and 65 status reports, zero faults and
zero pending samples. Its recording_complete=false is expected: no flight/landing
events occurred. Effective MPC_LAND_SPEED remained 0.7 throughout preparation.

Inside the pinned, isolated container, with source mounted from the release
archive and a new empty `/evidence` directory:

```bash
export CALIBRATION_IMAGE_SHA256=sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1
bash simulator/gazebo_trial/inside_the_box/run_hybrid.sh calibration_trial \
  scenarios/calibration/straight_lane_v1.json 30 \
  32dd89c21e95b45c497856b1c52f453947e7dac3 \
  scenarios/calibration/c0_hover_land.json /approval.json
```

Preparation uses `calibration_check` with an empty approval argument. That path
creates no command publishers, does not apply the landing-speed override, and
does not authorize arming. The approval artifact is the final step after review.
