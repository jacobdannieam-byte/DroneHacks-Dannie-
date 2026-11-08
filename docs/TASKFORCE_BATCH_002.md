# Taskforce batch 002

Delivery: `code-integration-hybrid`. Latest user boundary: continue implementation
and disarmed qualification; do not arm. Supervised taskforce roles remain Astra
low for policy diagnosis and independent review, Sol xhigh for implementation.
Map control, background registration and unattended lab loops remain off.

## Changes

- Stationary reobservation has an explicit bounded wait outcome after a
  reconstructed inspection target and D3-verified hold sweep. It carries no
  ranked movement IDs, grants no motion lease and does not fabricate progress.
  The mission preserves its original observation deadline across fresh frames.
- Hybrid native owner supervision uses a steady-clock timer so paused ROS time
  cannot suppress expiry checks. Active clock stall/rewind revokes authority;
  recovery does not restart the terminated owner. Final publication checks cover
  both ROS and wall lease expiry. Classic/S1 control timers remain unchanged.
- Offline observation-bundle auditing verifies original raw hashes and SQLite
  topic summaries, rejects omitted/split/concurrently added recordings, and
  labels its result as integrity evidence only.

## Independent findings and trial preparation

See `TASKFORCE_GEOMETRY_REVIEW_002.md` for generated world/model hashes and
reproducible static calculations. The actual phone-scan launch corridor has
about 0.489 m center-to-wall space. At a 0.20 m/s cap, the source-derived body
sphere and 0.75 s authority-reaction allowance leave only about 0.038 m before
braking, uncertainty and voxel conservatism. A 0.50 m/s envelope cannot fit.
Forward camera coverage also cannot certify the complete body volume at rest.
These are reasons to retain refusal, not lower safety bounds.

`BOUNDED_TRIAL_REVIEW_002.md` supersedes the old sequence as current preparation.
It remains blocked on feasible reviewed lane/uncertainty/observability,
single-owner calibration and event provenance, measured stopping and native
recovery. No moving command or fabricated clearance values are supplied.

S2/G1 remains the leading existing strategy for shadow evaluation based on the
prior paired synthetic benchmark. No new strategy promotion or physical
comparison was made. Classic/S1 remains the protected physical baseline.

## Evidence

The preserved Batch 001 raw observation passes the new integrity audit;
`artifacts/taskforce_batch_002/raw-observation-audit-reviewed.json` contains the
reviewed result. This reanalysis is not a new native observation or sensor
content/clearance qualification. Native raw recordings remain outside Git.

Final frozen-source suite: **640 passed in 35.23 s**. The preceding 639-test
run passed before an additional final-command clock guard/test was added.
Final log: `artifacts/taskforce_batch_002/repository-tests-final.txt`.

Private ROS watchdog probe passed twice, with the final run covering the final
source snapshot. In the pinned image, with network disabled and private domain
49, the actual RefNav steady timer expired a held target while ROS time stayed
at 160 ms, latched one authority yield, and published no further targets or
commands after ROS time resumed. Both runs emitted one initial private heartbeat
and setpoint and zero vehicle commands. This uses synthetic owner output and
private sinks, no PX4 or Gazebo. Source archive hashes and logs are under
`artifacts/taskforce_batch_002/`; every archived source file was compared with
the final tree. Initial Docker access required sandbox escalation; no probe
failed. Source archives remain in `../taskforce-native-002/inputs/`.

Independent review found no remaining code blocker in this batch. The watchdog
shares the owner executor; it fences output after delayed callbacks but is not
a hard OS deadline or proof of airborne PX4 fallback. No arming, Gazebo motion,
physical flight, or PX4 parameter changes occurred.
