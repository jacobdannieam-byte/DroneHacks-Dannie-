# Policy readiness

As of 2026-11-08, the identified ingestion and accounting bugs below have fixes, but **full Hybrid course runs are not ready or flight-qualified**. Waypoint scanning is additional observation behavior, not a clearance certificate. This snapshot separates delivered repairs from work being implemented and evidence still missing.

## Fixed, with bounded evidence

| Repair | Evidence and remaining limit |
|---|---|
| Inspection attempts no longer count an unchanged frame as a failed executed inspection; stationary reobservation has a bounded timer preserved across new-frame selection | `tests/test_inspection_attempt_accounting.py`, `tests/test_stationary_reobservation.py`; offline behavior, not a moving recovery result |
| Humble message-only shadow callbacks work; unavailable metadata stays unavailable | `tests/test_native_shadow_callback.py`; native subscriber-only ingestion subsequently succeeded |
| Native shadow failures retain exact clock-fence diagnostics; a dedicated delivered-ROS-clock service prevents expensive spatial work from blocking that receiver | Commits `8a3d3f3`, `497dacd`; the 50 ms fence and queue limits remain. Four native disarmed strategy windows completed without fatal ingestion faults. This repairs the shadow runner, not the command owner's scheduling qualification |
| Native camera minimum depth is propagated as 0.35 m instead of admitting evidence inside its near clip | Commit `7cb0587`, `examples/gazebo_sensor_contract.py`, H1/bag propagation and `tests/test_gazebo_hybrid_integration.py`; this tightens evidence and does not eliminate the blind region |

The native shadow run produced 116 decision records per strategy, only 3–6 full spatial assessments per window, and 22 fresh free voxels. Full assessments refused unsupported whole-body progress; S4 also had one planning deadline miss. Success means real native inputs reached consumers without fatal ingestion failure, not continuous planning at target rate or a positive motion decision. Profiles used diagnostic uncertainty assumptions. See [batch 004 evidence](TASKFORCE_BATCH_004.md) and `artifacts/taskforce_batch_004/shadow-clock-service/`.

## Incomplete implementation

- **Waypoint scan execution:** the optional initial/selected-waypoint scheduler is implemented and independently reviewed for shadow use; see [configuration and limits](WAYPOINT_SCAN.md). It requires measured stable pose, fresh sector observations and bounded drift/time. Native scan-enabled command ownership is explicitly rejected. Integrating and qualifying actual yaw execution remains outstanding; the existing core default `hover_s = 0` is not a stabilization guarantee. Expiry or cancellation does not advance a checkpoint.
- **Map-assisted clearance:** [the selected design](HYBRID_CLEARANCE_BOOTSTRAP_PLAN.md) requires a separate participant-supplied geometry certificate and current controlled-occupancy permission, a bounded map transform, full route/stopping coverage, live contradiction vetoes, and rechecks through proposal, held target and publication. The design document is not an enabled clearance backend. Do not mark map priors as newly observed FREE.
- **Launch and recovery geometry:** explicit takeoff/contact, stabilized-hover and landing contracts must cover the pre-navigation path as well as D3 route motion. The current core climb/hover phases do not alone prove a complete 3D swept clearance check.
- **Strategy promotion:** S0 is the first bounded owner candidate. Existing S2/S3/S4 shadow operation does not remove the native G0/D0 execution restrictions or supply the required model-aware owner rechecks.

An in-place scan changes viewing direction, but the forward optical origin and near clip leave the body origin unobserved at every yaw. Free cells also expire. Scan completion, a full turn, or an accumulated set of headings therefore cannot establish fresh whole-body/stopping clearance. Any scan success must mean only that the specified observation sequence completed under its own admission contract.

## Implemented preparation, still unflown validation

C1 braking and R1 owner-expiry/recovery drivers, recorder/audit checks and disarmed composition evidence exist. Their moving results remain outstanding in the cited batch. C0 takeoff/hover/landing evidence is narrower and supplies no translation/braking envelope. An approval package, schema-valid report or successful synthetic operator receipt is not a measured stopping or recovery result.

Before the first bounded policy segment, establish measured braking and recovery bounds, reviewed sensor/transform uncertainty, owner timing under load, complete launch/route/stopping/recovery coverage, and a fresh exact-source disarmed composition. Obtain approval for that concrete motion configuration. A successful bounded S0 segment would still precede full-course validation and separate promotion of other strategies. The narrow original phone-course start may fail the required margin even with an accurate map; preserve it as a negative case rather than reducing the margin to pass.

Protected Classic's saved four-checkpoint success demonstrates its different static-map route contract, not these new Hybrid gates. [Classic comparison](CLASSIC_HYBRID_BOOTSTRAP_COMPARISON.md). RTX acceleration can improve timing but cannot observe missing space or replace measured bounds.

This document records no new simulator launch, flight, runtime change, or claim that all bugs are fixed. Historical test totals belong to their pinned sources; rerun relevant checks after the scan implementation lands and retain its exact evidence separately.
