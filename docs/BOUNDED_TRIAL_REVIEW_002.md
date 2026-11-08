# Bounded moving-trial preparation — batch 002

Status: **NOT RELEASED FOR ARMING OR MOVEMENT**. This is a review package,
not executable authorization. The user's latest boundary remains disarmed-only.
Delivery branch: `code-integration-hybrid`. Classic/S1 remains the protected
physically tested baseline; these Hybrid findings do not regrade it.

## Pinned deployment and current evidence

Batch 001 native source: `c08481f` (full ancestry in Git); published evidence:
`a0c8663a8e58eedc6f12f3b25f7a4fd097f70b7a`.
Image: `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`.
World input: `drone_code/team_nav/course_maps/phone_scan_2026-11-06.json`,
SHA256 `74a3b67ed942292d880c79cc43eb2d0740466d0c27bfa42cb5045210e05f9ccf`.
Exact generated-world/vehicle review: `TASKFORCE_GEOMETRY_REVIEW_002.md`.

Fresh Batch 001 observation was healthy, landed and disarmed, with no motion
publisher and 40.616 seconds of recorded coverage. Batch 002 independently
checks preserved file hashes and the SQLite summary. Neither check establishes
positive whole-body sensor clearance or measured stopping.

## Geometry changes the proposed sequence

Do not execute the historical 0.20/0.35/0.50 m/s sequence merely because it is
listed in the old package. The selected launch corridor is too narrow for the
0.50 m/s all-direction reaction envelope even before braking and uncertainty.
At 0.20 m/s the remaining static budget is only about 0.038 m. This is not a
reviewed margin. Do not reduce vehicle dimensions, reaction time or sensing
uncertainty to manufacture a positive case.

Before proposing a calibration trial, identify a surveyed isolated calibration
lane and complete vehicle/attitude/sensor error bounds. A different simulated
calibration world must be explicitly identified and separately reviewed; it
cannot silently replace the selected course or establish course clearance.

## Required release record

Each future trial must name the full candidate commit, image digest, generated
world and vehicle hashes, profile/model/calibration hashes, exact single-owner
command, lane endpoints and altitude interval, maximum travel/time/speed,
fault trigger, recorder paths, reviewed recovery behavior and acceptance limits.
No runnable motion command or invented lane values are supplied here.

The release record remains incomplete until all of these are satisfied:

1. Positive native shadow case using real observed full-body/stopping volume,
   plus unknown, occupied, stale and insufficient-clearance refusals. Map truth
   is review evidence, not an onboard free-space sensor.
2. Reviewed dimensions, uncertainty, lane/overhead space and braking coverage.
3. One bounded calibration/fault owner and independent recording; private ROS
   rehearsal verifies event clocks and command provenance. The watchdog's
   cessation of output does not itself qualify PX4 recovery.
4. Native recovery observed under separately approved movement; no forced
   airborne disarm or PX4 safety-parameter changes.
5. Fresh landed/disarmed telemetry, valid estimator, advancing clock and idle
   motion graph on the exact candidate, followed by explicit team approval.

All trials remain isolated SITL, map control/background registration off,
actual 3D speed at most 0.50 m/s (a ceiling, not a selected speed), landing to
disarm at most 30 seconds, and stop on first failure. Keep failures and retries
separate. No unattended retries, hardware flights or policy promotion.
