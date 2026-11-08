# C1 Gazebo run success

The authorized corrected C1 retry passed on 2026-11-08. This branch preserves the corrected implementation and compact evidence; it does not claim a full Hybrid/PINN course success.

Runtime source: `38a9a36195c5bdc8c3531e0d31908fb89e92a69f`.
Trial: `calibration-c1-020-002`. One moving attempt. Registration and map control off.

## Results

- Brake entry: 0.202611 m/s; maximum braking excursion: 0.124079 m.
- Command to sustained settle: 0.872 s.
- Native LAND to disarm: 9.4 s including clock allowance, against 30 s.
- Peak simulator-truth speed: 0.332069 m/s, against 0.50 m/s.
- Recorder: 4,702 samples; raw command correlation PASS.
- No actual contact measurement available. Stopping/audit review status remains unreviewed.

## What made decisions

A deterministic calibration state machine checked fresh observations, climbed to 1 m, centered and settled, commanded 0.20 m/s north with bounded vertical proportional feedback, braked at north >= 1.50 m, confirmed sustained settling, and requested native LAND. PX4 closed the low-level control loops. No LLM, Hybrid D0 or PINN model chose the commands in this run. Camera tracking was simulated; this is not real VIO qualification.

The Hybrid predictor is intended to forecast candidate trajectories using nominal physics and learned residuals, reject unsafe candidates, and rank the remaining candidates for a single command owner. Its current comparison evidence is offline/shadow; this flight does not validate prediction accuracy or autonomous reasoning.

## Historical Classical comparison

The protected Classical run at `ad31ae55ebf7cd3b2eaf57917537242831e3c752` completed four checkpoints and landed in 84.3 s, traveling 17.974 m using a static grid and route follower. It is stronger evidence of end-to-end navigation. C1 is a narrower braking/altitude/landing experiment in an isolated lane and gives more specific stopping evidence. Its 9.4 s landing interval is not comparable to Classical's entire 84.3 s mission.

Current Classical at `a641deb50df2cb5a63280a33e82772856819636a` also adds live occupancy mapping and full-circle scans; it is distinct from the historical passing baseline.

## Evidence

See [result](../artifacts/c1_success/RESULT.json), [comparison report](../artifacts/c1_success/stack-and-evidence-report.html), and [recorded-depth replay](../artifacts/c1_success/C1-recorded-depth-telemetry-replay.mp4). The video is a derived replay, not original GUI footage.

Full raw evidence remains separately preserved under the workspace sibling `physical-gazebo-evidence/`:

- `simple-waypoint-c1-moving-002/`: successful raw bag, ULog and owner/recorder evidence.
- `simple-waypoint-c1-moving-001/`: previous failed moving attempt.
- `c1-altitude-retry-001/`: frozen source, offline and disarmed correction qualification.
- `c1-evidence-delivery-001/`: successful/correction ZIP, failed-attempt ZIP, Classical reference ZIP, video and delivery manifest.

The large raw archives are not stored in this Git branch. The copied evidence and delivery manifests retain their original scopes; their entries refer to the complete external folders, not only this compact Git subset. Archive hashes and CRC verification are in `package-verification.json`. All eight protected Classical hashes were unchanged.

Remaining gates: reviewed stopping bounds, separately bounded R1/recovery, native ground-contact/takeoff/landing clearance, matched waypoint trials, and native prediction/timing evidence before PINN promotion. No new flight was run while publishing this branch.
