# Hybrid simulation results

The hybrid planner combines metric geometry, semantic hints and D0 prediction
for diagnostics. Its handoff has been rehearsed offline. The existing ROS/PX4
command owner is the only native flight path.

## Runs

| Run | Scenario and revision | Result and evidence |
| --- | --- | --- |
| H-O-001 | Synthetic Phase 6 ablation on Windows at `a2c4db5f5b2cd3008601f70d4f115187c9d0b8a1` | D0 reported zero false-feasible candidates and zero 25 ms budget misses in 200 timed decisions. It also stopped in a moderate drift case where a geometric candidate had clearance. See the [ablation notes](../unified_phase6_planner.md) and [result JSON](../../artifacts/unified_phase6/ablation.json). |
| H-O-002 | Synthetic Phase 7 handoff rehearsal at `05508625b621745f526e3b0c2dccd0c683d35715` | The fixtures exercised eligibility and rejection without sending commands. See the [handoff review](../unified_phase7_review.md) and [rehearsal JSON](../../artifacts/unified_phase7/rehearsal.json). |
| H-W-001 | Planned managed course replay from native Workbench data | Reference the shared [C-M-001 smoke](classical.md#managed-reference). Retain a synchronized single-epoch depth and VIO bag and reviewed calibration for the Phase 2 and Phase 4/6 replays. Record geometry coverage, unknown and rejected candidates, and planning time. Follow the [SITL runbook](../unified_sitl_readiness.md). |

The [Phase 8 preflight](../../artifacts/unified_phase8/preflight.json) records
zero native qualification trials. The Windows timings are offline measurements.

## Issues, improvements and retests

| Issue | Improvement or investigation | Retest |
| --- | --- | --- |
| H-01 · D0 gave up progress in a moderate synthetic drift case | Check the clearance, rollout error and stop rationale on native course replay before proposing a change. Retain the unknown-geometry gate. | H-W-001 pending |
| H-02 · Applied-command timing and target compute budget remain unverified | Obtain Phase 5 effective-command timing in a reviewed moving capture. Profile the selected configuration with perception active on Orin separately. | Phase 5 replay and Orin profiling pending |
| H-03 · No native planner handoff or paired course qualification | Review native Phase 1–6 evidence and the owner handshake. Then seek team approval for bounded Phase 7 trials before running the [Phase 8 protocol](../unified_phase8_qualification.md). | Native execution gated |
