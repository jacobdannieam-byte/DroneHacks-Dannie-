# Prepared update for the existing unified roadmap

Destination: Master Implementation Roadmap, with cross-links to Autonomy Strategy
Tracker and Semantic Reasoning Integration. This is prepared text, not a published
Notion change. Preserve previous design checklists and negative/native results.

**2026-11-07 Pacific — offline goal comparison.** A new isolated
`codex/goal-reasoning-comparison` branch starts from canonical Hybrid `bfb09c8a`.
Phase 0 inspected all 19 remote heads plus local worktrees/unpushed changes;
canonical/P2 divergence remains 5/4. The reproduced baseline passed 423 tests.
Exact full SHAs, PR metadata, paths and executed commands are in the repository
audit and `artifacts/goal_comparison/`.

**Phases 2/4/6:** Implemented G0/G1/G2 selection, a bounded immutable inspection
contract, independent validation and semantic-to-metric resolution. G0 policy is
unchanged. Observations now target an inspection hypothesis rather than silently
approaching an opening center. D3 still checks motion; CourseMission does not
advance checkpoints for inspection subgoals. Proposal v1 remains compatible;
explicit v2 adds only an unsupported-inspection abstention.

**Offline paired evidence:** 60 untouched layouts × 7 regimes × 6 steps per
variant. Acquired evidence: G0 68; G1 260; G2 seeds 11/29/47: 259/244/243.
G1 gain over G0 is 0.457 items/episode, paired layout-bootstrap 95% interval
[0.367, 0.543]. G2 establishes no clear advantage over G1. These are abstract
synthetic observations, not course completion or native flight metrics.
Three reproducible MLP checkpoints, dataset fingerprints, configuration hashes,
latency and rejection/abstention/revisit results are retained.

**Phase 3:** Matched physics-consistency residual training reduces nominal error
versus an otherwise identical network, but regresses shifted/switched behavior
and remains substantially worse than D0 nominally. Existing ridge/history/MLP
comparisons and historical failures are retained. D0 remains the default.

**Phase 5:** The selected reasoner runs through H1/H2 with per-component host
timing. All nine complete synthetic message replay assessments per variant
still stop on unverified motion; the separate observed-corridor integration
fixture demonstrates a positive proposal and a justified G0/G1 downstream
difference. Neither result is native qualification.

**Phase 6 experiment:** Classical then learned-model MPC optimize bounded
two-segment position/yaw schedules, checked by D3 and diagnostic-only. Classical
MPC works on clear fixtures; learned-model MPC misses the 50 ms allocation in
clear cases in the saved run. D5 shows more progress in the clear corridor.
No optimized schedule can enter the existing command gateway.

**Selection and open gates:** G0/D0 remain configured defaults. G1 is the simpler
experimental candidate; G2 and physics-informed residuals are not promoted.
G3, yaw-only inspection, full-attitude visibility, P2/prior integration, native
clock/cadence/H1/shadow qualification, concurrent Jetson profiling, closed-loop
completion and physical flight remain open. Physical climb/tracking work remains
a separate priority. Repository documentation is authoritative for exact paths,
configurations, source/artifact hashes, test totals and rollback.
