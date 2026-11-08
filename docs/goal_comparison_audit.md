# Goal comparison: Phase 0 audit

Audit date: 2026-11-07 Pacific. New isolated worktree and branch:
`goal-reasoning-comparison`, `codex/goal-reasoning-comparison`. No existing
worktree, shared branch or flight configuration was overwritten.

The exact full heads, branch-relative commits, changed paths, local unpushed
commits and worktree locations are in
[`audit.json`](../artifacts/goal_comparison/audit.json). A successful remote
fetch preceded choosing the base. GitHub pull-request metadata was read through
the public API and saved in [`pulls.json`](../artifacts/goal_comparison/pulls.json).
The initial restricted-network fetch failed; the authorized retry succeeded.
`gh` is not installed. PRs 19–21 are closed, 22–24 remain open. No PR was merged.

## Base and donor decisions

Canonical Hybrid remains `integration/unified-sitl-readiness` at
`bfb09c8a` (full SHA in the manifest). Its H0–H3 squash `675ed13` has the same
tree as historical `aa215e26`; later commits add runtime, Gazebo integration
and native disarmed evidence. It is **5 commits ahead / 4 behind P2-C/D**.
P2 is not silently assumed merged.

| Branch | Purpose and inspected donor content | Reuse decision |
| --- | --- | --- |
| main | `ba299397`; Phase 1 specification targets `drone_code/team_nav`; source launch, staged mission and organizer wrapper | Separate Classical ownership/interface; not a Hybrid successor |
| integration/unified-sitl-readiness | `bfb09c8a`; H0/H1/H2/H3, CourseMission, guarded executor, Gazebo reports | Selected base |
| integration/hybrid-shadow-to-sitl | `aa215e26`; historical H2/H3, shadow and gateway | Already consolidated by tree equivalence |
| feature/h0-hybrid-contract | `d49cbb3e`; H0 assess, receipt validation, smoke tests | Already consolidated |
| feature/h1-native-observation | `1cb9c80d`; message clocks, state/geometry adapter | Already consolidated |
| feature/p2-a-reference-map | `e2425802`; PLY preprocessing and provenance | Available offline donor; not goal-selection input |
| feature/p2-b-map-localization | `90505e22`; capture-time ICP and anchor validation | Diagnostic correction only; not fresh collision evidence |
| feature/p2-cd-spatial-shadow | `e4274676`; topology, voxel queries, H2 sidecar | Diagnostic only; no verified route connectivity |
| cleanup/repository-audit | `3e89e1ba`; consolidation audit/history | Documentation provenance |
| classical | `56f5d323`; height and tracking changes, telemetry | Physical deployment priority; no threshold transplant |
| classical-dannie-changes | `61ec18ed`; comparison reports and divergent Classical work | Reference documentation; no blanket merge |
| dannie-classical | `79d4da2f`; depth/SLAM proposal | Historical proposal, not evidence of working Hybrid |
| dannie-classical-flyx | `c774ea69`; non-arming diagnostics and preflight guide | Documentation/tools only for this task |
| dannie-classical-flyx2 | `edd57eb0`; physical window-height warning | Preserve prior-vs-observed distinction |
| dannie-classicalimplementatioon | `2c2eee84`; detailed Classical Phase 1 specification | Design reference, separate package |
| fix/depth-no-return | `16ca455d`; uint16 65535 rejection in legacy decoder | H1's separate rectified decoder already rejects it; legacy fix not silently merged |
| soham-depth | `7781aff8`; four_clearances, no-return handling | Conservative probe ideas; no duplication of D3 |
| soham-mission | `dd09b79b`; stage/recovery pure state machine | Do not import its “rotation needs no clearance” assumption; reuse CourseMission |
| soham-stages | `3351c87d`; team_nav staged runtime and diagnostics | Separate command contract; not a Hybrid reasoner |

All 19 recorded remote heads remained at their recorded anchors after fetch.
The manifest additionally inventories local branches. In particular,
`codex/physical-height-regressions` and `codex/physical-p2-integration` contain
unpushed teammate work. The latter adds P2 reconciliation, a bounded worker,
disabled planner pose input and physical tracking evidence. It is preserved as
an independent integration candidate rather than treated as an approved new
canonical base. Its changes overlap `examples/h1_observation.py`,
`examples/hybrid_shadow_loop.py`, spatial input/pipeline/volume and H0 tests;
those need explicit reconciliation before combining branches.

## Capability and evidence checks

Read `AGENTS.md`, `AI_CONTEXT.md`, `CLAUDE.md`, README and runtime documents.
No nested AGENTS files exist in this checkout. `docs/READABLE_CODE.md` is absent
on the canonical branch; its current Classical version was read and followed.
`drone_code/TEST_SEQUENCE.md` is also absent on this base. Relevant source files
were read before editing. The user-specified seven Notion sources were fetched;
the roadmap's newer consolidation/P2 updates supersede older design checklists.
R3/R6/R9/R10/R11/R12 are research prompts, not finished findings.

The inspected source paths include all nine requested reasoning/planning/runtime
modules, `refnav/dynamics/*`, spatial observation/pipeline/volume and WorldState,
H1/H2/runtime entrypoints, baseline/validator/trajectory/planner/gateway tests,
the main Phase 1 specification and P2 spatial-shadow documentation. Exact donor
path inventories are recorded per branch in the audit manifest.

Verified before edits: G0 returns mostly observe/reject/insufficient; v1 forbids
connectivity and safe declarations; H1 directly constructed G0; CourseMission
owns progression/recovery; D5 has eight fixed candidates and a 25 ms budget;
H0 did not expose predictor/history options; the gateway reconstructs D5 IDs;
semantic resolution used entity centers and rebuilt context hashes.

Existing assets: D0, D1 residual MLP, D2 history MLP/GRUs and online ridge;
A1/A2 data generators, causal replay, checkpoints and negative results are
present. Python/NumPy/pytest run locally; no additional ML dependency is needed.
Native ROS deserialization, live Gazebo/PX4, target Jetson and aircraft evidence
were not available in this Windows offline session.

Baseline command: `python -m pytest -q tests` — **423 passed in 34.80 s**.
No baseline failure was hidden or repaired. Native disarmed captures remain
historical evidence: the frame fix did not close cadence, clock alignment,
H1 qualification, continuous shadow, handoff or closed-loop flight gaps.

## Actual call graph

```text
H1 ROS-shaped observations → SpatialWorld → immutable SpatialSnapshot
H1 select_goal → build_context → inspection candidates → configured G0/G1/G2
  → GoalProposal + hashed Selection → independent proposal validator
  → inspection resolver (rebuild state/map/camera identity)
  → D5 observation-objective schedules → D0 prediction → D3 swept checks
  → H0 receipt review → H2 read-only diagnostic

CourseMission retains checkpoint/recovery/terminal ownership.
Guarded execution reconstructs the same D5 schedules and rechecks D3.
MPC diagnostics have a different result type and never enter that gateway.
```

P2 map correction/topology remains a diagnostic sidecar on its donor branch;
it does not affect this planner. In this change the selected entity **does**
affect the resolved metric viewpoint, D5 schedule and D3 score. Tests demonstrate
different downstream objectives through H1 with G0 versus G1.
