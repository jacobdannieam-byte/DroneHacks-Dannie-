# Migration plan

The cleanup proceeds in incremental PRs. The documentation stays in
[draft PR #16](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/16)
against the SITL readiness branch. The first source change is the
[depth decoder fix](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/17).
Use readiness at `78ef913` as the comparison baseline and the pinned
checkout for the first disarmed capture. Source landing requires the
relevant owner review and the evidence listed below.

## Branches to use

- `main` is the stable managed ROS 2 reference.
- `integration/unified-sitl-readiness` is the managed test checkout for
  world semantics, dynamics prediction, hybrid planning and ROS 2 diagnostics.
- `classical` is the separate Docker/Humble emulator checkout. Review the
  staged mission there in [draft PR #18](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/18).

The published `Dannie-experimental` and `baseline-original` refs were
removed after checking all open PR bases and heads. Their commits,
`feb55b6a2485fb9f1aedeada371e8ca2abf96cd6` and
`c3c5df33bea400209e4bec95ba99a3acdb6d67f8`, are reachable from `main`.

## Recommended branch outcomes

| Ref | Outcome after acceptance | Reason |
| --- | --- | --- |
| `main` | Keep as stable shared managed code | Native reference mission and the common companion/perception stack. |
| `integration/unified-sitl-readiness` | Keep through native qualification | One test checkout for Phases 1–8. Its head already preserves both research lines. |
| `classical` at `1f60fb5` | Keep the existing ref; consider `strategy/classical` only after independent launch acceptance | The Humble/SIH emulator uses the organizer command interface and emulated observations. Keep the dependency pins and simulation fixtures together. |
| `dannie-classical` at `79d4da2` | Review with Dannie, retain the proposal, resolve script modes in the proper lane | The distinct change is a proposal plus executable-bit removals on four scripts. The runnable harness is inherited. |
| `soham-stages` at `0ef71ce` | Review into `classical` | Two commits add the `team_nav` ROS 2 package to the existing emulator. The package's offline tests pass. The staged flight still needs an isolated emulator run. |
| `soham-depth` and `soham-mission` | Preserve for focused owner reviews | Directional probes and course stages are independent contributions absent from readiness. |
| Hybrid | Start as a configuration of the managed stack after native acceptance | Uses the same state, metric map and command owner. A separate branch would add maintenance without a separate runtime. |
| Neural and RL | Defer branches | No independently runnable strategies found in the audited refs. |

The 14 feature refs and the Phase 4 integration base were retired after
verifying that all 15 tips are ancestors of readiness at `78ef913`.
PRs #1–14 were closed as superseded by
[draft PR #15](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/15),
which now targets `main`. The [branch inventory](BRANCH_INVENTORY.md)
retains the original branch names and commit IDs.

## Integration sequence and PR boundaries

1. **Collect the disarmed baseline.** Run the strict idle smoke on the
   authenticated managed companion at `78ef913`. It checks landed/disarmed
   telemetry, ownership, clock, calibration, VIO and EV. Save the report,
   capture and representative native depth samples with the source SHA.
   Record the depth encoding and how the camera marks missing returns
   before reviewing a moving trial.
2. **Review the depth decoder before a moving trial that uses saturated
   16-bit depth.** [PR #17](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/17)
   takes the `65535` fix from `soham-depth@7781aff` into the legacy
   decoder. The regression covers both 16-bit encodings, padded rows,
   valid depths and corridor coverage. The patched main baseline passed
   32 tests. The same two-file patch on readiness passed 335. Review the
   native sample alongside the patch. If the source uses the saturated
   code for missing depth, qualify the decoder fix in the moving checkout
   before arming. Review directional probes and any broader decoder
   unification separately with calibrated native fixtures and byte-order
   tests.
3. **Review the integrated draft by dependency.** In PR #15, review
   localization, world state and spatial replay, then dynamics and
   predictor selection, and then the trajectory, planning and handoff
   diagnostics. Use the superseded PRs for focused diffs and benchmark
   evidence. Keep source commits, checkpoints, seeds and benchmark
   contracts with their changes.
4. **Review runtime evidence before landing the integrated draft.**
   Use the Phase 4 merge at `d6c363c` and readiness at `78ef913` as
   comparison points. Review any future path that hands planner
   output to the PX4 command owner separately.
5. **Review the two classical lanes with their owners.** For the
   `classical` emulator, verify the documented build and landing command
   at `1f60fb5`, archive the image digest, OBJ, referee outputs and ULogs,
   and keep the organizer flight interface isolated. Review
   `dannie-classical`'s proposal and executable modes without replaying
   its inherited simulator commit. For `soham-mission@dd09b79`, preserve
   `refnav/stages.py`, `test_stages.py` and the core handoff evidence.
   Review line, flow, range and pad inputs and their time/frame contracts
   before adapting the stage policy. Wire a selector only after the
   baseline parity and owner tests pass.
6. **Qualify the managed hybrid in separate changes.** In order, verify
   moving Phase 1 timing and estimator fusion, Phase 2 native geometry,
   Phase 4/6 course replays and Phase 5 verified effective-command and
   Orin evidence. A separate Phase 7 change would connect validated
   proposals inside the existing owner after review. Require the staged
   isolated SITL trials and failure injections before Phase 8 runs.
   Keep D0 selected and retain ridge and learned models for comparisons.
   Native route, no-route recovery, landing and authority tests must pass
   before treating a configuration as independently runnable.
7. **Retire only after evidence and approval.** Recheck ownership, recent
   commits, worktrees, external asset availability and all open PR bases.
   Mark approved archival refs and publish the preserved SHAs before a
   separate deletion decision. Review local `experimental` and
   any remaining team branches with their owners.

## Conflicts and validation

- The published readiness delta and each of the four classical/Soham
  deltas have no common changed paths against `main@feb55b6`. That
  reduces textual overlap at the audited snapshot. It leaves runtime
  and safety interactions to review. In particular, the organizer's
  auto-Offboard interface and managed `refnav.node` are separate owners,
  and the optional Soham core hook touches supervisory control flow.
- The semantic world-model review in PR #1 forked one commit before
  the native baseline. Readiness preserves the native notebook, semantic
  commits, Phase 4 merge and Phase 1 merge parents.
- Future README and dependency changes may overlap the classical lane.
  Scope any consolidation to emulator files and review shell modes,
  container builds and source pins on their own. Retain its OBJ and
  generated table recipe. A textually clean merge is insufficient.
- Keep source tests with their modules. Before and after each approved
  merge, compare the pinned baseline and result with T1, the affected
  focused suites and the exact relevant runtime commands in the
  [simulation matrix](SIMULATION_MATRIX.md). Reproduce deterministic
  output, verify checkpoint and split hashes, then run the native
  scenarios when the runtime is available.
- Any flight-code conflict goes to the owners of the affected gate.
  Preserve estimator/depth freshness, Offboard priming, watchdog, landing,
  takeover and failsafe behavior. Keep simulator truth in the grader.

## Archival and rollback

Record full SHAs for every source, target, PR head and merge result. Obtain
separate approval for annotated archive tags and protect their refs where
the repository supports it. Verify that a fresh fetch can resolve each
archive and recover the pinned data and launch recipe. The retired
baseline and milestone tips are reachable through `main` or readiness.
Preserve active team branches until their owners confirm the archive and
PR dependencies are clear.

Use the last accepted build SHA and matching runtime configuration for
simulator rollback. Revert a faulty merged PR with a new reviewed commit
after checking dependents. Retain evidence and authorship. Keep the
pre-change worktree and managed entrypoint available for parity tests.
