# Migration plan

**Status: proposed.** The first review is the documentation-only
`cleanup/repository-audit` draft against `integration/unified-sitl-readiness`
at `78ef913`. The following integration steps require maintainer and
affected owner review. Keep native test collection on the pinned readiness
revision while reviewing the source changes.

## Recommended branch outcomes

| Ref | Outcome after acceptance | Reason |
| --- | --- | --- |
| `main` | Keep as stable shared managed code | Native reference mission and the common companion/perception stack. |
| `integration/unified-sitl-readiness` | Keep through native qualification | One test checkout for Phases 1–8. Its head already preserves both research lines. |
| `classical` at `1f60fb5` | Keep the existing ref; consider `strategy/classical` only after independent launch acceptance | The Humble/SIH emulator uses the organizer command interface and emulated observations. Keep the dependency pins and simulation fixtures together. |
| `dannie-classical` at `79d4da2` | Review with Dannie, retain the proposal, resolve script modes in the proper lane | The distinct change is a proposal plus executable-bit removals on four scripts. The runnable harness is inherited. |
| `soham-depth` and `soham-mission` | Preserve for focused owner reviews | Directional probes and course stages are independent contributions absent from readiness. |
| Hybrid | Start as a configuration of the managed stack after native acceptance | Uses the same state, metric map and command owner. A separate branch would add maintenance without a separate runtime. |
| Neural and RL | Defer branches | No independently runnable strategies found in the audited refs. |

All feature refs remain review anchors until the relevant PRs and archive
decisions are resolved. See the [branch inventory](BRANCH_INVENTORY.md) for
each named branch and immutable commit reference.

## Integration sequence and PR boundaries

1. **Audit and evidence.** Review this draft, then take a fresh ref and PR
   snapshot. Keep `main@feb55b6` and readiness `78ef913` as pre-migration
   baselines. Run the offline suites on both and run the strict idle smoke
   on the authenticated managed companion. The Phase 1 smoke checks
   landed/disarmed telemetry, ownership, clock, calibration, VIO and EV.
   Capture the corresponding run artifacts and preserve failure evidence.
2. **Review the existing shared stack in dependency order.** Review P1
   `feature/semantic-world-model-p1` #1 → I0–I4
   `feature/semantic-reasoning-i0-i1` #2 → Phase 2
   `feature/unified-phase2-spatial` #8. In parallel, review D0
   `feature/adaptive-dynamics-d0` #3 → D1
   `feature/adaptive-dynamics-d1` #4 → D2
   `feature/adaptive-dynamics-d2` #5 → A1
   `feature/adaptive-dynamics-a1` #6 → A2
   `feature/unified-phase3-adaptation` #9. Review Phase 1
   `feature/unified-phase1-localization` #7 separately. Keep all original
   owners, checkpoints, seeds and benchmark contracts.
3. **Reuse the existing joins for shared landing.** Phase 4 #10 already
   uses `integration/unified-phase4-base@d6c363c`, which merges the
   Phase 2 and Phase 3 heads. Its descendants are Phase 5 #11,
   Phase 6 #12, Phase 7 #13 and Phase 8 #14. Readiness #15 already
   includes the Phase 1 join at `5ef0e98`. After content acceptance,
   use a reviewed integration PR from a descendant of the accepted
   readiness revision to `main` or land approved slices through
   their original ancestry. Use regular merges where appropriate to
   preserve the source commits. Pick one landing approach after reviews
   and verify ancestry so no patch is replayed. Capture the exact merge
   parents and check runs. This is approval to land accepted modules,
   separate from any approval to activate the hybrid control path.
4. **Propose the shared depth fix in its own PR.** Compare
   `soham-depth@7781aff` and the Phase 2 decoder. Extract or merge the
   reviewed 65535 handling with a regression at the legacy
   `refnav.depth.decode_depth` entrypoint. Preserve the source commit and
   attribution. Verify row stride, big/little endian, zero and 65535
   no-return semantics, valid metric values and downstream coverage.
   Review directional probes as a separate capability with
   `test_depth_clearances.py` and calibrated native depth fixtures. Keep
   the original forward corridor and the Phase 2 bounded 3D map.
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
   separate deletion decision. Review deprecated branch pointers with
   their owners, especially `Dannie-experimental`, local
   `experimental` and `integration/unified-phase4-base`.

## Conflicts and validation

- The published readiness delta and each of the four classical/Soham
  deltas have no common changed paths against `main@feb55b6`. That
  reduces textual overlap at the audited snapshot. It leaves runtime
  and safety interactions to review. In particular, the organizer's
  auto-Offboard interface and managed `refnav.node` are separate owners,
  and the optional Soham core hook touches supervisory control flow.
- PR #1 forks one commit before the native baseline. Preserve the
  notebook on the target and the semantic commits on the head. Keep the
  Phase 4 integration base and the Phase 1 merge parents until their
  dependent reviews are reconciled.
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
archive and recover the pinned data and launch recipe. Example proposed
labels are `archive/baseline-original` and
`archive/phase4-integration-base`. Preserve active team branches until
their owners confirm the archive and PR dependencies are clear.

Use the last accepted build SHA and matching runtime configuration for
simulator rollback. Revert a faulty merged PR with a new reviewed commit
after checking dependents. Retain evidence and authorship. Keep the
pre-change worktree and managed entrypoint available for parity tests.
