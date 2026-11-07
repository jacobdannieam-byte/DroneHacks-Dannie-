# Cleanup checklist

## Current review

- [x] Snapshot the local and GitHub refs, tags, worktree and open draft PRs.
- [x] Test each distinct tip in an isolated archive and preserve simulator
  geometry and organizer test outcomes with runtime blockers.
- [x] Record capabilities, branch ancestry, owners, path deltas and PR bases.
- [x] Publish the documentation cleanup for draft review.
- [x] Verify identical patches and retarget the three starter drafts
  #1, #3 and #7 to `main`.
- [x] Publish the decoder regression and fix in
  [draft PR #17](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/17).

## P0 — Establish the integration baseline

- [ ] Review the audit with the repo maintainers and affected branch owners.
  Confirm the exact landing plan before changing the shared tree.
- [ ] Run the landed/disarmed smoke on `integration/unified-sitl-readiness`
  at `78ef913` and retain the report and capture.
- [ ] Review the native camera encoding and missing-return values. If it
  uses saturated 16-bit values, review and qualify PR #17 before the
  approved moving trial. Record takeoff, hover, landing and moving timing
  on the exact tested revision.
- [ ] Keep an explicit source, config, PX4/ROS build and artifact manifest
  for each accepted runtime. Verify checkpoint and split hashes before
  any branch is considered superseded.
- [ ] Define a pinned offline CI environment before adding workflows.
  Run the existing tests on clean checkouts and keep managed and
  emulator flight jobs separate from host CI.

## P1 — Review shared changes and the two runtimes

- [ ] Review the semantic and dynamics stacks plus Phase 1 in the order
  in the [migration plan](MIGRATION_PLAN.md). Reuse their existing joins.
- [ ] Review Soham's directional probes and staged mission separately.
  Require native detector and timing evidence before enabling
  `use_stages`.
- [ ] Confirm `dannie-classical`'s script mode changes with Dannie.
  Reproduce the `classical` Docker flight with Kenny's pinned runtime
  and record image/asset references, referee output and ULogs.
- [ ] Review native Phase 2 depth/calibration, Phase 5 applied commands
  and Orin, and the Phase 6 course replay. Resolve simulator-specific
  gate failures before connecting any proposal to control.
- [ ] Review the existing owner's exact Phase 7 proposal handshake and
  fallback tests. Require an approved isolated trial and Phase 8 paired
  evidence before declaring a runnable hybrid configuration.

## P2 — Archive approved history

- [ ] Confirm all original commits, assets and branch owners. Document
  which accepted refs include each capability.
- [ ] Review open PR bases and dependencies before tagging or retiring refs.
  Confirm the proposed archive SHAs can be checked out from a fresh clone.
- [ ] Keep the separate classical runtime until it can be reproduced by
  explicit configuration in the common repository.
- [ ] Approve any future deletion, history or flight behavior change
  separately after reviewing its exact scope.
