# Batch 008: conditional liftoff transition and calibration preparation

Frozen source `2ecbee3281727cff8c3791068f3503e66edf2bcf`, archive SHA256 `87e8781e0e26492b0f77e733a0d54cdd6bfc17c7e3016947c9538f5cff350e88`. Independent archive verification matched all 403 manifest members. Fresh native C1/R1 disarmed checks completed with exit code 0; the independent full-rate raw audit passed both recordings. No policy flight, calibration flight or approval artifact was created during this preparation. Classic remains protected. The work implements a conditional model-backed startup contract; it does not certify the numerical assumptions needed to enable that contract.

## Liftoff change

Batch 007 exposed the discontinuity where the conservative body envelope intersects the floor immediately after the native landed flag clears. The optional `ContactTransition` now distinguishes a designated support contact from other body collisions. It binds a fixed contact anchor, bounded support region, relative-height error, downward tracking excursion, attitude/rate reach and control horizon. Source, reference/reset identity, sample freshness, command lead, drift, rise, speed and both clock deadlines remain checked. A violation revokes the session; airborne handoff is irreversible. Non-support collisions, side/ceiling obstacles and ordinary airborne floor clearance remain obstacles.

Native attitude admission preserves source capture time and reset identity, rejects malformed/non-unit quaternion samples and clears admission on invalid input. The output fence checks the actual generated startup output, including the tick that changes mission phase. These are code and regression changes, not measurements of contact behavior, tracking error or recovery reach.

**No reviewed contact-transition deployment profile is enabled.** Sampled attitude/speed limits can contradict model assumptions but cannot establish continuous error/reach bounds. The binder requires exact transition-file identity and explicit deployment authorization; synthetic test parameters are not a reusable flight profile. Unconfigured deployments retain conservative refusal. Native landing/recovery admission, measured braking and recovery, reachable-speed bounds, participant survey/occupancy lease and owner timing remain separate S0 gates. [S0 proposal](S0_BOUNDED_FLIGHT_PLAN.md).

## Offline verification

The final frozen-source tracked suite passed **817 tests in 44.01 s**, including the real-`Mission` startup regression. Final parent focused checks passed **46 in 4.36 s**; independent launch/binder/stopping checks passed **34 in 1.04 s**. The earlier 816-pass run predates the last regression and is not the final count. [Validation record](../artifacts/taskforce_batch_008/validation.txt).

The calibration audit independently ran seven test modules: **117 passed in 1.09 s** on the pre-freeze inspected source. [Validation record](../artifacts/taskforce_batch_008/calibration-validation.txt). This verifies state-machine, recorder and audit behavior offline. It does not establish native braking, expired-owner recovery or a positive policy motion case.

## Refreshed C1/R1 package

[The concrete approval packet](BRAKING_RECOVERY_APPROVAL_008.md) preserves the bounded C1 0.20 m/s northward brake at 1.50 m and the separate R1 stationary 250 ms authority-expiry test. Parent selected one new immutable source for both, with the previously reviewed operator helper included in-tree, RTX rendering, fresh separate evidence directories and DDS domains 57/58. Calibration remains a deterministic velocity-command owner; the new Hybrid contact/map/scan mode does not control it.

The retained template records source/archive/profile/recorder/geometry/helper hashes. The finalized release manifest additionally binds both fresh native checks and their independent raw audit. It is not an executable approval. Historical C1/R1 source archives and native failures/checks remain retained. The new frozen source may supersede their deployment choice only after fresh disarmed checks and independent review.

Fresh C1/R1 checks ran on the frozen source in domains 57/58 and both exited 0. C1 recorded 44.032 s, 3,236 strict position samples and 64 statuses; R1 recorded 43.944 s, 3,270 positions and 65 statuses. Both had zero recorder faults, pending samples or trial events. `recording_complete=false` is expected because neither test armed or performed a landing transition. Both actual renderer logs identify `D3D12 (NVIDIA GeForce RTX 5070 Laptop GPU)`. Effective landing speed remained unchanged during checks. The independent raw audit decoded 87 statuses per recording, all disarmed, and found no recorded control messages. These are composition results, not stopping envelopes. [Raw audit](../artifacts/taskforce_batch_008/disarmed-raw-audit.json). [Native summary](../artifacts/taskforce_batch_008/native-summary.json).

The first external R1 runner preflight failed because its synthetic operator dry-run output directory was missing. The runner was corrected without changing frozen flight source, and the failure is retained under `taskforce-flight-prep-008/r1-preflight-failed`. It was not a flight failure or a successful native recovery trial.

The requested approval sequence remains C1 only, independent raw braking review, then a separate R1 approval and independent raw recovery review. Neither result automatically approves S0. The user must separately approve each concrete moving configuration; this batch performs only preparation and disarmed validation.
