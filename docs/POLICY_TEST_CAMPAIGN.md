# Curated policy test campaign after C0

Status: configured test sequence, not authorization for moving policy flights.
Current implementation direction is the explicit controlled-SITL map-assisted
mode in [the clearance plan](HYBRID_CLEARANCE_BOOTSTRAP_PLAN.md), with optional
initial/selected-waypoint scans. Sensor-only coverage remains a separate negative
control; completing a scan does not establish full-body clearance. The original
observed-only positive requirement below applies to observed-only qualification,
not as a mandatory handoff for a separately reviewed map-assisted run.
Setup source: `81ac530a029c7b39e3cef2e906b2eb7df8aedcf7`.
C0 passed only the bounded takeoff/settle/native-LAND path. Its evidence cannot
substitute for measured stopping, authority-loss recovery or spatial clearance.

## Comparison matrix

| ID | Configuration | Role and present execution boundary |
| --- | --- | --- |
| S1 | Protected Classic, `ad31ae55ebf7cd3b2eaf57917537242831e3c752` | User-confirmed baseline; preserve source and archived evidence; matched fresh comparisons still required |
| S0 | `scenarios/strategies/hybrid_existing.json` | Existing Hybrid G0/D0 control comparison; native activation still requires qualification |
| S2 | `scenarios/strategies/hybrid_information_gain.json` | Information-gain G1/D0; first experimental candidate after S0; shadow-only until reviewed promotion |
| S3 | `scenarios/strategies/hybrid_learned_ranker.json` | Pinned G2 ranker/D0; compare against S2 with identical observations and candidate sets; shadow-only |
| S4 | `scenarios/strategies/hybrid_physics_informed.json` | G1 plus pinned physics-informed residual; compare against S2's D0 predictor; shadow-only, D0-only gateway restriction retained |

The first campaign uses existing curated strategies. No new strategy is needed
to complete it. No LLM is introduced into the real-time control loop.

## Ordered execution stages

1. Offline consumer checks: run all four configured strategies through the
   existing H1/H0 fixture, with observed-volume and unknown-starting-volume
   cases. Require positive proposals and negative refusal, valid identities and
   no unintended model fallback. Completed in `artifacts/policy_test_setup_001`:
   all eight cases met expectations; portfolio tests: 8 passed in 1.02 s.
   These synthetic fixtures are not evidence of native sensor clearance or
   policy superiority, and the simple positive case does not distinguish goal
   selection quality.
2. Native disarmed shadow: use reviewed sensor calibration/TF, uncertainty,
   clock mapping, map identity/epoch and reset markers. Require a legitimate
   positive observed whole-body starting/stopping-volume case and negative
   unknown/stale/invalid-input refusals. Record selected goals, candidate
   rejection reasons, strategy/model hashes, fallback and timing. No command
   authority. Keep map control/background registration off. Spatial input must
   actually feed decisions; simulator truth remains evaluation-only.
3. Bounded calibration: prepare a separately approved C1 translation/braking
   plan with exact source/image/world/profile, speed/distance/altitude bounds,
   single owner, raw recorder, recovery and stop conditions. Measure stopping
   and repeatability; qualify owner/target expiry and clock/authority recovery
   in separately specified trials. C0 supplies no braking measurements. Do not
   create a fictitious measured envelope from configured limits.
4. Bounded policy execution: first S0, then separately reviewed S2 promotion.
   S3 and S4 remain shadow comparisons until their own implementation and
   qualification evidence supports promotion. Never bypass experimental-owner
   or D0-only execution restrictions to fill the matrix.
5. Matched course comparisons: pin common world/course, map permissions,
   vehicle, sensors, start, mission, seed, scoring and termination rules. Run
   each admitted strategy on the same predeclared scenario/seed set, with fresh
   worlds and one owner. Preserve failed attempts; stop the batch on first
   failure for diagnosis rather than silently retrying. Historical S1 success
   is context, not a fresh paired result. Final trial counts and exact moving
   configurations belong in the concrete approval package before execution.

## Metrics and acceptance

Record completion/checkpoints, native landing/disarm, elapsed simulation time,
path length, speed/position violations, observed whole-body clearance,
unknown-space refusals, recovery outcomes, source/receipt age, deadline misses,
compute load/real-time factor, selected goals and model fallback. Keep truth
metrics separate from policy-visible data. Missing evidence remains null, not
zero. Compare information acquired and useful progress separately from course
completion; inspection alone does not complete a checkpoint.

Retain raw bags, command/event correlation, manifests, pinned checkpoints and
independent review per attempt. The external comparison evaluator must reject
unmatched conditions. No superiority or hardware-readiness claim follows from
this setup or the synthetic smoke results.

## Reproduce the completed offline stage

From the repository root, run `python -m pytest tests/test_strategy_portfolio.py
-q -p no:cacheprovider`. For each configuration in the table, use
`python -m examples.run_strategy_smoke --strategy <configuration> --output
<new-output-path>`. Outputs are exclusive-create: choose a new directory for
each run and preserve previous evidence.

No simulator, ROS command owner or arming process was started during this setup.
Moving execution needs a concrete approved flight plan, as required by AGENTS.md.
