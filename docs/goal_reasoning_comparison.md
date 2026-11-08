# Goal reasoning comparison: offline implementation and evidence

This implements the authorized offline comparison on canonical Hybrid, without
changing its selected G0/D0 defaults. The [Phase 0 audit](goal_comparison_audit.md)
records provenance and donor decisions. Results live under
`artifacts/goal_comparison/`; historical A1/A2, native recordings and prior
reasoning results remain separate and unchanged.

## What runs

| Unified roadmap phase | Implemented here | Evidence boundary |
| --- | --- | --- |
| 0 | Fresh isolated branch, every available branch head and PR audit; baseline tests | Local/remote source evidence |
| 2/4/6 | Bounded inspection candidates, immutable input envelope, G1 utility, G2 MLP, independent resolution and D3/D5 | Offline synthetic tests |
| 3 | Matched D1 residual training with/without closed-loop physics consistency; causal A1 replay | Small synthetic predictor ablation |
| 5 | Configured H1/H2 reasoner path and component timing | Synthetic messages, not native qualification |
| 6 | Classical and learned-dynamics two-segment MPC diagnostic search | No execution artifact or gateway permission |
| 7/8 | Existing gates preserved | Native, closed-loop and physical results still absent |

The existing `baseline.py` is unchanged. The G0 comparison therefore preserves
its policy, including poor choices under missing support. All variants share
the same inspection resolver and validator, so a baseline proposal does not
automatically become movement toward an opening center.

## Contracts and actual behavior

`ReasoningContext` remains schema v1, with unknown height/connectivity preserved.
Existing v1 `GoalProposal` wire payloads and reason codes remain compatible.
A narrowly defined **proposal v2** adds `no_supported_inspection` only for an
`insufficient_information` result. A v1 reader should reject v2 rather than
silently reinterpret it. Tests cover old round trips and rejection of a v2
abstention falsely labeled v1. Neither version permits safe-traversable claims.

`InspectionInputs` v1 binds the context hash, frame/clock/epoch, map observation
and capture, full camera calibration, estimated camera pose, state capture,
alignment provenance, stage, previous target and at most 32 attempt outcomes.
`Selection` binds those inputs, proposal ID, model/configuration identity, score
components and selected target under a digest. Hashes establish identity inside
the trusted process, not authentication for an external sender.

The resolver reconstructs its inputs from current state/snapshot and refuses
changed pose, calibration ID, context, epoch or expired evidence. The semantic
context is independently rebuilt by D3 as before. Movement paths and stationary
holds remain subject to estimator and swept-volume checks; no collision evidence
comes from confidence, a prior map or the neural network.

Candidate generation projects the known opening center using camera intrinsics
and mounting. It proposes a body position at most 0.5 m away, maintaining an
estimated standoff, or reobservation from the current position. Missing upper
and lower boundaries remain explicit objectives. The current pose conversion
uses a **level-body yaw approximation**; visibility is a heuristic, not measured
full-attitude visibility or calibrated information gain. Candidates outside the
camera field of view abstain. D5 has no reviewed yaw-only inspection primitive;
orientation-only movement is deliberately unsupported. Other grounded openings
are available to ranking. A current-position inspection can still produce a D5
stop because D5 requires verified translational progress.

G1 first excludes blocked, stale, contradictory, unsupported and exhausted
candidates. It scores normalized mission-target support, missing-boundary gain,
movement cost, uncertainty proxy, failed attempts and switching with weights
`(1, 2, -0.25, -0.75, -1, -0.20)`. These are declared heuristic choices.
Distance affects observation cost, never route connectivity or checkpoint
progress. Three unsuccessful observations exhaust a candidate. History is
bounded to 32 records/10 seconds, clears on stage/reference change and does not
reset merely because the policy abstains. H1 infers acquisition only from an
actual reduction in reported missing evidence on a new observation; it does
not treat an issued goal as success.

CourseMission handles inspection as a temporary objective before its arrival
logic. It cannot advance a checkpoint and uses the existing bounded observation
timeout for terminal behavior. D5 generates and the gateway reconstructs the
same inspection-specific schedules. Experimental G1/G2 configurations are
rejected by `hybrid_sitl`; they are read-only shadow choices.

## Reproduce and configure

From the repository root, with the existing NumPy/pytest installation:

```sh
python -m pytest -q tests
python -m examples.train_goal_ranker
python -m examples.compare_goal_reasoners
python -m examples.compare_goal_shadow
python -m examples.h0_hybrid_smoke
python -m examples.compare_physics_residual
python -m examples.compare_diagnostic_mpc
```

No ROS process, motor command, network model download or GPU is involved.
Add the selected `reasoner` object from `artifacts/goal_comparison/configs/` to
an H1 profile; omitting it selects G0. G2 requires the exact checkpoint SHA-256.
Checkpoint loading occurs once during construction, not in the estimator tick.
Missing, malformed, mismatched or nonfinite models and expired inference results
fall back to a newly evaluated G0 proposal, still subject to independent gates.
Inference is a fixed-size local matrix calculation. Deadline checks reject late
results but cannot preempt a Python call; concurrent native scheduling remains
unverified. No unbounded inference queue or asynchronous network call was added.

## G2 training and feature schema

`learned.py` feature version 2 has 16 per-candidate inputs: six normalized G1
terms, age/stale horizon, observation count/8, missing-height mask, width/4 m,
verified bounded-by relation count/4, support count/8, context-omission mask and
three relative coordinates/6 m clipped to [-1,1]. Training-set mean and standard
deviation (minimum 0.001) are frozen in the checkpoint. IDs, map filenames,
layout/run names and candidate order are excluded. A 16→16→1 tanh MLP has
289 trainable parameters. Eligibility and proposal validation stay outside it.

The reproducible synthetic generator has 100 training layouts, 30 validation
layouts and 60 final layouts (IDs 300–359). Entire layout/run IDs are separated.
The abstract observation environment does not simulate dynamics regimes; D0 is
held fixed in the separate motion integration. Three training seeds use the same
300-epoch maximum and validation-only epoch selection. The teacher uses synthetic
expected measurement outcomes with hidden occlusion; this is not G0 imitation.
The analytic synthetic observation probabilities are not calibrated real-camera
probabilities. No evaluator truth is imported by runtime modules.

An exploratory evaluation used layouts 200–259 before adding the explicitly
requested relative-geometry features. Its result is retained separately; final
selection evidence uses untouched layouts 300–359. The final holdout was not
used to tune weights or choose a training seed. DeepSets was not added: G2 has
not established an advantage that justifies another encoder.

## Goal comparison findings

Final paired matrix: **60 layouts × 7 regimes × 6 interaction steps**, or 420
episodes/2,520 decisions per variant. Regimes include nominal alternatives,
occluded dead ends, stale evidence, contradictions, absent support, epoch reset
and context omissions. Existing seven-frame replay separately checks blocking,
occlusion and reobservation. Inherited tests cover frame/clock reset, new occupied
evidence and deadline cancellation. Wrong-map prior robustness is **not measured**:
this branch does not ingest P2/static priors as policy inputs.

| Variant | Useful evidence items acquired | Paired mean gain over G0 per episode, 95% layout-bootstrap interval |
| --- | ---: | --- |
| G0 | 68 | Reference |
| G1 | 260 | 0.457 [0.367, 0.543] |
| G2 seed 11 | 259 | 0.455 [0.374, 0.538] |
| G2 seed 29 | 244 | 0.419 [0.343, 0.500] |
| G2 seed 47 | 243 | 0.417 [0.336, 0.502] |

G1 is the simplest best-performing experimental choice by mean acquisition.
G2 seed 11 is statistically indistinguishable from G1 on this sample; its paired
interval versus G1 spans zero. No seed establishes a G2 advantage. G0 remains
the configured default because native resource, mission and deployment gates
have not passed. This is a negative result for adding learned ranking complexity,
not proof that learned ranking can never help.

The report includes abstentions, unnecessary abstentions, repeated observations,
switches, invalid targeted proposals, independent validator rejections and
p50/p95/p99/max timing. G0's invalid targeted proposal rate is 28.57% in this
deliberately adversarial matrix; independent validation rejects those proposals.
G1/G2 have zero invalid targeted proposals. Their 14.29% validator rejection rate
is the explicit non-action response to invalid epoch contexts, not unsafe motion.
Repeated observations are not automatically failures: total attempts also rise
when useful candidates are selected. Both acquisition and repetitions are kept.

Bootstrap resamples **layout-level paired means**, not individual frames or
training seeds as independent samples. Replay is reported separately and cannot
claim viewpoint benefit. Interactive results count abstract acquired boundary
evidence; mission progress, recovery, physical collision and completion are null.
Accuracy uses a fixed logical deadline clock for reproducibility; separate wall
timings count missed allocations. H2 and adversarial tests enforce real deadlines.

The H1/H2 message replay produced **9/9 stops for every variant**, preserving
the unknown-starting-volume limitation. Host inference was around 0.5–1.3 ms;
full H0 assessment around 15–19 ms in the saved run. These small sample counts
are not target-device capacity measurements. Context construction, inference,
spatial update, D3, D5, receipt review and total H0 timing are saved separately.
Python peak allocation is sampled; GPU utilization, concurrent CPU contention,
thermals and device memory are unmeasured. The independent positive corridor
test does propose a bounded inspection trajectory and proves different G0/G1
targets affect the H1→H0 planning path.

## Independent predictor result

The physics experiment reuses D1's 5,572-parameter residual MLP and A1 causal
replay. Both matched networks use 40 training/12 validation episodes, three seeds,
55 epochs, identical optimizer and rollout horizons (0.2/0.5/1/2 s). The soft
physical prior is the existing closed-loop position/yaw response:

```text
desired_velocity = saturate(position_gain * position_error)
nominal_acceleration = saturate((desired_velocity - velocity) / response_time)
corrected_acceleration = nominal_acceleration + bounded_residual
loss = mean((gate * residual - measured_target)^2)
       + 0.1 * mean((gate * residual)^2)
```

Yaw uses the corresponding bounded yaw-rate tracking law. This implements an
explicit nominal-response consistency loss; it is not thrust-driven dynamics or
a strong mechanistic PDE/PINN claim. The model does not resolve unknown command
delay; A1's effective-command/history contract is retained.

The saved comparison has four episodes per regime across nominal, in-range,
shifted, switched, delay and noise. It includes existing D0/D1/history-MLP/GRUs,
ridge and A2 learned-feature/gated-linear predictors.
Physics loss lowers matched-network nominal position error but hurts shifted
and switched performance for all three seeds. At one second nominal RMSE is
0.123–0.139 m for physics models versus **0.0127 m for D0**. Shifted physics
error is 0.226–0.238 m versus 0.212–0.223 m for matched plain models. Ridge wins
shifted position here but loses to D0 on delay (0.208 versus 0.190 m). Heading and
two-second errors are retained in JSON. Neither new model is promoted. Historical
A1/A2 nominal/delay/heading failures remain relevant; this small ablation is not
a complete rerun of every historical A2 gate.

The included A2 learned-feature model matches D0 on nominal data and improves
shifted position, but has worse delayed heading (0.331 versus D0 0.244 rad at
one second). Its gated-linear ablation is slightly better on shifted position
in this small sample. Those negative complexity/heading results are retained.

## Classical/neural MPC diagnostic result

Bounded coordinate search optimizes two position/yaw command segments, seven
candidates per iteration and three iterations. D3 independently rejects unknown,
occupied and insufficient-clearance sweeps. D0 runs first; the same optimizer
then uses the learned residual model. Solver failure/infeasibility/invalid values
and 50 ms deadline expiry return no candidate. It is derivative-free, so there
is no unvalidated derivative path. Results use `DiagnosticOptimization`, not
D5 IDs, and the gateway rejects that type.

On eight existing planner fixtures, classical MPC produces checked diagnostics
in clear cases and refuses unsupported cases. Learned-model MPC exceeded the
50 ms allocation on the clear cases in the recorded run. D5 retained greater
one-horizon progress in the clear corridor (about 0.699 m versus 0.301 m for the
two-segment optimizer). This is a small, unequal-search-cost exploratory test,
not a general solver ranking. No optimizer is selected for execution.

## Remaining gates and rollback

G3 is deferred: the evaluator has a toy synthetic observation model, but the
runtime has only heuristic visibility and no validated belief transition model.
Yaw-only inspection, full-attitude viewpoint prediction, verified topology,
P2/static-prior reconciliation, real observation outcomes, native ROS replay,
target concurrency/resource profiling, closed-loop mission completion and
physical flight remain unverified. The physical climb/tracking work remains a
separate deployment priority. No freshness threshold or command owner changed.

Rollback: omit the H1 `reasoner` setting or select G0 to disable G1/G2. D0 is
still the predictor default; do not supply experiment overrides. MPC has no
runtime hook to disable. To restore the old center-based observation behavior,
return to the audited base in a separate checkout; do not partially remove the
inspection identity/gateway reconstruction checks. No shared branch was rewritten.

For exact executed test totals see `artifacts/goal_comparison/tests.txt` and the
source/artifact hash manifest. The prepared Notion update is
[`goal_comparison_notion_update.md`](goal_comparison_notion_update.md); it is not
published and does not overwrite historical roadmap evidence.

## Reused and new source map

Reused unchanged: WorldState extraction/context construction, G0 policy, D0
closed-loop predictor, residual inference, history/ridge/A2 models, D3 collision
queries, H1 sensor health contracts and single command owner. Adapted: semantic
resolution and D5 candidate reconstruction, H1 selector/history, CourseMission
inspection handling and optional H0 predictor arguments. Newly implemented:
inspection/selection/bridge/learned modules, goal data/training/comparison runners,
matched physics-loss option and diagnostic coordinate-search MPC. The final
manifest identifies every changed source and every saved result/checkpoint.
