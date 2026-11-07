# Unified Phase 3 — adaptive prediction experiment

Phase 3 evaluates an optional frozen feature basis with bounded online
coefficients. It builds on `feature/adaptive-dynamics-a1@c36918e` (PR #6).
D0 remains the default and fallback. The existing online ridge fit is the
adaptive reference. Phase 1 and Phase 2 live gates remain separate.
[Unified roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) ·
[A1 baseline](adaptive_dynamics_a1.md)

## Model and causal contract

A2 uses the D0 tracking model, setpoint contract and semi-implicit integration.
It reads normalized nominal linear/yaw acceleration and velocity/yaw rate
from the existing D2 motion history. A frozen 8 → 16 → 8 ReLU/tanh MLP with
280 weights and biases adds two learned columns per axis to two classical
columns, nominal acceleration and negative normalized rate. Learned columns
vanish at zero demand and motion. The ablation zeroes these columns and uses
the same solver and gates.

For each axis, solve an exponentially weighted ridge problem over completed
20 ms transitions. Its target is normalized achieved acceleration minus
nominal acceleration. Forgetting is 0.90 per sample and the diagonal prior is
(0.30, 0.30, 0.70, 0.70) around zero residual. Four independent vectors have
bounds of [−0.65, 0.35] for the gain correction, [0, 1.5] for damping, and
[−0.35, 0.35] for each learned coefficient. A forecast binds the resulting
16 coefficients once and holds them fixed while rolling forward.

Eight to twenty completed observations are required. Per axis, the fit
requires 0.12 weighted energy in a classical column and a regularized
condition-number upper bound of 300. Fit a prefix and test its correction on
the most recent four completed transitions before allowing a full-window fit.
The held-back normalized MSE must improve over D0 by at least the greater of
0.0025 and 15%, with remaining MSE at most 0.16. The gate uses raw innovations
to expose inconsistent observations. A failed axis returns zero correction.
With no enabled axes the predictor returns the exact D0 result. The shared
D1 activity gate, residual caps, D0 acceleration bounds and yaw wrap still
apply.

Only measured endpoints and a setpoint effective throughout the transition
can enter `MotionHistory`. A gap or valid reference change starts a fresh
window. The caller must clear the history or advance the reference epoch on
an observed clock rewind. A2 inherits the existing D2 behavior for a late
transition spanning two epochs. It can clear the warm window, so the
predictor falls back to D0. The A1 replay explicitly tests this case.
No model owns ROS publication or PX4 commands.

## Reproducible experiment

The CPU trainer uses D2's 120 training and 24 validation episodes and causal
label construction. It differentiates through a ridge solve over the window
available at each training anchor and learns the basis by one-step normalized
acceleration residual MSE. The online eligibility gate is held fixed and is
evaluated separately in replay. Epoch selection uses validation MSE only.
Training exports the frozen basis to a versioned NPZ and checks PyTorch/NumPy
feature parity. Inference requires NumPy only.

The evaluator compares D0, D1, ridge, history MLP, GRU-32, GRU-64, A2 and its
gated linear ablation on all A1 cohorts. It requires the original fixture
fingerprints and exact original baseline metrics. Seed 202611470 generates a
new final holdout for in-range, shifted, switched, fixed delay, noisy and
matched-nominal regimes. The holdout excludes any A1 trajectory or random
dynamics regime. Nominal controls reuse the same physics on new trajectories.
The calibration split is kept fixed and separate from test and training.

The offline gate requires the A1 material improvement threshold of 20% over
ridge on clean shifted and switched 1 s pooled position in both comparisons,
with a paired bootstrap interval excluding zero. It also requires the A1
10% nominal gate across position and heading horizons, no fixed-delay
regression versus D0 at the one-second anchor, at least 80% holdout episode
coverage with zero false low-risk endpoints in shifted/switched/delayed/noisy
fixtures at 1 s, and 10 ms p95 for an active eight-candidate batch. Coverage is
an empirical offline diagnostic. Native and Orin qualification require
separate evidence. The reports include 0.2–2 s position/heading endpoints,
tails, post-switch curves, axis activation counts and timing distributions.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m pytest -q tests
python -m examples.train_adaptive_dynamics_a2 --output ../phase3-training
python -m examples.evaluate_adaptive_dynamics_a2 --checkpoint-dir ../phase3-training --output ../phase3-evidence --repeats 100
```

The benchmark calls each public API from the shared A1 anchor and also
times an explicitly active A2 context. The batch fits the context on each
public rollout. Windows timings measure synthetic offline inference.

## Decision and measured results

**NO-GO for promoting A2.** Keep D0 as the default and ridge as the adaptive
comparison baseline. Defer A3. The learned columns improved training's
one-step validation objective and did not add a repeatable rollout advantage
over the gated linear ablation. Both adaptive variants were slower than
ridge in the active batch benchmark.

The trainer selected epoch 12. The frozen basis reduced normalized
validation MSE from 0.01613168 for the linear ablation to 0.01515376.
Two CPU training runs produced identical checkpoint and manifest bytes.
The exported 2,336-byte checkpoint matched the PyTorch feature output to
a maximum absolute difference of 2.54×10⁻⁷. The evaluator reproduced all
original A1 baseline metrics exactly.

These are 1 s pooled position RMSEs, in centimeters, at the common anchor.

| Suite and regime | D0 | Ridge | Gated linear | A2 |
| --- | ---: | ---: | ---: | ---: |
| A1 shifted | 18.45 | 8.97 | 10.72 | 10.85 |
| A1 switched | 18.71 | 8.51 | 11.28 | 11.77 |
| A1 fixed delay | 12.27 | 12.77 | 10.66 | 10.16 |
| Holdout shifted | 20.72 | 8.92 | 11.81 | 11.90 |
| Holdout switched | 18.39 | 10.91 | 12.46 | 12.77 |
| Holdout fixed delay | 12.66 | 11.07 | 10.32 | 9.81 |
| Holdout noisy | 18.54 | 13.12 | 18.54 | 18.54 |
| Holdout matched nominal | 1.03 | 0.95 | 1.03 | 1.03 |

On the held-out shifted and switched sets, paired A2 minus ridge intervals
were [0.25, 6.29] cm and [0.29, 4.18] cm. A2 passed the nominal and
fixed-delay screens, with a holdout delayed heading RMSE of 0.1532 rad
versus D0's 0.1534 rad. Its 1 s confidence bounds covered 87.5–100% of
holdout episodes across the screened regimes and metrics. The bounds
exceeded the low-risk thresholds in all these cases, so the diagnostic
abstained from every low-risk declaration.

Active A2 eight-candidate latency was 26.40 ms p95, 29.86 ms p99 and
35.54 ms worst. Ridge was 16.82 ms p95 and D0 was 7.17 ms p95 on the same
Windows host. A2 exceeded the 10 ms allocation in all 100 measured active
batches. A2 can remain an optional research artifact for nominal protection
and delay analysis. These data do not support adding an A3 model to the
integration path. Native PX4, ROS, WorldState and Phase 2 code were unchanged.

[Training manifest](../artifacts/unified_phase3/training.json) ·
[Evaluation](../artifacts/unified_phase3/evaluation.json.gz) ·
[A1 rollout CSV](../artifacts/unified_phase3/a1_rollouts.csv.gz) ·
[Holdout rollout CSV](../artifacts/unified_phase3/holdout_rollouts.csv.gz) ·
[Test record](../artifacts/unified_phase3/test_results.txt)
