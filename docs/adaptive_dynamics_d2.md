# D2 causal dynamics adaptation

D2 estimates a changing closed-loop response from recent completed motion.
It retains D0's position/yaw setpoints and semi-implicit integration. Three
trained encoders and a classical online fit share the same measured history,
acceleration correction caps, and integration code. The implementation runs
offline on synthetic episodes.

## History and prediction

`MotionHistory.observe(before, command, after)` takes two measured `DroneState`
records in one local NED reference and a setpoint effective throughout their
20 ms interval. It retains 20 transitions, equivalent to 0.4 seconds at 50 Hz.
Each row holds four nominal accelerations, four observed accelerations inferred
from velocity and yaw-rate changes, four previous velocities and rates, and a
validity marker. All values use explicit scaling and clipping. Estimated-motion
records supply observations, and predicted states are a distinct type.

The history snapshot anchors to the last measured endpoint. Empty, warmup,
stale, unaligned and incompatible-reference states fall back to D0. Four
contiguous transitions are needed to enable adaptation. Gaps and new references
start a fresh window. Duplicate, reordered and older-epoch packets in the same
frame and clock are rejected. A transition spanning a reference reset clears the
window. Callers supply the authoritative stream and its effective command times.

The learned options are a 32-unit flattened-history MLP and one-layer GRUs of
32 and 64 units. Each produces eight tanh coordinates. Zero padding and a valid
marker represent incomplete history. The context joins the 17 current D1
features in a 48×32 residual head. Frozen weights transform current history
into a fresh context. Each rollout computes it once at the measured anchor and
holds it fixed across the predicted trajectory. Observations enter the history
only through `observe`.

The classical comparison fits gain and rate damping on each normalized
acceleration axis. It uses a ridge prior of gain 1 and damping 0 with
exponential recency weights over the same window. All variants apply D1's
activity gate and correction bounds, followed by D0's combined acceleration
limits. These are synthetic response limits.

```python
from refnav.dynamics.adaptive import AdaptiveDynamicsModel, AdaptiveNetwork, OnlineFitDynamicsModel
from refnav.dynamics.history import MotionHistory

history = MotionHistory()
network = AdaptiveNetwork.load("artifacts/adaptive_dynamics_d2/adaptive_gru64.npz")
learned = AdaptiveDynamicsModel(network)
classical = OnlineFitDynamicsModel()
# For each completed capture interval, before and after are measured DroneState records.
history.observe(before, effective_setpoint, after)
context = learned.adaptation(after, history)
forecast = learned.rollout(after, upcoming_setpoints, dt=0.02, history=history)
```

## Training and held-out evaluation

The simulator generates three-second episodes at 20 ms and integrates its
reference response with RK4 at 2 ms. Training uses 120 whole episodes, including
80 that change dynamics mid-run. Validation uses 24 separate switched episodes.
The test cohorts contain 32 in-range and 24 shifted switched episodes. Each
switch occurs at 1.30 s, changes tracking gains, lag, acceleration, drag and
delays, and keeps the state continuous. The shifted cohort draws the second
response from a range with slower tracking, greater drag and longer delays.
The checkpoint-selection script uses validation only. Episode configuration
fingerprints verify disjoint splits. One matched-nominal and one equilibrium
control provide additional comparisons.

At each training step the current input and history are assembled before
observing the next sample. That sample supplies the acceleration target and
enters the window for subsequent steps. Current-feature normalization uses
training data only. CPU PyTorch Adam trains all three models for 18 epochs,
with a checkpoint selected by normalized residual MSE. Portable NumPy inference
was checked against PyTorch on 32 validation windows for each encoder. The
maximum exported acceleration residual difference across models was 1.1×10⁻⁶.
[Training data and checkpoint hashes](../artifacts/adaptive_dynamics_d2/training.json)

| Encoder | Selected epoch | Validation MSE | Trainable parameters | Checkpoint bytes |
| --- | ---: | ---: | ---: | ---: |
| History MLP | 17 | 0.025215 | 11,564 | 49,852 |
| GRU-32 | 18 | 0.019569 | 7,724 | 34,992 |
| GRU-64 | 18 | 0.019520 | 18,636 | 78,640 |

GRU-64 won the validation criterion by a small margin. A frozen-context
ablation pins its last pre-switch context while continuing to use the measured
current state and supplied commands. That comparison tests the effect of
updating history after the dynamics switch.

| Model | In-range one-step velocity RMSE (m/s) | Shifted one-step velocity RMSE (m/s) | In-range rollout position RMSE (cm) | Shifted rollout position RMSE (cm) |
| --- | ---: | ---: | ---: | ---: |
| D0 | 0.0200 | 0.0307 | 9.11 | 20.31 |
| D1 | 0.0147 | 0.0204 | 6.01 | 12.38 |
| Online fit | 0.0130 | 0.0144 | **5.38** | **7.00** |
| History MLP | 0.0108 | 0.0151 | 7.42 | 9.54 |
| GRU-32 | 0.0104 | 0.0147 | 7.29 | 9.83 |
| GRU-64 | 0.0104 | 0.0147 | 7.20 | 9.75 |
| GRU-64 with pre-switch context | 0.0146 | 0.0184 | 10.41 | 12.44 |

The position columns pool the samples in one-second open-loop forecasts started
0.4 s after the switch. The one-step columns pool entire episodes and restart
from each reference state. The artifact includes position, velocity, heading,
and yaw-rate errors, several forecast anchors, and 0.2, 0.5 and 1.0 s endpoints.
All scored predictions were finite. On the matched-nominal control the online
fit and D0 each produced 1.34 cm rollout position RMSE, compared with 7.26 cm
for D1 and 12.79 cm for GRU-64. All variants held the equilibrium exactly.

On the shifted cohort, the selected GRU's post-switch velocity RMSE was 0.0145
m/s in the first 100 ms and 0.0160 m/s in the 0.4–0.8 s interval. The frozen
context had errors of 0.0195 and 0.0211 m/s over the same intervals. This shows
value from fresh history on these fixtures. The recovery criterion uses five
successive 100 ms RMS windows below 0.025 m/s. D0 recovered in 16 of 24 shifted
episodes at a median 0.54 s among recoveries. Every other variant recovered in
24 of 24 at a median 0.18 s, the metric's earliest possible time. That threshold
offers little separation among the residual approaches.

GRU-64's shifted-cohort median stepwise context displacement was 0.035 in
Euclidean norm, with median pre/post mean displacement 1.08. These describe
the learned coordinates under changing commands and dynamics. Their mapping to
physical parameters remains unverified. [Full evaluation](../artifacts/adaptive_dynamics_d2/evaluation.json)

## Measured host cost

Windows Intel AMD64, Python 3.11.9, NumPy 2.3.5 and one OpenBLAS thread. Each
prediction includes context construction when applicable. Each rollout encodes
once for its 50 steps. The benchmark uses 600 predictions, 60 rollouts and
300 standalone context updates per applicable model.

| Model | Context update median (ms) | One prediction median / p95 / worst (ms) | One-second rollout median / p95 / worst (ms) |
| --- | ---: | ---: | ---: |
| D0 | — | 0.017 / 0.019 / 0.069 | 0.834 / 0.913 / 1.039 |
| D1 | — | 0.038 / 0.043 / 0.076 | 1.893 / 2.124 / 2.406 |
| Online fit | 0.056 | 0.108 / 0.124 / 0.275 | 2.016 / 2.186 / 2.345 |
| History MLP | 0.022 | 0.071 / 0.083 / 0.347 | 2.122 / 2.390 / 2.436 |
| GRU-32 | 0.255 | 0.312 / 0.384 / 0.828 | 2.325 / 2.464 / 2.699 |
| GRU-64 | 0.274 | 0.346 / 0.373 / 0.586 | 2.534 / 2.853 / 2.893 |

GRU-64's rollout traced peak allocation was 49,152 bytes. Its context update
accounts for most of the single-step overhead. Training requires PyTorch.
Inference uses the committed NumPy checkpoints. Aircraft accuracy, Jetson
latency, and the cost of concurrent perception remain unmeasured.

The classical fit is the stronger starting predictor for later rollout
evaluation on this evidence. It wins the shifted forecast comparison and
handles the matched-nominal control with lower error. The trained encoders
remain useful for investigating one-step prediction and adaptation. Training
and checkpoint selection target one-step error, which can diverge from
open-loop accuracy as the predicted state drifts from the measured states.

## Reproduce

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m examples.train_adaptive_dynamics_d2
python -m examples.evaluate_adaptive_dynamics_d2
python -m pytest -q tests
```

The D2 branch passed 111 tests. A detached compatibility checkout with the
semantic branches passed 241. D1's default simulator outputs were compared
with the parent revision and matched across all four splits.
[Test evidence](../artifacts/adaptive_dynamics_d2/test_results.txt) · [Roadmap](adaptive_dynamics_roadmap.md)
