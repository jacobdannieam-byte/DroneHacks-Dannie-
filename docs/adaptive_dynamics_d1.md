# D1 acceleration residual

D1 adds an offline 64×64 MLP to the D0 position-setpoint predictor. The
checkpoint reduces position error on randomized synthetic responses, with a
regression on the matched-nominal fixture and higher rollout cost. The caller
chooses `DynamicsModel` or `ResidualDynamicsModel` explicitly.

## Prediction contract

D1 uses D0's `DroneState`, `PositionSetpoint`, `Prediction`, local NED reference,
and command effective-time checks. It uses the default D0 tracking parameters
and accepts steps between 10 and 40 ms. Training uses 10, 20, and 40 ms steps,
and the holdout evaluation uses 20 ms.

The 17 inputs are the three NED velocities, three position errors, three
nominal velocity references, three nominal accelerations, wrapped yaw error,
yaw rate, nominal yaw-rate reference, nominal yaw acceleration and step duration.
Each feature has an explicit clipping range and training-set normalization.
Only current state, effective command and nominal response enter inference.

Two ReLU layers feed four tanh outputs. The outputs correct the NED acceleration
components and yaw acceleration. The respective residual caps are
`(1.0, 1.0, 0.6) m/s²` and `2.0 rad/s²`. A component's activity gate uses the
sum of its absolute measured velocity and absolute nominal velocity reference
or their angular equivalents. The gate is `min(1, 4 * sum)` in the corresponding
SI units and preserves exact equilibrium. The combined accelerations are
clipped at the nominal D0 caps before the shared semi-implicit Euler update.
Those caps are synthetic model parameters.

The frozen checkpoint contains 5,572 trainable parameters and 34 input
normalization values. Its float32 arrays use a versioned, size-checked NumPy
archive loaded with pickling disabled. Its measured file size is 24,610 bytes.
Training runs separately from inference and uses NumPy Adam.

```python
from refnav.dynamics import DroneState, LocalNED, PositionSetpoint
from refnav.dynamics.residual import ResidualDynamicsModel, ResidualMLP

network = ResidualMLP.load("artifacts/adaptive_dynamics_d1/residual_64x64.npz")
model = ResidualDynamicsModel(network=network)
ref = LocalNED("offline_ned", 0, "replay")
state = DroneState(ref, 0, (0, 0, -1), (0, 0, 0), 0, 0)
command = PositionSetpoint(ref, 0, 2, (1, -1, -1.4), 0.5)
prediction = model.predict(state, command, dt=0.02)
trajectory = model.rollout(state, command, dt=0.02, steps=100)
```

## Data and selection

The independent simulator uses RK4 at one-tenth the prediction step. Randomized
episodes vary position and yaw tracking gains, velocity and yaw response lag,
acceleration limits, effective drag, and position and heading response delay.
Each episode samples an initial state and a piecewise-constant command schedule.
The simulator applies delay to its response to the current supplied setpoint.
The model observes the supplied setpoint and has no access to the hidden delay,
response parameters or future samples. This introduces ambiguity at command
changes, which a state-only residual cannot resolve in general.

The target is the reference change in velocity or yaw rate divided by `dt`,
clipped to the D0 acceleration limits, minus the nominal derivative. Targets
respect the residual caps and the activity gate. Normalization statistics use
training data only. Independent random streams assign 160 whole episodes to
training, 24 to validation, 48 to an in-range test and 24 to a shifted test.
Response fingerprints verify disjoint configurations. The shifted cohort
emphasizes slower response, lower acceleration, higher drag and longer delay.
The validation criterion selects epoch 4 of 55 with normalized residual MSE
0.085565, compared with 0.262392 for the zero-residual initialization. The
frozen checkpoint is then evaluated on both test cohorts and two fixed controls.
[Training record](../artifacts/adaptive_dynamics_d1/training.json)

## Measured accuracy

Each evaluation episode is 100 steps at 20 ms. One-step errors restart from
each reference state. Open-loop errors feed predictions back for two seconds.
Position and velocity use Euclidean error. Heading uses shortest-arc error.
The table reports pooled RMSE over prediction samples in each cohort.

| Cohort | One-step position mm, D0 → D1 | One-step velocity m/s, D0 → D1 | One-step heading rad, D0 → D1 | Rollout position m, D0 → D1 | Rollout heading rad, D0 → D1 |
| --- | ---: | ---: | ---: | ---: | ---: |
| In-range holdout, 48 episodes | 0.630 → 0.392 | 0.01901 → 0.01446 | 0.000784 → 0.000533 | 0.1460 → 0.0920 | 0.1133 → 0.0889 |
| Shifted holdout, 24 episodes | 0.901 → 0.626 | 0.03804 → 0.02533 | 0.001162 → 0.000852 | 0.3432 → 0.2257 | 0.3125 → 0.2556 |
| Matched nominal | 0.481 → 0.255 | 0.00057 → 0.01222 | 0.000373 → 0.000270 | 0.0118 → 0.1074 | 0.0106 → 0.0133 |
| Equilibrium hold | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 | 0 → 0 |

D1 reduced position rollout RMSE in 36 of 48 in-range episodes and 24 of 24
shifted episodes. The paired episode bootstrap intervals for the D1 minus D0
position RMSE difference are −0.070 to −0.036 m and −0.130 to −0.105 m,
respectively. All evaluated predictions stayed finite. In-range heading RMSE
at the two-second endpoint rose from 0.150 to 0.179 rad, despite the lower
pooled heading RMSE. The learned correction can degrade already accurate
nominal dynamics and individual endpoint quantities. The comparison uses the
fixed D0 settings. Aircraft response and the benefit over a fitted classical
baseline remain unmeasured. [Evaluation data](../artifacts/adaptive_dynamics_d1/evaluation.json)

## Host performance

Measured on Windows AMD64, Intel Family 6 Model 183, Python 3.11.9 and NumPy
2.3.5 with one OpenBLAS thread. Both models ran in the same process with the
same commands. Five warmups precede 1,000 single-step calls and 100 calls per
rollout workload. Peak traced allocation includes the outputs. The JSON also
records process CPU time from a separate sampling window. Jetson timing and
co-running perception workloads remain unmeasured.

| Operation | D0 median / p95 / worst ms | D1 median / p95 / worst ms | D1 traced peak bytes |
| --- | ---: | ---: | ---: |
| One prediction | 0.0172 / 0.0184 / 0.0578 | 0.0387 / 0.0448 / 0.2468 | 1,736 |
| One-second rollout | 0.816 / 0.873 / 0.951 | 1.907 / 2.142 / 2.280 | 40,312 |
| Eight one-second rollouts | 6.676 / 7.219 / 11.258 | 15.299 / 16.513 / 22.261 | 323,008 |

The eight-rollout D1 median exceeds the provisional 10 ms budget. D1 remains
an offline candidate. A larger network would need a new accuracy and cost case.

## Reproduce

Run from the repository root with NumPy and pytest installed. These environment
settings reproduce the host's single-threaded benchmark configuration.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m examples.train_adaptive_dynamics_d1
python -m examples.evaluate_adaptive_dynamics_d1
python -m pytest -q tests
```

The branch passed 101 tests and the detached check with the semantic branches
passed 231. The original D0 synthetic evaluation reproduced its saved results
exactly. The predictor, training and evaluation run offline. The roadmap
describes causal history for D2 and the separate requirements for later stages.
[Test evidence](../artifacts/adaptive_dynamics_d1/test_results.txt) · [Roadmap](adaptive_dynamics_roadmap.md)
