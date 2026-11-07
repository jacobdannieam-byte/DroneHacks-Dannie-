# D0 closed-loop dynamics baseline

D0 predicts short-horizon motion from an estimated state and a held position and
yaw setpoint. The model uses synthetic tracking gains, lag, and acceleration
limits. It runs offline with the Python standard library. The evaluation uses
NumPy for an independent reference simulator.

## Repository and interface audit

The branch starts at `experimental` commit `feb55b6`. At branch creation,
`main` pointed to the same commit and semantic PRs #1 and #2 were open drafts.
D0 uses the flight setpoint contract directly, so it builds on `experimental`
without depending on the semantic branches. A detached checkout combining the
D0 source with PR #2 verified the mission, WorldState, reasoning, and D0 suites.
[WorldState PR #1](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/1) ·
[Reasoning PR #2](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/2)

| Existing path | Implication for D0 |
| --- | --- |
| `main.py`, `refnav/launch.py` | Mission configuration and the standard ROS node launch are outside the predictor's entry point. |
| `refnav/core.py` | `Mission.step()` generates `(north, east, down, yaw)` setpoints on route legs. Its vehicle summary holds estimated position, heading, attitude and validity/authority flags. A future dynamics adapter needs a synchronized velocity and yaw-rate estimate as well. |
| `refnav/node.py` | The 20 Hz mission timer uses the ROS node clock. Offboard uses position mode with `TrajectorySetpoint.position` and `yaw`. Velocity and acceleration are NaN. PX4-bound messages receive system-clock stamps, so their stamp alone does not identify command effective time in simulation. |
| `refnav/depth.py`, `refnav/core.py`, `onboard/adapters/px4_health.py` | Depth coverage, calibrated projection, estimator validity and freshness are evaluated by the flight software. D0 receives a numeric state for prediction. |
| `onboard/adapters/px4_odom.py`, `onboard/adapters/geometry.py` | The odometry bridge accepts PX4 NED pose, converts it to ENU and FLU, and stamps ROS odometry at receipt. Published twist linear velocity is in the child frame when a supported NED velocity is available. |
| `onboard/slam/`, `onboard/vio/`, `onboard/config/` | cuVSLAM uses capture stamps and local odometry. The external-vision bridge validates, aligns and maps timestamps and references before producing PX4 vision odometry. NED alignment needs an established heading reference. |
| `refnav/world_model/`, `refnav/reasoning/` on the semantic branches | WorldState has local geometry and temporal evidence. Reasoning has bounded read-only context and non-executable goal proposals. An evaluator can consume a D0 trajectory and a separate, synchronized context in D3. |

The Notion architecture separates environment belief, predictive dynamics,
candidate evaluation, and command authority. D0 implements the dynamics box.
[Architecture blueprint](https://app.notion.com/p/411f393aa768814e97a4cb94b726398c) ·
[Reasoning integration](https://app.notion.com/p/411f393aa76881e78dd6d7beb6592060) ·
[Research roadmap](https://app.notion.com/p/410f393aa76881c0a464eaf3d7fd750f) ·
[Predictive dynamics research](https://app.notion.com/p/411f393aa768811182d8e74979a39594)

## State, action and time

| Record | Contract |
| --- | --- |
| `LocalNED` | Local frame identifier, nonnegative reference epoch, caller-owned clock identifier. Equality binds the frame origin, reset epoch and clock domain. |
| `DroneState` | Estimated capture timestamp in seconds, local NED position in metres, NED world velocity in m/s, yaw in radians, yaw rate in rad/s, and the reference. |
| `PositionSetpoint` | Absolute position and yaw in the same local NED reference, held over `[effective_from_s, effective_until_s)`. The interval describes the command assumed effective at the controller. |
| `PredictedState` | Distinct model output with the same physical fields at the prediction endpoint. It can feed the next model step. |
| `Prediction` | Predicted state, bounded velocity and yaw-rate references, and accelerations applied by the model over that step. |

Yaw is positive from north toward east about the down axis and normalized to
`[-π, π)`. Vertical motion uses positive down. Producer code must establish NED
coordinates and radians before constructing the records. Reference validation
checks labels and epochs. The correctness of the supplied transform is the
producer's responsibility.

The numeric clock has no system-time dependency. Each step must fit entirely
inside one effective command interval. Two timestamp ULPs accommodate rounding
at adjoining endpoints, and excessively coarse timestamps are rejected. A
rollout computes timestamps from its initial time and step index to avoid
accumulated floating-point drift. Clock or estimator resets require a new
reference epoch from the producer.

## Tracking model and integration

For each NED axis, with symmetric component clipping,

```text
v_ref = clip(k_p * (p_cmd - p), +/- v_limit)
a     = clip((v_ref - v) / tau_v, +/- a_limit)
v'    = v + a * dt
p'    = p + v' * dt
```

Heading uses the same structure. Compute `omega_ref` from the wrapped shortest
yaw error times `k_yaw`, clip it by the yaw-rate reference limit, and track it
with time constant `tau_yaw` and bounded angular acceleration. Then integrate yaw
with the updated rate and wrap the result. Exactly antipodal headings choose
the negative arc by the half-open wrap convention.

The integrator is semi-implicit Euler. The time step is in 1–50 ms, at most
half the smaller response time constant, with `dt * max(k_p, k_yaw) <= 1`.
These are explicit numerical guards. The limits apply to references and
acceleration, so an initially fast state decelerates over time. Axis-wise limits
define a box and allow a larger diagonal speed than a single horizontal-axis
limit. Real tilt and thrust couple the axes.

| Synthetic nominal parameter | Value |
| --- | --- |
| Position gain, velocity time constant | 1.2 s⁻¹, 0.35 s |
| NED speed reference limits | 1.2, 1.2, 0.7 m/s |
| NED acceleration limits | 2.0, 2.0, 1.2 m/s² |
| Heading gain, yaw-rate time constant | 2.0 s⁻¹, 0.18 s |
| Yaw-rate reference, angular acceleration limits | 1.5 rad/s, 4.0 rad/s² |

This abstraction describes the response of the vehicle and PX4 tracking loops
as a single reduced-order system. It omits explicit drag to avoid adding an
unidentified term on top of the effective controller response. It also omits
attitude and thrust dynamics, actuator delays, gravity/thrust balance, contact,
wind, and estimator dynamics. Gains and limits are fixture values. Their
relationship to aircraft parameters remains to be measured.

```python
from refnav.dynamics import DroneState, DynamicsModel, LocalNED, PositionSetpoint

ref = LocalNED("offline_ned", 0, "replay")
state = DroneState(ref, 0.0, (0, 0, -1), (0, 0, 0), 0.0, 0.0)
command = PositionSetpoint(ref, 0.0, 1.0, (1, 0, -1), 0.2)
model = DynamicsModel()
prediction = model.predict(state, command, dt=0.02)
trajectory = model.rollout(state, command, dt=0.02, steps=50)
```

For a varying schedule, pass an ordered iterable containing one setpoint per
step and omit `steps`. Repeating a setpoint can cover several adjacent steps.
Maximum length is 250 steps and maximum duration is 5 s. A malformed value,
reference mismatch, uncovered interval, or exhausted budget raises `ValueError`
and yields no partial result. Rollouts return immutable endpoint records and
consume at most one extra iterable item to detect oversize input.

## Reproduce and interpret the evaluation

```sh
python -m pytest -q tests
python -m examples.adaptive_dynamics_d0_demo --output artifacts/adaptive_dynamics_d0 --repeats 200
```

The fixture independently integrates continuous synthetic responses with RK4 at
2 ms. The nominal fixture shares the tracking law and constants by construction.
A 1 ms refinement changed the nominal trajectory by at most 0.14 μm position
and 0.28 μrad yaw on the sampled grid. The predictor uses its default parameters
across all seven cases. Each perturbed case changes only the reference simulator.

One-step errors start from each reference state. The 5 s rollout starts once
and feeds back predictions. Position and velocity errors use Euclidean norms,
and heading error uses the shortest wrapped difference. The artifact includes
RMSE, maximum, final and horizon-specific errors, plus sampled NED traces.
[Evaluation data](../artifacts/adaptive_dynamics_d0/evaluation.json)

| Reference response | One-step position RMSE (mm) | One-step velocity RMSE (m/s) | One-step yaw RMSE (rad) | 5 s rollout position RMSE (m) | 5 s rollout yaw RMSE (rad) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Matched nominal | 0.329 | 0.00053 | 0.00031 | 0.0195 | 0.0090 |
| 55% acceleration limits | 0.579 | 0.01721 | 0.00031 | 0.6358 | 0.0090 |
| 0.65 s translational time constant | 0.502 | 0.01348 | 0.00031 | 0.2793 | 0.0090 |
| Effective drag at 0.45 s⁻¹ | 0.385 | 0.01086 | 0.00031 | 0.2631 | 0.0090 |
| 0.30 s heading command delay | 0.329 | 0.00053 | 0.00064 | 0.0195 | 0.2226 |
| Equilibrium hold | 0 | 0 | 0 | 0 | 0 |
| Reversal of internal velocity demand via position steps | 0.260 | 0.00038 | 0 | 0.0078 | 0 |

With the lower acceleration limits, absolute position error reached 1.14 m and
ended at 0.976 m. The delayed-heading case peaked at 0.463 rad. All sampled
trajectories stayed finite. The suite also checks 60 s of chained rollouts.
Small one-step errors coexist with substantial open-loop drift. This is the
central limitation to address with measured tracking data and held-out rollouts.

## Host timings and tests

Measured on Windows AMD64, Intel Family 6 Model 183, Python 3.11.9 and NumPy
2.3.5. Timings use five warmups, 2,000 single steps and 200 calls for each rollout
workload. The 20 ms step is fixed. Traced peak includes output storage, and CPU
sampling ran separately in continuous half-second windows. The JSON records
source hashes, timer statistics, CPU and allocation evidence. The scalar Python
workload used the host CPU. GPU utilization telemetry and Jetson timings are
unavailable for this run. [Benchmark data](../artifacts/adaptive_dynamics_d0/benchmark.json)

| Workload | Median (ms) | p95 (ms) | Worst observed (ms) | Traced peak (bytes) |
| --- | ---: | ---: | ---: | ---: |
| One prediction | 0.0172 | 0.0180 | 0.1273 | 1,392 |
| 0.2 s rollout | 0.1657 | 0.1779 | 0.2925 | 6,392 |
| 1 s rollout | 0.8193 | 0.9190 | 1.1023 | 38,288 |
| 2 s rollout | 1.6276 | 1.6940 | 2.0954 | 78,336 |
| 5 s rollout | 4.1243 | 4.4008 | 7.6660 | 202,736 |
| Eight 1 s rollouts | 6.5791 | 7.5239 | 10.2039 | 317,216 |

The eight-rollout maximum crossed the provisional 10 ms rollout budget on this
host. At 20 Hz for 40 in-process cycles, one 1 s rollout met all 50 ms periods.
Release lateness p95 was 0.559 ms, compute p95 was 1.392 ms, and synthetic
capture-to-call age p95 was 0.083 ms. Process CPU usage was approximately 5.6%
of one core during the paced sample. Effective-window checks rejected future,
expired, and mid-step-expiring commands. ROS transport age and co-running
perception workloads require a separate measurement.

The base suite passed 30 tests, the D0 suite passed 92, and the detached check
with the semantic branches passed 222. All commands were offline Python tests
and synthetic evaluation. The branch adds only D0 source, tests, examples,
documentation and evidence. [Test evidence](../artifacts/adaptive_dynamics_d0/test_results.txt)

See the roadmap for D1–D5 contracts, telemetry alignment, and prerequisites.
[Adaptive dynamics roadmap](adaptive_dynamics_roadmap.md)
