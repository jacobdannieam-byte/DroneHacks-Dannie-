# A1 offline predictor selection

A1 compares the six shipped predictors through their public APIs. D0 is the
nominal and fallback reference. D2 online ridge remains the adaptive position
benchmark. Keep D0 as the default reference for downstream offline integration
until an adaptive candidate clears the nominal and runtime gates.

## How the tracks fit

| Track | Current milestone | Next dependency |
| --- | --- | --- |
| A, research and validation | A0 audited the implementation. A1 compares and selects predictor baselines. | A2 tests a bounded adaptive feature model against ridge. A3 is conditional on a remaining rollout gap. |
| D, vehicle prediction and planning | D0 nominal, D1 residual MLP and D2 history models and ridge are implemented offline. A1 is the D2.5 benchmark. | A4 delivers D3 trajectory scoring, A5 delivers D4 read-only ROS replay and profiling, and A6 delivers D5 simulation-only planning. A7 is the optional D6+ research lane. |
| S, semantic world model and reasoning | WorldState P1 and offline I0–I4 are in separate stacked draft PRs. They expose hypotheses and validated, non-executable goal hints. | A4 combines those interfaces with predicted trajectories and verified metric geometry. A8 explores future observation and information gain after geometry and replay evidence. |

[Master roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) ·
[A1 prompt](https://app.notion.com/p/411f393aa76881c4a006d2d3c695488c) ·
[WorldState](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/1) ·
[Reasoning](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/2) ·
[D2](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/5)

## Replay protocol

- Evaluate D0, D1, D2 ridge, history MLP, GRU-32 and GRU-64 at 20 ms. D2
  receives the same measured history across variants. All models get the same
  estimated anchor and effective position/yaw setpoint schedule. The schedule is
  assumed available for candidate forecasting. The simulator applies response
  delays internally. Future measured states and response settings stay outside
  inference. History is checked for unchanged contents after prediction.
- Preserve the published D2 post-switch anchor and compare pooled position
  RMSE to its recorded values. New 4.5 s episodes allow 0.2, 0.5, 1 and 2 s
  forecasts at the same five timestamps. On stationary episodes, switch anchor
  labels name the corresponding evaluation timestamps. Use 16 trajectories
  each for in-range, shifted, switched and fixed delay, plus eight new matched
  nominal controls, the legacy nominal trace and the equilibrium trace.
- The fixed-delay fixture uses stationary in-range gains and 120 ms position
  and 180 ms heading response delays. Paired switch replays add independent
  Gaussian position, velocity, heading and yaw-rate noise, no history, an
  eight-capture dropout, an eight-transition history stall, a reference
  epoch change, or a delayed transition spanning epochs. Dropouts omit
  forecasts at unavailable anchors. Epoch changes affect reference metadata,
  so candidate schedules keep the anchor's coordinates and reference through
  each forecast. D2 fallback is checked for exact equality to D0.
- Check checkpoint hashes and regenerate both models' train/validation
  fingerprints. Reject calibration/test overlaps with the model fit splits
  using full response, individual pre/post response, and state/command trace
  hashes. Calibration uses 24 independent clean switched episodes. The test
  manifest identifies deliberate paired fault replays by their shared source ID.
- Report vector-norm one-step linear velocity and acceleration errors,
  yaw-rate and yaw-acceleration errors, wrapped heading errors and position
  errors. Acceleration truth is the average finite difference over the
  upcoming capture interval. Summaries include RMSE, mean, median, p95,
  worst, and rollout counts above the synthetic markers of 0.5 m and 30°.
  Endpoint metrics use the last step, and pooled metrics use all forecast
  steps within the horizon. The post-switch curve reports one-step errors in
  elapsed-time bins. Report a learned model as materially better only if it
  improves 1 s pooled position RMSE by at least 20% in both clean shifted and
  switched cohorts, the paired episode bootstrap interval excludes zero, and
  it clears the 10% nominal position and heading regression gate.
- Confidence diagnostics take the finite-sample 90% empirical upper bound
  over calibration episode maxima at each model and horizon. Report test
  endpoint and episode coverage and counts where the bound was within the
  synthetic marker but observed error exceeded it. These fixed bounds are
  calibrated on clean switches and evaluated for transfer across other
  conditions. They provide no per-trajectory clearance certificate.

## Results

Position is pooled RMSE across 50 samples starting 0.4 s after the switch or
at the same timestamp in stationary replays. Heading is evaluated separately.

| Model | Published shifted | A1 stationary shifted | A1 clean switch | A1 fixed delay | A1 noisy switch |
| --- | ---: | ---: | ---: | ---: | ---: |
| D0 | 20.31 cm | 18.45 cm | 18.71 cm | 12.27 cm | 18.94 cm |
| D1 | 12.38 cm | 11.74 cm | 11.51 cm | 9.26 cm | 11.79 cm |
| D2 ridge | **7.00 cm** | **8.97 cm** | **8.51 cm** | 12.77 cm | **11.05 cm** |
| D2 history MLP | 9.54 cm | 10.67 cm | 8.57 cm | 8.78 cm | 13.97 cm |
| D2 GRU-32 | 9.83 cm | 11.03 cm | 9.55 cm | 8.90 cm | 14.54 cm |
| D2 GRU-64 | 9.75 cm | 11.50 cm | 9.16 cm | **8.24 cm** | 15.37 cm |

All six reproduce the published position metrics on both original D2 cohorts.
Ridge leads pooled position from 0.2 to 2 s on the clean shifted and switched
cohorts at the selected anchor. The history MLP leads the switched 0.5 s
position endpoint, and GRU-32 leads switched pooled heading at 0.5–2 s.
The history MLP nearly ties ridge at 1 s on clean switches and lowers the
worst observed pooled position error from 44.47 cm to 26.91 cm. Its paired
difference interval spans zero. No learned candidate clears the accuracy gate.

Ridge fails the expanded matched-nominal heading gate at 2 s. Endpoint
heading RMSE is 0.0332 rad versus D0's 0.0206 rad, and pooled RMSE is 0.0200 rad
versus 0.0153 rad. All learned models fail the nominal gate, and all models
preserve exact equilibrium. Fixed response delay puts ridge behind D0 on
1 s pooled position. In the noisy cohort ridge exceeds the 0.5 m synthetic
marker once at the selected 1 s anchor, and its clean-calibrated position bound
reports one false low-risk endpoint across the five anchors. In 16 of 16 late
cross-epoch replays an old crossing packet cleared a ready current-epoch
history and triggered D0 fallback. A1 records that existing behavior.

The Windows benchmark times 1,000 predictions and 100 batches of eight
distinct one-second forecasts for each model. Public calls include context
recomputation per candidate. D0 meets the provisional 10 ms batch allocation
at p95 on this host. Ridge and all learned candidates exceed it at p95.
D0 and ridge batch p95 are 7.02 ms and 17.01 ms, respectively. Ridge's
single-rollout p95 is 2.15 ms.
See the timing record for medians, p95, p99, maxima and over-budget counts.
These timings use synthetic input on an Intel Windows host.

## Reproduce and next gate

Run from the repository root with NumPy and pytest. Matplotlib is required only
for the optional SVG. The full JSON and CSV evidence is stored compressed
under the A1 artifact directory.

```powershell
$env:OPENBLAS_NUM_THREADS='1'
$env:OMP_NUM_THREADS='1'
python -m pytest -q tests
python -m examples.evaluate_adaptive_dynamics_a1 --output ../a1-replay --repeats 100 --plot
```

Use `--skip-timing` for an accuracy-only run. The JSON records fixture seeds,
checkpoint hashes and split fingerprints. The CSV records per-episode forecasts
and their anchor, history status, endpoint errors and pooled RMSE.

[Evaluation JSON](../artifacts/adaptive_dynamics_a1/evaluation.json.gz) ·
[Forecast CSV](../artifacts/adaptive_dynamics_a1/rollouts.csv.gz) ·
[Rollout plot](../artifacts/adaptive_dynamics_a1/rollout_summary.svg) ·
[Test evidence](../artifacts/adaptive_dynamics_a1/test_results.txt)

**GO for A2 as an offline experiment after review.** Use A1's frozen cohorts,
compare against ridge and D0, and make command-lag sensitivity, near-nominal
heading, and batch cost explicit acceptance criteria. Keep an untouched
holdout for final model selection. Defer A3 until A2 establishes whether
learned history is needed. ROS, PX4, model weights and semantic modules were
outside the A1 change set. Aircraft, obstacle-course and concurrent Orin
performance remain unmeasured.
