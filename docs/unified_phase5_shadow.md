
# Phase 5 read-only dynamics replay

This branch stacks on Phase 4 at `294ee7d`. It adds an opt-in ROS 2 subscription,
a rosbag2 and JSONL reader, and a causal replay engine. Outputs are JSON lines on
stdout. The ROS entry point creates a depth-four best-effort subscription and
disables ROS logging and parameter services. With no source flag it exits before
loading ROS. D0 stays the default predictor in the trajectory evaluator. Ridge
and the existing GRU-64 checkpoint are comparisons.

## Recorder and bag manifest

An upstream reviewed recorder or exporter must provide one ordered
`std_msgs/msg/String` event stream on `/dronehacks/diagnostics/shadow_input`.
The manifest's `stream` object specifies `frame_id`, `clock_id`, `state_source`,
`command_source`, `mapping_id`, and integer `clock_origin_ns`. Archive it with
the bag, data hashes, recorder revision, native topic and type versions,
estimator and camera configuration, origin and native-to-common clock
calibration (including residuals and valid interval), frame/heading
calibration, reset log and effective command provenance. The synthetic example
generates a minimal manifest and a matching JSONL event stream.

All events carry `kind`, `source`, `frame_id`, `clock_id`, `mapping_id`, `axes`,
`epoch`, `sequence`, and `received_ns`. A single recorder assigns monotonic
sequence numbers across both kinds and increases the epoch on an estimator or
clock reset. Time values are integer nanoseconds relative to the manifest
origin. Map capture and receive times separately and preserve acquisition
latency. Live callback time is checked in the same clock, using ROS sim time
when explicitly requested. Offline replay uses recorded receive order and
timestamps embedded by the recorder.

- `state` adds `capture_ns`, `healthy`, `position_m`, `velocity_mps`, `yaw_rad`,
  `yaw_rate_rad_s`, `twist_frame="world"` and
  `yaw_rate_kind="heading_derivative"`. ENU converts to NED with the existing
  frame transform and a heading reference change. Supply a synchronized
  estimated heading derivative. A raw body angular rate or body-frame twist
  fails validation. The recorder must resolve PX4 twist frame, FRD/FLU
  orientation, reset counters and estimator validity upstream.
- `effective` adds `effective_from_ns`, `effective_until_ns`, `position_m`,
  `yaw_rad` and `acceptance="verified_effective"`. A completed interval must
  arrive before the closing state event and span its preceding 20 ms
  transition. A ROS publication stamp for an issued trajectory setpoint
  supplies neither PX4 acceptance nor an effective interval. Native bags
  need a reviewed source of applied-command evidence and a recorder/exporter
  for this schema. Keep the original native streams alongside normalized
  events for review.

A forecast holds the last verified position/yaw for one counterfactual second.
The adapter scores it only after 50 aligned measured transitions verify that
same applied command. It retains one pending endpoint comparison, one
effective interval and the existing bounded history. Sequence loss, clock
rewind, reset, stale or unhealthy input, and incomplete command coverage
discard partial evidence. Warmup uses the existing D0 fallback inside the
adaptive models. Deadline overruns discard the forecast. The Python budget is
checked after a bounded batch and does not preempt synchronous NumPy work.

## Reproduction and gate

```sh
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m pytest -q tests
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python -m examples.unified_shadow_benchmark \
  --checkpoint artifacts/adaptive_dynamics_d2/adaptive_gru64.npz \
  --output phase5.json --fixture-directory phase5_fixture
python -m examples.unified_shadow_replay --manifest phase5_fixture/manifest.json \
  --jsonl phase5_fixture/events.jsonl
python -m examples.unified_shadow_replay --manifest reviewed-manifest.json --bag reviewed-bag
python -m examples.unified_shadow_replay --manifest reviewed-manifest.json --enable-shadow --use-sim-time
```

Four matched synthetic holds on the clean shifted fixture produced 1 s
position RMSE of 7.75 cm for D0, 6.55 cm for ridge and 9.32 cm for GRU-64.
Heading RMSE was approximately 0.165 degrees for all three. Three forecasts
were excluded after the verified command changed. Windows CPU p95 for eight
50-step rollouts was 6.92 ms, 16.51 ms and 20.19 ms respectively. The evidence
JSON records the p50, p99, worst and deadline counts. These are model-only CPU
measurements. This experiment uses deterministic synthetic measurements and
has no aircraft, target-device or concurrent perception measurements.

The Phase 5 native gate remains open. On Jazzy, first replay a reviewed bag
and inspect graph ownership, transport sequence gaps and native timestamp/frame
alignment. Then collect Orin Nano runs with stereo, cuVSLAM, mapping and
PX4 traffic active. Archive p50/p95/p99/worst end-to-end and eight-rollout
latencies, input age and jitter, 20 Hz misses, CPU/GPU/RAM and workload settings.
Check that frame changes, stale VIO and backlog clear predictions, and inspect
the node graph for any application publishers or command services. Compare
accelerated inference only if CPU profiling justifies it.
