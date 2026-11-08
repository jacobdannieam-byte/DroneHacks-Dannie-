# Simulation matrix

No new flight was executed in this audit. Each row distinguishes a
historically reported native run, a reproduced offline command and an
unavailable runtime. All flight entrypoints require the team's approved
isolated run and a sole command owner. Use the [branch inventory](BRANCH_INVENTORY.md)
for full revision pins and branch test totals.

| Lane | Entrypoint and prerequisites | Local evidence | Native acceptance and limitations |
| --- | --- | --- | --- |
| Managed classical reference on `main@feb55b6` | `python3 main.py` in the managed companion, with the reviewed deployment clock, depth/camera TF, estimator service profile and matching Jazzy/PX4 messages. `flight.yaml`, `route.json` and deployment parameters configure the route. | T1, 30 passed including kinematic FakePX4 and ray-cast tests. | Native entrypoint inherited across most branches. Repository reports an isolated 1 m takeoff/hover/AUTO_LAND/disarm using `python3 onboard/examples/takeoff_land.py` with actual cuVSLAM, and safe refusal with VIO off. That run's moving RTF was about 0.53 and failed timing. There is no new full course proof here. |
| Emulator on `classical@1f60fb5` | On the documented Mac/Colima/Docker setup, `simulator/build_simulator_image.sh` then `simulator/run_test_flight.sh landing_test.py --with-push-down`. It builds Humble, PX4 SIH 1.16.2, MicroXRCE, pinned px4_msgs, the organizer interface and the course package. The flight runner launches an isolated graph. | T1, 30 passed. T2, five geometry checks passed. T3, 88 passed and 2 skipped. T4 hit the rclpy blocker at collection. | Branch reports six successful landing flights on an M1 Mac and approximately 0.92–1.01 RTF. Includes 0.8 m rise, 1 m forward and landing. Generates referee JSON, pose CSV, simulator log and PX4 ULog. Camera tracking is emulated and collisions are scored by the referee. Full Docker flight, course completion, genuine VIO and hardware performance remain unverified here. |
| `dannie-classical@79d4da2` | Same source harness plus an Argon proposal. Direct shell invocation under Linux/Mac needs a review of four removed execute bits, including the build and run scripts. | T1, 30 passed. The geometry checker and inputs are identical to the classical tip that passed T2. | No separate launch demonstrated. Preserve the changed script modes as evidence until the owner confirms intent. |
| `soham-depth@7781aff` | Inherited baseline entrypoint. Directional probes exercised using synthetic depth fixtures. | T1, 57 passed. | Candidate shared module. No native four-probe subscriber or new flight entrypoint. |
| `soham-mission@dd09b79` | Inherited `main.py` and ROS adapter. Optional `Params.use_stages` selects a pure course policy. It needs native line, gate, optical-flow/range and pad inputs with capture stamps. | T1, 83 passed, including stage inputs and supervisory cases. | Missing live detector wiring means the staged strategy is unqualified. Preserve the default owner, stop and landing behavior when reviewing the adapter. |
| Managed hybrid research at readiness `78ef913` | T5 starts a read-only 30 s managed smoke. Phase 2 accepts a single-epoch native bag with a reviewed pose/depth profile. Phase 5 needs actual applied-command timing and normalized state. D0 is the default, ridge and NPZ weights supply research comparisons. | T1, 333 passed. The readiness record also documents reproducible Phase 2/4 outputs, synthetic shadow comparisons, offline ablation/rehearsal and Phase 8 preflight. | No connected planner authority or closed-loop course entrypoint. Require fresh observed geometry, state/epoch consistency, owner and landing/takeover gates, native Phase 1–6 evidence, Phase 7 activation review and Phase 8 qualification. The current qualification record has zero native trials. |
| Standalone neural or RL | No runnable policy entrypoints or qualified checkpoints found in the audited refs. | Learned predictor weights and evaluator tests are in the hybrid stack. | Defer independent strategy configurations. |

The managed companion uses real rendered sensor streams and cuVSLAM in the
documented environment. The emulator produces synthetic sensor messages
from an isolated simulated state. Keep the two launch recipes, clock
mappings, graph ownership checks and acceptance claims separate.

## Commands executed

Host was Windows, CPython 3.11.9, NumPy 2.3.5, SciPy 1.17.0 and pytest 9.0.3.
ROS2 Python, px4_msgs, rosbag2_py, cuVSLAM and Docker were unavailable to the
local audit. No managed course session or Orin device was used.

**T1** extracted `git archive --format=zip <full-head-sha>` into a separate
temporary directory for each of the 23 distinct head commits. From each
extracted root, with `OPENBLAS_NUM_THREADS=1`, `OMP_NUM_THREADS=1`,
`PYTHONUTF8=1` and `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`, the command was

```bash
python -m pytest -q tests
```

All 23 exited 0. Pytest emitted a warning per run while writing its cache
in the isolated Windows temp directory. Tests used the archives' own
tracked fixtures, imports and checkpoint blobs.

**T2**, from the isolated classical archive root, ran

```bash
python simulator/check_course_model.py
```

The initial run failed when the sandbox denied the atomic rename of a
generated distance table. The isolated retry completed with five of five
checks passing. The maximum error over finite reference rays was 7 mm.
This check writes only derived simulator tables and uses no ROS runtime.

**T3**, from `simulator/organizers_code/skybeta_flight_interface` in the
same archive with that directory on `PYTHONPATH`, ran

```bash
python -m pytest -q -p no:cacheprovider test/test_auto_offboard.py test/test_envelope.py test/test_frames.py test/test_topics.py
```

Result was 88 passed and 2 skipped. **T4**, from the archive root with the
same package on `PYTHONPATH`, ran

```bash
python -m pytest -q simulator/organizers_code/skybeta_flight_interface/test
```

It exited 2 during collection when `test_flight_interface.py` imported
`rclpy`. The node-level suite and `test/integration_offboard.py` need the
Humble image and matching messages for their intended runtime.

**T5** is the next managed Workbench test from readiness, while landed and
disarmed. See the [readiness runbook](../unified_sitl_readiness.md).

```bash
python3 -m onboard.diagnostics.phase1_probe capture \
  --require-idle --seconds 30 --window settled-disarmed --output phase1-settled
```

Capture the report and metadata with the exact source SHA. A smoke pass is
the prerequisite for a reviewed moving trial. Obtain clock, estimator and
ownership evidence, a corresponding bag and PX4 ULog, and keep failures.
For native and simulator grading, use the matching referee and log source.

## Strategy results logs

- [Classical](../simulation_results/classical.md) covers the managed reference
  and the independent emulator in separate sections.
- [Hybrid](../simulation_results/hybrid.md) covers predictive planning replay
  and eventual native trials. Link shared evidence by run ID.

Append a row for each attempt with its run ID, UTC date, strategy and runtime,
scenario and course revision, exact Git SHA and config, reset state and seed
when applicable, pass/fail outcome, metrics, and evidence links. Label the
evidence as offline, emulator, or managed simulator. Keep bags, ULogs and
other raw evidence in team-approved storage.

Keep failed runs. Record each issue with its run ID, the proposed improvement
and the fix revision when available. Repeat the same scenario and conditions
where possible, then link the new run and compare the affected metrics. Record
any changed conditions. Track the first baseline, fix and retest separately.

## Independent strategy acceptance

Retain a separate emulator ref until one checkout reproduces its launch
and the managed companion with explicit runtime profiles. Before assigning
a `strategy/*` ref, record one runnable launch, the simulator and ROS/PX4
dependency pins, sensor services, strategy config and scenario IDs, any
checkpoint hashes, before/after tests, diagnostic logs and the expected
stop/landing/authority behavior. Require an independently replayable
closed-loop scenario for that strategy. The staged and hybrid modules
currently need native adapter and flight evidence.
