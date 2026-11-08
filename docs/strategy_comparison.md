# Strategy comparison implementation

This work is isolated on `codex/strategy-comparison`, starting at
`c8bb4eb2ddcf62ca687ef6a0c75a446a52bf01cf`. The refreshed remote canonical
Hybrid head is `bfb09c8a44607d0c7efbf3532749207b25500385`. Remote heads were
read with `ls-remote`, without fetching or moving teammate refs. Canonical
Hybrid remains five commits ahead and four behind P2-C/D. No blanket donor
merge was performed. Baseline verification: 444 tests passed in 25.60 s.

## Protected S1

The user confirmed `run-AdMalPxA` on exact revision
`ad31ae55ebf7cd3b2eaf57917537242831e3c752` as the working Classical baseline.
The saved report and trial approval agree. World SHA256 is
`74a3b67ed942292d880c79cc43eb2d0740466d0c27bfa42cb5045210e05f9ccf`.
Phone-scan world, side-gap-first mission, isolated domain 37. Four checkpoints,
landing on pad, disarm at 84.3 seconds, zero touches, no course or height-limit
violation; maximum height-estimate error .078 m; horizontal distance 17.974 m.
Minimum sampled propeller-point distance .0751 m is NOT D3 swept-body clearance.
One successful run is not repeated evidence or a paired comparison.

All Classical source/worktrees/services are protected. No Classical code is
instrumented or adapted. `examples/strategy_audit.py` reads hashes and tracked
state. Saved bag analysis runs in a separate container with no network and
read-only source/bag mounts. Outputs are solely in this Hybrid worktree.

## Capability and consumer matrix

| Capability | Actual consumer | Reused evidence | Change / remaining boundary |
| --- | --- | --- | --- |
| S0 G0/D0 | H1 → H0 → D5/D3 → receipt → existing owner | 444 baseline tests; previous H0 positive fixture | Default preserved; no experimental model or yaw selection |
| S2 G1/D0 | H1 `select_goal` → immutable selection → resolver → CourseMission → D5 | Prior held-out abstract outcomes; positive metric inspection | Named selector; bounded yaw hypothesis with independent reconstruction |
| S3 G2/D0 | Same candidate set, resolver and validator as S2 | Three existing 289-parameter MLP checkpoints; layout-held-out outcome labels | Pinned configuration; load/schema/deadline fallback retained; no superiority claim |
| S4 G1/residual | H1 `prepare` passes selected model to H0/D5 | Matched 5572-parameter plain/physics training and six-regime ablation | Actual predictor consumer added; matched S2/D5 decision report; execution remains shadow-only |
| MPC | `diagnostic_mpc.optimize` → D3, no gateway artifact | Classical optimizer before neural model, matched schedules | Cancellation added; arbitrary schedules remain diagnostic |
| P2 | Separate donor sidecar / physical integration branches | P2-C/D reports; later teammate integration commits inspected read-only | Not merged: separate map correction and registration work needs its own runtime reconciliation |
| S1 | External saved report, bag and common evaluator | User-confirmed successful full course | No Hybrid schema requirement; no Classical modifications |

R3/R6/R9/R10/R11/R12 in the Notion research library are research assignments,
not completed studies. No paper-performance claim is inferred from those prompts.

## Configurations and commands

Run from this worktree. Strategy files live in `scenarios/strategies/`:
`hybrid_existing.json`, `hybrid_information_gain.json`,
`hybrid_learned_ranker.json`, `hybrid_physics_informed.json`.
S0 remains the default. Supplying both a strategy and contradictory legacy
reasoner configuration is rejected. Classical is not a selector value.

For each strategy, compose an independently reviewed H1 profile:

```sh
python -m examples.configure_strategy --profile reviewed-h1.json \
  --strategy scenarios/strategies/hybrid_information_gain.json --output s2-h1.json
python -m examples.hybrid_sitl native-shadow --profile s2-h1.json --seconds 30 --output s2-shadow.json
```

For the isolated Gazebo launcher, use `simulator/gazebo_trial/make_hybrid_profile.py`
with that composed H1 profile and a reviewed course, then:

```sh
bash simulator/gazebo_trial/run_hybrid.sh --mode hybrid_shadow --domain 73 \
  --profile reviewed-s2-launcher.json --seconds 30
```

The launcher copies pinned ranker/predictor artifacts into its isolated source
copy. Profiles still require measured calibration/TF, clock mapping, map and
epoch, reset markers, vehicle bounds and source bindings. Templates are not
reviewed configurations. The selected strategy does not grant command authority.
Experimental S2/S3/S4 are rejected by the existing SITL owner guard; S4 is also
rejected by the D0-only execution gateway. Those promotion gates are retained.

```sh
python -m pytest tests -q
python -m examples.run_strategy_smoke --strategy scenarios/strategies/hybrid_information_gain.json --output s2-smoke.json
python -m examples.compare_strategy_predictors
python -m examples.strategy_comparison classical-run.json hybrid-run.json --output paired.json
bash examples/run_recorded_strategy_inventory.sh
```

The external evaluator refuses missing/mismatched course, permitted map, start,
vehicle, sensors, seed, mission, scoring or termination conditions. Missing
metrics stay null, never zero. It never launches a system or accesses truth in
runtime policy code. Descriptive matched results do not imply statistical superiority.

## Observation and physics contracts

Observation candidates use fresh observed support, calibration/FOV, map identity,
pose, missing evidence, stage and bounded attempt history. Yaw is at most .45 rad;
nearby translation remains at most .5 m. Invalid/contradictory/unsupported and
exhausted candidates are filtered first. A target outside a supported yaw/FOV
hypothesis still abstains. Geometry, including the stationary vehicle volume,
must pass D3. Angular progress is measured in radians separately from metres.
Candidate reconstruction revalidates the selection digest and current inputs;
an `inspection-yaw` ID alone is not sufficient. Inspection does not complete a
course checkpoint. Current camera reconstruction assumes level yaw, not full
attitude; native roll/pitch qualification remains necessary.

Physics training is the previous explicit residual consistency ablation:
closed-loop D0 acceleration/yaw response plus gated learned residual, with
data MSE plus lambda times residual consistency MSE. Lambda .1 versus zero,
identical training splits, seeds, network sizes and horizons. This is a nominal
response-law prior, not motor-thrust dynamics or a PDE PINN. See
`goal_reasoning_comparison.md` and `artifacts/goal_comparison/predictors/`.
Frozen models load once before observations, with byte bounds and SHA256 checks.
Load failures explicitly fall back to D0; invalid rollout output is rejected by
D3. There are no unbounded inference queues. Deadline expiry discards output.

## Evidence and limits

Prior reports/checkpoints are preserved byte-for-byte under
`artifacts/goal_comparison/`; fresh evidence is under `artifacts/strategy_comparison/`.
The prior abstract held-out evidence acquisition was G0 68, G1 260, and G2
259/244/243 across 420 episodes per variant. This is not physical viewpoint or
course completion evidence. Physics consistency improved some nominal errors
relative to plain residuals but did not beat D0 robustly; D0 remains default.
The new predictor decision ablation holds S2 goal and D5 candidates fixed and
records all candidate predictions, rankings, rejection reasons and host time.

No Hybrid-versus-Classical course ranking is justified yet. Repeated matched
interactive runs, native deadlines under load, GPU/CPU/RAM/RTF profiles,
physical information-gathering outcomes, and hardware qualification remain
separate acceptance gates. Archived results are not promoted to new-version
evidence merely because their checkpoints are reused.

Rollback: use the unchanged `codex/goal-reasoning-comparison` worktree at the
starting SHA or select `hybrid_existing` in this worktree. Do not reset or
switch any Classical worktree. No changes are merged to main or protected refs.

## Final verification and retained native failure

The updated suite passed **451 tests in 26.65 seconds**. The fresh abstract
held-out rerun reproduced 68/260/259/244/243 useful-evidence acquisitions for
G0/G1/G2 seeds 11/29/47. The seven-model S2/D5 ablation chose `forward` in every
positive case and stopped every sensor-loss case. Ten timing samples per case,
p50/p95/p99/max and a separate real-deadline result are in
`predictor_decisions.json`; these are host measurements, not target qualification.

The saved Classical bag was deserialized successfully with native ROS messages
in a network-isolated container, without playback or publishers. Archived map,
mission, launch and sensor/model files have exact byte hashes in
`classical_baseline.json`. They match `ad31ae5` after CRLF normalization; raw
archive hashes differ from Git blob hashes due to Windows line endings.

Fresh native observation used a standalone Hybrid launch copy (`strategy-native`)
at `aa6c77b76a29e8e6340939e2b02ffaf192c17af7`, because Linux Git cannot resolve
the Windows linked-worktree metadata. Only that copy's local line-ending setting
was adjusted. Run `run-MC0GeHVr`, domain 73, built four ROS packages and reached
all required topics with an advancing clock at approximately 1.0 RTF. It exited
1 after twelve preflight attempts: simulated-estimator health was false and the
landed report was older than the 300 ms admission limit. Last report: landed
sample at 91.444 s, current health receipt at 92.400 s. The received landed value
was true, but stale; it was correctly not accepted as fresh. No command graph
issues or command publishers appeared. No flight owner/arming was started.
The isolated container exited through its launcher cleanup; teammate services
were never controlled. Full native evidence remains in that launch copy and
the compact report/hashes are in `native_observe.json`.

This is a measured Hybrid health/freshness integration blocker, not evidence
against the successful Classical environment. Do not increase age limits or
invent health to pass it. Qualification needs a reviewed source-timing contract,
current native H1/course profile, and subsequent separately approved flight.
No experimental strategy is advertised as closed-loop Gazebo-qualified.

`preservation.json` confirms identical protected tracked-file hashes, worktree
status and HEADs, and unchanged non-task refs between snapshots. Remote heads
were also reread unchanged. Prior goal-comparison artifacts were not modified.
The external evaluator's `external_comparison_status.json` explicitly refuses
comparison until matching Hybrid course evidence and missing start/seed/map-
permission/termination conditions are supplied.
