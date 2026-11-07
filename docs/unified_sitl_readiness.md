
# First test in the DroneHacks simulator

Use the [SITL readiness branch](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/tree/integration/unified-sitl-readiness).
It joins the Phase 1 observer at `ef22933` with the Phase 2–8 stack at
`0a4069e` so the companion needs one checkout. The first test is a
30-second observation of the settled, landed and disarmed Sky Beta in the
supplied course. The observer has no command authority.

## Set up the Workbench checkout

Sign into [DroneHacks Connect](https://dronehacks.groups.stanford.edu/connect/challenge)
and open the native terminal in the running simulator Workbench. Keep
existing team files in place and use a separate checkout. With the sim ready,
leave the flight program stopped for the full smoke window. If keyboard RC was
used, land, release RC and reset the world in the simulator UI first.

```bash
git clone --single-branch --branch integration/unified-sitl-readiness \
  https://github.com/jacobdannieam-byte/DroneHacks-Dannie-.git \
  "$HOME/dronehacks-sitl-readiness"
cd "$HOME/dronehacks-sitl-readiness"
git rev-parse HEAD
git status --short
python3 --version
printenv ROS_DISTRO ROS_DOMAIN_ID
ros2 topic list -t
```

Run this from that checkout. Keep the output path new for each attempt.

```bash
evidence="$HOME/dronehacks-sitl-evidence"
mkdir -p "$evidence"
run="$evidence/settled-$(date -u +%Y%m%dT%H%M%SZ)"
python3 -m onboard.diagnostics.phase1_probe capture \
  --require-idle --seconds 30 --window settled-disarmed --output "$run"
```

The smoke check measures RTF, stream rates and gaps, stereo pairs, static
transforms, VIO tracking and EV stamp matching, estimator validity and graph
ownership. Strict idle mode also requires landed and disarmed reports throughout
and flags command publishers on the standard or alternate PX4 command topics.
Exit 0 means the metadata passed the smoke screen. Exit 1 retains a no-go
capture and report. Exit 2 means the runtime or inputs were unavailable or
invalid. Archive `capture.json` and `report.json` alongside the exact Git SHA,
course revision, ROS/PX4/cuVSLAM versions, VIO/depth parameters and reset state.
Keep raw sensor evidence in the team's authorized storage.

Use the discovered stream names in `capture.json` to collect the first
single-epoch Phase 2 bag. The following Bash example takes the list from a
completed capture and adds dynamic TF. Check disk space first. Stop recording
with Ctrl+C after the bounded observation window, then review the topic counts.

```bash
mapfile -t topics < <(python3 -c 'import json, sys; print(*sorted(set(json.load(open(sys.argv[1]))["topics"].values()) | {"/tf"}), sep="\n")' "$run/capture.json")
ros2 bag record --use-sim-time -o "$run/rosbag" --topics "${topics[@]}"
ros2 bag info "$run/rosbag"
```

Keep any independently recorded PX4 ULog with the corresponding test. A
bag can be replayed through Phase 2 after reviewing the clock span, depth
profile and optical-to-body extrinsics. Preserve unknown geometry unless
the depth-footprint and pose bounds have been validated. A trajectory
setpoint publication stamp supplies issue time. Phase 5 requires an
independent record of the interval when PX4 actually applied a setpoint.
A reviewed recorder or exporter must supply the normalized stream and
manifest defined in the Phase 5 runbook.

## Local verification

`python -m pytest -q tests` with one BLAS thread passed all 333 tests on
the combined Windows checkout. The Phase 2 map and replay and the Phase 4
replay matched byte for byte across two runs. Phase 5 produced four
verified comparisons from its synthetic JSONL stream. The Phase 6
ablation and 16-case Phase 7 rehearsal reproduced. The Phase 8
preflight reported zero recorded native trials and an open gate.

## Gates after the first smoke

| Phase | State and next evidence |
| --- | --- |
| 0 and 3 | Frozen D0/ridge/adaptive benchmarks. D0 remains the selected default. Review the exact combined revision and preserve the frozen checkpoints. |
| 1 | Run settled idle smoke, then obtain approved isolated takeoff/hover/landing and moving VIO timing on the same simulator configuration. The older run reported moving RTF about 0.53. |
| 2 and 4 | Replay native synchronized depth and VIO with a reviewed calibration. Verify conservative clearance against the actual course and retain unknown and blocked outcomes. |
| 5 | Capture effective command and estimator timing evidence for causal replay. Qualify concurrent Orin timing separately. |
| 6 | Check candidate rejection, recovery requests and latency against recorded course observations. Use the diagnostic planner with D0. |
| 7 | Review the native gates and the integration inside the existing single command owner. Obtain team approval for the exact isolated trial before arming. Follow the staged sequence in the [Phase 7 runbook](unified_phase7_review.md). |
| 8 | Run the preregistered qualification after repeatable Phase 7 acceptance. See the [Phase 8 protocol](unified_phase8_qualification.md). |

For the native commands and log review, see the
[Phase 1 observer](unified_phase1_localization.md),
[Phase 2 bag replay](unified_phase2_spatial.md) and
[Phase 5 shadow replay](unified_phase5_shadow.md).
