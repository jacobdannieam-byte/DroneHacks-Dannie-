# Branch inventory

The pre-cleanup snapshot contains **23 published branches, 16 local branches,
23 distinct tips, 15 open draft PRs and zero published or local tags**. The
default branch is `main`. The single worktree started clean at `78ef913`.
Fifteen local feature and readiness refs matched their published counterparts.
The local `experimental` branch points to `a254248`. Its stale
`origin/experimental` tracking ref points to `feb55b6`; the remote no longer
advertises that branch. The configured single-branch fetch failed for that
reason. An explicit wildcard fetch retrieved the current heads. Preserve the
local ref pending review.

The comparison uses the nearest review boundary or immediate parent.
For the two integration merges, the delta includes the merged side relative
to the first parent. Reachability shows Git preservation. Branch retirement
also requires accepted behavior and a verified archive.

| Branch | Tip | Compare with · merge base | Commits / paths | In readiness | Offline T1 | Proposed action and target | Purpose and dependencies |
| --- | --- | --- | ---: | --- | --- | --- | --- |
| `baseline-original` | [`c3c5df3`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/c3c5df33bea400209e4bec95ba99a3acdb6d67f8) | `fa2eacb` · `fa2eacb` | 1 / 43 | Yes | 30 passed | ARCHIVE → history | Pristine starter, policies, package and lessons. Preserved as ancestor. |
| `experimental` | [`a254248`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/a25424805ad79fbf122f4d84c9859ab3477b5c62) | `baseline-original` · `c3c5df3` | 1 / 8 | Yes | 30 passed | ARCHIVE → history | Local walkthrough and setup snapshot. Stale tracking ref and P1 fork point. |
| `main` | [`feb55b6`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/feb55b6a2485fb9f1aedeada371e8ca2abf96cd6) | `experimental` · `a254248` | 1 / 1 | Yes | 30 passed | KEEP → main | Shared managed Jazzy/VIO reference mission. |
| `Dannie-experimental` | [`feb55b6`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/feb55b6a2485fb9f1aedeada371e8ca2abf96cd6) | `main` · `feb55b6` | 0 / 0 | Yes | 30 passed | ARCHIVE → main | Same tip as main. Retain while #1, #3 and #7 depend on it. |
| `classical` | [`1f60fb5`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae) | `main` · `feb55b6` | 1 / 71 | No | 30 passed | KEEP → strategy/classical after acceptance | Independent Docker/Humble PX4 SIH harness, course OBJ, referee and landing demo. Full launch pending locally. |
| `dannie-classical` | [`79d4da2`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/79d4da2f0339e998dcbb01874ab1c37ca9c22243) | `classical` · `1f60fb5` | 1 / 5 | No | 30 passed | REVIEW → classical or design archive | Argon proposal and four shell mode changes. No separate strategy implementation. |
| `soham-depth` | [`7781aff`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/7781aff884bf9a0c47e03873dad543de3aa33d6a) | `main` · `feb55b6` | 1 / 3 | No | 57 passed | CONSOLIDATE → main after focused review | Four directional probes and legacy uint16 sentinel fix. Synthetic tests, no native wiring. |
| `soham-mission` | [`dd09b79`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/dd09b79bff4c272137f0a9f98f7001cf3f589219) | `soham-depth` · `7781aff` | 1 / 3 | No | 83 passed | REVIEW → classical configuration | Course stages and opt-in core hook. Requires depth branch and detector feeds. |
| `feature/semantic-world-model-p1` | [`8f71328`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/8f71328ed022377b45ee3a936df67272a5d65fbf) | `experimental` · `a254248` | 2 / 16 | Yes | 93 passed | CONSOLIDATE → main / representation | Persistent geometry-grounded semantics and evidence; fork lacks the later main notebook. |
| `feature/semantic-reasoning-i0-i1` | [`b026465`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/b026465d1c57a056a423e51872164b7474abbf9a) | `feature/semantic-world-model-p1` · `8f71328` | 2 / 19 | Yes | 160 passed | CONSOLIDATE → main / reasoning | I0–I4 contexts, deterministic proposals and independent validation on P1. |
| `feature/adaptive-dynamics-d0` | [`e99e617`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/e99e6172290231956be499aa47e99273af80a65f) | `main` · `feb55b6` | 2 / 14 | Yes | 92 passed | CONSOLIDATE → main / dynamics | D0 closed-loop nominal rollout and effective setpoint schema. |
| `feature/adaptive-dynamics-d1` | [`8f695cd`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/8f695cdbb60fdbdbf815cb340fa9c69b3b95c97d) | `feature/adaptive-dynamics-d0` · `e99e617` | 2 / 12 | Yes | 101 passed | CONSOLIDATE → main / dynamics | D1 residual MLP, checkpoint and training provenance on D0. |
| `feature/adaptive-dynamics-d2` | [`ed02901`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/ed029018d08ae91c4fb173c6b50f5b076e048bec) | `feature/adaptive-dynamics-d1` · `8f695cd` | 2 / 15 | Yes | 111 passed | CONSOLIDATE → main / dynamics | D2 causal history MLP/GRUs and ridge. Keep weights and split evidence. |
| `feature/adaptive-dynamics-a1` | [`c36918e`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/c36918e44c0cfa6712e71bf2240683b44ed55b13) | `feature/adaptive-dynamics-d2` · `ed02901` | 1 / 10 | Yes | 120 passed | CONSOLIDATE → main / research tools | A1 common benchmark and model selection on D2. |
| `feature/unified-phase1-localization` | [`ef22933`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/ef2293368d43e974ab588a9aee1d87bcf388efd9) | `main` · `feb55b6` | 1 / 6 | Yes | 46 passed | CONSOLIDATE → main / diagnostics | Phase 1 read-only graph, frame, estimator and clock observation. Native runtime required. |
| `feature/unified-phase2-spatial` | [`67b70ed`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/67b70eda292a2302cb5338a2c21e3bd6642eead3) | `feature/semantic-reasoning-i0-i1` · `b026465` | 1 / 12 | Yes | 180 passed | CONSOLIDATE → main / representation | Phase 2 metric/semantic join and single-epoch bag replay on I0–I4. |
| `feature/unified-phase3-adaptation` | [`5816a04`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/5816a0405a40f5411ad1690d52b0b6ba4bca59a8) | `feature/adaptive-dynamics-a1` · `c36918e` | 1 / 14 | Yes | 129 passed | CONSOLIDATE → main / research tools | Phase 3 A2 feature residual on A1. Preserve negative result and frozen holdout. |
| `integration/unified-phase4-base` | [`d6c363c`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/d6c363c89a465d24a345687495dfad05e80c2b89) | `feature/unified-phase2-spatial` · `67b70ed` | 10 / 58 | Yes | 279 passed | ARCHIVE → history after #10 | Existing Phase 2/3 merge, retained as the base of #10. |
| `feature/unified-phase4-trajectory` | [`294ee7d`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/294ee7dfc62280210bd62de38325f0cc82999bb0) | `integration/unified-phase4-base` · `d6c363c` | 1 / 8 | Yes | 291 passed | CONSOLIDATE → main / validation | D3 diagnostics on the combined Phase 2/3 base. |
| `feature/unified-phase5-shadow` | [`5ce2595`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/5ce25958c7ebfc91d3431b638293e589cbc1a06b) | `feature/unified-phase4-trajectory` · `294ee7d` | 1 / 6 | Yes | 299 passed | CONSOLIDATE → main / diagnostics | D4 effective-command replay. Native normalized stream and Orin evidence pending. |
| `feature/unified-phase6-planner` | [`a2c4db5`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/a2c4db5f5b2cd3008601f70d4f115187c9d0b8a1) | `feature/unified-phase5-shadow` · `5ce2595` | 1 / 6 | Yes | 308 passed | CONSOLIDATE → hybrid configuration | D5 bounded candidate fan and diagnostic stop results. Synthetic ablation. |
| `feature/unified-phase7-review` | [`0550862`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/05508625b621745f526e3b0c2dccd0c683d35715) | `feature/unified-phase6-planner` · `a2c4db5` | 1 / 6 | Yes | 310 passed | CONSOLIDATE → main / diagnostics | Read-only ownership and disconnected handoff rehearsal. Native integration pending. |
| `feature/unified-phase8-qualification` | [`0a4069e`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/0a4069e1768a72ff5cf4735428f562f484e3c649) | `feature/unified-phase7-review` · `0550862` | 1 / 5 | Yes | 315 passed | CONSOLIDATE → main / qualification | 100-trial protocol and manifest reviewer, with zero recorded native trials. |
| `integration/unified-sitl-readiness` | [`78ef913`](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/commit/78ef9131b2ac539042b3bb7d63ea0666724a7fbd) | `feature/unified-phase8-qualification` · `0a4069e` | 3 / 7 | Yes | 333 passed | KEEP → managed SITL integration | Joins Phase 1 and Phases 2–8. Strict idle smoke and native runbook. |

T1 was reproduced on an isolated archive of each distinct tip. The
[simulation matrix](SIMULATION_MATRIX.md) defines the exact command and live
blockers. Research branches share the inherited native entrypoint and add
offline modules. The classical emulator has a distinct launcher and
historical short-flight evidence. The staged Soham mission and hybrid
planner lack a verified native navigation path on their current tips.

## Pull requests and checks

All 15 were open drafts and GitHub reported them as mergeable. This is a Git
status. Runtime acceptance and team review are separate. PR #1 targets
`Dannie-experimental` with a source fork at `a254248`, one commit behind
the current target. PRs #3 and #7 also target that ref. The published
classical and Soham tips have no PR heads among these 15.

| PR | Head | Base | Status |
| --- | --- | --- | --- |
| [#1](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/1) | `feature/semantic-world-model-p1` | `Dannie-experimental` | Open draft |
| [#2](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/2) | `feature/semantic-reasoning-i0-i1` | `feature/semantic-world-model-p1` | Open draft |
| [#3](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/3) | `feature/adaptive-dynamics-d0` | `Dannie-experimental` | Open draft |
| [#4](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/4) | `feature/adaptive-dynamics-d1` | `feature/adaptive-dynamics-d0` | Open draft |
| [#5](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/5) | `feature/adaptive-dynamics-d2` | `feature/adaptive-dynamics-d1` | Open draft |
| [#6](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/6) | `feature/adaptive-dynamics-a1` | `feature/adaptive-dynamics-d2` | Open draft |
| [#7](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/7) | `feature/unified-phase1-localization` | `Dannie-experimental` | Open draft |
| [#8](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/8) | `feature/unified-phase2-spatial` | `feature/semantic-reasoning-i0-i1` | Open draft |
| [#9](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/9) | `feature/unified-phase3-adaptation` | `feature/adaptive-dynamics-a1` | Open draft |
| [#10](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/10) | `feature/unified-phase4-trajectory` | `integration/unified-phase4-base` | Open draft |
| [#11](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/11) | `feature/unified-phase5-shadow` | `feature/unified-phase4-trajectory` | Open draft |
| [#12](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/12) | `feature/unified-phase6-planner` | `feature/unified-phase5-shadow` | Open draft |
| [#13](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/13) | `feature/unified-phase7-review` | `feature/unified-phase6-planner` | Open draft |
| [#14](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/14) | `feature/unified-phase8-qualification` | `feature/unified-phase7-review` | Open draft |
| [#15](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/15) | `integration/unified-sitl-readiness` | `feature/unified-phase8-qualification` | Open draft |

GitHub Actions reported zero configured workflows and zero runs. All 23
distinct tips returned zero GitHub check runs and legacy commit statuses.
The inspected trees have no
workflow paths, submodule declarations, LFS attribute files or LFS pointer
blobs. The tracked model weights, compressed reports, OBJ and PNG are Git
blobs. Installed ROS/PX4 interfaces, cuVSLAM, native run logs and simulator
images are external. The classical Dockerfile clones upstream sources and
pins PX4 v1.16.2, MicroXRCE v2.4.3 and px4_msgs at `392e831`. Record the
Humble image digest and package versions for a reproducible build.

## Exact commits and changed paths

Each expansion gives the full head, comparison tip, merge base, commits beyond
the comparison tip and path delta from the merge base.

<details>
<summary>baseline-original · c3c5df3</summary>

Head `c3c5df33bea400209e4bec95ba99a3acdb6d67f8`. Comparison tip `fa2eacbee639f6d18ec41196e2324ce28f2dddc8`. Merge base `fa2eacbee639f6d18ec41196e2324ce28f2dddc8`.

- c3c5df3 Dannie Jacob: Import original DroneHacks baseline starter package

```text
A	AGENTS.md
A	AI_CONTEXT.md
A	CLAUDE.md
A	CONNECT_FROM_LAPTOP.md
A	FLIGHT_CODE.md
A	README.md
A	flight-source.json
A	flight.yaml
A	learn/flyx-flight-lab-v2.ipynb
M	main.py
A	onboard-source.json
A	onboard/README.md
A	onboard/__init__.py
A	onboard/adapters/__init__.py
A	onboard/adapters/geometry.py
A	onboard/adapters/px4_health.py
A	onboard/adapters/px4_odom.py
A	onboard/config/common.yaml
A	onboard/config/services.json
A	onboard/config/vio.yaml
A	onboard/config/vision_bridge.yaml
A	onboard/examples/takeoff_land.py
A	onboard/services.py
A	onboard/slam/__init__.py
A	onboard/slam/cuvslam_lib_node.py
A	onboard/slam/pose_cov.py
A	onboard/slam/stereo_pairing.py
A	onboard/slam/vio_evidence.py
A	onboard/vio/__init__.py
A	onboard/vio/ev_bridge.py
A	onboard/vio/frames.py
A	onboard/vio/ownership.py
A	onboard/vio/ros_common.py
A	onboard/vio/validation.py
A	onboard/vio/vio_health.py
A	refnav/__init__.py
A	refnav/core.py
A	refnav/depth.py
A	refnav/gps.py
A	refnav/launch.py
A	refnav/node.py
A	route.json
A	tests/test_refnav_core.py
```

</details>

<details>
<summary>experimental · a254248</summary>

Head `a25424805ad79fbf122f4d84c9859ab3477b5c62`. Comparison tip `c3c5df33bea400209e4bec95ba99a3acdb6d67f8`. Merge base `c3c5df33bea400209e4bec95ba99a3acdb6d67f8`.

- a254248 Dannie Jacob: Add codebase walkthrough, Python file architecture guide, and setup docs

```text
A	.gitignore
A	Dannie_Computer_Setup/CODEBASE_WALKTHROUGH.md
A	Dannie_Computer_Setup/CONNECT_FROM_LAPTOP.md
A	Dannie_Computer_Setup/Session_context.md
A	Dannie_Computer_Setup/Website_context.txt
A	Dannie_Computer_Setup/repomix-output.txt
M	README.md
A	repomix-output.txt
```

</details>

<details>
<summary>main · feb55b6</summary>

Head `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Comparison tip `a25424805ad79fbf122f4d84c9859ab3477b5c62`. Merge base `a25424805ad79fbf122f4d84c9859ab3477b5c62`.

- feb55b6 Dannie Jacob: Add flight notebook to Dannie_Computer_Setup

```text
A	Dannie_Computer_Setup/dronehacks-flight-notebook.ipynb
```

</details>

<details>
<summary>Dannie-experimental · feb55b6</summary>

Head `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Comparison tip `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Merge base `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`.

Both refs point to the same commit.



</details>

<details>
<summary>classical · 1f60fb5</summary>

Head `1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae`. Comparison tip `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Merge base `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`.

- 1f60fb5 Kenny Guo: Add drone simulator, flight plan and readable code guide

```text
A	.claude/settings.json
M	.gitignore
M	CLAUDE.md
M	README.md
A	docs/ARCHITECTURE.md
A	docs/READABLE_CODE.md
A	docs/course_map.png
A	drone_code/README.md
A	drone_code/install_on_drone_16.sh
A	drone_code/team_nav/README.md
A	drone_code/team_nav/launch/team_nav.launch.py
A	drone_code/team_nav/package.xml
A	drone_code/team_nav/resource/team_nav
A	drone_code/team_nav/setup.py
A	drone_code/team_nav/team_nav/__init__.py
A	simulator/.dockerignore
A	simulator/Dockerfile
A	simulator/README.md
A	simulator/build_simulator_image.sh
A	simulator/check_course_model.py
A	simulator/course_model/dronehacks-course-v3.obj
A	simulator/inside_the_box/README.md
A	simulator/inside_the_box/build_our_packages.sh
A	simulator/inside_the_box/load_ros_environment.sh
A	simulator/organizers_code/README.md
A	simulator/organizers_code/VERSION
A	simulator/organizers_code/px4_odom_bridge.py
A	simulator/organizers_code/skybeta_flight_interface/package.xml
A	simulator/organizers_code/skybeta_flight_interface/resource/skybeta_flight_interface
A	simulator/organizers_code/skybeta_flight_interface/setup.cfg
A	simulator/organizers_code/skybeta_flight_interface/setup.py
A	simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/__init__.py
A	simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/auto_offboard.py
A	simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/envelope.py
A	simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/flight_interface.py
A	simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/frames.py
A	simulator/organizers_code/skybeta_flight_interface/skybeta_flight_interface/topics.py
A	simulator/organizers_code/skybeta_flight_interface/test/integration_offboard.py
A	simulator/organizers_code/skybeta_flight_interface/test/test_auto_offboard.py
A	simulator/organizers_code/skybeta_flight_interface/test/test_envelope.py
A	simulator/organizers_code/skybeta_flight_interface/test/test_flight_interface.py
A	simulator/organizers_code/skybeta_flight_interface/test/test_frames.py
A	simulator/organizers_code/skybeta_flight_interface/test/test_topics.py
A	simulator/plot_flight.py
A	simulator/px4_changes/README.md
A	simulator/px4_changes/drone_16_settings.sh
A	simulator/px4_changes/let_simulator_read_true_position.py
A	simulator/px4_changes/lower_simulated_motion_sensor_noise.py
A	simulator/px4_changes/make_simulated_floor_solid.py
A	simulator/px4_changes/start_px4.sh
A	simulator/ros_packages/course_simulator/README.md
A	simulator/ros_packages/course_simulator/course_simulator/__init__.py
A	simulator/ros_packages/course_simulator/course_simulator/course_model.py
A	simulator/ros_packages/course_simulator/course_simulator/paths.py
A	simulator/ros_packages/course_simulator/course_simulator/pretend_bottom_laser.py
A	simulator/ros_packages/course_simulator/course_simulator/pretend_camera_tracking.py
A	simulator/ros_packages/course_simulator/course_simulator/pretend_depth_camera.py
A	simulator/ros_packages/course_simulator/course_simulator/pretend_remote_control.py
A	simulator/ros_packages/course_simulator/course_simulator/referee.py
A	simulator/ros_packages/course_simulator/course_simulator/true_drone_state.py
A	simulator/ros_packages/course_simulator/launch/whole_simulator.launch.py
A	simulator/ros_packages/course_simulator/package.xml
A	simulator/ros_packages/course_simulator/resource/course_simulator
A	simulator/ros_packages/course_simulator/setup.cfg
A	simulator/ros_packages/course_simulator/setup.py
A	simulator/ros_packages/isaac_ros_visual_slam_interfaces/CMakeLists.txt
A	simulator/ros_packages/isaac_ros_visual_slam_interfaces/msg/VisualSlamStatus.msg
A	simulator/ros_packages/isaac_ros_visual_slam_interfaces/package.xml
A	simulator/run_test_flight.sh
A	simulator/test_flights/README.md
A	simulator/test_flights/landing_test.py
```

</details>

<details>
<summary>dannie-classical · 79d4da2</summary>

Head `79d4da2f0339e998dcbb01874ab1c37ca9c22243`. Comparison tip `1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae`. Merge base `1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae`.

- 79d4da2 Dannie Jacob: Proposal for depth perception and SLAM

```text
A	docs/PROPOSAL_DEPTH_SLAM_ARGON.md
M	drone_code/install_on_drone_16.sh
M	simulator/build_simulator_image.sh
M	simulator/px4_changes/start_px4.sh
M	simulator/run_test_flight.sh
```

</details>

<details>
<summary>soham-depth · 7781aff</summary>

Head `7781aff884bf9a0c47e03873dad543de3aa33d6a`. Comparison tip `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Merge base `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`.

- 7781aff Soham Kolhe: Add four_clearances to refnav/depth.py; fix 16UC1 65535 decode

```text
A	DEPTH_HANDOFF.md
M	refnav/depth.py
A	tests/test_depth_clearances.py
```

</details>

<details>
<summary>soham-mission · dd09b79</summary>

Head `dd09b79bff4c272137f0a9f98f7001cf3f589219`. Comparison tip `7781aff884bf9a0c47e03873dad543de3aa33d6a`. Merge base `7781aff884bf9a0c47e03873dad543de3aa33d6a`.

- dd09b79 Soham Kolhe: Add refnav/stages.py course stages with a small core.py hook

```text
M	refnav/core.py
A	refnav/stages.py
A	tests/test_stages.py
```

</details>

<details>
<summary>feature/semantic-world-model-p1 · 8f71328</summary>

Head `8f71328ed022377b45ee3a936df67272a5d65fbf`. Comparison tip `a25424805ad79fbf122f4d84c9859ab3477b5c62`. Merge base `a25424805ad79fbf122f4d84c9859ab3477b5c62`.

- 8f71328 Aditya Patra: Document world model contracts and record offline replay benchmarks
- 2f3ef4c Aditya Patra: Add offline geometry-grounded semantic world model and deterministic tests

```text
A	artifacts/world_model/.gitignore
A	artifacts/world_model/benchmark.json
A	artifacts/world_model/replay.json
A	artifacts/world_model/snapshot.json
A	artifacts/world_model/test_results.txt
A	artifacts/world_model/top_down.svg
A	docs/semantic_world_model.md
A	examples/semantic_world_model_demo.py
A	refnav/world_model/__init__.py
A	refnav/world_model/geometry.py
A	refnav/world_model/observations.py
A	refnav/world_model/queries.py
A	refnav/world_model/state.py
A	tests/test_world_model_geometry.py
A	tests/test_world_model_memory.py
A	tests/test_world_model_semantics.py
```

</details>

<details>
<summary>feature/semantic-reasoning-i0-i1 · b026465</summary>

Head `b026465d1c57a056a423e51872164b7474abbf9a`. Comparison tip `8f71328ed022377b45ee3a936df67272a5d65fbf`. Merge base `8f71328ed022377b45ee3a936df67272a5d65fbf`.

- b026465 Aditya Patra: Document I0-I4 contracts, replay decisions and host benchmark evidence
- 93830cb Aditya Patra: Add bounded semantic contexts, deterministic goals and independent validation

```text
A	artifacts/reasoning/benchmark.json
A	artifacts/reasoning/decisions.md
A	artifacts/reasoning/evaluation.json
A	artifacts/reasoning/replay.json
A	artifacts/reasoning/test_results.txt
A	docs/semantic_reasoning_integration.md
A	examples/semantic_reasoning_demo.py
A	refnav/reasoning/__init__.py
A	refnav/reasoning/baseline.py
A	refnav/reasoning/context.py
A	refnav/reasoning/evaluation.py
A	refnav/reasoning/schema.py
A	refnav/reasoning/validation.py
M	refnav/world_model/state.py
A	tests/conftest.py
A	tests/test_reasoning_baseline.py
A	tests/test_reasoning_context.py
A	tests/test_reasoning_evaluation.py
A	tests/test_reasoning_validation.py
```

</details>

<details>
<summary>feature/adaptive-dynamics-d0 · e99e617</summary>

Head `e99e6172290231956be499aa47e99273af80a65f`. Comparison tip `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Merge base `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`.

- e99e617 Aditya Patra: Record D0 synthetic results and document the adaptive dynamics roadmap
- 6c0e325 Aditya Patra: Add offline D0 position-setpoint dynamics predictor and evaluation

```text
A	artifacts/adaptive_dynamics_d0/benchmark.json
A	artifacts/adaptive_dynamics_d0/evaluation.json
A	artifacts/adaptive_dynamics_d0/test_results.txt
A	docs/adaptive_dynamics_d0.md
A	docs/adaptive_dynamics_roadmap.md
A	examples/adaptive_dynamics_d0_demo.py
A	examples/d0_synthetic.py
A	refnav/dynamics/__init__.py
A	refnav/dynamics/physics.py
A	refnav/dynamics/schema.py
A	refnav/dynamics/validation.py
A	tests/test_dynamics_physics.py
A	tests/test_dynamics_rollout.py
A	tests/test_dynamics_schema.py
```

</details>

<details>
<summary>feature/adaptive-dynamics-d1 · 8f695cd</summary>

Head `8f695cdbb60fdbdbf815cb340fa9c69b3b95c97d`. Comparison tip `e99e6172290231956be499aa47e99273af80a65f`. Merge base `e99e6172290231956be499aa47e99273af80a65f`.

- 8f695cd Aditya Patra: Document D1 accuracy trade-offs and compatibility evidence
- 67c4472 Aditya Patra: Train and evaluate the offline D1 acceleration residual model

```text
A	artifacts/adaptive_dynamics_d1/evaluation.json
A	artifacts/adaptive_dynamics_d1/residual_64x64.npz
A	artifacts/adaptive_dynamics_d1/test_results.txt
A	artifacts/adaptive_dynamics_d1/training.json
A	docs/adaptive_dynamics_d1.md
M	docs/adaptive_dynamics_roadmap.md
A	examples/d1_synthetic.py
A	examples/evaluate_adaptive_dynamics_d1.py
A	examples/train_adaptive_dynamics_d1.py
M	refnav/dynamics/physics.py
A	refnav/dynamics/residual.py
A	tests/test_dynamics_residual.py
```

</details>

<details>
<summary>feature/adaptive-dynamics-d2 · ed02901</summary>

Head `ed029018d08ae91c4fb173c6b50f5b076e048bec`. Comparison tip `8f695cdbb60fdbdbf815cb340fa9c69b3b95c97d`. Merge base `8f695cdbb60fdbdbf815cb340fa9c69b3b95c97d`.

- ed02901 Aditya Patra: Record D2 adaptation results and rollout trade-offs
- 8107a64 Aditya Patra: Add causal D2 adaptive dynamics and compare learned and online estimators

```text
A	artifacts/adaptive_dynamics_d2/adaptive_gru32.npz
A	artifacts/adaptive_dynamics_d2/adaptive_gru64.npz
A	artifacts/adaptive_dynamics_d2/adaptive_mlp.npz
A	artifacts/adaptive_dynamics_d2/evaluation.json
A	artifacts/adaptive_dynamics_d2/test_results.txt
A	artifacts/adaptive_dynamics_d2/training.json
A	docs/adaptive_dynamics_d2.md
M	docs/adaptive_dynamics_roadmap.md
M	examples/d1_synthetic.py
A	examples/d2_data.py
A	examples/evaluate_adaptive_dynamics_d2.py
A	examples/train_adaptive_dynamics_d2.py
A	refnav/dynamics/adaptive.py
A	refnav/dynamics/history.py
A	tests/test_dynamics_adaptive.py
```

</details>

<details>
<summary>feature/adaptive-dynamics-a1 · c36918e</summary>

Head `c36918e44c0cfa6712e71bf2240683b44ed55b13`. Comparison tip `ed029018d08ae91c4fb173c6b50f5b076e048bec`. Merge base `ed029018d08ae91c4fb173c6b50f5b076e048bec`.

- c36918e Aditya Patra: Add A1 offline dynamics benchmark and predictor selection evidence

```text
A	artifacts/adaptive_dynamics_a1/evaluation.json.gz
A	artifacts/adaptive_dynamics_a1/rollout_summary.svg
A	artifacts/adaptive_dynamics_a1/rollouts.csv.gz
A	artifacts/adaptive_dynamics_a1/test_results.txt
A	docs/adaptive_dynamics_a1.md
A	examples/a1_data.py
A	examples/a1_metrics.py
M	examples/d1_synthetic.py
A	examples/evaluate_adaptive_dynamics_a1.py
A	tests/test_dynamics_a1.py
```

</details>

<details>
<summary>feature/unified-phase1-localization · ef22933</summary>

Head `ef2293368d43e974ab588a9aee1d87bcf388efd9`. Comparison tip `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`. Merge base `feb55b6a2485fb9f1aedeada371e8ca2abf96cd6`.

- ef22933 Aditya Patra: Add read-only unified Phase 1 localization evidence checks

```text
A	artifacts/unified_phase1/synthetic_checks.json
A	artifacts/unified_phase1/test_results.txt
A	docs/unified_phase1_localization.md
A	onboard/diagnostics/phase1_health.py
A	onboard/diagnostics/phase1_probe.py
A	tests/test_phase1_localization.py
```

</details>

<details>
<summary>feature/unified-phase2-spatial · 67b70ed</summary>

Head `67b70eda292a2302cb5338a2c21e3bd6642eead3`. Comparison tip `b026465d1c57a056a423e51872164b7474abbf9a`. Merge base `b026465d1c57a056a423e51872164b7474abbf9a`.

- 67b70ed Aditya Patra: Add unified Phase 2 spatial and semantic replay

```text
A	artifacts/unified_phase2/benchmark.json
A	artifacts/unified_phase2/map.svg
A	artifacts/unified_phase2/replay.json
A	artifacts/unified_phase2/test_results.txt
A	docs/unified_phase2_spatial.md
A	examples/spatial_bag_input.py
A	examples/unified_spatial_replay.py
A	refnav/world_model/spatial_input.py
A	refnav/world_model/spatial_pipeline.py
A	refnav/world_model/spatial_volume.py
A	tests/test_spatial_bag_input.py
A	tests/test_spatial_world.py
```

</details>

<details>
<summary>feature/unified-phase3-adaptation · 5816a04</summary>

Head `5816a0405a40f5411ad1690d52b0b6ba4bca59a8`. Comparison tip `c36918e44c0cfa6712e71bf2240683b44ed55b13`. Merge base `c36918e44c0cfa6712e71bf2240683b44ed55b13`.

- 5816a04 Aditya Patra: Evaluate bounded adaptive features in unified Phase 3

```text
A	artifacts/unified_phase3/a1_rollouts.csv.gz
A	artifacts/unified_phase3/evaluation.json.gz
A	artifacts/unified_phase3/feature_basis.npz
A	artifacts/unified_phase3/holdout_rollouts.csv.gz
A	artifacts/unified_phase3/test_results.txt
A	artifacts/unified_phase3/training.json
A	docs/unified_phase3_adaptation.md
M	examples/a1_data.py
M	examples/a1_metrics.py
M	examples/evaluate_adaptive_dynamics_a1.py
A	examples/evaluate_adaptive_dynamics_a2.py
A	examples/train_adaptive_dynamics_a2.py
A	refnav/dynamics/adaptive_features.py
A	tests/test_dynamics_a2.py
```

</details>

<details>
<summary>integration/unified-phase4-base · d6c363c</summary>

Head `d6c363c89a465d24a345687495dfad05e80c2b89`. Comparison tip `67b70eda292a2302cb5338a2c21e3bd6642eead3`. Merge base `67b70eda292a2302cb5338a2c21e3bd6642eead3`.

- d6c363c Aditya Patra: Join Phase 2 geometry and Phase 3 dynamics for offline Phase 4
- 5816a04 Aditya Patra: Evaluate bounded adaptive features in unified Phase 3
- c36918e Aditya Patra: Add A1 offline dynamics benchmark and predictor selection evidence
- ed02901 Aditya Patra: Record D2 adaptation results and rollout trade-offs
- 8107a64 Aditya Patra: Add causal D2 adaptive dynamics and compare learned and online estimators
- 8f695cd Aditya Patra: Document D1 accuracy trade-offs and compatibility evidence
- 67c4472 Aditya Patra: Train and evaluate the offline D1 acceleration residual model
- e99e617 Aditya Patra: Record D0 synthetic results and document the adaptive dynamics roadmap
- 6c0e325 Aditya Patra: Add offline D0 position-setpoint dynamics predictor and evaluation
- feb55b6 Dannie Jacob: Add flight notebook to Dannie_Computer_Setup

```text
A	Dannie_Computer_Setup/dronehacks-flight-notebook.ipynb
A	artifacts/adaptive_dynamics_a1/evaluation.json.gz
A	artifacts/adaptive_dynamics_a1/rollout_summary.svg
A	artifacts/adaptive_dynamics_a1/rollouts.csv.gz
A	artifacts/adaptive_dynamics_a1/test_results.txt
A	artifacts/adaptive_dynamics_d0/benchmark.json
A	artifacts/adaptive_dynamics_d0/evaluation.json
A	artifacts/adaptive_dynamics_d0/test_results.txt
A	artifacts/adaptive_dynamics_d1/evaluation.json
A	artifacts/adaptive_dynamics_d1/residual_64x64.npz
A	artifacts/adaptive_dynamics_d1/test_results.txt
A	artifacts/adaptive_dynamics_d1/training.json
A	artifacts/adaptive_dynamics_d2/adaptive_gru32.npz
A	artifacts/adaptive_dynamics_d2/adaptive_gru64.npz
A	artifacts/adaptive_dynamics_d2/adaptive_mlp.npz
A	artifacts/adaptive_dynamics_d2/evaluation.json
A	artifacts/adaptive_dynamics_d2/test_results.txt
A	artifacts/adaptive_dynamics_d2/training.json
A	artifacts/unified_phase3/a1_rollouts.csv.gz
A	artifacts/unified_phase3/evaluation.json.gz
A	artifacts/unified_phase3/feature_basis.npz
A	artifacts/unified_phase3/holdout_rollouts.csv.gz
A	artifacts/unified_phase3/test_results.txt
A	artifacts/unified_phase3/training.json
A	docs/adaptive_dynamics_a1.md
A	docs/adaptive_dynamics_d0.md
A	docs/adaptive_dynamics_d1.md
A	docs/adaptive_dynamics_d2.md
A	docs/adaptive_dynamics_roadmap.md
A	docs/unified_phase3_adaptation.md
A	examples/a1_data.py
A	examples/a1_metrics.py
A	examples/adaptive_dynamics_d0_demo.py
A	examples/d0_synthetic.py
A	examples/d1_synthetic.py
A	examples/d2_data.py
A	examples/evaluate_adaptive_dynamics_a1.py
A	examples/evaluate_adaptive_dynamics_a2.py
A	examples/evaluate_adaptive_dynamics_d1.py
A	examples/evaluate_adaptive_dynamics_d2.py
A	examples/train_adaptive_dynamics_a2.py
A	examples/train_adaptive_dynamics_d1.py
A	examples/train_adaptive_dynamics_d2.py
A	refnav/dynamics/__init__.py
A	refnav/dynamics/adaptive.py
A	refnav/dynamics/adaptive_features.py
A	refnav/dynamics/history.py
A	refnav/dynamics/physics.py
A	refnav/dynamics/residual.py
A	refnav/dynamics/schema.py
A	refnav/dynamics/validation.py
A	tests/test_dynamics_a1.py
A	tests/test_dynamics_a2.py
A	tests/test_dynamics_adaptive.py
A	tests/test_dynamics_physics.py
A	tests/test_dynamics_residual.py
A	tests/test_dynamics_rollout.py
A	tests/test_dynamics_schema.py
```

</details>

<details>
<summary>feature/unified-phase4-trajectory · 294ee7d</summary>

Head `294ee7dfc62280210bd62de38325f0cc82999bb0`. Comparison tip `d6c363c89a465d24a345687495dfad05e80c2b89`. Merge base `d6c363c89a465d24a345687495dfad05e80c2b89`.

- 294ee7d Aditya Patra: Rank bounded Phase 4 trajectory diagnostics against observed geometry

```text
A	artifacts/unified_phase4/benchmark.json
A	artifacts/unified_phase4/replay.json
A	artifacts/unified_phase4/test_results.txt
A	docs/unified_phase4_trajectory.md
A	examples/unified_trajectory_replay.py
A	refnav/trajectory_evaluation.py
M	refnav/world_model/spatial_volume.py
A	tests/test_trajectory_evaluation.py
```

</details>

<details>
<summary>feature/unified-phase5-shadow · 5ce2595</summary>

Head `5ce25958c7ebfc91d3431b638293e589cbc1a06b`. Comparison tip `294ee7dfc62280210bd62de38325f0cc82999bb0`. Merge base `294ee7dfc62280210bd62de38325f0cc82999bb0`.

- 5ce2595 Aditya Patra: Add opt-in read-only Phase 5 dynamics replay

```text
A	artifacts/unified_phase5/benchmark.json
A	docs/unified_phase5_shadow.md
A	examples/unified_shadow_benchmark.py
A	examples/unified_shadow_replay.py
A	refnav/shadow_replay.py
A	tests/test_shadow_replay.py
```

</details>

<details>
<summary>feature/unified-phase6-planner · a2c4db5</summary>

Head `a2c4db5f5b2cd3008601f70d4f115187c9d0b8a1`. Comparison tip `5ce25958c7ebfc91d3431b638293e589cbc1a06b`. Merge base `5ce25958c7ebfc91d3431b638293e589cbc1a06b`.

- a2c4db5 Aditya Patra: Add bounded simulation-only Phase 6 predictive planner

```text
A	artifacts/unified_phase6/ablation.json
A	docs/unified_phase6_planner.md
A	examples/unified_planner_ablation.py
A	refnav/predictive_planner.py
M	refnav/trajectory_evaluation.py
A	tests/test_predictive_planner.py
```

</details>

<details>
<summary>feature/unified-phase7-review · 0550862</summary>

Head `05508625b621745f526e3b0c2dccd0c683d35715`. Comparison tip `a2c4db5f5b2cd3008601f70d4f115187c9d0b8a1`. Merge base `a2c4db5f5b2cd3008601f70d4f115187c9d0b8a1`.

- 0550862 Aditya Patra: Rehearse Phase 7 SITL handoff without command authority

```text
A	artifacts/unified_phase7/rehearsal.json
A	docs/unified_phase7_review.md
A	examples/unified_sitl_rehearsal.py
A	onboard/diagnostics/sitl_graph.py
A	refnav/sitl_review.py
A	tests/test_sitl_review.py
```

</details>

<details>
<summary>feature/unified-phase8-qualification · 0a4069e</summary>

Head `0a4069e1768a72ff5cf4735428f562f484e3c649`. Comparison tip `05508625b621745f526e3b0c2dccd0c683d35715`. Merge base `05508625b621745f526e3b0c2dccd0c683d35715`.

- 0a4069e Aditya Patra: Preregister Phase 8 trials and review qualification evidence

```text
A	artifacts/unified_phase8/preflight.json
A	docs/unified_phase8_protocol.json
A	docs/unified_phase8_qualification.md
A	examples/unified_qualification.py
A	tests/test_unified_qualification.py
```

</details>

<details>
<summary>integration/unified-sitl-readiness · 78ef913</summary>

Head `78ef9131b2ac539042b3bb7d63ea0666724a7fbd`. Comparison tip `0a4069e1768a72ff5cf4735428f562f484e3c649`. Merge base `0a4069e1768a72ff5cf4735428f562f484e3c649`.

- 78ef913 Aditya Patra: Prepare unified stack for a disarmed SITL smoke test
- 5ef0e98 Aditya Patra: Merge branch 'feature/unified-phase1-localization' into integration/unified-sitl-readiness
- ef22933 Aditya Patra: Add read-only unified Phase 1 localization evidence checks

```text
A	artifacts/unified_phase1/synthetic_checks.json
A	artifacts/unified_phase1/test_results.txt
A	docs/unified_phase1_localization.md
A	docs/unified_sitl_readiness.md
A	onboard/diagnostics/phase1_health.py
A	onboard/diagnostics/phase1_probe.py
A	tests/test_phase1_localization.py
```

</details>
