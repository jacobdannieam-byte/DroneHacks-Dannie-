# Capability matrix

The module column names the proposed home. Existing paths serve as the
first stable boundary. Test names below are relative to the relevant branch
unless a simulator path is shown. The [branch inventory](BRANCH_INVENTORY.md)
pins every source revision and the [simulation matrix](SIMULATION_MATRIX.md)
gives the execution environment and observed results.

| Capability | Source and classification | Current behavior and dependencies | Permanent home and regression evidence |
| --- | --- | --- | --- |
| Stereo, IMU, cuVSLAM | `main` · shared perception | Pairs stereo by capture stamp, initializes from camera and rig calibration, feeds IMU, publishes odometry and tracking diagnostics. Requires native ROS, cuVSLAM and compatible image/GPU build. | `onboard/slam/`, `onboard/config/`. `test_phase1_localization.py` covers normalized metadata and health evidence. Native cuVSLAM and IMU timing require the managed simulator. |
| External vision, PX4 state and frames | `main` · shared perception | `ev_bridge` gates fresh, sane VIO and supplies PX4 EV odometry. `px4_odom` exposes ROS ENU/FLU with receipt time; the native VLP/PX4 sample clock and reset counters need explicit preservation in a new planner adapter. | `onboard/vio/`, `onboard/adapters/`. `test_phase1_localization.py`, native ROS graph and ULog review. |
| Baseline forward clearance | `main` · shared input and classical baseline | `refnav/depth.py` levels depth with latest attitude and computes corridor hits and coverage for the route state machine. | `refnav/depth.py`. `test_refnav_core.py`. |
| Directional and low-band probes | `soham-depth` · shared candidate | `four_clearances` gives ahead/left/right/low results with observed/unknown distinctions, visibility bounds and a uint16 sentinel fix. Low band needs a valid floor reference. | Keep tests and review integration in `refnav/depth.py`. `test_depth_clearances.py` and `DEPTH_HANDOFF.md`. |
| Staged course mission | `soham-mission` · classical policy | `Course` has line, gate, tunnel, chicane and pad stages, pose-jump supervision and a time limit. Opt-in `Params.use_stages` delegates through the existing core. Native line/pad/flow/range inputs are unfinished. | Preserve `refnav/stages.py` in review; design its adapter before routing output through the single owner. `test_stages.py` and existing core tests. |
| Geometric/semantic memory | P1 then Phase 2 · shared representation with optional semantics | P1 tracks observed walls, corridors, openings, objects and persistent entity evidence. Phase 2 joins rectified depth with capture-time estimated pose, maintains a bounded 3D evidence volume and requires explicit calibration before allowing free cells. | `refnav/world_model/`; `test_world_model_*.py`, `test_spatial_world.py`, `test_spatial_bag_input.py`. |
| Bounded context and goal reasoning | I0–I4 · hybrid policy | Deterministic contextual goal hints with an independent validator. I4 is a design comparison. Semantics supplies a goal or rejection hint. Collision clearance comes from metric evidence. | `refnav/reasoning/`; `test_reasoning_*.py`. |
| Nominal prediction | D0 · shared dynamics | Reduced closed-loop position/heading tracking with explicit NED references, half-open effective command intervals and bounded rollouts. Default predictor. | `refnav/dynamics/physics.py`, `schema.py`; `test_dynamics_physics.py`, `test_dynamics_rollout.py`, `test_dynamics_schema.py`. |
| Learned residual and causal adaptation | D1/D2 · hybrid components | D1 MLP predicts acceleration residuals. D2 adds history MLP, GRU-32, GRU-64 and online ridge over completed transitions. Unavailable context falls back through the existing nominal path. These models predict motion inside a classical control stack. | `refnav/dynamics/residual.py`, `adaptive.py`, `history.py`; `test_dynamics_residual.py`, `test_dynamics_adaptive.py`; preserve `artifacts/adaptive_dynamics_d1/` and `adaptive_dynamics_d2/`. |
| Model selection | A1, Phase 3/A2 · research tools | Shared benchmark, checkpoint/hash and split checks, frozen feature basis with bounded online fit. A2 failed promotion; D0 stays selected, ridge remains a comparison and A3 is deferred. | `examples/evaluate_adaptive_dynamics_*.py`, `refnav/dynamics/adaptive_features.py`, `artifacts/adaptive_dynamics_a1/`, `artifacts/unified_phase3/`; `test_dynamics_a1.py`, `test_dynamics_a2.py`. |
| Trajectory geometry and score | Phase 4/D3 · shared validation | Checks aligned NED schedules, continuous swept volume, uncertainty margin and fresh metric evidence. Returns candidate diagnostics and feasible rank. Rejects unknown or occupied sweeps. | `refnav/trajectory_evaluation.py`; `test_trajectory_evaluation.py`. |
| Shadow replay | Phase 5/D4 · diagnostics | Disabled by default, read-only normalized input, causal state/verified effective interval join. Host benchmark evidence. Native recorder/exporter and Orin profiling pending. | `refnav/shadow_replay.py`, `examples/unified_shadow_replay.py`; `test_shadow_replay.py`. |
| Bounded predictive candidates | Phase 6/D5 · hybrid strategy | Up to eight bounded choices, D0 default, safety diagnostics and stop request. Accepts optional semantic goals and preserves deterministic ranking and deadline behavior. Outputs carry IDs and reasons. | `refnav/predictive_planner.py`; `test_predictive_planner.py` and `examples/unified_planner_ablation.py`. |
| Shared execution and supervision | `main`, Phases 1/7, readiness · shared execution | `refnav.node` publishes through the existing mission and its health/landing/takeover gates. `takeoff_land.py` is a separate alternative owner for its own trial. The read-only graph observer and in-process receipt rehearsal supply review evidence. | `refnav/core.py`, `refnav/node.py`, `onboard/diagnostics/`, `refnav/sitl_review.py`; `test_refnav_core.py`, `test_phase1_localization.py`, `test_sitl_review.py`. |
| Qualification | Phase 8 · research tools | Fixed paired experiment matrix and evidence reviewer with artifact integrity checks. Needs repeatable native trials and Orin profiling. | `examples/unified_qualification.py`, `docs/unified_phase8_protocol.json`; `test_unified_qualification.py`. |
| Course simulator and referee | `classical` · independent simulator tooling | PX4 SIH, Humble, course mesh, distance tables, synthetic depth/range and tracking, isolated organizer flight interface, scripted landing, external logs. The organizer image/pins differ from the managed Jazzy system. | Keep pinned in `simulator/` on the preserved lane. Geometry check, pure organizer tests, then authorized Docker flight. |
| Physical organizer launch scaffold | `classical` · isolated deployment reference | `team_nav` starts organizer `flight_interface` with automatic Offboard disabled and no mission. Documented install tool targets physical drone 16. | Preserve `drone_code/` as a distinct deployment reference. Organizer tests and team deployment review. |
| Argon proposal | `dannie-classical` · research design | Depth/SLAM proposal with estimated budgets and executable-mode changes. Distinct document; simulator algorithms inherited. | Retain proposal with author attribution; confirm intended modes before integration. |
| Standalone neural or RL strategy | Searched audited refs | D1/D2/A2 are predictors. The audit found no independently runnable learned navigation policy or RL strategy. | Defer strategy branches until real entrypoints and evidence exist. |

## Duplication and distinct work

- The managed VIO and mission files shared by `main`, the research stack
  and the classical fork come from the same Git history. Readiness adds
  separate modules. The classical emulator's organizer bridge, flight
  interface and pretend sensors solve different runtime contracts. Keep
  that stack isolated and retain its provenance.
- `refnav/depth.decode_depth` on the managed baseline treats 16-bit zero
  as invalid. Soham adds the 65535 rejection there. Phase 2 independently
  handles both sentinels and byte order in `decode_rectified_depth`.
  A shared decoder can reduce drift after verifying row stride, endian,
  invalid-return behavior and native camera encoding. The 2D directional
  probes and the bounded 3D map are distinct consumers. Preserve both.
- The emulator's course geometry is a known scene used for simulated sensor
  synthesis and scoring. Phase 2 reconstructs evidence from observations.
  Keep truth and referee interfaces outside autonomy imports.
- Frame helpers in the organizer interface and managed stack should stay
  behind their respective boundaries until sign, axis, timestamp and
  installed message contracts match. Keep third-party copies pinned.
- The depth handoff records possible gaps around unobserved-corridor dwell,
  IMU-only health and latest-attitude depth alignment. Reproduce against
  the pinned runtime before changing gates. Keep the evidence and assign
  safety review separately from the decoder change.
- Preserve the historical I4 VLA comparison as provenance. Active
  milestones and strategy work exclude VLA and WAM.
