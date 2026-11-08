# Depth-guided sections and real telemetry review

Reviewed 2026-11-08. Classical source: `a641deb50df2cb5a63280a33e82772856819636a`.
This is an implementation specification and offline data audit, not qualification of a new flight policy.

## Findings

Use depth geometry to identify passages and a dynamics predictor to choose motion. The existing residual network has no image, point-cloud or occupancy input and cannot identify a corridor or tunnel. Adding such inputs would require a different model, labels and validation; it is not a capability gained by retraining its present weights.

The successful protected Classical simulator baseline is `ad31ae55ebf7cd3b2eaf57917537242831e3c752`, run `run-AdMalPxA`. All eight protected evidence hashes were checked unchanged. Latest Classical hardware work is a different revision; its telemetry is not proof that the old simulator success transferred to hardware.

Useful existing components:

- Classical `depth_camera.py` and `stereo_depth.py`: calibrated stereo to capture-stamped obstacle points; invalid matches remain unknown.
- Classical `live_map.py`: a small occupancy grid, useful for local free/occupied/unknown decisions.
- Classical `opening_crossings.py`: separates approach, alignment, level crossing, and clearance before changing height. It plans from a known map; it is not a learned tunnel detector.
- Our `refnav/world_model/geometry.py`: parallel-wall corridor hypotheses with ray evidence. It explicitly leaves full-volume coverage and connectivity unconfirmed. The current semantic schema has no tunnel type.
- Our `refnav/waypoint_scan.py`: partial looks at -20, +20, then 0 degrees already exist. Use partial mode only after the entire rotating vehicle envelope is checked; do not use its default full revolution.

Do not copy Classical wholesale: latest `depth_to_map.py` calls `correct_drift()` and updates the start frame using `WallAlignment`. That is registration and remains excluded. The proposed maze/tunnel document in Classical is explicitly a proposal, not tested capability. Its assumed dimensions and clearance values are not vehicle measurements.

## Section policy to implement

Use named sections matching the mission: first corridor, maze, tunnel, chicane, landing. Keep intermediate path points for smooth control; major-section observation gates are distinct from every interpolated waypoint.

Each section has entry/inspection, approach, passage and verified-exit waypoints. These are derived from measured geometry in a consistent local frame, with the old map used only as a prior in the authorized map-assisted experiment. They must not be extracted from simulator truth or referee checkpoints by the flight algorithm.

State sequence: APPROACH -> SETTLE -> OBSERVE -> VALIDATE -> TRAVERSE -> CONFIRM_EXIT. On insufficient observations, hold only while fresh state and a clear hold volume remain available; make bounded partial looks when their swept volume is clear. On timeout or lost authority, invoke the existing stop/recovery contract. A named section never grants permission to enter unobserved space.

Corridor validation needs two supported side surfaces, width and heading with uncertainty, obstacle/free-space evidence through the intended swept path, and enough observed distance to stop before the unobserved boundary. A gap between two clusters is not enough. The current detector's observer-between-walls requirement also needs an entrance-hypothesis path before it can detect corridors from outside.

Tunnel validation additionally needs floor/roof or curved-ceiling evidence, the narrowest cross-section over the observed interval, entrance lip clearance and a safe exit region. A 2D occupancy grid alone cannot supply this: retain a small local 3D or height-layer representation for roof/floor checks. No roof return means unknown overhead clearance, not an open tunnel. The roof need not be planar.

Compute admissibility from the full body/propeller envelope, stopping trajectory, pose error, depth error, tracking error and discretization error. Use directional bounds backed by calibration; do not shrink the vehicle or lower coverage thresholds to pass sparse real imagery. If the space cannot contain a safe in-place turn, the planner must avoid that turn, rather than discovering this after arrival.

The PINN scores bounded candidate motion only after perception and geometry admit the swept volume. Revalidate depth and state during traversal and record every candidate prediction, rejection, selected command and actual response. It does not substitute for obstacle sensing, the safety gate, or native PX4 stabilization.

## What the current PINN training actually does

`examples/compare_physics_residual.py` generates 40 synthetic training episodes and 12 validation episodes, each 100 steps: 4,000 and 1,200 samples. These randomize closed-loop gains, response lag, acceleration scale, drag and command delay. This is not real-flight training.

The network is a 17 -> 64 -> 64 -> 4 MLP (5,572 parameters), ReLU hidden layers and bounded tanh output. Features describe velocity, position error, nominal velocity/acceleration demand, yaw error/rate/demand and timestep. Outputs correct three acceleration components and yaw acceleration. An activity gate suppresses residuals at rest with zero demand.

Targets are finite-difference acceleration minus nominal predicted acceleration, clipped to the actuator-model and residual limits. NumPy Adam uses learning rate .001, batches of 256 and up to 55 epochs; the best validation checkpoint is selected. The physics variant adds a .1-weight penalty on residual magnitude, favoring the nominal response law. This is a soft dynamics prior, not a full aerodynamic model or a perception model.

The attempted full-course configuration selected `physics_11.npz`, SHA256 `e935d56dfd0a9cf898c15ab2611afe9525fa9507f859f813121a9e181ed23375`, epoch 8, normalized validation MSE .091539368. That loss is not a flight success rate or an error in metres. Real-flight telemetry has not been used to retrain this checkpoint.

## Real telemetry pulled and audited

Exported all `telemetry`, team navigation source and docs from the pinned Classical revision to a separate evidence folder. No merge into or changes to the protected baseline. Export contains 6,844 telemetry files, about 1.54 GB, with 87 session metadata files. Session names and run-mode labels do not establish flights. Nineteen sessions contain recorded armed status together with command and local-position records; their synchronized training eligibility remains unqualified.

| Recorded evidence | Audit result |
| --- | --- |
| Trajectory setpoints | 188,159 readable rows in 86 sessions |
| Finite XYZ position commands | 113,594 rows |
| Finite XYZ velocity commands | 74,565 rows; not interchangeable with position-command training |
| PX4 local position | 781,491 rows in 62 sessions |
| Sessions with observed reset-counter transitions | 50 of those 62 |
| Rows with all XYZ position/velocity validity and heading-good flags | 8,161; not an airborne-qualified count |
| Malformed rows rejected in four inspected flight streams | 48 |
| Depth diagnostic records | 38,986 valid; 8 malformed rejected |
| Saved depth pictures | 3,569 PGM files |
| Depth calculation duration | median 58.3 ms; p95 71.9 ms; maximum 719.6 ms |
| Whole-image trusted fraction | median 14.1%; p95 21.6%; maximum 42.1% |

Low whole-image coverage is a warning for perception, not a measurement of corridor-ray coverage. The latter must be measured in the intended swept volume. Calculation time excludes some acquisition/queue/transport delay and is not end-to-end freshness. These aggregates mix bench tests and flight sessions; they are not flight-only performance metrics.

The saved PGM encoding is `uint8(clip(255*(1-depth/3.5),0,255))`, after invalid depth rejection. Black merges unknown and the far end of the range. Positive intensity permits only an approximate depth interval (~13.7 mm quantization, in addition to stereo error), not the original float depth or confidence. RGB/stereo picture paths appear in camera indexes, but the corresponding raw pictures are omitted from this Git export. Do not feed PGM bytes to a metric-depth reader. Calibration, capture pose and original raw depth/stereo must accompany any training-grade perception dataset.

The audit stores per-file SHA256, per-session gaps, malformed rows, reset transitions and command-type counts. Git archive applied normal text newline conversion to three Markdown files; compressed telemetry is preserved. Audit metadata and original exports are separate. No training-ready dataset or trained hardware model is claimed.

## Implementation order and acceptance evidence

1. **Data qualification:** join capture/source timestamps with commands, armed/landed/offboard state, VIO/estimator health and reset epochs. Resolve clock offsets and frame/calibration identities; reject corrupt, stale, collision, failsafe and reset-spanning windows. Keep velocity-only control in a separately modeled cohort. Label carry/bench sessions explicitly. Output every exclusion reason and eligible duration per flight.
2. **Perception recording and replay:** preserve metric depth, validity/confidence, CameraInfo, TF and capture-time pose. Replay existing corridor extraction on calibrated recordings; add entrance and tunnel cross-section hypotheses with unknown floor/roof cases and occlusions. Establish false-clear rates on annotated real passages, including mesh, plain walls, slanted surfaces and partial views. PGM previews support annotation, not free-space acceptance.
3. **Section state machine:** bind approach/inspection/entry/exit goals to observation identities and reference/reset epochs. Add partial-look and turn-envelope checks. Test stale depth, missing roof, blocked exit, narrowing passage, pose reset, occluded sidewall and no-space-to-turn cases; each must refuse traversal without weakening the existing safety gates.
4. **Real dynamics evaluation:** first replay qualified real windows with D0 and the frozen synthetic model. Fit nominal lag/drag on training flights, then train the bounded residual on acceleration targets with filtering uncertainty. Split by whole flight/session/code/configuration, keeping overlapping recorder sessions and repeated manoeuvres in the same split. Compare held-out .2/.5/1/2-second errors, worst-case errors and runtime before promoting any weights. Keep collision data as negative evaluation cases, not normal-motion targets.
5. **Integration and moving evaluation:** resolve the independent recorder clock failure, then evaluate section perception and dynamics together on the actual full course. Registration stays off. Preserve raw capture, command-selection proof, referee results and actual video. A synthetic replay or disarmed check is not the final flight deliverable.

## Latest actual Gazebo result

Full-course implementation and offline test suite existed before this audit (853 tests passed; synthetic course replay reached all 22 transit goals). Actual Gazebo attempt 002 at runtime `75c2cab094f7f6ed70f08e41192c9e0bcf02ef22` FAILED: recorder `clock_wait_expired` caused `owner_admission_lost` after arming, before model navigation. Recorded armed interval was ROS 23.2–36.3 s; referee height remained 0 m. Zero PINN commands and zero checkpoints. Final recovery confirmed landed and disarmed. This does not test the PINN's course-following performance.

The original bag, ULog, telemetry, decisions, failures and RGB frames remain intact. A 45-second MP4 made from all 90 actual observer frames at 2 fps was decoded successfully. No further moving attempt was made after this failure.
