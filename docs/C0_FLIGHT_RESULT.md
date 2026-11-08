# C0 simulated flight: execution and architecture

The user explicitly approved the C0 plan in chat. One attempt, `calibration-c0-001`, executed from flight-source commit `32dd89c21e95b45c497856b1c52f453947e7dac3`. The native owner completed takeoff, hover settling, LAND and confirmed disarm. No retry, C1 translation, policy flight or hardware flight was performed. Final independent review is recorded in `artifacts/calibration_c0_001/independent-review.md`.

## What controlled this flight

C0 is a deterministic calibration state machine, not an LLM or strategy reasoning policy. It checks fresh native telemetry and ownership, primes Offboard, requests arm, commands a bounded vertical climb, waits for the height/speed settling condition, sends one native LAND command and verifies landing/disarm. PX4 supplies the estimator, stabilization and native landing controller. Only one calibration owner publishes motion commands.

Classic/S1 remains the protected, user-confirmed physically tested comparison baseline. Neither Classic nor the synthetic-screen candidate S2/G1 controls C0. No goal ranker, strategy selector, spatial-policy trajectory selection, map control or background registration chooses motion here. AI taskforce models operate outside the flight loop for engineering and independent review.

The spatial input layer is not in C0's motion-decision path. The hybrid environment supplies simulated camera tracking and a bottom laser, with an external-vision bridge into PX4; this is simulated tracking, not a qualified physical VIO implementation. Camera/depth streams are available for recording, but C0 does not interpret them as obstacle clearance or free space. An independent truth observer evaluates the isolated lane and never supplies truth to the flight algorithm. This run does not validate spatial reasoning, registration, obstacle avoidance or course navigation.

## Exact experiment

- ROS Humble, Gazebo 8.15.0; PX4 `54f0455ffcd755534539a7cf33a09a20bf71d29d`; px4_msgs `392e831c1f659429ca83902e66820d7094591410`.
- Image `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`. Archive SHA256 `71724c6e0fef949d94344b7a9155e33afc098ff50a393c41cbfca5c362a9342e`. Configuration and geometry identities are preserved with the approval and profile.
- Fresh, separate empty 6 x 8 m lane, roof 3.048 m above floor; launch course coordinates (0,-2). Resting body origin is 0.04 m above floor. This is not the phone-scan course.
- Approximate drone16-like PX4 x500-derived vehicle, estimated mass 1.47 kg and 7-inch props. Generated collision geometry has conservative all-attitude radius 0.303078 m; design uses a 0.35 m sphere plus 0.25 m position reserve. These are simulation/design assumptions, not measured hardware uncertainty.
- Simulated OAK-D S2 depth: 512 x 320, 10 Hz, approximately 77.8 degree field of view, 0.35–10 m range, optical center 0.131 m forward. No depth-based command selection in C0.
- Target height 1.0 m above the local body origin, accepted hover band 0.8–1.2 m, climb command at most 0.20 m/s, zero commanded horizontal translation. Settling requires 3D speed at most 0.05 m/s for 0.5 s before LAND. Consequently landing can begin below exactly 1.0 m.
- Native NED bounds: north/east each [-0.5,+0.5] m, down [-1.25,+0.02] m. Actual 3D speed limit 0.50 m/s, including native landing. No braking segment or stopping-distance measurement.
- Approved ephemeral simulator override: MPC_LAND_SPEED 0.7 to 0.3 m/s, verified by readback. Existing COM_OF_LOSS_T=1 s and COM_OBL_RC_ACT=0 (Position) were preserved; offboard loss is not assumed to mean automatic LAND.
- Owner budget 60 simulation seconds / 120 wall seconds, LAND-to-disarm budget 30 s. Freshness, position validity, reset counters, command age and clock progression are guarded. The independent recorder checks timestamp mapping, data continuity and command correlation.
- Docker network disabled, all capabilities dropped, no-new-privileges, localhost DDS domain 52, immutable source archive and approval mounts. No hardware transport. The flight container exited 0 and no simulator remained running after cleanup.

## Observed evidence

Native arm command timestamp was 22.628 s; LAND 38.808 s; accepted LAND acknowledgment 38.816 s; native disarm 48.584 s. LAND-to-disarm was 9.776 s. PX4 reported takeoff, landing detected and disarm by landing. No emergency recovery or forced airborne disarm was used.

Raw flight-window local-position review contains 2,596 samples: peak estimated 3D speed 0.344915929 m/s, north [-0.001938,+0.069957] m, east [-0.003912,+0.010296] m, down [-0.873314,-0.003696] m. Thus maximum estimated height was about 0.873 m above local origin. Sample gaps were 8–16 ms; no invalid position samples, reset-counter changes or recorder/raw value mismatches were found. The strict recorder completed with 3,437 samples, 75 statuses and no faults. Raw command correlation matched the single LAND event.

The truth observer's final JSONL contains 2,240 rows; its periodically flushed summary contains 2,225 and is explicitly a checkpoint, not the final row count. Peak truth speed was 0.355894306 m/s, with no side/roof intrusion or speed exceedance. Floor-sphere overlap at rest is expected. No measured Gazebo contact stream was available: geometric checks are not contact proof.

The initial independent decoder flagged a zero-gap native status timestamp. Its initial output is preserved. The follow-up audit confirmed the single duplicate at 38.840 s matched every field; no regression, conflicting state or excessive status gap remained. The final independent verdict passes this single bounded C0 experiment. This audit rerun did not start ROS nodes or repeat the flight.

## Qualification boundary and next step

C0 establishes evidence for this single bounded native takeoff/settle/landing path and its recording/ownership checks. It does not establish stopping envelopes, policy superiority, full-course performance, physical readiness or spatial-layer flight qualification. Existing policies must still be compared against Classic/S1 before new strategies are promoted.

No C1 or subsequent flight is authorized by this C0 result. The next moving trial needs a separate concrete bounded plan and approval. Map control, background registration and persistent autonomous loops remain off.

Raw evidence is retained in the sibling workspace directory `taskforce-c0-001/evidence`; compact reports, approval, decoder, identities and evidence hashes are published under `artifacts/calibration_c0_001`. The manifest hashes the original files relative to `taskforce-c0-001`, not Git-normalized report copies; text copies may have normalized line endings. Raw bags are retained outside Git.
