# Hybrid PINN command-controlled Classical-prefix trial

Prepared 2026-11-08. **One moving trial proposed; no moving flight authorized or executed.**
Registration remains off. The protected Classical success and C1 success are unchanged.

## Exact candidate and purpose

Runtime source: `32fe9f6f599360eb645310166633d61f2ed5382c` on `codex/hybrid-model-flight`.
Frozen archive and candidate identity: sibling workspace
`physical-gazebo-evidence/hybrid-model-integration-001/source-final.tar` and `candidate-final.json`.
Use the frozen archive, not a later documentation HEAD.

Image: `sha256:32650af6bc9424e7aa9a1977fe4f386ec201f23bef0c6cea587ba70e2c8ad5d1`.
Trial ID: `hybrid-pinn-classical-prefix-001`; isolated ROS domain 62, loopback only, no hardware devices.
Profile: `scenarios/model_flight/h1_pinn_prefix.json`; recording contract: `h1_recorder.json` alongside it.

This trial asks whether the frozen physics-informed residual predictor can select executable position commands and complete a short path in the successful Classical Gazebo world. It is not another C1 velocity script and is not a full-course or hardware qualification.

## What controls the aircraft

1. The existing native owner performs fresh-telemetry admission, Offboard/arming, climb and settled entry. These lifecycle actions are deterministic and explicitly distinguished from model navigation.
2. During transit, actual PX4 position, velocity and heading feed `ModelSelector`. Heading is joined to the same captured position message, including clock deferral and duplicate rejection.
3. The frozen `physics_11.npz` network, SHA256 `e935d56dfd0a9cf898c15ab2611afe9525fa9507f859f813121a9e181ed23375`, augments the existing nominal dynamics. It predicts 30 steps at 20 ms (0.6 s horizon) for four bounded held-position targets: hold, 0.08 m, 0.14 m and 0.20 m lead toward the next waypoint. Vertical lead is bounded to 0.04 m; yaw holds the observed heading. This restricted candidate family is specific to the first straight route segment; it is not the complete D5 maneuver set.
4. Candidates must stay inside the reviewed static corridor, including their full target and predicted trajectory, and predict speed no greater than 0.35 m/s. The PINN cost is terminal goal distance plus 0.15 times terminal speed. The eligible minimum-cost candidate determines the actual position/yaw setpoint.
5. The same candidates are scored by nominal D0 for a recorded counterfactual only. D0 never supplies a fallback navigation command. PX4 retains its native low-level flight controllers.
6. Model failure, invalid checkpoint, no admissible candidate or excessive inference time ends the experiment; when authority is valid the owner requests LAND. It cannot silently complete using the Classical route follower.

This is a model-predictive command-selection experiment, not an LLM pilot. The higher-level semantic/G1 reasoning stack and general D3/PINN promotion gates remain unchanged. The separate experimental owner requires a new explicit `hybrid_model_trial` approval; C1 approvals are rejected.

## Classical setup and spatial contract

The participant map is the exact archived phone-scan map from successful Classical source `ad31ae55ebf7cd3b2eaf57917537242831e3c752`, SHA256 `74a3b67ed942292d880c79cc43eb2d0740466d0c27bfa42cb5045210e05f9ccf`. Its first route segment starts at course ENU `(-2.5146, 0.9144, 0.8)` and points toward `(-2.3588, 2.4912, 0.8)`. The two proposed goals lie on that segment.

The map, vehicle/camera geometry and PX4 settings come from the successful Classical setup; the command owner and ROS sensor adapter are the existing isolated native integration. The world is the full Classical course, not the C1 empty lane. Simulated tracking and actual Gazebo depth are explicitly labeled; this does not qualify real VIO. The model reads PX4 estimates, not referee truth. Registration is off, and no prior map cell is presented as observed FREE.

The first corridor is narrow. An axis-aligned launch box intersects M1, so a broad-box clearance claim was rejected. The proposed corridor follows the archived segment with a **±0.025 m east-coordinate deviation from its centerline**, and that guard is applied to actual state as well as predictions. The horizontal geometry calculation reserves 0.35 m for body/design envelope plus another 0.10 m, and subtracts a conservative lattice-cover error that includes the segment's slope. The remaining margin beyond those reservations is about **0.002 m**; this is not the body-to-wall distance. It is a deliberately tight experimental corridor and may reject the trial because of drift.

The geometry check covers the entire corridor from the authored floor to 1.25 m height, including static low obstacles, so landing is not permitted over an unchecked boulder. Floor contact is explicitly permitted on the authored flat support plane. These are simulator design bounds, not measured statistical uncertainties or a solution to the general Hybrid ground-contact/swept-volume gate. Approval must explicitly accept this static-prior experiment. No safety threshold in the general gateway has been relaxed.

## Proposed bounded flight

One attempt, stop on first failure, no automatic retry:

1. Require current exact approval, fresh healthy landed/disarmed telemetry, idle command graph, pinned checkpoint and live recorder readiness.
2. Climb at no more than 0.20 m/s toward 0.80 m; center within 0.03 m and settle for 0.60 s.
3. Give the PINN command-selection authority for goals in native NED: `(0.30, 0.0296423135, -0.80)`, then `(0.60, 0.059284627, -0.80)`. Require distance at most 0.06 m and speed at most 0.05 m/s for sustained settling at each goal.
4. Brake with the unchanged all-axis zero-velocity command, verify settling, request native LAND and confirm disarm.

Actual 3D speed cap: **0.50 m/s**. Navigation height band: **0.65–0.95 m**. Model decision budget: **40 ms**; native publication fence: **50 ms**. Transit deadline: **20 s**. Brake settle deadline: **4 s**. LAND/disarm deadline: **30 s on both clocks**. Overall: **60 s ROS / 120 s wall**. Outer NED box: north `[-0.05, 0.85]`, east `[-0.05, 0.18]`, down `[-1.05, 0.02]`, in addition to the narrow centerline corridor. The approved ephemeral LAND speed would be 0.30 m/s, applied only after authorization; no such parameter mutation occurs in disarmed mode.

## Proof of model authority

`model_owner.decisions.jsonl` is append-only with a SHA256 chain and durable writes. Each decision records the actual state and timestamps, waypoint, checkpoint hash, every candidate command, predicted trajectory, PINN/D0 scores and selection. A separate receipt is written only after the actual publisher accepts the selected setpoint. The raw ROS bag captures `/fmu/in/trajectory_setpoint`, Offboard mode, vehicle commands, events and sensor/authority data.

`examples.audit_model_flight` independently reloads the checkpoint, replays decisions and costs, verifies the hash chain, and correlates every position publication with raw float32 PX4 command messages by timestamp and values. It rejects missing/extra commands, altered selections, command mismatches, fallback commands inside the model-control interval, model aborts and fewer than ten model commands. Flight success additionally requires both waypoints, owner completion, speed within limit, disarm and complete clean recorder/bag evidence.

Counterfactual PINN/D0 differences are reported; identical choices do not invalidate model authority, but neither do they prove PINN improvement. The frozen checkpoint was trained on synthetic dynamics; native prediction accuracy remains experimental.

## Testing and evidence

- Full repository suite: 850 passed. Focused model/lifecycle suite after the final conservative lattice-bound correction: 24 passed.
- Three closed-loop synthetic response plants (time constants 0.20/0.35/0.60 s): all reached both goals and completed landing; 99/104/116 model commands respectively.
- The same three rehearsals through private ROS PX4 messages passed exact command audit, with an idle flight command graph. PINN differed from D0 on two selected commands in the middle case; p95 selection time was approximately 9–10 ms. These are synthetic plants and private ROS transport, not Gazebo moving success.
- Native Gazebo disarmed evidence and independent raw-bag audit are in `native-disarmed-final/` and `disarmed-audit-final.json`. No model or other flight commands are issued in disarmed mode. Ground-state model hypotheses are rejected instead of inventing an airborne position.
- `native-disarmed-001/` preserves the map-byte mismatch refusal. Intermediate successful checks and all frozen candidate archives remain separate.
- `package-summary.json`, `approval-template.json` and `evidence-manifest.json` in the evidence root bind the final candidate and results. The template intentionally does not grant approval.

## Recovery and approval

The inherited owner requests ordinary LAND for model refusal and valid-authority aborts. If telemetry or authority is lost, it stops publishing instead of inventing a recovery state. Native Offboard-loss behavior is unchanged. The supervisor records recovery for a bounded 60 s after owner exit. If landed/disarmed cannot be confirmed, it marks failure and terminates only the isolated simulator under the explicitly approved termination permission. No forced airborne disarm and no retry.

The reviewer must approve this exact candidate, the experimental PINN/static-prior authority, one-attempt limits, named recovery operator, expiry and bounded failed-simulator termination. The repository's `AGENTS.md` requires: “Explain a flight plan and obtain the team's approval before arming.” This document is the proposed plan, not that approval.

After approval, execute `examples/run_model_trial.ps1` with the frozen candidate, archive, new evidence output and completed approval file. The launcher checks archive identity; the native owner rechecks exact world, vehicle, map, model, profile, recorder, expiry and live admission. Use `-DisarmedCheck` only for subscriber-only checks. No extra disarmed cycle is required solely to request approval of the frozen candidate.
