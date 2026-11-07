# Classical simulation results

The managed reference and the classical emulator have separate runtimes and
acceptance criteria. See the [simulation matrix](../repository_audit/SIMULATION_MATRIX.md)
for launch instructions and the run record format.

## Managed reference

| Run | Scenario and revision | Result and evidence |
| --- | --- | --- |
| C-M-H01 | Historical isolated-companion 1 m takeoff, hover and landing with cuVSLAM and EV | The repository reports AUTO_LAND and confirmed disarm. Moving RTF was 0.53 and failed the sustained timing check. See the [onboard report](../../onboard/README.md#october29-native-application-check). |
| C-M-001 | Planned shared Workbench smoke, Sky Beta on the prepared course, readiness `78ef9131b2ac539042b3bb7d63ea0666724a7fbd` | Collect 30 seconds while landed and disarmed with flight programs stopped. Save the capture and report, review native depth encoding and missing returns, and record the exact runtime and settings. Follow the [SITL runbook](../unified_sitl_readiness.md). |
| C-M-002 | Planned isolated takeoff, hover and landing with cuVSLAM | Requires accepted smoke evidence, depth decoder review for the tested source and team flight approval. Compare moving clock, stream timing and estimator evidence with C-M-H01. Record the exact checkout, PX4 ULog and relevant telemetry. |

## Classical emulator

| Run | Scenario and revision | Result and evidence |
| --- | --- | --- |
| C-E-001 | Offline checks on `classical@1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae` | Five course geometry checks passed with 7 mm maximum finite-ray error. Organizer tests had 88 passes and 2 skips. Node-level test collection stopped at the missing `rclpy` dependency. See [T2–T4](../repository_audit/SIMULATION_MATRIX.md#commands-executed). |
| C-E-002 | Planned isolated landing demo on the pinned Docker/Humble/PX4 SIH runtime | Reproduce the [emulator landing recipe](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/blob/1f60fb55c6f9b7d9bb9240c6919b7a4634ac74ae/simulator/README.md). Record the image digest, course asset revision, referee output, pose CSV, simulator log and PX4 ULog. |

## Staged classical mission

| Run | Scenario and revision | Result and evidence |
| --- | --- | --- |
| C-S-001 | Offline ROS package checks at `soham-stages@0ef71cefd21a748a463d4f40ee896d1db0b6d0c6` | `python -m pytest -q drone_code/team_nav/test` passed 64 tests using stand-ins for ROS and the aircraft. See the [mission package](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/blob/0ef71cefd21a748a463d4f40ee896d1db0b6d0c6/drone_code/team_nav/README.md). |
| C-S-002 | Planned staged takeoff, hold and landing in the classical emulator | Use the existing emulator flight interface as the sole command owner. Retain the referee output and PX4 ULog for [PR #18](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/18). |

## Issues, improvements and retests

| Issue | Improvement or investigation | Retest |
| --- | --- | --- |
| C-01 · Moving timing failed in C-M-H01 | Use C-M-001 to establish a clean sensor and estimator baseline. Inspect moving RTF, sample gaps and estimator validity in the approved bounded trial. | C-M-002 pending |
| C-02 · Native missing-depth representation is unverified | Review a native sample. If missing depth uses saturated 16-bit values, qualify [the decoder fix](https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/pull/17) on the moving checkout before arming. | C-M-001, then C-M-002 pending |
| C-03 · C-E-001 could only exercise the offline subset | Run the node tests and landing demo inside the pinned emulator runtime. | C-E-002 pending |
| C-04 · The staged package has offline evidence and missing detector inputs | Verify the one-owner emulator launch and the bounded hold and landing first. Review live detector inputs before testing course stages. | C-S-002 pending |
