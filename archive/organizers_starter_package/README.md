# DroneHacks Autonomous Drone Companion Project

This is the team's persistent DroneHacks companion project. It runs inside the companion container (Linux / ROS 2 Jazzy) on the NVIDIA Jetson Orin Nano / simulation environment, communicating with the PX4 autopilot via Micro XRCE-DDS in Offboard Mode.

---

## 🚀 Quick Start & Native Commands

Open the native companion terminal (Ctrl + \` in Workbench) and run:

```bash
# 1. Run local decision unit tests (without starting simulator or motors)
python3 -m pytest -q tests

# 2. Check perception services (cuVSLAM, EV bridge)
python3 onboard/services.py status

# 3. Run the autonomous mission
python3 main.py
```

> [!IMPORTANT]
> **The Reset World Rule:** After using keyboard RC, land and release RC, then use **Reset world** in the web UI before running an automated program. Simulator Reset is a web UI action, not a flight code API.

---

## 📁 Python (`.py`) Architecture & Complete File Manifest

To make development clear and prevent accidental breakage, the codebase is divided into **Original Baseline Python Files** and **Autonomous Competition Modules**.

### 1. Root Entrypoint
| File | Role | Status |
| :--- | :--- | :--- |
| `main.py` | Program entrypoint. Configures takeoff height (`HEIGHT_M = 1.2`), hover duration (`HOVER_S = 5.0`), and starts `refnav.node`. | **Baseline / Editable** |

### 2. Autonomy & Navigation (`refnav/`)
The local package responsible for flight decisions, state transitions, and Offboard setpoints.
| File | Responsibility | Status |
| :--- | :--- | :--- |
| `refnav/__init__.py` | Package initialization and module exports. | **Baseline** |
| `refnav/core.py` | **The Flight Brain.** Pure Python state machine (`wait` -> `arm` -> `climb` -> `hover` -> `turn` -> `check` -> `go` -> `hold` -> `land`). Gated on estimator and depth health. Contains zero ROS imports. | **Baseline / Primary Modification Target** |
| `refnav/node.py` | **The ROS 2 Wrapper.** Subscribes to PX4 telemetry (`/fmu/out/*`) and depth (`/camera/depth/*`); publishes 20 Hz Offboard setpoints (`/fmu/in/trajectory_setpoint`) and commands. | **Baseline / Primary Modification Target** |
| `refnav/depth.py` | **Geometric Perception.** Decodes depth images (32FC1 / 16UC1), corrects camera tilt using attitude quaternion, and computes forward corridor clearance. | **Baseline / Primary Modification Target** |
| `refnav/launch.py` | ROS 2 launch argument builder; merges `flight.yaml` and deployment parameters. | **Baseline / Editable** |
| `refnav/gps.py` | Utility to wait for GPS fix for outdoor lessons. (Note: GPS is strictly forbidden indoors by Rule 6.1). | **Baseline / Leave Untouched** |

### 3. Perception, VIO & Services (`onboard/`)
Maintains the drone's position fix indoors by fusing stereo cameras and IMU into PX4.
| Subfolder / File | Responsibility | Status |
| :--- | :--- | :--- |
| `onboard/services.py` | Service manager daemon to start/stop/status background perception processes. | **Baseline / Leave Untouched** |
| `onboard/examples/takeoff_land.py` | Standalone raw ROS 2 takeoff, hover, and land reference script. | **Baseline Reference** |
| `onboard/adapters/geometry.py` | Frame conversions and quaternion math for PX4. | **Baseline / Leave Untouched** |
| `onboard/adapters/px4_health.py` | Monitors estimator status flags and publishes `/estimator/healthy`. | **Baseline / Leave Untouched** |
| `onboard/adapters/px4_odom.py` | Broadcasts ROS 2 TF transforms from PX4 local position odometry. | **Baseline / Leave Untouched** |
| `onboard/slam/cuvslam_lib_node.py` | ROS 2 wrapper for NVIDIA cuVSLAM (visual-inertial odometry). | **Baseline / Leave Untouched** |
| `onboard/slam/pose_cov.py` | Computes pose covariance matrices for visual tracking. | **Baseline / Leave Untouched** |
| `onboard/slam/stereo_pairing.py` | Matches left and right stereo camera frames. | **Baseline / Leave Untouched** |
| `onboard/slam/vio_evidence.py` | Tracks visual capture health and diagnostic logging. | **Baseline / Leave Untouched** |
| `onboard/vio/ev_bridge.py` | **External Vision Bridge.** Gates and converts SLAM odometry into `/fmu/in/vehicle_visual_odometry`. | **Baseline / Leave Untouched** |
| `onboard/vio/frames.py` | Coordinate transforms between FLU, FRD, and NED frames. | **Baseline / Leave Untouched** |
| `onboard/vio/ownership.py` | Process ownership verification and PID locks. | **Baseline / Leave Untouched** |
| `onboard/vio/ros_common.py` | Common message utilities and conversion routines. | **Baseline / Leave Untouched** |
| `onboard/vio/validation.py` | Validates odometry freshness, covariance, and velocity limits. | **Baseline / Leave Untouched** |
| `onboard/vio/vio_health.py` | Computes visual estimator health scores and quiet-window gates. | **Baseline / Leave Untouched** |

### 4. Tests (`tests/`)
| File | Responsibility | Status |
| :--- | :--- | :--- |
| `tests/test_refnav_core.py` | Unit tests for `refnav.core` and `refnav.depth` using a mock kinematic `FakePX4` and synthetic ray-cast camera. Runs in ~1 second. | **Baseline / Add Tests Here** |

---

## 🛠️ Planned Autonomous Challenge Modules (`refnav/`)

These new modules will be added to conquer the 5 course checkpoints:

1. **`refnav/line_tracker.py` (Checkpoints 1 & 2: Maze Corridors)**
   - Computer vision node detecting the floor tape through the maze to provide lateral tracking setpoints.
2. **`refnav/planner.py` (Obstacle Avoidance)**
   - Replaces the default `_land(now, out, "route blocked")` with dynamic lateral detours around 18-inch boulders and corridor walls.
3. **`refnav/tunnel.py` (Checkpoint 3: Dark Tunnel)**
   - Dead-reckoning and downward lidar (`VL53L1X`) altitude hold for transiting the unlit tunnel where visual features drop.
4. **`refnav/window_detector.py` (Checkpoint 4: Chicane Window [+5 Bonus Pts])**
   - 3D depth contrast detector to identify and navigate through the window opening instead of Gate B.
5. **`refnav/pad_detector.py` (Checkpoint 5: Precision Landing)**
   - Concentric circle landing pad detector ensuring the entire drone touches down and disarms inside the outer ring.
6. **`refnav/return_bonus.py` (Return Bonus [+25% Score Multiplier])**
   - Autonomous re-arm, vertical climb above 68 in, return transit over the barrier, and precision landing at launch.

---

## 📚 Team Setup & Session Documentation (`Dannie_Computer_Setup/`)

- [`Dannie_Computer_Setup/CODEBASE_WALKTHROUGH.md`](Dannie_Computer_Setup/CODEBASE_WALKTHROUGH.md): Deep-dive architectural walkthrough and step-by-step roadmap.
- [`Dannie_Computer_Setup/Session_context.md`](Dannie_Computer_Setup/Session_context.md): Clean session context for AI coding assistants.
- [`Dannie_Computer_Setup/Website_context.txt`](Dannie_Computer_Setup/Website_context.txt): Full competition rulebook, scoring breakdown, and hardware specifications.
- [`Dannie_Computer_Setup/CONNECT_FROM_LAPTOP.md`](Dannie_Computer_Setup/CONNECT_FROM_LAPTOP.md): SSH and browser terminal connection reference.
