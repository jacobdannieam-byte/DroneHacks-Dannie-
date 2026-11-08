# FlyX Companion Preflight Debugging & Root Cause Analysis Guide

**Author:** Gemini 4 Argon — Principal Autonomous Flight Systems Architect for DroneHacks  
**Date:** November 7, 2026  
**Target Branch:** `dannie-classical-flyx`  
**Compliance:** Strict adherence to `docs/READABLE_CODE.md` and `AGENTS.md`

---

## 1. Executive Summary & Plain-English Problem Definition

### The Core Problem
In the FlyX cloud simulation workbench (`/var/lib/flyx-lab/workspace`), the raw test script [`onboard/examples/takeoff_land.py`](../onboard/examples/takeoff_land.py) successfully arms, hovers, and lands, but executing [`main.py`](../main.py) with [`route.json`](../route.json) (the `refnav` safety node) refuses to fly and remains stuck on the ground.

### Words Used in This Guide
* **Arming:** Giving the flight controller permission to spin the motors. A disarmed drone cannot spin its motors or take off.
* **Offboard Mode:** A PX4 autopilot flight mode where motor commands and position targets are dictated over a network bridge by an onboard companion computer rather than a handheld radio transmitter.
* **EKF2 (Extended Kalman Filter 2):** The mathematical algorithm running inside PX4 firmware that fuses noisy sensor measurements (accelerometer, gyroscope, camera tracking, and laser rangefinder) into a single estimated drone position and velocity.
* **VIO (Visual-Inertial Odometry):** Using camera frames and high-speed motion sensors together to calculate where the drone has moved in 3D space without GPS. In this lab, NVIDIA cuVSLAM provides this tracking.
* **TF (Transform Frame Tree):** Standard ROS 2 system that calculates geometric relationships between physical parts of the robot (such as the distance and rotation from `base_link` at the drone's center to `camera_link` on its nose).
* **Clock Type / Sim Time:** Simulation physics runs on simulated time published on the `/clock` ROS topic. If ROS nodes use wall-clock time (`SYSTEM_TIME`) while physics uses `/clock`, sensor measurements appear millions of seconds in the past or future and are rejected.

---

## 2. Root Cause Analysis: Bare-Metal Bypass vs. Production Gating

`takeoff_land.py` and `main.py` interact with the vehicle through completely different architectural layers:

```
  ========================================================================
  PATH A: onboard/examples/takeoff_land.py (Bare-Metal Autopilot Script)
  ========================================================================
  [PX4 Autopilot SITL] 
         │  Streams /fmu/out/vehicle_local_position_v1 (Raw EKF2)
         ▼
  [takeoff_land.py] (Ignores camera, ignores TF, ignores /estimator/healthy)
         │  Immediately streams TrajectorySetpoint (target_z = -1.0 metres)
         ▼  Sends VEHICLE_CMD_DO_SET_MODE (Offboard) & VEHICLE_CMD_ARM
  [Vehicle Arms & Takes Off] (Blind flight without perception safeguards)

  ========================================================================
  PATH B: main.py -> refnav.node -> refnav.core (Safety-Gated Architecture)
  ========================================================================
  [PX4 Autopilot SITL]               [Stereo Camera]         [VIO Pipeline]
         │                                  │                      │
  /fmu/out/vehicle_status_v1     /camera/depth/image_raw    /estimator/healthy
  /fmu/out/vehicle_local_position   /camera_info + TF              │
         │                                  │                      │
         ▼                                  ▼                      ▼
  ┌──────────────────────────────────────────────────────────────────────┐
  │                    refnav.core Mission State Machine                 │
  │                         INITIAL STATE: "wait"                        │
  │                                                                      │
  │ Gate 1: PX4 Position Valid? (xy_valid, z_valid, heading_aligned)    │
  │ Gate 2: Estimator Healthy? (/estimator/healthy == True for >= 3.0 s) │
  │ Gate 3: Depth Frame Usable? (valid_fraction >= 0.30, TF ok)          │
  │ Gate 4: Timestamps Fresh? (now - depth_timestamp <= 0.5 s)           │
  └──────────────────────────────────┬───────────────────────────────────┘
                                     │
           ┌─────────────────────────┴─────────────────────────┐
           ▼                                                   ▼
     [ALL 4 GATES PASS]                               [ANY 1 GATE FAILS]
  Transitions to "arm" -> "climb"                Remains in "wait" forever
  Executes planned route.json legs               Logs refusal reason every 10 s
```

### The Four Qualification Gates in `refnav/core.py`

1. **Gate 1 — Position Fix (`Vehicle.pos_ok`):**
   * **Requirement:** PX4 must report valid horizontal position (`xy_valid`), valid height (`z_valid`), and aligned heading (`cs_yaw_align` or `heading_good_for_control`).
   * **Failure Symptom:** `Mission.why == "waiting for PX4 position and heading"`.

2. **Gate 2 — Estimator Health (`Health.arm_ok`):**
   * **Requirement:** The topic `/estimator/healthy` must publish `std_msgs/Bool(data=True)` continuously for `health_quiet_s = 3.0 seconds` (**our own choice**).
   * **Failure Symptom:** `Mission.why == "estimator health: no report has arrived"` or `"estimator health: latest report is unhealthy"`.

3. **Gate 3 — Usable Metric Depth & Geometric Transform (`Mission.depth_ready_for_arm`):**
   * **Requirement:** `/camera/depth/image_raw` must deliver valid pixel returns (`min_valid_frac >= 0.30`, **measured from flight data: >= 0.76**), matching `/camera/depth/camera_info`, and a valid ROS TF transform between `base_link` and the depth image's `header.frame_id`.
   * **Failure Symptom:** `Mission.why == "waiting for live depth: ..."` with `depth_reject` citing `"no_camera_info"` or `"no_tf"`.

4. **Gate 4 — Clock Synchronization & Freshness (`Mission.depth_fresh`):**
   * **Requirement:** Depth frames must not be older than `depth_stale_s = 0.5 seconds` (**our own choice**) and not in the future by more than `depth_future_tol_s = 0.05 seconds` (**our own choice**).
   * **Failure Symptom:** If [`flight.yaml`](../flight.yaml) specifies `use_sim_time: false` while the simulator physics publishes simulated timestamps on `/clock`, the wall clock timestamp difference exceeds millions of seconds, causing all incoming frames to be rejected as `"stale"`.

---

## 3. Dealing with Large Logs Without Cluttering Git

Because full simulation ULogs, ROS 2 rosbags, and console dumps generate tens of megabytes per minute and are rightfully listed in [`.gitignore`](../.gitignore), **do not upload or inspect raw log files**. 

Instead, observe the flight software through its two ultra-low-bandwidth diagnostic interfaces:

### Interface A: The State Oracle Topic (`/refnav/status`)
`refnav.node` publishes a single structured JSON message at 2 Hz on `/refnav/status`. Querying this topic **once** reveals the exact internal gate state and reject counters without reading any file:

```bash
ros2 topic echo /refnav/status --once
```

Example JSON response when gated:
```json
{
  "phase": "wait",
  "why": "waiting for live depth: no depth frame has arrived",
  "depth_reject": "no_camera_info: no camera_info yet on /camera/depth/camera_info",
  "rejects": {"no_camera_info": 42},
  "health_bad_s": null,
  "depth_age_s": null
}
```

### Interface B: First-Message Telemetry Logs
Commit `00d2e14` on `dannie-classical-flyx` introduced `_log_first_input(topic)` into `refnav/node.py`. The node logs the exact moment the first message arrives on each subscribed channel:
```
[INFO] [refnav]: first message received on vehicle_local_position_v1
[INFO] [refnav]: first message received on /estimator/healthy
[INFO] [refnav]: first message received on /camera/depth/image_raw
```
If a required topic never produces a `"first message received"` line within 5 seconds of node startup, its upstream driver is either misconfigured or stopped.

---

## 4. Step-by-Step FlyX Companion Debugging Workflow

Execute this 4-step checklist inside the FlyX companion terminal:

```bash
# ==============================================================================
# STEP 1: Verify and Start Perception Services
# ==============================================================================
# refnav requires cuVSLAM tracking and the external vision bridge to be active.
python3 onboard/services.py status

# If services report stopped or failed, restart them:
python3 onboard/services.py stop
python3 onboard/services.py start

# Verify processes are running in Linux:
ps -ef | grep -E "cuvslam|ev_bridge"

# ==============================================================================
# STEP 2: One-Shot Sensor Topic Verification
# ==============================================================================
# 1. Verify simulation clock is publishing
ros2 topic hz /clock

# 2. Verify estimator health (must return "data: true")
ros2 topic echo /estimator/healthy --once

# 3. Verify camera depth image frequency (should be ~15 Hz)
ros2 topic hz /camera/depth/image_raw

# 4. Verify camera info metadata exists
ros2 topic echo /camera/depth/camera_info --once

# 5. Verify optical transform from vehicle center to camera
ros2 run tf2_ros tf2_echo base_link camera_link

# ==============================================================================
# STEP 3: Run the Autonomous Node Preflight Checker
# ==============================================================================
# Run the dedicated preflight script added in this branch:
python3 onboard/debug_flyx_preflight.py

# ==============================================================================
# STEP 4: Launch main.py with Explicit Simulation Clock
# ==============================================================================
# In Terminal 1: Launch the mission node
python3 main.py --ros-args -p use_sim_time:=true

# In Terminal 2: Interrogate the status topic once
ros2 topic echo /refnav/status --once
```

---

## 5. Diagnostic Remediation Matrix

| Symptom / `why` String | Probable Cause | Actionable Fix |
| :--- | :--- | :--- |
| `"waiting for estimator healthy"` | `/estimator/healthy` topic has never published `True` | Run `python3 onboard/services.py start`. Inspect `onboard/config/services.json` to verify the cuVSLAM wrapper and EV-bridge nodes are enabled. |
| `"no transform from base_link to ..."` | ROS TF tree lacks static camera transform | In [`flight.yaml`](../flight.yaml), check `base_frame: base_link`. Broadcast fallback transform if missing: `ros2 run tf2_ros static_transform_publisher 0.1 0 0 -1.57 0 -1.57 base_link camera_link`. |
| `"no camera_info yet on ..."` | Camera metadata topic name mismatch | Run `ros2 topic list \| grep camera_info`. Set `depth_info_topic` in [`flight.yaml`](../flight.yaml) to match the actual published camera info topic. |
| `"the latest usable frame is ... s old"` | **Clock desynchronization** | Ensure `use_sim_time: true` is passed on the command line or set in [`flight.yaml`](../flight.yaml). |
| `"waiting for PX4 position and heading"` | Autopilot EKF2 has not completed yaw alignment | Click **Reset world** in the FlyX UI, release keyboard RC, and wait 5 seconds before starting `main.py`. |
