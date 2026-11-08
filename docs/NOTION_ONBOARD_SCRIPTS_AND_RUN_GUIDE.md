# DroneHacks 2026: Onboard Script Inventory & Course Execution Guide

**Author:** Gemini 4 Argon — Principal Autonomous Flight Systems Architect for DroneHacks  
**Date:** November 7, 2026  
**Branch:** `dannie-classical-flyx2`  
**Purpose:** Team reference document formatted for Notion sync. Explains every runnable script, preflight qualification, and the exact procedures to fly the competition course in the FlyX companion environment.

---

## 1. Executive Summary: Running the Course in FlyX

In the FlyX cloud simulation workbench (`/var/lib/flyx-lab/workspace`), the team has **two distinct execution strategies** to complete the course:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│ STRATEGY A: Pre-Planned Waypoints (route.json + main.py)                        │
│ • Planned by 5 cm Dijkstra search over measured course maps (clearance >= 35 cm)│
│ • Flies each leg while verifying forward depth clearance                        │
│ • Halts and lands safely if an unexpected obstacle is closer than 0.9 metres    │
└──────────────────────────────────────┬──────────────────────────────────────────┘
                                       │
┌──────────────────────────────────────┴──────────────────────────────────────────┐
│ STRATEGY B: Dynamic Sensor-Driven Stages (refnav.stages + main.py)              │
│ • 7-stage state machine: line -> gate_a -> tunnel -> chicane -> pad -> land    │
│ • Actively probes clearances: ahead, left (+20 deg), right (-20 deg), and low   │
│ • Squares up to openings, crosses tunnel blind-room, and centers over pad       │
└─────────────────────────────────────────────────────────────────────────────────┘
```

### Exact Terminal Commands to Run in FlyX

#### Option A: Run Pre-Planned Waypoints
1. In the FlyX browser top bar, click **Reset world** (or **Session → Reset simulation**).
2. Wait 3–5 seconds for PX4 EKF2 and sensor topics to settle.
3. In the FlyX companion terminal:
   ```bash
   python3 main.py --ros-args -p use_sim_time:=true
   ```
   *(Or click the green **Run** button in the web UI for an official graded flight recording).*

#### Option B: Run Dynamic Course Stages
1. In the FlyX browser top bar, click **Reset world**.
2. In the FlyX companion terminal:
   ```bash
   python3 main.py --ros-args -p use_sim_time:=true -p use_stages:=true -p tunnel_enabled:=true
   ```

---

## 2. The Course Navigators (Scripts Attempting the Course Itself)

These are the primary control algorithms responsible for solving and flying the 5-checkpoint course:

### 1. `main.py` — The Companion Entrypoint
* **Path:** `main.py`
* **Role:** Primary ROS 2 launch wrapper inside the FlyX companion container.
* **How It Works:** Validates local package resolution for `refnav`, loads ROS parameters from `flight.yaml`, loads route coordinates from `route.json`, and starts `refnav.node`.
* **Key Parameters:**
  * `HEIGHT_M = 1.0` (metres above takeoff point).
  * `HOVER_S = 5.0` (seconds to hold before starting navigation).

### 2. `refnav/core.py` — Pure Flight Decision Engine
* **Path:** `refnav/core.py`
* **Role:** Pure mathematical state machine (zero ROS dependencies, zero system clock reads).
* **How It Works:**
  * **Preflight Safety Gates:** Requires valid PX4 position and heading (`Vehicle.pos_ok`), continuous estimator health report (`/estimator/healthy == True` for at least 3.0 seconds), and fresh, valid depth frames (`min_valid_frac >= 0.30`).
  * **Airborne Waypoint Tracking:** Turns the vehicle toward the target waypoint, advances a lookahead carrot at `speed_mps = 0.5 m/s`, and queries `refnav.depth` for forward clearance.
  * **Obstacle Braking:** If an obstacle is detected within `stop_m = 0.9 m` for `confirm_frames = 2` consecutive frames, it arrests horizontal motion and commands an immediate checked landing.

### 3. `refnav/stages.py` — The 7-Stage Course State Machine
* **Path:** `refnav/stages.py`
* **Role:** Dynamic, sensor-driven course sequence activated when `-p use_stages:=true`.
* **The 7 Sequential Stages:**
  1. `follow_line`: Tracks the floor tape down the first corridor (`speed_mps = 0.5 m/s`).
  2. `gate_a`: Balances left and right wall clearances to center through Gate A into the maze.
  3. `tunnel_align`: Squares up against the tunnel entrance face using side clearance probes.
  4. `tunnel_cross`: Bounded dead-reckoning crossing through the dark tunnel (`tunnel_len_m = 2.5 m`).
  5. `tunnel_exit`: Waits for stereo depth and visual odometry to settle upon emerging.
  6. `chicane`: Navigates past the divider wall toward Gate B.
  7. `find_pad`: Rotates and flies directly to center over the 5 ft landing pad, then triggers landing.

### 4. `drone_code/team_nav/team_nav/route_planner.py` — 3D Course Solver
* **Path:** `drone_code/team_nav/team_nav/route_planner.py`
* **Role:** Offline global path planner using 5 cm grid Dijkstra search.
* **How It Works:**
  * Takes map files (`organizers_drawing.json` or `phone_scan_2026-11-06.json`) and rulebook requirements (`missions/dronehacks_2026.json`).
  * Enforces `0.35 m` clearance from all walls (`0.30 m` drone propeller radius + `0.05 m` safety margin).
  * Penalizes paths passing within `0.25 m` of floor boulders to prevent EKF2 downward laser drop faults.
  * Generates a 4-waypoint perpendicular climb/descent through the window opening (`0.81 m` to `1.56 m` height).

### 5. `drone_code/team_nav/team_nav/route_follower.py` — Lookahead Carrot Controller
* **Path:** `drone_code/team_nav/team_nav/route_follower.py`
* **Role:** Online waypoint tracking and trajectory generator.
* **How It Works:** Slides a target position `0.65 m` ahead of the drone along the polyline. Features collinear pass-through logic (`STRAIGHT_ON_LARGEST_TURN_DEGREES = 10.0 deg`) so the drone flies steadily across the window sill without hovering over the wall.

### 6. `simulator/test_flights/route_test.py` — Full Simulator Flight Test
* **Path:** `simulator/test_flights/route_test.py`
* **Role:** Scripted end-to-end mission test for the standalone simulator.
* **How It Works:** Solves the route, takes off to 0.8 m, follows all checkpoints via `RouteFollower`, lands on the pad, and queries the referee (`referee.py`) to report wall touches, checkpoint splits, and time above boulders.

---

## 3. Flight Tests & Single-Purpose Verification Scripts

| Script | Location | Description |
| :--- | :--- | :--- |
| **`takeoff_land.py`** | `onboard/examples/takeoff_land.py` | **Bare-metal baseline.** Connects directly to PX4 uXRCE-DDS topics (`/fmu/in/*`), climbs to 1.0 m, hovers 2.0 s, and commands landing without checking depth cameras or estimator health. |
| **`landing_test.py`** | `simulator/test_flights/landing_test.py` | **Touchdown qualification test.** Descends at 0.35 m/s, verifies laser floor distance within 3 cm, and issues a 1.0 m/s downward velocity push to trigger PX4 disarm in ~1 second. |
| **`run_px4_in_gazebo.sh`** | `simulator/gazebo_trial/run_px4_in_gazebo.sh` | **Gazebo hover test.** Runs PX4 SITL and software-rendered depth camera inside Docker to measure rendering frame rates and hover stability. |
| **`run_full_route_in_gazebo.sh`** | `simulator/gazebo_trial/run_full_route_in_gazebo.sh` | **Gazebo full course test.** Flies the complete route inside Gazebo Harmonic with rigid mesh collision physics. |

---

## 4. Diagnostics, Preflight & Perception Daemons

| Script | Location | Description |
| :--- | :--- | :--- |
| **`debug_flyx_preflight.py`** | `onboard/debug_flyx_preflight.py` | **Non-arming preflight diagnostic.** Listens to ROS 2 topics for 3 seconds and checks all 4 safety gates: PX4 telemetry, `/estimator/healthy`, `/camera/depth/image_raw`, and `/clock`. Outputs a clean PASS/FAIL report. |
| **`services.py`** | `onboard/services.py` | **Companion process daemon manager.** Starts, stops, and queries background nodes listed in `onboard/config/services.json` (`cuvslam`, `vision_bridge`, `px4_odometry`). |
| **`SIM_SENSOR_EXAMPLE.py`** | `SIM_SENSOR_EXAMPLE.py` | **Simulation oracle probe.** Reads the simulated drone's true rigid-body pose and 5 cm static collision occupancy voxel grid from `dronehacks_sim`. |
| **`listen_to_every_sensor.py`** | `drone_code/hardware_checks/on_the_drone/listen_to_every_sensor.py` | **Hardware sensor validator.** Reads live topics on physical drone 16 to verify camera, IMU, and laser health before flight. |

---

## 5. Course Mapping & Route Generation Tools

Offline tools located in [`course_map_tools/`](course_map_tools/):

* **`make_map_from_organizers_model.py`:** Extracts walls, tunnel, window opening (`0.32–1.20 m`), and pad coordinates from CAD `dronehacks-course-v3.obj` to produce `organizers_drawing.json`.
* **`make_map_from_phone_scan.py`:** Aligns the 600,000-point LiDAR phone scan (`.ply`) to the course coordinate frame and outputs `phone_scan_2026-11-06.json`.
* **`draw_planned_routes.py`:** Solves the route across maps and saves high-resolution top-down PNG diagrams with clearance corridors.
* **`make_boulder_surprise_worlds.py`:** Builds synthetic course maps with moved boulders to verify that obstacle-stopping logic reacts safely.

---

## 6. Preflight Checklist Summary Table

Before commanding an automated flight in FlyX:

| Step | Action | Expected Result |
| :---: | :--- | :--- |
| **1** | Run `python3 onboard/debug_flyx_preflight.py` | All 3 reported gates show `[PASS]`. |
| **2** | Check `python3 onboard/services.py status` | State reports `running` (`cuvslam`, `vision_bridge` alive). |
| **3** | Click **Reset world** in the FlyX UI | Physics, topics, and EKF2 yaw alignment reset cleanly. |
| **4** | Execute `python3 main.py --ros-args -p use_sim_time:=true` | Vehicle arms, climbs, executes navigation, and lands. |
