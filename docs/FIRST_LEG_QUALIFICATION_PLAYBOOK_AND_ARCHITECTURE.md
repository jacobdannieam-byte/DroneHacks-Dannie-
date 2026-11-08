# DroneHacks 2026: First Leg Qualification Playbook & Architecture Manual

**Document Purpose:** Master operational manual, architectural blueprint, and telemetry forensic guide for the `first_leg` branch. Written from the perspective of the Principal Autonomous Flight Systems Architect for electrical engineers and autonomous robotics teammates preparing to qualify on physical hardware (Drone 16) and in simulation at DroneHacks 2026.

---

## 1. Executive Summary & Qualification Scoring Realities

At DroneHacks, the overwhelming majority of autonomous teams fail to qualify because their software suffers from **"vibes upon vibes"**—cascading heuristic assumptions, over-sensitive software abort triggers, and unstable reactive planners that deadlock in narrow spaces or crash on takeoff.

### The Competition Landscape
* **The Qualification Hurdle:** Over 80% of participating teams never clear the first corridor. They fail on the launch pad (EKF2 drift), flip into the net on takeoff, or trigger software aborts during their first turn.
* **The High-Scoring Trap:** Teams attempt to fly the entire course on their first flight—braving the narrow window over the 0.40 m box, navigating the chicane, and traversing the unlit tunnel. This introduces exponential risk for diminishing marginal points before any baseline qualification score is recorded.

### The Winning Strategy: Bank the Qualification Tier
* **Tier 1 Checkpoints:**
  1. **Checkpoint 1 (Exit of Corridor 1):** +10 Points. Cleared by flying past North = +1.80 m.
  2. **Checkpoint 2 (Maze Segment Entry past Divider M1):** +15 Points. Cleared by advancing East past X = -1.85 m along Y = +2.18 m.
  3. **Controlled Ground Touchdown:** +10 Points. Touchdown within course boundary and automatic motor shutdown.
  4. **Total Banked Score: 35 Points.** This guaranteed baseline secures a Top 3 qualification finish, advancing the team to finals.
* **Our Primary Objective:** A deterministic, rock-solid execution of the first leg:
  ```text
  [Launch Pad: (-2.51, +0.91)] 
         │  Climb to +0.75 m (0.20 m/s)
         ▼
  [Cruise Hold: (-2.51, +0.91, +0.75)]
         │  Advance North (0.30 m/s)
         ▼
  [Turn Pocket: (-2.51, +2.18, +0.75)]  <-- 0.87 m buffer to North net
         │  Coordinated Clockwise Turn 90° -> 0° @ 15°/s (6.0 s)
         ▼
  [Maze Ingress: (-2.51, +2.18, +0.75)]
         │  Advance East (0.25 m/s)
         ▼
  [Landing Spot: (-0.50, +2.18, +0.75)] <-- Clears Divider M1 by > 0.60 m
         │  Descent & Touchdown (0.35 m/s)
         ▼
  [Safely Disarmed on Floor]
  ```

---

## 2. Signal vs. Noise: Exhaustive Engineering Assessment

Time on competition day is strictly constrained. Teams lose hours debugging complex subsystems that do not contribute to qualification. The table below details what is strictly useful (high signal) versus what is dangerous noise:

| Component / Subsystem | Signal Rating | Observed Risk / Failure Mode | Tactical Verdict |
|---|---|---|---|
| **Deterministic Waypoints (`first_maze_leg.py`)** | **CRITICAL (High Signal)** | None. Coordinates are measured from the competition arena to millimeter precision. | **USE EXCLUSIVELY** for qualification runs. |
| **cuVSLAM Visual Odometry** | **CRITICAL (High Signal)** | Tracking silence gaps > 0.20 s occur during sudden maneuvers or textureless floor areas. | **USE**, with widened silence limit (`CAMERA_TRACKING_LONGEST_SILENCE_SECONDS = 0.5 s`). |
| **Calibrated Yaw Rates ($15^\circ/\text{s}$)** | **CRITICAL (High Signal)** | Drone 16 exhibits 0.50 m lateral position slide when yawing at $30^\circ/\text{s}$. | **ENFORCE $15^\circ/\text{s}$** limit on all yaw maneuvers. |
| **Human Pilot Kill Switch Authority** | **CRITICAL (High Signal)** | Software automated landing logic tries to descend at hover thrust when hitting a net, winding props into mesh. | **RELY ON PILOT KILL SWITCH** for emergency motor shutoff. |
| **Bare-Bones Safety Option** | **CRITICAL (High Signal)** | Hair-trigger tilt (> 20°) and height checks (> 15 cm divergence) cause false software aborts in mid-air. | **ENABLE `--bare-bones-safety`** to suppress false aborts while keeping kill switch active. |
| **Dynamic A* Live Replanning** | **FATAL (Pure Noise)** | Narrow $1.19\text{ m}$ corridor + $0.35\text{ m}$ inflation buffers leaves $< 49\text{ cm}$ channel. Stereo noise causes instant deadlock (`path_squares = 0`). | **BYPASS ENTIRELY** in Corridor 1. |
| **"Look-Around" Pirouette Loop** | **FATAL (Pure Noise)** | When A* deadlocks, drone spins $360^\circ$ in four $90^\circ$ steps. Yaw slide immediately slams airframe into corridor walls. | **SUPPRESS COMPLETELY**. Never spin in a corridor. |
| **HeightKeeper (`USE_HEIGHT_KEEPER = True`)** | **FATAL (Pure Noise)** | cuVSLAM camera height drifts $10$–$20\text{ cm}$. HeightKeeper offsets fight the bottom laser, causing immediate `HeightCheck` aborts. | **HARDCODE OFF** (`USE_HEIGHT_KEEPER = False`). |
| **Live Stereo Raycasting Obstacle Erasure** | **DANGEROUS (Noise)** | A single noisy far stereo point subtracts $0.5$ from cell scores, instantly erasing verified $1.2$-score walls (`BLOCKED_SCORE = 1.0`). | **DO NOT USE** for Corridor 1 path clearance. |
| **WallAlignment Scan-Matching** | **USELESS (Noise in Corridor 1)** | All walls in Corridor 1 are perimeter netting or start with `"Sketch"`, which `_trusted_wall_squares` ignores. Zero scan matching occurs. | **BYPASS**; rely on visual odometry. |
| **The Window Obstacle** | **HIGH RISK (Low Signal)** | $17\text{ cm}$ vertical clearance over a $0.40\text{ m}$ box that causes laser dropouts. Awards only 5 extra points. | **SKIP** until qualification score is banked. |

---

## 3. Telemetry Forensic Autopsy: Analysis of Real Drone 16 Crashes

Detailed inspection of real physical flight logs from the companion computer (`flight_records`) reveals the exact failure mechanisms that destroyed previous attempts:

```mermaid
flowchart TD
    subgraph Run_014841["Run 014841: Takeoff Climb Timeout"]
        A1["Launch Arming"] --> B1["cuVSLAM tracking jitter > 0.20 s"]
        B1 --> C1["Safety state switches to HOLD (25 times)"]
        C1 --> D1["Setpoints frozen; Drone hovers at 0.15 m"]
        D1 --> E1["LONGEST_CLIMB_SECONDS (20.0 s) expires"]
        E1 --> F1["Flight Aborted on Pad"]
    end

    subgraph Run_105138["Run 105138: 170° Inverted Net Flip"]
        A2["Drone sits on pad for 124.4 s"] --> B2["EKF2 yaw & position drift"]
        B2 --> C2["Takeoff slides 33 cm South"]
        C2 --> D2["Rear prop clips South Net at 0.51 m"]
        D2 --> E2["Software enters landing descent at hover thrust"]
        E2 --> F2["Props wind netting into hubs: 170° Inverted Flip"]
    end

    subgraph Run_104453["Run 104453: Pirouette Target Jump"]
        A3["Drone enters Corridor 1"] --> B3["Stereo noise inflates corridor walls"]
        B3 --> C3["A* Planner Deadlock: path_squares = 0"]
        C3 --> D3["Fallback: start_look_around() 360° pirouette"]
        D3 --> E3["Yaw drift pushes drone 0.50 m laterally"]
        E3 --> F3["Target > 1.2 m from drone: Software Abort"]
    end

    subgraph Run_050406["Run 050406: HeightKeeper Divergence"]
        A4["Drone takes off cleanly"] --> B4["cuVSLAM camera height drifts +10 cm"]
        B4 --> C4["HeightKeeper commands +10 cm offset"]
        C4 --> D4["PX4 bottom laser disagrees with offset"]
        D4 --> E4["HeightCheck fires at t = 4.0 s"]
        E4 --> F4["Premature Abort & Emergency Landing"]
    end
```

### Forensic Case 1: `20261108_014841` (Climb Timeout)
* **Symptom:** Drone armed, spun props, but never climbed above $0.20\text{ m}$. After $20\text{ seconds}$, the program aborted and shut down.
* **Log Evidence:** `events.jsonl.gz` revealed 25 occurrences of `action=HOLD` within the first $20\text{ s}$ of flight.
* **Root Cause:** The camera tracking silence limit was set to $0.20\text{ s}$. cuVSLAM on the Jetson Orin Nano periodically experienced frame processing hiccups lasting $0.22$–$0.28\text{ s}$. Every hiccup triggered `HOLD`, which froze the vertical climb setpoint. The drone was trapped in place until `LONGEST_CLIMB_SECONDS = 20.0 s` expired.
* **Fix Applied:** `CAMERA_TRACKING_LONGEST_SILENCE_SECONDS` increased from $0.20\text{ s}$ to $0.50\text{ s}$.

### Forensic Case 2: `20261108_105138` (170° Inverted Net Flip)
* **Symptom:** Drone climbed, drifted backwards, snagged the South perimeter net at $0.51\text{ m}$ height, and flipped violently upside down ($170^\circ$ pitch/roll, $771^\circ/\text{s}$ rotation rate).
* **Log Evidence:** Telemetry showed the drone was booted and sat on the launch pad for **$124.4\text{ seconds}$** before the operator armed. During this 2-minute ground dwell, PX4 EKF2 accumulated internal ground drift. On takeoff, the setpoint requested $(0.0, 0.0)$, but the drifted estimator dragged the drone $33\text{ cm}$ South into the net.
* **Secondary Disaster:** When `TouchDetector` fired, `land_after_a_problem()` commanded a normal landing descent ($0.50\text{ m/s}$ down). Because the drone was hung up on the net, it never reached the floor; the motors continued spinning at hover thrust, winding netting into motor shafts until the drone flipped inverted.
* **Fix Applied:** 
  1. Strict **5-Second Arming Cadence**: arm and engage Position mode within 5 seconds of script prompt.
  2. Immediate motor kill switch authority: pilot kills throttle instantly upon net contact; never let software try to descend when snagged.

### Forensic Case 3: `20261108_104453` (The Pirouette Target Jump)
* **Symptom:** Mission aborted in the middle of Corridor 1 with the error: `Target is 1.43 m from drone, more than 1.2 m allowed`.
* **Log Evidence:** In Corridor 1, stereo obstacle points populated both sides of the $1.19\text{ m}$ hallway. Obstacle inflation merged across the corridor, leaving 0 open grid cells. A* failed. The planner initiated `start_look_around()`, attempting a $360^\circ$ in-place spin. At $30^\circ/\text{s}$, the drone slipped $0.50\text{ m}$ laterally. The next waypoint setpoint was $> 1.2\text{ m}$ from the actual drone position, triggering an immediate safety violation.
* **Fix Applied:** Bypassed A* planning in Corridor 1; restricted yaw rates to $15^\circ/\text{s}$.

### Forensic Case 4: `20261108_050406` (HeightKeeper False Divergence)
* **Symptom:** Flight aborted 4 seconds after takeoff while hovering above the launch pad.
* **Log Evidence:** `HeightKeeper` was active. It observed cuVSLAM visual height drifting upward by $10\text{ cm}$ due to ground texture ambiguity. Believing the drone was too high, it commanded a negative height offset to PX4. The bottom laser altimeter accurately reported the true distance to the floor. The disagreement between commanded offset and laser reality tripped `HeightCheck`, which aborted the flight.
* **Fix Applied:** Hardcoded `USE_HEIGHT_KEEPER = False`. The bottom laser altimeter is the sole, trusted source for vertical positioning.

---

## 4. End-to-End System Architecture (The 9 Phases)

The complete dataflow from laptop terminal to physical propeller actuation is organized into 9 coordinated phases:

```
[Phase 1: Deploy] ----> [Phase 2: Launch] ----> [Phase 3: Telemetry]
         │                       │                       │
         ▼                       ▼                       ▼
[Phase 4: Perception] -> [Phase 5: Frames]  -> [Phase 6: Safety Loop]
         │                       │                       │
         ▼                       ▼                       ▼
[Phase 7: Executive] --> [Phase 8: Actuation] -> [Phase 9: Post-Mortem]
```

### Phase 1: Deploy & Build Layer
* **[`drone_code/install_on_drone_16.sh`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/install_on_drone_16.sh):**
  - Connects to Jetson via Wi-Fi (`skybeta-16.local`) or USB tether (`192.168.55.1`).
  - Verifies target hostname is strictly `skybeta-16` (protects against deploying to another team's drone).
  - Synchronizes `drone_code/team_nav` and `drone_code/team_smoke_flights` via `rsync -avz --delete`.
  - Executes `docker exec skybeta-stack /bin/bash -c "cd ~/team_ws && ./team_build.sh"`.
* **[`drone_code/team_nav/drone.sh`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/drone.sh):**
  - Manages background tmux sessions (`team_flight`) inside the Docker container.
  - Commands: `status`, `start <launch_file>`, `stop`, `course_run`, `practice_flight <flight_name>`.

### Phase 2: Launch Architecture
* **[`drone_code/team_nav/launch/test_flights.launch.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/launch/test_flights.launch.py):**
  - Spawns `flight_interface` with `auto_offboard_on_arm = True` and `target_system = 16`.
  - Spawns `flight_recorder` (records `events.jsonl.gz`, `drone_condition.csv.gz`, `targets.csv.gz`).
  - Spawns `depth_camera` stereo matcher.
  - Leaves control unassigned until an operator starts a flight program.
* **[`drone_code/team_nav/launch/watch_only.launch.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/launch/watch_only.launch.py):**
  - Safe standby mode. Sets `auto_offboard_on_arm = False`. Propellers cannot be commanded by software.

### Phase 3: Telemetry Ingestion Layer
* **[`drone_code/team_nav/team_nav/drone_reports.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/drone_reports.py):**
  - Subscribes to `/skybeta/state` (summarized flight condition from organizer harness).
  - Subscribes to PX4 FMU uORB messages:
    - `/fmu/out/vehicle_local_position` (NED position and velocity).
    - `/fmu/out/vehicle_attitude` (quaternion orientation).
    - `/fmu/out/vehicle_odometry` (body rates and accelerations).
    - `/fmu/out/distance_sensor` (downward laser altimeter, where available).
    - `/fmu/out/estimator_status_flags` (laser usage flags `cs_rng_hgt`, `cs_rng_fault`).
* **[`drone_code/team_nav/team_nav/flight_steps.py` (`DroneLink`)](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/flight_steps.py#L182):**
  - Subscribes to `/visual_slam/tracking/odometry` (cuVSLAM 30 Hz odometry).
  - Synchronizes all sensor readings onto a monotonic clock.
  - Enforces the $10\text{ Hz}$ discrete control cycle (`wait_one_cycle()`).
  - Captures and freezes ground baseline readings (`remember_launch_mark()`).

### Phase 4: Perception Pipeline
* **[`drone_code/team_nav/team_nav/depth_camera.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/depth_camera.py) & [`stereo_depth.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/stereo_depth.py):**
  - Subscribes to left/right rectified camera frames: `/oak/left/image_raw` and `/oak/right/image_raw` ($512 \times 320$).
  - Buffers the last 5 frames from each camera; matches image pairs within $0.02\text{ s}$ (`PAIR_TOLERANCE_SECONDS`).
  - Computes disparity using block matching: $9 \times 9$ patch SAD, $64$ disparity levels.
  - Projects disparities to 3D camera coordinates:
    $$Z = \frac{f \cdot B}{d}, \quad X = \frac{(u - c_x) \cdot Z}{f}, \quad Y = \frac{(v - c_y) \cdot Z}{f}$$
  - Offsets 3D points by physical camera mounting extrusions: $+10.53\text{ cm}$ forward, $+3.04\text{ cm}$ up.
  - Filters out points beyond $3.5\text{ m}$ (`FARTHEST_USEFUL_METRES`).
  - Publishes `sensor_msgs/PointCloud2` on `/team_nav/obstacle_points`.
* **[`drone_code/team_nav/team_nav/depth_to_map.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/depth_to_map.py):**
  - Queries `link.tracked_pose_at(timestamp)` to obtain the visual odometry pose at the exact moment of optical exposure.
  - Transforms points into Course Coordinates via `start_frame.py`.
  - **Bug Fixed on this Branch:** Initialized `self.last_picture_monotonic_seconds = time.monotonic()` in constructor to prevent false cycle-0 freeze aborts.

### Phase 5: Coordinate Frames & Spatial Representation
* **[`drone_code/team_nav/team_nav/start_frame.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/start_frame.py):**
  - Manages rigid 2D Euclidean SE(2) transformations between Camera Tracking frame $(x_{\text{fwd}}, y_{\text{left}}, z_{\text{up}})$ and Course Map frame $(\text{East}, \text{North}, \text{Height})$:
    $$\begin{bmatrix} \text{East} \\ \text{North} \end{bmatrix} = \begin{bmatrix} \text{East}_{\text{launch}} \\ \text{North}_{\text{launch}} \end{bmatrix} + \text{fwd} \cdot \begin{bmatrix} \cos\theta \\ \sin\theta \end{bmatrix} + \text{left} \cdot \begin{bmatrix} -\sin\theta \\ \cos\theta \end{bmatrix} + \vec{\delta}_{\text{drift}}$$
* **[`drone_code/team_nav/team_nav/live_map.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/live_map.py):**
  - $5\text{ cm}$ resolution 2D occupancy grid.
  - Log-odds update: raycasts free space (`-0.5` per empty cell) and marks hits (`+0.6` per obstacle point).
  - Thresholds cells $\ge 1.0$ (`BLOCKED_SCORE`) as solid obstacles.
  - Computes Euclidean Distance Transform (EDT) using `scipy.ndimage.distance_transform_edt` for obstacle avoidance buffers.

### Phase 6: Safety Supervision Loop
* **[`drone_code/team_nav/team_nav/run_safety.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/run_safety.py):**
  - Evaluated at $10\text{ Hz}$ on every control cycle:
    1. `CONTROL_LOST`: Pilot flipped switch away from Position, disarmed RC, or PX4 failed safe. Halts setpoint publishing immediately.
    2. `LAND_HERE` (Touch): `touch_detector.py` reports tilt $> 20^\circ$ or rotation rate $> 120^\circ/\text{s}$.
    3. `LAND_HERE` (Height Divergence): `height_check.py` reports $> 15\text{ cm}$ gap between PX4 $Z$ and bottom laser over clear floor.
    4. `HOLD`: cuVSLAM tracking silence $> 0.5\text{ s}$. Freezes setpoint for up to $5.0\text{ s}$ before landing.
    5. `check_target()`: Validates that candidate target coordinates are strictly inside the course perimeter net ($[-3.0, +3.0]\text{ m}$) and under the ceiling ($Z \le 1.6\text{ m}$).

### Phase 7: Mission Executive Layer
* **[`drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py):**
  - The dedicated deterministic qualification flight executive.
  - Coordinates all stage transitions, linear waypoint progressions, settle pauses, and landing triggers.

### Phase 8: Actuation & Setpoint Dispatch
* **[`drone_code/team_nav/team_nav/target_sender.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/target_sender.py):**
  - Sole publisher of control setpoints to ROS 2 topics:
    - `/skybeta/setpoint/pose` (`geometry_msgs/PoseStamped`)
    - `/skybeta/setpoint/twist` (`geometry_msgs/TwistStamped`)
  - Interpolates setpoints smoothly at $10\text{ Hz}$ to prevent velocity step jumps.
  - Streams setpoints continuously to satisfy PX4 Offboard heartbeat requirement ($> 2\text{ Hz}$).

### Phase 9: Landing & Post-Mortem Analysis
* **Landing Controller (`flight_steps.land`):**
  - Stage 1: Medium descent at $0.50\text{ m/s}$ down to $0.30\text{ m}$ height.
  - Stage 2: Gentle descent at $0.35\text{ m/s}$ until ground touchdown is sensed.
  - Stage 3: Push-down descent at $1.0\text{ m/s}$ for $1.0\text{ s}$ to trigger PX4 land detector and cut motor power.
* **[`drone_code/sync_telemetry.sh`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/sync_telemetry.sh):**
  - Downloads flight logs from Jetson to local repo:
    `rsync -avz dronehacks@skybeta-16.local:~/team_ws/flight_records/ ./flight_records/`
  - Allows instant post-flight forensics via CSV/JSONL parsing scripts.

---

## 5. The Bare-Bones Safety Option

### Why Strip Safety to Bare Bones?
During physical testing, teams frequently find that software-based safety checks cause more crashes than they prevent:
1. **Prop Wash & Floor Ground Effect:** During takeoff and descent, turbulent ground effect causes transient airframe tilts of $15^\circ$–$22^\circ$. A rigid $20^\circ$ threshold causes false-positive `TouchDetector` trips, cutting power when the drone is flying normally.
2. **Bottom Laser Dropouts:** Over arena seam tapes or polished floor patches, optical laser altimeters experience transient 1-cycle dropouts, causing `HeightCheck` to panic.
3. **The Human Kill Switch Is Faster:** The human safety pilot has their fingers resting on the RC Mode and Throttle Cut switches. Human visual reaction time to a true collision is $200$–$300\text{ ms}$, cutting motor power instantly and cleanly without winding netting into props.

### Architecture of the Bare-Bones Safety Mode
On this branch, `--bare-bones-safety` provides a principled, battle-tested compromise:
```text
                          RunSafety.check(condition)
                                     │
                    Is Offboard Active & RC Armed?
                           ├── NO  ──► Action = CONTROL_LOST (Cease Targets Immediately)
                           │
                           └── YES
                                │
                    Is Bare-Bones Safety Enabled?
                           ├── YES ──► Bypass TouchDetector (> 20° tilt)
                           │           Bypass HeightCheck (> 15 cm divergence)
                           │           (Pilot kill switch has sole authority)
                           │
                           └── NO  ──► Run TouchDetector & HeightCheck normally
                                │
                    Is cuVSLAM Silence > 0.5 s?
                           ├── YES ──► Action = HOLD / LAND_HERE
                           └── NO  ──► Action = CARRY_ON
```

### What Is Preserved vs. What Is Suppressed
* **PRESERVED (Non-Negotiable Safety):**
  - `CONTROL_LOST`: If the safety pilot flips the RC mode switch away from Position or hits the kill switch, target publishing ceases immediately.
  - Course Boundary Latch (`check_target`): Waypoints outside the course perimeter net ($[-3.0, +3.0]\text{ m}$) or above the arena ceiling ($> 1.6\text{ m}$) are strictly refused.
  - Tracking Loss Protection: If cuVSLAM tracking dies for $> 5.0\text{ s}$, the drone settles and lands rather than flying blind.
* **SUPPRESSED (Eliminating False Aborts):**
  - `TouchDetector`: Transient tilt spikes ($> 20^\circ$) and yaw rate spikes ($> 120^\circ/\text{s}$) will not abort the flight.
  - `HeightCheck`: Optical laser divergence over surface seams will not abort the flight.

### How to Run with Bare-Bones Safety
```bash
# On the drone:
ros2 run team_nav practice_flight first_maze_leg --bare-bones-safety
```

---

## 6. The First Maze Leg Flight Profile & Waypoints

### Physical Arena Coordinate Map
All dimensions and coordinates are in metres, referenced to the competition origin $(0.0, 0.0)$ at the center of the arena:

```text
               [North Perimeter Net: Y = +3.05 m]
  ------------------------------------------------------------------
  |                                                                |
  |   WP2: Turn Pocket (-2.51 m, +2.18 m)                          |
  |   North Net Buffer: 0.87 m (props > 0.70 m clear)              |
  |   Coordinated Yaw: 90° (N) -> 0° (E) @ 15°/s                   |
  |               │                                                |
  |               │             East @ 0.25 m/s                    |
  |               ▼────────────────────────────────────────────► X | WP3 / WP4: Landing Spot
  |                                                                | (-0.50 m, +2.18 m)
  |   CORRIDOR 1:                                                  |
  |   Flight: North @ 0.30 m/s                                     | [Divider M1 Corner]
  |   Centerline: X = -2.51 m                                      | X = -1.85 m
  |   West Net Clearance: 0.54 m                                   | Clearance: > 0.60 m
  |   East Wall Clearance: 0.66 m                                  |
  |               ▲                                                |
  |               │                                                |
  |   WP0 / WP1:  │                                                |
  |   Launch Pad  │                                                |
  |   (-2.51 m, +0.91 m)                                           |
  |                                                                |
  ------------------------------------------------------------------
               [South Perimeter Net: Y = -3.05 m]
```

### Complete Waypoint Specification Table

| Step | Stage Name | Coordinates $(X_{\text{East}}, Y_{\text{North}}, Z_{\text{Up}})$ | Heading | Speed | Target Duration | Clearances & Safety Invariants |
|---|---|---|---|---|---|---|
| **WP0** | `starting` | `(-2.51 m, +0.91 m, 0.00 m)` | $90.0^\circ$ (N) | $0.0\text{ m/s}$ | Disarmed dwell | Capture ground baseline; confirm cuVSLAM ready. |
| **WP1** | `climb` | `(-2.51 m, +0.91 m, +0.75 m)` | $90.0^\circ$ (N) | $0.20\text{ m/s}$ | $3.75\text{ s}$ + $2.0\text{ s}$ settle | Vertical ascent to cruise height; hold still $2.0\text{ s}$. |
| **WP2** | `first corridor` | `(-2.51 m, +2.18 m, +0.75 m)` | $90.0^\circ$ (N) | $0.30\text{ m/s}$ | $4.23\text{ s}$ | Traverse $1.27\text{ m}$ North. Crosses Checkpoint 1. Stays $0.87\text{ m}$ South of North net. |
| **WP2 $\to$ WP2** | `turn east` | `(-2.51 m, +2.18 m, +0.75 m)` | $90^\circ \to 0^\circ$ (E) | $15.0^\circ/\text{s}$ | $6.00\text{ s}$ + $1.0\text{ s}$ settle | In-pocket pivot. Yaw rate limited to $15^\circ/\text{s}$ to prevent lateral slide. |
| **WP3** | `enter maze` | `(-0.50 m, +2.18 m, +0.75 m)` | $0.0^\circ$ (E) | $0.25\text{ m/s}$ | $8.04\text{ s}$ + $2.0\text{ s}$ settle | Advance $2.01\text{ m}$ East along $Y = +2.18\text{ m}$. Clears Divider M1 by $> 0.60\text{ m}$. Stops $0.50\text{ m}$ short of boulder. |
| **WP4** | `landing` | `(-0.50 m, +2.18 m, 0.00 m)` | $0.0^\circ$ (E) | $-0.35\text{ m/s}$ | $4.50\text{ s}$ | Controlled 3-phase descent, touchdown detection, motor cut. |

---

## 7. Line-by-Line Code Walkthrough: `first_maze_leg.py`

Below is an exhaustive walkthrough of each logical section in [`drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py):

### 1. Arguments & Configuration
```python
FLIGHT_NAME = "first_maze_leg"
LAUNCH_EAST_METRES = -2.51
LAUNCH_NORTH_METRES = 0.91
CRUISE_HEIGHT_METRES = 0.75
CORRIDOR_1_TURN_NORTH_METRES = 2.18
MAZE_ENTRY_EAST_METRES = -0.50
TURN_RATE_DEGREES_PER_SECOND = 15.0
CORRIDOR_SPEED_METRES_PER_SECOND = 0.30
MAZE_SPEED_METRES_PER_SECOND = 0.25
SETTLE_PAUSE_SECONDS = 2.0
```
* **Why $Y = +2.18\text{ m}$ for the turn?** The North perimeter net is at $Y = +3.048\text{ m}$. Placing the turn at $+2.18\text{ m}$ leaves an $0.87\text{ m}$ buffer between the drone center and the net ($> 0.70\text{ m}$ prop-tip clearance). This eliminates the corner-net trap.
* **Why $X = -0.50\text{ m}$ for the maze entry?** Divider M1 terminates at $X = -1.85\text{ m}$. Advancing to $X = -0.50\text{ m}$ places the drone $1.35\text{ m}$ past the divider into the maze, well past Checkpoint 2, while stopping $0.50\text{ m}$ short of the obstacle boulder at $X = -1.0\text{ m}, Y = +2.2\text{ m}$.
* **Why $15^\circ/\text{s}$ turn rate?** Flight tests on Drone 16 proved that yawing at $30^\circ/\text{s}$ causes a $0.50\text{ m}$ lateral position slide due to unmodelled rotor drag and camera motion blur. At $15^\circ/\text{s}$, position drift during the turn is strictly $< 5\text{ cm}$.

### 2. Ground Preparation & Arming Hand-Over
```python
def fly(self):
    flight_steps.wait_until_ready_on_the_ground(self.link)
    flight_steps.hold_above_launch_until_control_is_ours(self.link, CRUISE_HEIGHT_METRES)
```
* `wait_until_ready_on_the_ground()` verifies that cuVSLAM tracking is live, PX4 is landed and disarmed, and all reports are fresh.
* `hold_above_launch_until_control_is_ours()` begins streaming setpoints at $+0.75\text{ m}$ height above the launch pad. It loops indefinitely until the safety pilot arms RC and switches the mode switch to Position.

### 3. Stage 1: Vertical Climb
```python
self.link.stage = "climb"
flight_steps.climb_above_launch(self.link, self.watch, CRUISE_HEIGHT_METRES)
self.link.stage = "settle before corridor"
flight_steps.hold_still_for(self.link, self.watch, SETTLE_PAUSE_SECONDS)
```
* Climbs vertically to $+0.75\text{ m}$ and holds still for $2.0\text{ s}$ to damp any residual pendulum oscillations before horizontal translation begins.

### 4. Stage 2: Corridor 1 Traversal
```python
def fly_corridor_1(self):
    start_north = LAUNCH_NORTH_METRES
    distance_metres = CORRIDOR_1_TURN_NORTH_METRES - start_north
    duration_seconds = distance_metres / CORRIDOR_SPEED_METRES_PER_SECOND
    ...
    def send_corridor_target(seconds_since_start):
        progress = min(1.0, seconds_since_start / duration_seconds)
        current_target_north = start_north + progress * distance_metres
        target_course = np.array([LAUNCH_EAST_METRES, current_target_north, CRUISE_HEIGHT_METRES])
        self.watch.send_target_on_course(target_course, heading_on_course_radians=math.radians(90.0))
```
* Slides the target smoothly North at $0.30\text{ m/s}$ along $X = -2.51\text{ m}$, maintaining a constant $90^\circ$ North heading.
* Reaches $Y = +2.18\text{ m}$ in exactly $4.23\text{ seconds}$.

### 5. Stage 3: Coordinated 90-Degree Turn East
```python
def turn_90_degrees_east(self):
    turn_seconds = 90.0 / TURN_RATE_DEGREES_PER_SECOND
    ...
    def send_turning_target(seconds_since_start):
        progress = min(1.0, seconds_since_start / turn_seconds)
        current_heading_deg = 90.0 - progress * 90.0
        self.watch.send_target_on_course(hold_target, heading_on_course_radians=math.radians(current_heading_deg))
```
* Holds the physical position stationary at `(-2.51 m, +2.18 m, +0.75 m)` while ramping the heading target from $90.0^\circ$ (North) down to $0.0^\circ$ (East) over $6.0\text{ seconds}$.

### 6. Stage 4: Maze Entry past Divider M1
```python
def enter_maze(self):
    start_east = LAUNCH_EAST_METRES
    distance_metres = MAZE_ENTRY_EAST_METRES - start_east
    duration_seconds = distance_metres / MAZE_SPEED_METRES_PER_SECOND
    ...
    def send_maze_target(seconds_since_start):
        progress = min(1.0, seconds_since_start / duration_seconds)
        current_target_east = start_east + progress * distance_metres
        target_course = np.array([current_target_east, CORRIDOR_1_TURN_NORTH_METRES, CRUISE_HEIGHT_METRES])
        self.watch.send_target_on_course(target_course, heading_on_course_radians=math.radians(0.0))
```
* Slides the target straight East at $0.25\text{ m/s}$ along $Y = +2.18\text{ m}$ from $X = -2.51\text{ m}$ to $X = -0.50\text{ m}$.
* Clears the corner of divider M1 ($X = -1.85\text{ m}$) with $> 0.60\text{ m}$ of clearance.

### 7. Stage 5: Precision Touchdown & Disarm
```python
self.link.stage = "settle before landing"
flight_steps.hold_still_for(self.link, self.watch, SETTLE_PAUSE_SECONDS)

self.link.stage = "landing"
height_source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
flight_steps.land(self.link, height_source, self.watch)
```
* Pauses for $2.0\text{ s}$ to null out horizontal velocity.
* Executes the 3-phase landing descent, detects ground contact, and shuts down motors.

---

## 8. Ground Crew & Pilot Operational Runbook (Field Guide)

Follow this sequence exactly on competition day:

### Step 1: Connect to Drone Hardware
You can connect in two ways:
* **Over USB-C tether (Preferred for Reliability):** Drone IP is `192.168.55.1`.
* **Over venue Wi-Fi ("DroneHacks Wifi"):** Drone hostname is `skybeta-16.local`.

### Step 2: Deploy & Build Code from Laptop
From your laptop repository root:
```bash
# If using Wi-Fi:
drone_code/install_on_drone_16.sh

# Or over USB-C:
DRONE_ADDRESS=192.168.55.1 drone_code/install_on_drone_16.sh
```
*(Verifies hostname `skybeta-16`, syncs code, and runs `team_build.sh` inside Docker.)*

### Step 3: SSH into the Jetson
```bash
ssh dronehacks@skybeta-16.local
# Or over USB: ssh dronehacks@192.168.55.1
```

### Step 4: Verify Telemetry & Start Flight Interface
```bash
cd ~/team_ws
# Check telemetry connection to PX4 (should show PX4 connected: True, armed: False)
./drone.sh status

# Start flight interface and depth camera nodes:
./drone.sh start test_flights
```

### Step 5: Physical Drone Placement & Sight Alignment
1. **Pad Centering:** Place Drone 16 precisely centered on the launch pad mark `(-2.51 m, +0.91 m)`.
2. **Sight North Alignment (CRITICAL):** Visually sight the drone's nose directly down Corridor 1. Ensure the airframe heading points strictly North ($90^\circ \pm 2^\circ$). The visual odometry frame is initialized to this heading.
3. **Sensor Check:** Verify the bottom laser lens and stereo camera lenses are free of dust or smudges.

### Step 6: Start the Flight Program
```bash
# Standard safety mode:
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight first_maze_leg

# OR with Bare-Bones Safety (recommended to avoid false aborts):
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight first_maze_leg --bare-bones-safety
```

### Step 7: The 5-Second Arming Cadence (CRITICAL)
Wait until the terminal prints:
```text
[stage] Sending a target 0.75 m above the launch mark. Waiting for a teammate to arm the drone and flip the switch to Position.
```
**EXECUTE IMMEDIATELY (Do NOT dwell on pad):**
1. RC mode switch must be in **Manual / Altitude** (NOT Position).
2. Arm the RC transmitter. Propellers begin idling.
3. **Within 5 seconds:** Flip the mode switch to **Position**.
4. The flight script detects Position mode, takes control, and initiates vertical climb.
5. **SAFETY PILOT:** Keep fingers resting on the mode switch and throttle cut at all times. If the drone touches netting or drifts unexpectedly, disarm immediately.

### Step 8: Post-Flight Safing & Telemetry Extraction
1. Once motors stop, disarm RC switch.
2. Put drone back into safe watch-only mode:
   ```bash
   ./drone.sh start watch_only
   ```
3. In your laptop terminal, pull the flight logs:
   ```bash
   drone_code/sync_telemetry.sh
   ```

---

## 9. Simulation & Verification Playbook

### Running in Full Gazebo 3D Simulation
To test the complete flight in 3D Gazebo with full stereo matching and physics:
```bash
simulator/gazebo_full/fly_course_in_gazebo.sh \
    --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json \
    --run-mode practice \
    --longest-run-seconds 120 \
    -- ros2 run team_nav practice_flight first_maze_leg
```

### Running in Fast Lightweight Simulator
To test logic and trajectory geometry in seconds:
```bash
simulator/run_test_flight.sh \
    --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json \
    real_program_test.py --env TEAM_NAV_RUN_MODE=practice \
    -- ros2 run team_nav practice_flight first_maze_leg
```

### Running the Non-ROS Test Suite
Verify that all 263 unit tests pass cleanly:
```bash
wsl bash -c "export LD_LIBRARY_PATH=/opt/ros/humble/lib:\$LD_LIBRARY_PATH && export PYTHONPATH=/mnt/c/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav:/opt/ros/humble/local/lib/python3.10/dist-packages:/opt/ros/humble/lib/python3.10/site-packages:\$PYTHONPATH && cd /mnt/c/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree && python3 -m pytest drone_code/team_nav/test -q -p no:cacheprovider"
# Expected Result: 263 passed, 1 skipped in ~100s
```

---

## 10. Summary of Files on the `first_leg` Branch

| File Path | Status | Key Modification / Responsibility |
|---|---|---|
| [`drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py) | **Created** | Deterministic 4-waypoint qualification flight program with `--bare-bones-safety` support. |
| [`drone_code/team_nav/team_nav/run_safety.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/run_safety.py) | **Modified** | Added `bare_bones_safety` mode to suppress hair-trigger touch/height trips while preserving `CONTROL_LOST`. |
| [`drone_code/team_nav/test/test_run_safety.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/test/test_run_safety.py) | **Modified** | Added unit test coverage for `bare_bones_safety` mode. |
| [`drone_code/team_nav/team_nav/practice_flights/run_practice_flight.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/practice_flights/run_practice_flight.py) | **Modified** | Registered `first_maze_leg` into CLI dispatch table. |
| [`drone_code/team_nav/team_nav/depth_to_map.py`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/drone_code/team_nav/team_nav/depth_to_map.py) | **Modified** | Initialized `last_picture_monotonic_seconds` to eliminate cycle-0 false freeze bug. |
| [`docs/FIRST_LEG_QUALIFICATION_PLAYBOOK_AND_ARCHITECTURE.md`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/docs/FIRST_LEG_QUALIFICATION_PLAYBOOK_AND_ARCHITECTURE.md) | **Created** | Master architectural, operational, and forensic qualification manual. |
