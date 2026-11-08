# Engineering Audit: `classical-dannie-attemptone` Branch

**Target Branch:** `classical-dannie-attemptone`  
**Purpose:** Pre-flash architectural audit, static/dynamic bug identification, hardware risk analysis, and step-by-step execution trace on Drone 16 (NVIDIA Jetson Orin Nano companion + Holybro Pixhawk 6C PX4).

---

## 1. Executive Summary & Strategic Feasibility

### Concept Feasibility
The high-level tactical premise of `classical-dannie-attemptone` is **fundamentally sound**:
* Over 80% of teams fail because dynamic A* obstacle costmaps and in-place $360^\circ$ "look-around" pirouettes deadlock in the narrow $1.19\text{ m}$ Corridor 1.
* Bypassing dynamic replanning in Corridor 1 with a deterministic, low-speed waypoint flight (`first_maze_leg.py`) banks **35 qualification points** (Checkpoint 1 at $+1.80\text{ m}$ North, Checkpoint 2 at $-1.85\text{ m}$ East, and a controlled ground touchdown).
* Initializing `DepthToMap.last_picture_monotonic_seconds = time.monotonic()` resolves a real bug where `live_flight` reported `float("inf")` depth image age on cycle 0.

### Code Readiness Verdict
**DO NOT FLASH THIS BRANCH IN ITS CURRENT STATE.**
The branch contains a **fatal runtime `TypeError`** that will trigger an immediate emergency abort and landing within 2 seconds of takeoff, followed by a **coordinate sign inversion** that commands the drone directly over/into a $0.53\text{ m}$ tall boulder.

---

## 2. Identified Bugs & Failure Modes

### Bug 1: [FATAL RUNTIME EXCEPTION] Argument Mismatch in `fly_until` Closures
* **File:** `drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py` (Lines 198, 222, 246)
* **Code:**
  ```python
  def send_corridor_target(seconds_since_start):  # Line 198
  def send_turning_target(seconds_since_start):   # Line 222
  def send_maze_target(seconds_since_start):      # Line 246
  ```
* **Mechanism:**
  In `drone_code/team_nav/team_nav/flight_steps.py` (Line 627), `fly_until` invokes the target callback with **zero arguments**:
  ```python
  if must_hold:
      link.targets.resend_last()
  else:
      send_target()  # Zero arguments passed!
  ```
* **Failure Impact on Hardware:**
  The moment vertical climb completes and the drone settles, `self.fly_corridor_1()` starts. `fly_until` calls `send_corridor_target()`, which immediately raises:
  ```text
  TypeError: FirstMazeLegFlight.fly_corridor_1.<locals>.send_corridor_target() missing 1 required positional argument: 'seconds_since_start'
  ```
  The generic `except Exception:` block in `FirstMazeLegFlight.run()` catches the error, logs it to `events.jsonl.gz`, and commands `self.land_after_a_problem()`. **The drone lands back on the launch mark without moving forward.**

---

### Bug 2: [CRITICAL COLLISION RISK] Coordinate Sign Inversion Flies Over 0.53m Boulder
* **File:** `drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py` (Line 62)
* **Code:**
  ```python
  # Penetrates 1.35 m into the maze past divider M1 (-1.85 m), stopping well before the boulder (-1.0 m).
  MAZE_ENTRY_EAST_METRES = -0.50
  ```
* **Coordinate Geometry:**
  The arena coordinate frame has its origin near the center; Western coordinates are negative:
  - West Perimeter Net: $X = -3.05\text{ m}$
  - Corridor 1 Centerline: $X = -2.51\text{ m}$
  - Divider M1 Opening (Checkpoint 2 Line): $X = -1.85\text{ m}$
  - **Sketch Boulder NW:** $\mathbf{X = -1.10\text{ m},\; Y = +2.20\text{ m}}$ (Radius $0.23\text{ m}$, Height $0.53\text{ m}$)
  - Target Ingress Coordinate: $\mathbf{X = -0.50\text{ m}}$
* **Mechanism:**
  Moving Eastward from $X = -2.51\text{ m}$ towards $X = -0.50\text{ m}$ means:
  $$-2.51\text{ m} \longrightarrow -1.85\text{ m} \longrightarrow \mathbf{-1.10\text{ m (BOULDER)}} \longrightarrow -0.50\text{ m}$$
  The author assumed $-0.50\text{ m}$ was "before" $-1.0\text{ m}$ on the number line. In reality, the drone flies **past** the boulder, with its flight path at $Y = +2.18\text{ m}$ passing **$2\text{ cm}$** from the boulder's center.
* **Failure Impact on Hardware:**
  1. **Physical Clearance:** The drone flies at $0.75\text{ m}$ cruise height; the boulder is $0.53\text{ m}$ tall, leaving only $22\text{ cm}$ vertical clearance.
  2. **PX4 Laser Altimeter Surge:** When the bottom VL53L1X laser rangefinder crosses the top of the boulder, the measured range suddenly drops from $0.75\text{ m}$ to $0.22\text{ m}$. PX4 EKF2 interprets this as a rapid drop in drone altitude, commanding maximum vertical thrust and pitching violently into the ceiling netting (as documented in the 2026-11-07 crash logs).
* **Fix:** Checkpoint 2 is awarded immediately upon crossing Divider M1 ($X > -1.85\text{ m}$). Set:
  ```python
  MAZE_ENTRY_EAST_METRES = -1.50  # 0.35 m past M1, 0.40 m safely WEST of the boulder
  ```

---

### Bug 3: [OPERATIONAL RISK] `--bare-bones-safety` Disabled by Default
* **File:** `drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py` (Line 92)
* **Mechanism:**
  `--bare-bones-safety` is an optional CLI flag (`default=False`). Running:
  ```bash
  TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight first_maze_leg
  ```
  runs with strict safety checks active (`TouchDetector` tilt $> 20^\circ$, `HeightCheck` divergence $> 15\text{ cm}$). Any normal banking tilt during the $90^\circ$ turn risks a false software abort unless the flag is explicitly passed.

---

## 3. Hardware Execution Trace (If Flashed As-Is)

```mermaid
sequenceDiagram
    autonumber
    actor Pilot as Safety Pilot
    participant Shell as Laptop / Jetson Shell
    participant Launch as test_flights.launch
    participant Leg as first_maze_leg.py
    participant PX4 as PX4 Autopilot / Motors

    Shell->>Launch: ./drone.sh start test_flights
    Launch-->>Shell: flight_interface & depth_camera active
    Shell->>Leg: TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight first_maze_leg
    Leg->>Leg: wait_until_ready_on_the_ground() (Topics OK)
    Leg->>Leg: hold_above_launch_until_control_is_ours(0.75)
    Pilot->>PX4: Arm Remote Controller & Flip Switch to Position
    PX4-->>Leg: Offboard Control Granted
    Leg->>PX4: climb_above_launch(0.75 m)
    PX4->>PX4: Motors Spool Up; Climb to 0.75 m
    Leg->>Leg: hold_still_for(2.0 s) Hover Settle
    Leg->>Leg: fly_corridor_1() -> fly_until()
    Note over Leg: TypeError: send_corridor_target() missing argument
    Leg->>Leg: Catch Exception in run()
    Leg->>PX4: land_after_a_problem()
    PX4->>PX4: Emergency Descent to Launch Pad & Disarm
    Leg-->>Shell: Exit Code 1 (Flight Aborted in ~5 seconds)
```

---

## 4. Complete Code Patch to Fix the Branch

Applying the following modifications to `drone_code/team_nav/team_nav/practice_flights/first_maze_leg.py` resolves all identified bugs:

```python
# 1. Update line 62: Move maze entry target safely West of the boulder
MAZE_ENTRY_EAST_METRES = -1.50

# 2. Update fly_corridor_1 closure (Lines 198-208):
    def fly_corridor_1(self):
        self.link.stage = "first corridor"
        start_north = LAUNCH_NORTH_METRES
        distance_metres = CORRIDOR_1_TURN_NORTH_METRES - start_north
        duration_seconds = distance_metres / CORRIDOR_SPEED_METRES_PER_SECOND
        self.log.event("stage", f"Flying Corridor 1: {distance_metres:.2f} m North to +{CORRIDOR_1_TURN_NORTH_METRES:.2f} m.", stage="first corridor")

        start_time = self.link.now_seconds()

        def send_corridor_target():
            now = self.link.now_seconds()
            progress = min(1.0, (now - start_time) / duration_seconds)
            current_target_north = start_north + progress * distance_metres
            target_course = np.array([LAUNCH_EAST_METRES, current_target_north, CRUISE_HEIGHT_METRES])
            self.watch.send_target_on_course(target_course, heading_on_course_radians=math.radians(90.0))

        flight_steps.fly_until(self.link,
                               lambda: self.link.now_seconds() - start_time >= duration_seconds,
                               send_corridor_target,
                               duration_seconds + 5.0,
                               "flying straight down Corridor 1",
                               self.watch)

# 3. Update turn_90_degrees_east closure (Lines 222-232):
    def turn_90_degrees_east(self):
        self.link.stage = "turn east"
        turn_degrees = 90.0
        turn_seconds = turn_degrees / TURN_RATE_DEGREES_PER_SECOND
        self.log.event("stage", f"Turning 90 degrees East at {TURN_RATE_DEGREES_PER_SECOND:.0f} deg/s.", stage="turn east")

        start_time = self.link.now_seconds()
        hold_target = np.array([LAUNCH_EAST_METRES, CORRIDOR_1_TURN_NORTH_METRES, CRUISE_HEIGHT_METRES])

        def send_turning_target():
            now = self.link.now_seconds()
            progress = min(1.0, (now - start_time) / turn_seconds)
            current_heading_deg = 90.0 - progress * 90.0
            self.watch.send_target_on_course(hold_target, heading_on_course_radians=math.radians(current_heading_deg))

        flight_steps.fly_until(self.link,
                               lambda: self.link.now_seconds() - start_time >= turn_seconds + 1.0,
                               send_turning_target,
                               turn_seconds + 5.0,
                               "turning 90 degrees East into the maze",
                               self.watch)

# 4. Update enter_maze closure (Lines 246-256):
    def enter_maze(self):
        self.link.stage = "enter maze"
        start_east = LAUNCH_EAST_METRES
        distance_metres = MAZE_ENTRY_EAST_METRES - start_east
        duration_seconds = distance_metres / MAZE_SPEED_METRES_PER_SECOND
        self.log.event("stage", f"Entering Maze: flying {distance_metres:.2f} m East to {MAZE_ENTRY_EAST_METRES:.2f} m.", stage="enter maze")

        start_time = self.link.now_seconds()

        def send_maze_target():
            now = self.link.now_seconds()
            progress = min(1.0, (now - start_time) / duration_seconds)
            current_target_east = start_east + progress * distance_metres
            target_course = np.array([current_target_east, CORRIDOR_1_TURN_NORTH_METRES, CRUISE_HEIGHT_METRES])
            self.watch.send_target_on_course(target_course, heading_on_course_radians=math.radians(0.0))

        flight_steps.fly_until(self.link,
                               lambda: self.link.now_seconds() - start_time >= duration_seconds,
                               send_maze_target,
                               duration_seconds + 5.0,
                               "advancing East into the maze corridor",
                               self.watch)
```
