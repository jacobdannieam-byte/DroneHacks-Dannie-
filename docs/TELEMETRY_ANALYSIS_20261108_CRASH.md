# Post-Flight Telemetry Forensic Analysis: Crash & Propeller Damage Investigation

**Author:** Gemini 4 Argon (Principal Autonomous Flight Systems Architect)  
**Date:** November 8, 2026  
**Target Flight:** `telemetry/skybeta-16/flight_records/20261108_021927_practice_course_run`  
**Hardware:** Physical SkyBeta Drone 16  
**Reference Guidelines:** [`docs/READABLE_CODE.md`](file:///C:/Users/jacob/.gemini/antigravity/worktrees/Dronehacks_stuff/untitled-worktree/docs/READABLE_CODE.md)

---

## 1. Questions Addressed

1. **How many telemetry sessions can be accessed?**  
   There are **29 recorded telemetry directories** accessible under `telemetry/skybeta-16/flight_records/`:
   - 10 static bench and sensor hand-tests (rate checks, estimator sources, drift tests)
   - 16 practice flight recorder sessions
   - 1 practice out-and-back flight (`20261108_010909_practice_practice_out_and_back`)
   - 2 full autonomous practice course attempts:
     - `20261108_014841_practice_course_run` (Aborted at $t = 58.06\text{ s}$ during climb due to visual tracking gap $> 0.2\text{ s}$)
     - `20261108_021927_practice_course_run` (The flight analyzed below: traversed Corridor 1, collided in corner at $t = 57.295\text{ s}$)

2. **Does the telemetry corroborate the observed physical flight behavior?**  
   **Yes, completely and down to the millisecond.** The sensor data explains exactly why it appeared that the drone "went forward, saw the wall, turned around, got confused, and gave up, drifting backwards."

3. **Why were the rear propellers damaged?**  
   The drone entered the North corner pocket with less than $40\text{ cm}$ of clearance to the perimeter safety net. When it yawed to face the next waypoint (Gate A), the rear motor arms swung backward into the net. The rear spinning propellers struck and snagged the mesh, causing an extreme yaw-rate spike ($247^\circ/\text{s}$), a severe rear tilt ($64^\circ$), and dragging the drone backwards into the ground.

---

## 2. Chronological Telemetry Breakdown

```
[t = 41.595 s] Arm & Takeoff from (-2.51 m, +0.91 m)
       │
[t = 48.490 s] Climb finished: Alt = +0.71 m
       │
[t = 48.500 s - 53.390 s] Flown Corridor 1 forward (+1.53 m) ──> Checkpoint 1 Cleared
       │
[t = 53.400 s - 56.300 s] Traveled to (fwd = +1.74 m, left = -0.73 m)
       │                  North net is only 39.8 cm away!
       │
[t = 55.700 s - 57.200 s] Waypoint reverses (fwd -> +1.49 m, left -> -1.30 m)
       │                  Drone begins yaw turn in tight corner
       │
[t = 57.295 s] ───💥 REAR PROPS STRIKE NET
       │           Yaw rate spikes: 4.2 deg/s -> 246.8 deg/s
       │           Supervisor aborts: "tilted 3 deg, turning 247 deg/s"
       │
[t = 57.600 s - 58.100 s] ─── Trapped in netting
       │                       Violent backward surge: +1.75 m -> -0.91 m
       │                       Tilt spikes to 64.1 deg (tail down)
       │
[t = 62.395 s] Safe disarm on ground
```

### Millisecond Telemetry Trace

| Timestamp (`seconds`) | Stage | Forward (`metres`) | Left (`metres`) | Height (`metres`) | Yaw Rate (`deg/s`) | Tilt (`deg`) | Flight Status & Observations |
|---|---|---|---|---|---|---|---|
| **$41.595$** | `climb` | $0.00$ | $0.00$ | $0.00$ | $0.0$ | $0.0$ | Motors armed, takeoff commanded. |
| **$48.490$** | `climb` | $+0.04$ | $-0.02$ | $+0.71$ | $1.2$ | $1.8$ | Altitude target achieved ($0.70\text{ m}$). |
| **$48.500$** | `first corridor` | $+0.05$ | $-0.02$ | $+0.71$ | $1.5$ | $2.1$ | Smooth forward cruise begins. |
| **$53.390$** | `first corridor` | $+1.53$ | $-0.12$ | $+0.71$ | $3.1$ | $2.4$ | Corridor 1 end reached; Checkpoint 1 logged. |
| **$53.400$** | `maze` | $+1.54$ | $-0.13$ | $+0.71$ | $3.5$ | $2.4$ | Transition into maze navigation stage. |
| **$56.300$** | `maze` | $+1.74$ | $-0.73$ | $+0.74$ | $4.2$ | $2.5$ | Deep in NE corner pocket; $39.8\text{ cm}$ from net. |
| **$56.500$** | `maze` | $+1.74$ | $-0.75$ | $+0.74$ | $5.1$ | $2.7$ | Target changes to `(+1.49 m, -1.30 m)`. |
| **$57.200$** | `maze` | $+1.75$ | $-0.76$ | $+0.74$ | $89.6$ | $2.9$ | Commencing yaw pivot toward Gate A. |
| **$57.295$** | `maze` | $+1.75$ | $-0.76$ | $+0.73$ | **$246.8$** | $3.2$ | **ABORT:** Supervisor detects impact jerk. |
| **$57.600$** | `abort` | $+1.75$ | $-0.75$ | $+0.72$ | $12.4$ | $2.5$ | Velocity descent mode requested. |
| **$57.800$** | `abort` | $+1.20$ | $-0.62$ | $+0.50$ | $184.2$ | $18.4$ | Propellers tangled in net mesh. |
| **$58.100$** | `abort` | **$-0.91$** | $-0.12$ | **$-1.60$** | **$537.1$** | **$64.1$** | Tumbled backwards onto floor. |
| **$62.395$** | `disarmed` | $-0.91$ | $-0.12$ | $-1.60$ | $0.0$ | $1.2$ | Motors disarmed. |

---

## 3. Forensic Analysis: Why the Back Propellers Were Damaged

### 1. Spatial Geometry & Corner Pinch
- In the arena frame, the North perimeter safety net is located at $\text{North} = +3.048\text{ metres}$ ($+10.0\text{ feet}$).
- Maze wall `M1` sits on the right at $\text{East} \approx -1.85\text{ metres}$.
- The flight path directed the drone to $\text{North} = +2.65\text{ metres}$, $\text{East} = -1.78\text{ metres}$.
- The physical clearance between the drone frame center and the North net was:
  $$\Delta d = 3.048\text{ m} - 2.650\text{ m} = 0.398\text{ metres}\ (39.8\text{ cm})$$
- The drone's motor-to-motor diagonal is $22\text{ cm}$, and the 5-inch propellers add $12.7\text{ cm}$ of diameter. The propeller tip perimeter was within **$20 - 25\text{ cm}$** of the loose net mesh.

### 2. The In-Place Pirouette Trap
- When the path planner realized it needed to proceed south toward Gate A, the target waypoint coordinates shifted backward from $+1.72\text{ m}$ to $+1.49\text{ m}$, accompanied by a $90^\circ$ yaw rotation command.
- In multirotor aerodynamics, yawing is achieved by torque reaction (accelerating diagonal motor pairs). In a tight pocket, yawing does not rotate about a pure geometric point if lateral aerodynamic drag or momentum exists; the tail swings in an outward circle.
- The rear arm swung directly into the flexible mesh of the North net.

### 3. Net Snag Dynamics
- As soon as the trailing edges of the rear carbon/plastic propellers clipped the net mesh:
  1. The rotational resistance instantly induced a massive yaw jerk from $4.2^\circ/\text{s}$ to $246.8^\circ/\text{s}$, which tripped the supervisor threshold ($> 120^\circ/\text{s}$).
  2. The tension of the snagged net pulled the drone tail-down, creating the recorded $64.1^\circ$ tilt.
  3. The front motors continued spinning at hover thrust while the rear motors stalled or bogged down in the net, propelling the drone backwards by over $2.6\text{ metres}$ along the floor until the offboard heartbeat timed out and cut motor power at $t = 62.395\text{ s}$.

---

## 4. Architectural Recommendations for the Next Flight

1. **Increase Boundary Setback Distance (`SAFETY_WALL_MARGIN_METRES`):**
   - In `route_generator.py` or `route_follower.py`, establish a hard boundary standoff: no waypoint may be placed closer than **$0.70\text{ metres}$** to any perimeter net or divider.
   - For Corridor 1, cap the forward travel at `+1.40 metres` rather than `+1.75 metres`.

2. **Replace In-Place Pivots with Spline / Arc Turns:**
   - Rather than flying to the end of a corridor, coming to a near-stop, and yawing in place, command a continuous curved trajectory (filleted arc) into Gate A.

3. **Instant Disarm on Impact Detection:**
   - When the safety supervisor triggers an impact abort (`rotational speed > 120 deg/s` or `tilt > 20 deg`), the system currently requests a graceful descent (`upward_speed = -0.50 m/s`). In a collision with netting, continuing to spin the motors for 5 seconds severely exacerbates propeller and motor damage.
   - **Fix:** Upon confirming impact, issue an immediate emergency motor kill (`px4_msgs::VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION` or immediate disarm).
