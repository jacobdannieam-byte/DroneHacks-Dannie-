# maybe_important_changes

## Comprehensive Engineering Audit & Potential Problems: `codex/hybrid-model-flight`

**Date:** 2026-11-08  
**Scope:** Review of frozen full-course PINN flight (`cfdffc79b0cb79dab6abd81d130f1e0f78715067`), runtime architecture, timing fences, safety bounds, and hardware readiness.  
**Repository Test Status:** All 879 unit and regression tests pass (`pytest tests/ -q`).

---

### Executive Summary

While the full-course Gazebo diagnostic reached 22 post-launch waypoints, cleared all four referee checkpoints, and landed inside the pad with zero contacts, **this flight passed exclusively within a synthetic diagnostic sandbox (`scenarios/model_flight/full_course_diagnostic.json`)**. 

To achieve a passing run after 11 consecutive failed attempts (attempts 001–011), nearly all critical safety gates—including map collision reservations, depth perception requirements, flight boundary boxes, speed limits, inference budgets, and clock deferrals—were either relaxed or completely bypassed. Running this branch under the production safety configuration (`scenarios/model_flight/full_course.json`) or on physical hardware will fail almost immediately.

---

### 1. The Core Model: What Controlled the Flight vs. D0 & Classical

* **Route Geometry is 100% Classical Prior:**
  The PINN does not plan paths, explore unknown space, recognize tunnels, or avoid obstacles. All 22 post-launch waypoints and the 0.8 m flight ceiling originate from the Classical route planner and the phone scan map (`phone_scan_2026-11-06.json`).
* **99.32% Identity to Nominal D0 Dynamics:**
  Out of 1,617 published commands:
  * **1,606 decisions were identical** to nominal D0 physics rollouts without the learned residual.
  * **Only 11 decisions (0.68%) differed from D0.**
  * In all 11 cases, the PINN merely chose candidate `0` (hold horizontal position) instead of candidate `1` (advance 0.08 m) when the drone was already within 0.49 cm to 3.62 cm of the waypoint.
  * The learned network functions essentially as a subtle braking/settling filter at the tail end of waypoint approaches, rather than an active flight controller.
* **Trained on Synthetic Dynamics Only:**
  The 5,572-parameter MLP (`physics_11.npz`) was trained on synthetic simulation dynamics. Real physical telemetry exported from hardware currently has only two overlapping candidate windows and has not been used to train the network.
* **82.2% Slower Course Time (153.57 s vs. 84.30 s):**
  Hybrid is 69.27 s slower than Classical. Why?
  1. The controller forces a full stop and settle ($d \le 0.06\text{ m}$, $v \le 0.05\text{ m/s}$ held for 0.6 s) at **every single one of the 22 waypoints**. In contrast, Classical continuously flies through straight-line waypoints with a 0.65 m forward lead at 0.5 m/s.
  2. The maximum forward command lead in Hybrid is capped at 0.20 m (vs. 0.65 m).

---

### 2. Critical Safety Fences Bypassed in Diagnostic Mode

In the campaign logs (`campaign-attempts.json`), attempts 001 through 011 failed due to clock deferrals, observation supersession, and safety trips. Attempt 012 succeeded because `_diagnostic_motion_unbounded` and `simulator_diagnostic: true` disabled the following safety gates:

1. **Map Clearance Disabled (`refnav/full_course_model.py`):**
   ```python
   # Line 78-79 in refnav/full_course_model.py
   if self.config.get('simulator_diagnostic'):
       allowed = True
   ```
   * **Problem:** In the successful flight, **10 published predictions had negative map margins** (penetrating up to -1.9 mm into the 0.35 m obstacle reservation buffer).
   * In non-diagnostic mode, these candidates are marked ineligible. If no candidate is eligible, the selector raises `ValueError('No model candidate admissible')` and aborts the mission.
2. **Depth Perception Completely Bypassed (`refnav/full_course_model.py`):**
   ```python
   # Line 106-107 in refnav/full_course_model.py
   if self._diagnostic_motion_unbounded:
       self.selector.require_depth = False
   ```
   * **Problem:** Depth was recorded passively but excluded from command authority.
   * If depth requirements are enabled (`scenarios/model_flight/full_course.json`), `refnav/section_policy.py` requires 3 consecutive independent frames verifying that the forward swept volume is clear and passage side-walls (and tunnel roof/floor) are confirmed. Any dropout or unconfirmed surface triggers `section_depth_refused` and forces an autonomous land within 3 seconds.
3. **Flight Envelope Limits Bypassed (`refnav/calibration_trial.py`):**
   * Lane boundary aborts bypassed: `if not in_lane and not diagnostic:`
   * Speed limit aborts bypassed: `if speed > self.plan.actual_speed_limit_mps and not diagnostic:`
   * Hover height / band departure bypassed: `elif height_error < -self.plan.hover_tolerance_m and not diagnostic:`
4. **Progress Watchdog Bypassed (`refnav/full_course_model.py`):**
   * The 4-second no-progress abort ($< 0.02\text{ m}$ forward progress) in `transit()` is completely bypassed when `_diagnostic_motion_unbounded` is set.
5. **Turn Alignment Bypassed (`refnav/full_course_model.py`):**
   * `admit_goal()` immediately returns `None` in diagnostic mode, bypassing yaw alignment before advancing toward goals.

---

### 3. Real-Time Timing, Scheduling & Concurrency Bottlenecks

1. **205 Discarded Decisions (11.2% Drop Rate):**
   * 205 out of 1,822 computed decisions expired before publication because input observation age exceeded 100 ms (`model_state_expired`).
   * When discarded, the control loop invokes `continue`, publishing **no setpoint** on that tick.
2. **1,134 Publications (70.1%) Exceeded the 50 ms Production Budget:**
   * Median inference was 36.5 ms, p95 was 48.1 ms, and maximum was **106.3 ms**.
   * In production mode, elapsed wall time $\ge 50\text{ ms}$ closes the publication fence with `computation_deadline`. Diagnostic mode relaxed this fence to **300 ms**.
3. **Severe Scheduler Delays (up to 581.9 ms):**
   * 5 wall-clock delays exceeded 300 ms (largest: **581.9 ms**).
   * PX4 Offboard mode requires continuous setpoints. If setpoints cease for $> 500\text{ ms}$ (`COM_OF_LOSS_T`), PX4 triggers failsafe mode (failsafe land / RTL). This flight was operating right on the brink of failsafe loss of control.
4. **Per-Tick ROS Graph Query Overhead (`examples/calibration_trial_owner.py`):**
   * Line 389 calls `node.get_publishers_info_by_topic()` on every tick (20 Hz).
   * Synchronous DDS discovery graph queries inside the real-time flight loop cause CPU spikes, context switching, and timing jitter.
5. **Clock Deferral Limit Raised to 300 ms:**
   * In `ConcurrentNativeInputs`, `defer_limit_ns` was increased from 50 ms to 300 ms and coalescing was enabled to prevent `overlapping_px4_clock_deferral` faults that terminated attempt 011.

---

### 4. Disqualifications for Real Physical Hardware

As established in `AGENTS.md` and `AI_CONTEXT.md`, a passing simulator diagnostic is **not hardware qualification**:

1. **Jetson Orin Compute Limits:** Pure Python MLP rollouts taking up to 106 ms on x86 will run significantly slower on the Jetson Orin CPU cores without TensorRT / C++ bindings. This will cause persistent `model_state_expired` discards and offboard command starvation.
2. **Estimator Noise vs. Ground Truth:** Gazebo used clean simulated odometry. Real physical flights with OAK-D cuVSLAM suffer from vibration, lighting shifts, motion blur, and covariance jumps. The strict `section_policy.py` checks would fail repeatedly under real sensor conditions.
3. **Airframe Oscillation:** Stopping and settling at all 22 waypoints causes heavy cyclic pitching and rolling, inducing camera motion blur, degrading VIO tracking, and rapidly draining battery capacity.

---

### 5. Actionable Recommendations for Next Phase

1. **Vectorize Rollouts:** Batch the 4 candidate rollouts into a single vectorized NumPy operation to cut rollout time from ~40–100 ms to $< 10\text{ ms}$.
2. **Cache ROS Graph Queries:** Remove per-tick `get_publishers_info_by_topic()` calls from the inner control loop.
3. **Restore Continuous Route Following:** Allow the drone to pass through straight-line intermediate waypoints with lead distance (e.g. 0.5 m) rather than coming to a dead stop and settling for 0.6 s at every single point.
4. **Pad Obstacle Clearance Buffers:** Apply a 2–3 cm repulsive margin offset to the candidate targets so that predicted trajectories do not penetrate the negative margin boundary.
5. **Train on Real Telemetry:** Retrain the residual model on real flight logs before attempting physical qualification.
