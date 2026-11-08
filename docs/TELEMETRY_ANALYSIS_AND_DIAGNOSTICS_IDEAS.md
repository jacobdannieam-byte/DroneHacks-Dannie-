# Telemetry Analysis & Diagnostics Ideas for Autonomous Flight

> [!NOTE]  
> **Author Attribution:** Generated entirely by **Gemini** (AI Assistant, Gemini 4 Argon persona). This is **not authored by a human team member**.  
> **Purpose:** Practical engineering methodologies for extracting actionable flight dynamics, sensor health, and collision-avoidance metrics from Drone 16's telemetry archives.

---

## 1. Depth Camera Sensor Confidence Diagnostics

To determine whether the OAK-D depth camera is reliable enough for reactive corridor centering and tunnel detection, we should evaluate the newly recorded `depth_camera` telemetry sessions (e.g., `20261108_062406_depth_camera`, `20261108_062508_depth_camera`).

### Diagnostic Metrics to Calculate:
1. **Valid Return Ratio:**
   - **Formula:** $\text{Valid Return \%} = \frac{\text{Number of Non-Zero Depth Pixels}}{\text{Total Sensor Pixels}} \times 100\%$
   - **Significance:** Black nylon safety nets and smooth glossy barriers often absorb or scatter IR stereo illumination. If the return ratio drops below $30\%$ when pointing toward nets, the camera cannot be trusted to see boundary walls purely from depth.
2. **Temporal Range Stability on Static Walls:**
   - For a drone sitting on the launch pad or holding still, track the range to a fixed wall over a 5-second window ($50$ frames).
   - If the standard deviation $\sigma > 0.05\text{ metres}$ ($5\text{ cm}$), apply temporal exponential moving average filtering before using point clouds for lateral steering.
3. **Effective Distance Cutoff:**
   - Stereo depth uncertainty grows quadratically with distance ($\Delta Z \propto Z^2$).
   - Identify the threshold where depth noise exceeds $10\text{ cm}$. On a $7.5\text{ cm}$ baseline camera, points beyond **$2.2\text{ metres}$** should be discarded from obstacle avoidance calculations.

---

## 2. In-Flight Turn Slide & Position Lag Diagnostic

During flight `20261108_050406`, Drone 16 slid $0.5\text{ metres}$ sideways during a 90° yaw turn. We can formally quantify this dynamics flaw across all flight records.

### The Turn Slide Metric:
1. Extract time intervals where `activity == "turning to face the path"`.
2. Compute the Euclidean distance between the instantaneous estimated position and the designated hold anchor:
   $$\text{Slide Error}(t) = \sqrt{(x_{\text{drone}}(t) - x_{\text{hold}})^2 + (y_{\text{drone}}(t) - y_{\text{hold}})^2}$$
3. **Correlation Analysis:** Plot peak $\text{Slide Error}$ versus yaw rate ($\omega_{\text{yaw}}$).
   - This validates whether reducing turn rate to $15^\circ/\text{s}$ successfully bounds the slide to $< 0.10\text{ metres}$.

---

## 3. Real-Time Corridor Clearance & Proximity Logging

Currently, `drone_condition.csv` logs broad state variables (tilt, speed, PX4 mode). Adding bilateral clearance metrics will give immediate visibility into near-misses.

### Proposed Telemetry Fields:
- `clearance_left_metres`: Minimum Euclidean distance to any point in the left $60^\circ$ forward sector.
- `clearance_right_metres`: Minimum Euclidean distance to any point in the right $60^\circ$ forward sector.
- `clearance_front_metres`: Minimum distance to obstacles directly on the current path line.
- `distance_to_tunnel_metres`: Estimated range to the detected tunnel entrance.

**Safety Benefit:** If `clearance_left_metres` or `clearance_right_metres` trends below $0.25\text{ metres}$, the flight log flags a yellow proximity warning before a collision supervisor threshold is tripped.

---

## 4. Vertical Tracking & Laser Consistency Check

Our post-flight investigation revealed that visual odometry altitude drifted by $8 - 10\text{ cm}$, leading to the deactivation of `height_keeper.py` in commit `b93c478`.

### Continuous Validation Check:
$$\Delta z(t) = |z_{\text{camera}}(t) - z_{\text{downward\_laser}}(t)|$$
- **Health Criterion:** Over flat arena flooring, $\Delta z(t)$ must remain steady.
- **Boulder Detection Metric:** When passing over a low boulder obstacle ($0.20\text{ m}$ height), $z_{\text{downward\_laser}}$ drops by $0.20\text{ m}$ while $z_{\text{camera}}$ remains flat. This difference can be used to positively confirm that the drone successfully negotiated an obstacle without confusing it for drift.

---

## 5. Instantaneous Motor Disarm on Net Touch

In flight `20261108_021927`, the touch detector aborted at $t = 57.295\text{ s}$ when yaw rate hit $246.8^\circ/\text{s}$, but requested a gentle 5-second descent. During those 5 seconds, the still-spinning motors tangled in the netting, causing severe propeller damage.

### Recommended Telemetry Alarm Trigger:
- Add an angular acceleration check:
  $$\alpha_{\text{yaw}} = \left|\frac{\Delta \omega_{\text{yaw}}}{\Delta t}\right| > 1500^\circ/\text{s}^2$$
- Net strikes create an immediate, sharp jerk ($\alpha > 2000^\circ/\text{s}^2$) long before tilt reaches $20^\circ$.
- Triggering an **immediate emergency motor disarm** upon detecting this jerk cuts motor power before propellers can spool net strands around the motor bells.
