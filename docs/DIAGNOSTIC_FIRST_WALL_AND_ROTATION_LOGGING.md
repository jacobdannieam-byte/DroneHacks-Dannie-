# Diagnostic Blueprint: First Wall Detection, Wall-Passed State & 90° Turn Logging

> [!NOTE]  
> **Author Attribution:** Generated entirely by **Gemini** (AI Assistant, Gemini 4 Argon persona). This is **not authored by a human team member**.  
> **Status:** Local diagnostic specification only. **Strictly NOT pushed to Git.**  
> **Tagging Standard:** Every diagnostic message in terminal/SSH and `events.jsonl` is explicitly prefixed with `[GEMINI DEBUG]`.

---

## 1. Overview & Purpose

This diagnostic blueprint instruments Drone 16 with high-visibility real-time logging across SSH and `events.jsonl` to pinpoint why corner transitions fail. Specifically, it logs:
1. **First Wall Detection (`[GEMINI DEBUG - FIRST_WALL]`):** Distance to the front boundary as the drone advances down Corridor 1.
2. **Wall Cleared / Passed Event (`[GEMINI DEBUG - WALL_PASSED]`):** Explicit notification when the algorithm considers the first wall / corridor checkpoint passed, including the exact signed distance past the wall line.
3. **Raw Rangefinder LiDAR Reporting (`[GEMINI DEBUG - LIDAR]`):** Instantaneous raw bottom laser distance (`/fmu/out/distance_sensor` current distance) and PX4's `dist_bottom` fused reading.
4. **90-Degree Turn Execution & Slide Tracker (`[GEMINI DEBUG - TURN_90]`):** Continuous monitoring of yaw rate, heading angle error, and Euclidean lateral slide ($\sqrt{\Delta x^2 + \Delta y^2}$) from the hold anchor.
5. **Post-Turn Settlement (`[GEMINI DEBUG - SETTLING]`):** Real-time measurement of drift recovery before forward flight resumes.

---

## 2. Real-Time SSH Console & Event Output Example

When executed over SSH, the flight script streams structured lines formatted as follows:

```text
[02:19:50] [GEMINI DEBUG - LIDAR] Raw bottom LiDAR: 0.812 m | PX4 dist_bottom: 0.846 m | Signal quality: 100% | Baro fallback: False.
[02:19:52] [GEMINI DEBUG - FIRST_WALL] Front wall sighted at 1.14 m. Drone at (East = -2.51 m, North = +1.80 m).
[02:19:53] [GEMINI DEBUG - FIRST_WALL] Approaching corridor end. Front clearance = 0.52 m. Raw LiDAR = 0.809 m.
[02:19:54] [GEMINI DEBUG - WALL_PASSED] Passed 'end of first corridor' checkpoint! Distance past wall line: +0.26 m (North = +2.45 m, Line North = +2.19 m). Raw LiDAR = 0.805 m.
[02:19:54] [GEMINI DEBUG - TURN_90] Start turn: Current heading = 90.0°, Target heading = 0.0°. Hold anchor point: (-2.51 m, +2.45 m).
[02:19:55] [GEMINI DEBUG - TURN_90] Progress: Heading = 75.1° (Remaining: 14.9°) | Yaw rate = 14.9°/s | Slide from anchor = 0.04 m | Raw LiDAR = 0.808 m.
[02:19:56] [GEMINI DEBUG - TURN_90] Progress: Heading = 60.0° (Remaining: 30.0°) | Yaw rate = 15.0°/s | Slide from anchor = 0.11 m | Raw LiDAR = 0.810 m.
[02:19:57] [GEMINI DEBUG - TURN_90] Progress: Heading = 45.2° (Remaining: 44.8°) | Yaw rate = 15.2°/s | Slide from anchor = 0.22 m | Raw LiDAR = 0.807 m.
[02:19:58] [GEMINI DEBUG - TURN_90] Progress: Heading = 30.0° (Remaining: 60.0°) | Yaw rate = 14.8°/s | Slide from anchor = 0.35 m | Raw LiDAR = 0.811 m.
[02:19:59] [GEMINI DEBUG - TURN_90] Progress: Heading = 15.1° (Remaining: 74.9°) | Yaw rate = 15.1°/s | Slide from anchor = 0.44 m | Raw LiDAR = 0.814 m.
[02:20:00] [GEMINI DEBUG - TURN_90] Turn completed: Heading = 0.2° (Target = 0.0°). Elapsed: 6.1 s. Total slide during turn = 0.47 m.
[02:20:00] [GEMINI DEBUG - SETTLING] Holding position to recover slide. Current drift = 0.47 m (Settled threshold <= 0.10 m). Raw LiDAR = 0.810 m.
[02:20:02] [GEMINI DEBUG - SETTLING] Settled: Drone returned to within 0.09 m of target. Ready for forward drive into Maze.
```

---

## 3. Diagnostic Code Injections

These zero-risk logging hooks are designed for insertion into `drone_code/team_nav/team_nav/live_navigation.py` and `live_flight.py`.

### 3.1 Raw Rangefinder LiDAR Helper
Add to `live_flight.py` or `live_navigation.py` to extract both raw and fused LiDAR telemetry:

```python
def _format_raw_lidar_telemetry(self):
    """Extracts the raw laser rangefinder and PX4's fused bottom distance in metres."""
    raw_distance = None
    if "laser" in self.watch.latest and self.watch.latest["laser"] is not None:
        raw_distance = float(self.watch.latest["laser"].current_distance)
    
    px4_dist_bottom = None
    if "px4_position" in self.watch.latest and self.watch.latest["px4_position"] is not None:
        px4_dist_bottom = float(self.watch.latest["px4_position"].dist_bottom)
        
    raw_text = f"{raw_distance:.3f} m" if raw_distance is not None else "None (topic missing)"
    fused_text = f"{px4_dist_bottom:.3f} m" if px4_dist_bottom is not None else "None"
    return raw_distance, px4_dist_bottom, raw_text, fused_text
```

---

### 3.2 First Wall Detection & "Wall Passed" Log Hooks
In `live_navigation.py` inside `_aim_at_path()` and stage transition handler:

```python
# -----------------------------------------------------------------------------
# [GEMINI DEBUG] First Wall Sighting & Wall-Passed Logger
# -----------------------------------------------------------------------------
raw_lidar, px4_bottom, lidar_str, fused_str = self._format_raw_lidar_telemetry()

# Check distance to end of first corridor line (North = 2.191 m)
CORRIDOR_1_WALL_NORTH_METRES = 2.1912

if self.current_goal.stage_name == "first corridor":
    forward_direction = np.array([math.cos(drone_heading_radians), math.sin(drone_heading_radians)])
    front_clearance_metres = self.live_map.free_distance_metres(
        drone_metres, forward_direction, room_needed_metres=0.35, max_distance_metres=3.0
    )
    
    # Distance remaining along corridor to the wall
    distance_remaining_metres = CORRIDOR_1_WALL_NORTH_METRES - drone_metres[1]
    
    self.link.log.event(
        "gemini_debug_first_wall",
        f"[GEMINI DEBUG - FIRST_WALL] Front clearance = {front_clearance_metres:.2f} m | "
        f"Dist to wall line = {distance_remaining_metres:.2f} m | "
        f"Raw LiDAR = {lidar_str} | PX4 dist_bottom = {fused_str}",
        front_clearance_metres=round(front_clearance_metres, 3),
        distance_to_wall_metres=round(distance_remaining_metres, 3),
        raw_lidar_metres=raw_lidar,
        px4_dist_bottom_metres=px4_bottom,
        east_metres=round(drone_metres[0], 3),
        north_metres=round(drone_metres[1], 3)
    )

# Detection of crossing/passing the first wall
if self.previous_stage == "first corridor" and self.current_goal.stage_name != "first corridor":
    distance_past_wall_metres = drone_metres[1] - CORRIDOR_1_WALL_NORTH_METRES
    self.link.log.event(
        "gemini_debug_wall_passed",
        f"[GEMINI DEBUG - WALL_PASSED] Passed 'end of first corridor'! "
        f"Distance past wall line: {distance_past_wall_metres:+.2f} m | "
        f"Drone at North = {drone_metres[1]:.2f} m | "
        f"Raw LiDAR = {lidar_str}",
        distance_past_wall_metres=round(distance_past_wall_metres, 3),
        raw_lidar_metres=raw_lidar,
        px4_dist_bottom_metres=px4_bottom,
        drone_pose=[round(x, 3) for x in drone_metres]
    )
```

---

### 3.3 90-Degree Turn & Slide Tracker Hook
In `live_navigation.py` inside `_step_turning()`:

```python
# -----------------------------------------------------------------------------
# [GEMINI DEBUG] 90-Degree Turn & Dynamic Slide Tracker
# -----------------------------------------------------------------------------
if self.hold_point is not None:
    slide_distance_metres = float(np.linalg.norm(drone_metres[:2] - self.hold_point[:2]))
    angle_error_degrees = math.degrees(abs(_angle_between(drone_heading_radians, wanted_heading_radians)))
    raw_lidar, px4_bottom, lidar_str, fused_str = self._format_raw_lidar_telemetry()
    yaw_rate = math.degrees(self.watch.drone_yaw_rate())
    
    self.link.log.event(
        "gemini_debug_turn_90_progress",
        f"[GEMINI DEBUG - TURN_90] Heading: {math.degrees(drone_heading_radians):.1f}° "
        f"(Remaining: {angle_error_degrees:.1f}°) | "
        f"Slide from anchor: {slide_distance_metres:.2f} m | "
        f"Yaw rate: {yaw_rate:.1f}°/s | "
        f"Raw LiDAR: {lidar_str}",
        heading_degrees=round(math.degrees(drone_heading_radians), 1),
        remaining_angle_degrees=round(angle_error_degrees, 1),
        slide_distance_metres=round(slide_distance_metres, 3),
        yaw_rate_degrees_per_second=round(yaw_rate, 1),
        raw_lidar_metres=raw_lidar,
        px4_dist_bottom_metres=px4_bottom
    )
```

---

### 3.4 Settlement & Post-Turn Hold Hook
In `live_navigation.py` inside `_settled()`:

```python
# -----------------------------------------------------------------------------
# [GEMINI DEBUG] Settlement / Recovery Monitor
# -----------------------------------------------------------------------------
if not is_settled:
    raw_lidar, px4_bottom, lidar_str, fused_str = self._format_raw_lidar_telemetry()
    self.link.log.event(
        "gemini_debug_settling",
        f"[GEMINI DEBUG - SETTLING] Slide recovery in progress. "
        f"Current drift: {slide_distance_metres:.2f} m (Threshold: <= {SETTLED_DISTANCE_METRES:.2f} m) | "
        f"Raw LiDAR: {lidar_str}",
        drift_metres=round(slide_distance_metres, 3),
        elapsed_seconds=round(self.now_seconds - self.settling_since_seconds, 2),
        raw_lidar_metres=raw_lidar
    )
```

---

## 4. Diagnostic Value & Debugging Power

With these tags active, inspecting logs or monitoring live flight output gives instantaneous answers to:
1. **Did the drone actually pass the wall?**  
   `[GEMINI DEBUG - WALL_PASSED]` logs the precise distance past the boundary ($+0.26\text{ m}$).
2. **Did the bottom laser see the floor or glitch?**  
   `[GEMINI DEBUG - LIDAR]` prints the direct raw reading from `/fmu/out/distance_sensor` on every step.
3. **Did the drone slide into the net during the turn?**  
   `[GEMINI DEBUG - TURN_90]` records the exact growth of `Slide from anchor` as the turn progresses from $90^\circ \to 0^\circ$.
