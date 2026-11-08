# Visuals pack: how our drone flies the course

Eleven Excalidraw diagrams, one per topic. The diagrams carry labels only. The
explanation for each one is below. Open a file at excalidraw.com (File → Open),
or with the Excalidraw extension in VS Code.

Colours mean the same thing in every diagram:

- Blue: our code.
- Grey: the organizers' code.
- Orange: PX4 (the flight controller).
- Green: hardware, or a good result.
- Red: a problem, or a bad result.
- Purple: the pretend world in our test bench.

The camera latency work is in diagrams 2, 3 and 4. Read those first.

---

## 1. System overview — `01_system_overview.excalidraw`

**What it shows.** The parts of the drone and how data moves between them.

- The stereo camera sends pictures to the Jetson (the computer on the drone).
- The organizers' tracking program (cuVSLAM) turns the pictures into a position report.
- PX4 combines the position report with its own motion sensor and the laser height sensor.
- Our depth node turns the camera pictures into a list of nearby obstacles.
- Our course program reads the position and the obstacles. It sends one target point to PX4, ten times each second.
- PX4 flies the drone to the target. Our code never arms the drone and never changes the PX4 mode.

---

## 2. Camera transport — `02_camera_transport.excalidraw`

**The problem.** Position reports often arrived late. A late report makes PX4 trust the camera less, and the drone drifts.

**What we found.**

- One camera picture is 1.97 MB (1024 × 640 × 3 bytes).
- The shared-memory block that carries the pictures is only 549 KB.
- If a reader is slow, the camera driver waits for it. Then the tracking program also waits.
- The tracking maths takes only 1.5 ms. So the delay was in moving the pictures, not in the maths.

**What we changed.**

- Our depth node and our recorder now get the pictures over the local network loopback (127.0.0.1). The setting is in `transport/pictures_over_network.xml`.
- The tracking program keeps its fast path. Only our extra readers moved.
- In competition mode the recorder saves no pictures. The depth node takes at most 10 pictures each second.

**Result: position reports more than 180 ms late.**

| Who reads the pictures | Late reports |
|---|---|
| None of our programs | 5.7 % to 6.7 % |
| Recorder, shared memory | 25.1 % |
| Depth node, shared memory | 25.7 % |
| Depth node, loopback | 8.1 % |

---

## 3. Latency timeline — `03_latency_timeline.excalidraw`

**What it shows.** How old a position report is when PX4 uses it.

- Normal age: 60 ms (median). 95 % of reports are younger than 75 ms.
- Bad periods: stalls of 200 ms to 430 ms, about 67 each minute.
- Old PX4 limit: it rejected reports older than about 235 ms. So most stalls made PX4 drop the camera.
- New PX4 limit: about 445 ms. Now PX4 keeps the camera through the normal stalls.
- Our own hold starts when a report is older than 450 ms. This is just above the PX4 limit, so we never hold while PX4 still trusts the camera.

**The PX4 settings we changed.** The flight controller must restart before these work.

- `EKF2_EV_DELAY` from 70 ms to 0 ms. The report time stamp is already the time the picture was taken. The old value counted the delay two times.
- `EKF2_DELAY_MAX` from 200 ms to 350 ms. This raises the limit above, from about 235 ms to about 445 ms.

**In our code.** We measure the age of each report from its picture time stamp, not from the time it arrived. That is the true age.

---

## 4. Heading and compass — `04_heading_compass.excalidraw`

**The problem.** Indoors, the compass is wrong by 135° to 180°.

- With the compass on, PX4 sometimes reset its heading by 36° to 64°.
- In one flight the drone turned 23° in 0.6 s because of this.

**What we changed.**

- `EKF2_MAG_TYPE` from 0 to 5. This turns the compass off.
- Now heading comes only from the camera tracking and the motion sensor.

---

## 5. Depth pipeline — `05_depth_pipeline.excalidraw`

**What it does.** It turns the left and right camera pictures into a list of obstacle points.

**The steps.**

1. Pair a left and a right picture taken within 20 ms of each other.
2. Shrink them to 512 pixels wide. This makes it faster.
3. Match the two pictures on the GPU (64 shift steps, 9-pixel patches). The shift tells the distance.
4. Keep a match only if it is clearly the best one (uniqueness 0.9).
5. Check left-to-right against right-to-left. Remove a pixel if they disagree by more than 1 pixel.
6. Remove small specks of fewer than 150 pixels. These are mostly false points. We use OpenCV for this: 0.7 ms, against 39 ms for the first version.
7. Use every 4th pixel, and keep only points between 0.45 m and 3.5 m away.

**Speed.** About 59 ms for each picture, about 8.6 pictures each second.

**Result.** About 90 % fewer false near points. Before, the "nearest object ahead" read 0.376 m in every picture, even in open space.

**The live map.**

- The floor is a grid of 5 cm squares.
- A square becomes a wall after 3 pictures see an obstacle there.
- Scores fade by 0.1 each second, so old mistakes go away.
- Each point is placed with the drone position at the time of the picture, not the current position.
- The path planner (A*) runs every 0.5 s on this map.

---

## 6. Safety rules — `06_safety_rules.excalidraw`

**The idea.** One bad reading must not end the run. Do the cheap thing at once. Land only if the problem lasts.

**Cheap actions, at once.**

- Hold still.
- Keep the last target.
- Start no turn.
- Pull a far target back to 1.2 m.

**Landing, only if the problem lasts.**

- Position report older than 0.45 s, for 3 s.
- PX4 heading and camera heading differ by more than 20°, for 3 s.
- PX4 not using the camera, for 2 s.
- Target more than 1.2 m away, for 0.3 s.

**Always land at once.** Touch (tilt or spin), lost control, or a target outside the course.

**Why.** In the 10:20 flight the heading gap lasted 1 s. The old code landed after 0.4 s. The new code holds, and then carries on.

---

## 7. Climb profile — `07_climb_profile.excalidraw`

**The problem.** The old code asked for 0.8 m at once. The drone drifted 0.11 m to 0.72 m sideways during takeoff.

**What we do now.**

- Climb at 0.30 m/s.
- Pause for 1.5 s at 0.30 m.
- If the drift is more than 0.30 m, wait up to 3 s for PX4 to pull the drone back. Do not land.
- Climb to 0.8 m. Settle for 2 s.
- Keep the launch heading during the climb. Every target has a heading.
- A climb step is done when the height is at least the target minus 0.10 m.

---

## 8. Navigation loop — `08_navigation_loop.excalidraw`

**The loop runs 10 times each second.**

- Read the position and the live map.
- Get the next point on the planned path, at most 0.35 m ahead.
- Send it to PX4.

**Turning in place.** A turn starts only if all of these are true:

- The drone is almost still (less than 0.08 m/s).
- Two plans in a row agree on the direction, within 20°.
- There is at least 0.50 m of room.

Then the drone turns at 15°/s, settles within 0.10 m, and waits for new depth pictures. While flying forward the nose turns at most 5°/s.

**When to give up and land where it is.**

- 165 s of the time used.
- 25 s with less than 0.3 m of progress.
- 20 s with no path.

---

## 9. Course plan — `09_course_plan.excalidraw`

**What it shows.** The sketch map of the course, and a full run of our real program in the test bench (blue line).

**Checkpoint 1 (the inset).**

- On the sketch, line 1 is at y = 2.19 m and the wall face is at y = 2.77 m.
- The gap is 0.58 m. The drone is 0.60 m wide. So on the sketch, the whole drone cannot be past the line without touching the wall.
- Our fix: past the line, the depth camera decides where the wall is, not the sketch. The drone creeps forward with a 0.05 m margin and stops 0.40 m from the wall.
- **Someone must measure the real line and the real wall.** The sketch may be wrong.

---

## 10. Noise bench — `10_noise_bench.excalidraw`

**What it is.** Our real flight code, running against a pretend drone, pretend tracking, pretend depth and a pretend world. The pretend parts add the faults we saw on the real drone.

- Pretend drone and PX4: slow response, yaw lag 0.40 s, 2 cm hover wobble, 0.1 m slide in turns.
- Pretend tracking: 60 ms normal age, and stalls of 200 ms to 430 ms.
- Pretend depth: 15 % of pixels, 2 % noise, false specks.
- Pretend world: walls moved by up to ±0.10 m. Wall 1 gap set between 0.58 m and 0.75 m.
- A referee scores each run: whole drone past each line, touches, 180 s limit.
- 40 runs with different random seeds. One 180 s flight takes about 20 s to run.

**Results.**

| | Before fixes | After fixes |
|---|---|---|
| Landed on the pad | 14 / 40 | 32 / 40 |
| Whole drone past lines 2 to 4 | 14 / 40 | 33 / 40 |

Mean time: 128 s.

**Bugs it found, now fixed.**

- Checkpoint 1 deadlock.
- A boulder blocked turns.
- The exit margin did not match between two parts of the code.
- The camera hold never ended.
- The time limit was skipped during a hold.

---

## 11. Simulator screenshots — `11_sim_screenshots.excalidraw`

**What it shows.**

- Four views from our 3D flight viewer of a full course run: see-through, from above, the maze and tunnel, and the window and tunnel exit.
- The drone model.
- A depth picture from the Gazebo simulator (yellow = near, dark = far).

**Note.** These are simulator pictures. The real course may be different. The simulator is a tool; the real flight is the goal.
