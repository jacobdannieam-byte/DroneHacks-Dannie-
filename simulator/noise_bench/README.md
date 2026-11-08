# Noise bench (PRETEND / SIMULATOR ONLY)

Nothing in this folder ever runs on the drone. It refuses to start if the real ROS (`rclpy`) is
loaded.

## What it is for

It flies our **real** competition program (`drone_code/team_nav/team_nav/course_run.py`,
competition mode, live navigation) through the whole course, many times, on the Mac, faster than
real time, with the noise we measured on drone 16. It answers: does the program finish, and if
not, why and where does it get stuck? One 3-minute trial takes about 5-15 s of computer time.

## What is real and what is pretend

Real, unchanged: everything in `team_nav` from the moment `course_run` starts: planning, the
ready check, the hand-over, the climb, `LiveFlight`, `LiveNavigator`, `LiveMap`, `DepthToMap`,
wall alignment, `run_safety`, the landing, and the real `DroneLink` and `DroneReports`. The
program writes its normal records (events.jsonl, live_navigation.csv, live maps) into a folder
per trial (printed as `records_folder`).

Pretend:

| File | What it pretends |
|---|---|
| `ros_stand_ins.py` | ROS itself: empty message classes, and `rclpy.spin_once`, which moves simulated time forward. |
| `bench_link.py` | The ROS node our `DroneLink` talks to: delivers camera tracking, PX4 reports, the organizers' summary and depth points; takes our targets; plays the teammate who arms; runs the referee. Also swaps `time.monotonic()` in every `team_nav` module for the simulated clock. |
| `pretend_drone.py` | Drone 16 + PX4: sideways and up/down response, heading lag, hover wander, turn slide, takeoff drift, climb delay, land detector. Each number says where it came from. |
| `pretend_tracking.py` | Camera tracking: a trial starts "clean" or "bad" and switches state after minutes (mean 7 min clean, 3 min bad); 60 ms age when clean; in "bad" periods 0.20-0.43 s freezes about 67 a minute, released in a burst; 1 %/m drift; PX4 dropping a report older than 0.445 s and dead-reckoning (compass OFF, so no heading jumps). |
| `pretend_world.py` | The TRUE course (the map file with every wall, the tunnel and the boulders moved by about 0.10 m, and the north-west top wall 0.58-0.75 m past line 1), and the depth camera: a 3D ray-cast against boxes and the floor, 15 % of pixels, 0.37-3.5 m, 2 % range noise, a mostly see-through net, false speck clusters at 0.45-1.2 m ("filtered" = about 1.5 small clusters a picture, "old" = about 15 big ones). |
| `bench_referee.py` | The referee: checkpoints in order (whole drone past the line), touches (0.30 m radius reaching a true solid), crash, finish on the pad, the 180 s limit, the organizers cutting the motors after 30 s without 0.3 m of progress. |
| `run_one_flight.py` | One seeded trial, one result row, optional trace CSV. |
| `monte_carlo.py` | Many trials in parallel, and the results table. |

Read them in that order.

## How to run

Use the Python that has numpy, scipy, pandas and opencv (no ROS needed):

```bash
cd simulator/noise_bench
python run_one_flight.py --seed 3 --tracking bad --specks filtered --trace /tmp/trace.csv
python monte_carlo.py --trials 20                      # 20 seeds x (clean, bad) x (filtered, old)
python monte_carlo.py --trials 40 --tracking bad --specks filtered --output bad_filtered.csv
python pretend_drone.py                                # the pretend drone's step and turn, next to the measured numbers
```

Result columns: `checkpoints` (scored, in order, 0-5), `lines_passed_any_order` (diagnosis: lines
the whole drone got past, in any order), `touches`, `flight_seconds` (from liftoff), `score`
(checkpoint points; a finish uses an assumed fastest finish of 60 s), `referee_end`,
`program_stop` (the program's own last word), `last_stage`, the end position, and the share of
navigation cycles per activity.

## How to tune a constant with it

1. Run a sweep and keep the CSV: `python monte_carlo.py --trials 40 --output before.csv`.
2. Change one constant in `drone_code` (for example a clearance or a speed).
3. Run the **same seeds** again: `python monte_carlo.py --trials 40 --output after.csv`. Seed n
   gives the same pretend course and the same noise both times, so differences come from the
   change (and from the program reacting differently, which changes later noise draws).
4. Compare the two tables, and open the records folder of any seed that changed.

## Where it touches the real code (update here after code changes)

- `run_one_flight._fly_the_real_program`: repeats `course_run.plan_then_fly` (`parse_arguments`,
  `plan_the_course`, `flight_steps.DroneLink(node, log)`, `CourseRun(...).run()`), because
  `plan_then_fly` builds a real ROS node itself.
- `bench_link.py`: topic names and the message fields that `drone_reports.py`, `flight_steps.py`,
  `target_sender.py` and `depth_camera.points_from_message` read; `node.get_clock().now()`.
- `bench_link.use_simulated_time`: replaces the `time` module in every `team_nav` module.
- `ros_stand_ins._get_package_share_directory` points at `drone_code/team_nav` for maps and missions.

## Known gaps (it is kinder or harsher than reality here)

- The program's own computing takes no simulated time (on the Jetson a cycle takes up to 30 ms
  and sometimes 300 ms).
- No tilt dynamics, battery sag, contact forces or prop wash near walls. A crash is only
  "propeller 0.10 m into a solid".
- Depth pixels are kept at random, evenly; real stereo is clumped on texture and blind on plain
  walls. No people, poles or lights beyond the net. Tunnel = two flat sides and a flat roof;
  boulders = square boxes. The tunnel is not dark.
- No bottom-laser jumps over boulders or the window sill; PX4 always uses the laser.
- Compass off: no heading jumps (EKF2_MAG_TYPE=5). Tracking never jumps or gets lost.
- Checkpoint lines are our map's lines, not the organizers' (which have no coordinates).
- The motor-cut rule (30 s without 0.3 m of progress) is our guess at what the organizers do.
- The drift correction (wall_alignment.py) never fired in the bench (0 pictures used). On the
  real drone at 10:44 it used 29 of 208 pictures. So the bench may show more drift than reality.
- A run ending with "landed onto the Boulder 2" means the program chose to land while hovering
  over a boulder; the touch it causes on the way down is counted in `touches` too.
- The real code is being changed by other engineers while this bench runs; a sweep uses
  whatever is on disk when each trial starts.
- We have not compared a bench run side by side with a real flight log yet.
