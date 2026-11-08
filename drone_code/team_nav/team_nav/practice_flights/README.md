# Practice flights

## A* intermediate waypoints (`hardcoded` branch)

`astar_five_waypoints` keeps the five main course goals and computes its intermediate
points from the participant map before arming. Read `missions/astar_five_goals.json`,
then `astar_route.py`, `astar_geometry.py`, and `astar_five_waypoints.py`.

Every flight waypoint is at **0.75 m** above launch; the final landing descends to **0 m**.
Each run saves `astar_plan.json` and `waypoints_with_heights.csv`, including every generated
point's east, north and height. Extra points are added where the route or turn needs them.

The first stop is north = **2.451 m**, about 5 cm earlier than the previous 2.50 m stop,
while still clearing the first checkpoint with the full airframe. It turns at that stop and
departs forward. There is no reverse segment. Tight turns use a 5 degree/second heading-target
slew; other turns keep 15 degrees/second. The clearance check uses the convex hull of the
measured motor positions expanded by the estimated propeller radius, with the same 5 cm
body clearance, rather than treating the empty corners of a rectangle as aircraft.

A* searches positions and preserves heading in tight spaces. In open space all headings
fit, so those states are merged. Path shortcuts must preserve turn, body and bottom-laser
clearance. The final chicane is also checked against the previous branch's declared drift
cases: 25 cm northward and 3 cm east/west. These are specific tested error cases, not a
guarantee against arbitrary localization errors.

This version plans from the supplied map. Live depth still pauses for obstacles or stale
data; it does not yet rebuild the route from an unreported obstacle. The moved-obstacle
comparison explicitly updates both the participant map and Gazebo world.

```bash
# Original obstacle positions:
bash simulator/gazebo_full/fly_course_in_gazebo.sh \
  --world drone_code/team_nav/course_maps/sketch_2026-11-08.json \
  --run-mode practice --longest-run-seconds 900 -- practice_flight astar_five_waypoints \
  --map sketch_2026-11-08.json

# Same five goals; north-west boulder moved 0.45 m south:
bash simulator/gazebo_full/fly_course_in_gazebo.sh \
  --world drone_code/team_nav/course_maps/astar_moved_obstacle.json \
  --run-mode practice --longest-run-seconds 900 -- practice_flight astar_five_waypoints \
  --map astar_moved_obstacle.json
```

## Previous five-waypoint controller

`five_waypoints` is the heading-aware version on `codex/attemptone-five-waypoints`.
Read `missions/five_waypoints.json` for its five main targets: First corridor, Maze exit,
Tunnel exit, Chicane exit, and Landing pad. Intermediate points avoid walls; connecting
only the five endpoints with straight lines would cut through the course.

| Main target | East (m) | North (m) | Cruise height (m) |
|---|---:|---:|---:|
| First corridor | -2.51 | 2.50 | 0.75 |
| Maze exit | 1.65 | 2.12 | 0.75 |
| Tunnel exit | 2.06 | -0.65 | 0.75 |
| Chicane exit | -1.20 | -1.35 | 0.75 |
| Landing pad | -2.0422 | -0.7925 | 0.75, then descend |

Read `five_waypoints.py` for the controller. At each corner it holds position, changes the
heading target at 15 degrees/second, waits for measured position and heading to settle, then advances at
0.25 m/s. It does not advance to the next point until measured arrival. `five_waypoint_plan.py`
checks the rotating airframe and every segment against the participant map before flight.
The first checkpoint has insufficient turning room, so a bounded 0.4 m retreat along the
just-flown corridor precedes the turn. Other segments face the direction of travel.

The existing `HeightKeeper` closes the loop on measured camera height during turns and
travel. This mode caps the added height target at 0.15 m. It compensates small offsets;
it does not repair PX4's estimator or erase its error from the referee. Existing sensor,
authority, contact and height checks remain active, including their documented handling
of height compensation. Climb and landing keep their existing controllers.

The final chicane corners provide extra clearance for tracking drift. A regression checks
their turning and travel footprints with a 25 cm northward offset and 3 cm east/west error.
This is a specific error margin, not a guarantee against arbitrary localization error.
Wall matching is not used: its trial pulled the position estimate incorrectly near a wall tip.

`forward_depth.py` requires recent depth with a matching tracked pose and pauses for nearby
obstacles ahead. Missing/empty pictures are not treated as clear space. The retreat requires
fresh depth but has no rear obstacle detection; its protection is the checked, recently flown
path. The controller stops after a persistent hold rather than replanning around obstacles.
Gazebo supplies simulated tracking; this does not validate real cuVSLAM tracking through turns.

Run from the repository root in the supported Linux/Docker simulator environment:

```bash
bash simulator/gazebo_full/fly_course_in_gazebo.sh \
  --world drone_code/team_nav/course_maps/sketch_2026-11-08.json \
  --run-mode practice --longest-run-seconds 600 -- practice_flight five_waypoints \
  --map sketch_2026-11-08.json
```

`full_course` extends the first-leg experiment through all four checkpoint lines and onto the
landing pad on `sketch_2026-11-08.json`. It validates the fixed, north-facing airframe footprint
and bottom-laser clearance before starting, then waits for measured arrival at each waypoint.
The first checkpoint requires a short northward excursion followed by backing south; the path
then goes around the north-west boulder rather than over it. This avoids the range discontinuity
that caused PX4's height estimate to diverge in the previous first-leg run. It does not change
PX4's estimator configuration. The referee must still confirm each actual flight.

The full-course landing caps airborne descent at 0.15 m/s. The first full-course Gazebo
trial cleared all checkpoints but reached 12.7 cm of height-estimate error during the old
0.5/0.35 m/s descent. With the slower descent, run `20261108_100150` cleared all four lines,
landed inside the pad, recorded zero contacts and no boulder overflight, and had 7.2 cm peak
in-flight height-estimate error. These are simulation results, not hardware qualification.

Run in Gazebo with `-- practice_flight full_course` after the launcher options. Normal touch and
height checks are enabled unless `--bare-bones-safety` is explicitly supplied.

From the repository root, run the tested full-course configuration:

```bash
bash simulator/gazebo_full/fly_course_in_gazebo.sh \
  --world drone_code/team_nav/course_maps/sketch_2026-11-08.json \
  --run-mode practice --longest-run-seconds 300 -- practice_flight full_course
```

The new flight is named `full_course`; `first_maze_leg` remains the shorter original route.
The fixed route assumes the sketch-map geometry and a north-facing aircraft throughout.
For the WSL clock conflict observed during testing, see `simulator/gazebo_full/README.md`.

Short real-drone flights for the practice steps in `docs/ARCHITECTURE.md`, section 11 ("Test order"): flying 1 m out and back to measure how quickly the drone follows its target, and flying a mock gap and a mock window built in the net. They run on drone 16 exactly as written. So far they have flown only in our simulator (the 3D rendered sim, `simulator/`).

The hover-and-land flight is not here. It is in `drone_code/smoke_flights/`.

## Files, in the order to read them

| File | What it does |
|---|---|
| `run_practice_flight.py` | Starts one flight by name, after checking practice mode. With no name, it lists the flights. |
| `out_and_back.py` | From a low hover, 1 m forward as a step and back as a slide, then land. Measures the response delay and how far the drone trails its target. |
| `response_measurements.py` | The arithmetic behind those two measurements, kept apart so it can be tested on its own. |
| `mock_gap_and_window.py` | The competition program, `course_run.py`, with the practice course's map and mission. |

The practice course itself is two files outside this folder: `course_maps/practice_mock_gap_and_window.json` and `missions/practice_mock_gap_and_window.json`. The tests are `test/test_practice_flights.py` and `test/test_practice_flights_measurements.py`.

Both flights use the same building blocks as the competition program (`flight_steps.py`): they wait until the drone is ready, hold a target above the launch mark until a teammate arms the drone and flips the switch to Position, and then fly. They send only targets. They never arm the drone, never ask PX4 to change mode, and never send PX4 a command. Every target passes the same safety checks as in the competition run (`run_safety.py`), and any problem, error or Ctrl+C lands the drone where it is.

## Before any flight on the drone

- The team approves the flight, a teammate holds the remote with the kill switch, and the drone is in the net.
- Practice flights only run in practice mode, so their records are never filed as a competition run. Set it in the terminal first:

  ```bash
  export TEAM_NAV_RUN_MODE=practice
  ros2 run team_nav practice_flight            # lists the flights
  ```

- The program prints what it is waiting for. When it says it is sending a target above the launch mark and waiting for the start switch, the teammate arms the drone and flips the switch to Position. To take over at any time, flip the mode switch or use the kill switch.
- Exit codes: 0 finished with the motors off, 1 stopped early or failed, 2 refused to start, 130 Ctrl+C.
- Each run's records go to a new folder in `~/team_ws/flight_records/`, named with the time, `practice` and the flight's name.

## Flight 1: out and back

**What it proves.** The drone takes off under our program, holds a hover, follows a target forward and back, and lands with our landing. And it gives our first tuning measurement: how quickly the drone follows its target.

**What it does.** It climbs to 0.5 m, hovers for 3 s, then:

1. **Out, a step.** The target jumps 1 m forward at once and stays there for 10 s. The **response delay** is the time from that jump until the drone has covered 63% of the metre. If PX4's position control behaves like a simple low-pass filter (section 9), this is its time constant. We also record how long until the drone stays within 5 cm of the target, and how far it overshoots.
2. **Back, a slide.** The target slides back to above the launch mark at 0.25 m/s, then stays there for 6 s. The **trailing distance** is how far the drone runs behind the moving target once its speed is steady. Divided by the speed, it should be close to the response delay. Section 9 expects about 1.3 s.

Then it lands where it started.

**Where to fly it.** At a launch mark with at least 1 m of clear floor ahead, and the map of that place. Before anything moves, the program checks on the map that the 1 m line keeps 0.35 m from every wall and keeps the bottom laser 0.25 m from every boulder, and refuses to start if not. The default is the course's launch mark on our phone-scan map, facing down the first corridor:

```bash
ros2 run team_nav practice_flight out_and_back
```

Options: `--map <file>` and `--start-heading-degrees <angle>` for another place, `--hover-height-metres 0.8` to measure at the course's cruise height (0.4 to 1.0 m allowed), and `--ignore-raw-laser` to test the checks the real drone gets (see below). The mock course's launch mark does not have 1 m clear ahead (the gap wall is 0.8 m in front of it), so the program refuses it there.

**What to look at afterwards.**

- The `measurement` lines on the screen and in `events.jsonl`, and `out_and_back_result.json`: the response delay, the time to settle, the overshoot and the trailing distance.
- `out_and_back_samples.csv`: the target and the drone's forward position every 0.1 s during the step and the slide. Plot `drone_forward_metres` and `target_forward_metres` against `seconds_since_start` to see the response.
- `drone_condition.csv`: the sideways (`from_launch_left_metres`) and height wander while hovering, the tilt (`tilt_degrees`), and whether PX4 kept using the laser for height (`px4_uses_laser_for_height`).

Positions come from camera tracking, because that is what our program steers by. So the delay includes camera tracking's own delay, which is the delay our program really has to live with. Positions are sampled 10 times a second, and the 63% point is found between samples, to about 0.05 s.

Once measured on the real drone, these numbers decide the route follower's target lead and speed (`route_follower.py`) and whether the window's level stretch is long enough (`opening_crossings.py`). Write each new value into its file by hand, with a note naming the flight it came from.

## Flight 2: the mock gap and window

**What it proves.** The whole competition program (planner, route follower, safety checks, landing) works on a real obstacle: it threads a gap a little wider than the drone, crosses a window whose bottom edge makes the bottom laser's reading jump by 0.4 m, and lands on a pad. Section 7 explains why that jump matters: PX4 may stop trusting the laser for height if the drone slows down soon after it.

**The layout.** The drone starts facing north. Distances are from the launch mark, "east" is to the drone's right.

- **Practice area:** 4 m wide by 6 m long of free floor. The launch mark is in the middle of the width, 0.5 m from the south end.
- **Gap:** two walls in a line across the area, 0.8 m ahead of the launch mark. Each wall is 1.0 m long, 1.2 m tall and at most 10 cm deep. The gap between their ends is **0.90 m** wide, and its middle is 0.4 m west of the launch mark. The drone needs 0.70 m, so this leaves 10 cm to spare on each side.
- **Window:** a second line of walls 2.3 m beyond the gap (3.1 m ahead of the launch mark): two pieces 0.6 m long, 1.2 m tall and at most 10 cm deep, with a **0.90 m** opening between them, its middle 0.4 m east of the launch mark. A box **0.40 m tall** fills the bottom of the opening, the full 0.90 m width. There is no top bar: the opening reaches the walls' 1.2 m top.
- **Landing pad:** a circle of 0.76 m radius (the real pad's size), centred 4.7 m ahead of the launch mark and 0.4 m east, straight past the window.

The planned route is about 5 m: climb to 0.8 m, slant west into the gap, through it, slant east to line up 1.3 m before the window, straight through the window's middle at 0.8 m, on to the pad, and land. The tightest place is the gap, where the drone's centre is 0.36 m from a wall end; it needs 0.35 m.

**Build it, then measure it.** Every number in the map is a proposal, and its `source` note says so. Build the course, then measure what was really built with a tape measure, and change the map file (or a copy, given with `--map`) before flying:

1. The free floor's size, and where the launch mark is on it. Put the origin in the middle of the free floor, x to the east (the drone's right), y to the north (the drone's nose).
2. Both ends of each wall, measured along the floor, and each wall's height and depth. **Use the real depth.** The planner treats each wall end as rounded by half the wall's depth, so deeper walls narrow the gap and the window, and they must then be built wider.
3. The window opening's two edges, and the height of the top of the box in it (the bottom edge).
4. The landing pad's centre and radius.
5. That the drone's nose points straight along +y on the launch mark. If it doesn't, give `--start-heading-degrees` (90 means along +y).

Then check that the planner still finds a route and has room, on a laptop with the package's tests:

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 -m pytest drone_code/team_nav/test/test_practice_flights.py -q -p no:cacheprovider
```

A changed map must be copied onto the drone and the package rebuilt there (`drone_code/README.md`), or given with `--map` as a full path.

**Run it.**

```bash
ros2 run team_nav practice_flight mock_gap_and_window
```

To practise one part at a time, stop after a stage and land there: `export TEAM_NAV_STOP_AFTER_STAGE="mock gap"` (or `climb`, or `mock window`). Any `course_run` option may follow the flight's name, for example `--ignore-raw-laser` or `--map <file>`.

**What to look at afterwards.**

- `route.txt`: the route the planner chose, and its least room.
- `events.jsonl`: each stage's start and finish, any hold, touch or height problem, and the landing.
- `drone_condition.csv` around the window: `raw_laser_metres` (in the simulator), `px4_uses_laser_for_height` and `px4_height_above_floor_metres`. Did PX4 keep using the laser after crossing the box? If it stopped, did the flight land early, as designed?
- `safety_checks.csv`: the tilt and turn rate in the gap and the window, against the touch limits (20 degrees, 120 degrees per second).
- Whether the drone landed inside the pad, measured with a tape.

## Running them in the simulator

The simulator plays the teammate who arms the drone and flips the switch (`simulator/test_flights/real_program_test.py`). Run these from the top folder of the repository on the Mac.

Out and back, in the default world (the organizers' drawing), with the matching map:

```bash
simulator/run_test_flight.sh real_program_test.py --env TEAM_NAV_RUN_MODE=practice -- \
    /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/practice_flight out_and_back --map organizers_drawing.json
```

The mock course needs one extra step first. The simulator's referee always judges the rulebook's five checkpoints and stops if the world's map lacks their lines, so `make_world_for_the_referee.py` writes a simulator-only copy of the practice map with five tiny pretend lines in a corner. The drone's program is still given the real practice map.

```bash
python3 simulator/test_flights/make_world_for_the_referee.py drone_code/team_nav/course_maps/practice_mock_gap_and_window.json
simulator/run_test_flight.sh --world simulator/generated_files/practice_worlds/practice_mock_gap_and_window.json \
    real_program_test.py --env TEAM_NAV_RUN_MODE=practice -- \
    /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/practice_flight mock_gap_and_window --ignore-raw-laser
```

`--ignore-raw-laser` makes the program behave as on drone 16, which does not publish the laser's raw reading. The program's records go to `simulator/generated_files/team_runs/`.

## What has been tested

All in the simulator on 2026-11-07, on an M1 Mac, with drone 16's PX4 settings. The simulator's clock ran at 0.97 of real time. "The referee's height check" is the simulator's own test that PX4's height estimate stays within 10 cm of the true height.

| Flight | Program's exit | Touches | PX4's largest height error | Result |
|---|---|---|---|---|
| Out and back, organizers' drawing, 6 s step watch | 0 | 0 | 0.07 m | Response delay 1.81 s; never settled within 5 cm in 6 s; overshoot 0.16 m. Trailing 0.33 m at 0.25 m/s (1.33 s). |
| Out and back, organizers' drawing, 10 s step watch | 0 | 0 | 0.08 m | Response delay 1.68 s; settled within 5 cm after 8.5 s; overshoot 0.09 m. Trailing 0.31 m at 0.25 m/s (1.22 s). |
| Mock course, `--ignore-raw-laser` | 0 | 0 | 0.13 m, just past the window | Through the gap and the window, landed on the pad, 41 s from hand-over to motors off. Closest propeller to a wall 0.16 m. The referee's height check failed. |
| Mock course, with the raw laser | 0 | 0 | 0.14 m, just past the window | The same, and the same failure. |

What this shows:

- **The response delay is about 1.7 to 1.8 s**, longer than section 9's 1.3 s, and the step overshoots by 9 to 16 cm and takes over 8 s to settle within 5 cm, not 2.5 s. The slide's trailing distance gives 1.2 to 1.3 s. A plain low-pass filter would give the same number both ways, so the drone is not quite one. While the drone accelerated forward, its height also sagged by about 8 cm below the 0.5 m target.
- **The mock window reproduces the real window's problem.** Over the 0.4 m box, PX4 kept using the laser, so part of the 0.4 m drop in the laser's reading went into its height estimate: PX4 thought the drone had sunk, and the drone climbed to about 1.0 m true height just past the window, 0.2 m above its target, with PX4's height wrong by 0.13 to 0.14 m by the referee's measure. Neither form of our height check stopped the run: the raw-laser check ignores places near walls by design, and PX4's "uses the laser" flag never changed. The landing was still normal. This is the effect section 7 describes, now seen on a course we can build and fly in practice.

## What has not been tested

- Neither flight has flown on the real drone. The real drone wobbles more, its camera tracking is real, and its response delay is what we want to measure.
- Nobody has built the mock course, so its sizes are untested proposals.
- In the simulator the walls are not solid: a touch is counted by the referee, but the drone flies on. On the real course a touch is a real collision.
