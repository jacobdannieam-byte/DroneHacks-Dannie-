# The simulator: start here

This folder lets us test flight code on a Mac, with no drone, in a copy of the competition course. Nothing in this folder ever runs on the real drone. The code that goes on the drone lives in [`../drone_code`](../drone_code/README.md).

**Status:** see "What we have tested" at the bottom. That section says what has actually been run, so trust it over anything else here.

## The idea in one paragraph

A simulator is only useful if it behaves like the real thing, so we run as much of the real thing as we can. The flight controller's software, called PX4, is the same version as on drone 16 and uses drone 16's settings. The organizers' programs that accept our targets and apply the safety limits are the same files as on the drone. What is pretend is everything PX4 normally gets from the physical world: the motors and the air, the cameras, the bottom laser, and the person holding the remote control. Each pretend part lives in its own short file, and its name starts with `pretend_`.

## How to run it

These steps run on a Mac, from the top folder of this repository. You need [Homebrew](https://brew.sh), and you only need to install and build once.

Install the tools that run Linux programs on a Mac. Docker runs a program inside a sealed Linux "box", and Colima provides the Linux for Docker to use.

```bash
brew install colima docker docker-credential-helper
```

Start Colima, and give it this repository's folder. This is the one step you repeat after restarting the Mac.

```bash
colima start --cpu 6 --memory 10 --mount "$PWD:w"
```

Build the box. This compiles PX4 and the other programs from source, which took about 20 minutes on an M1 Mac, and uses about 6 GB of disk.

```bash
simulator/build_simulator_image.sh
```

Run a test flight. The first run takes about a minute longer because it prepares the course tables and builds our ROS programs. After that, a flight takes about 40 seconds.

```bash
simulator/run_test_flight.sh landing_test.py --with-push-down
```

Fly our real mission program, [`drone_code/team_nav`](../drone_code/team_nav/README.md), started by the same launch file the drone uses. The script starts the simulator without its own copy of the organizers' flight interface, because our launch file starts that. It launches with two simulator-only settings:
- `target_system:=1`, because the simulator's PX4 is system 1 (drone 16's is 16);
- `vision_check:=false`, because the simulator has no camera images, so nothing reports camera tracking health.

The mission accepts `vision_check:=false` only from this script, inside the box. The script then presses start through the pretend remote control and watches the flight.

```bash
simulator/run_team_nav.sh          # the course stages
simulator/run_team_nav.sh route    # the route in drone_code/team_nav/missions/route_waypoints.json
```

With the stages, today it should show these steps, in order:
1. the hand-over;
2. a climb to 0.8 m;
3. a hold of about 5 s, because there is no line input yet;
4. the landing, with the push-down;
5. the motors switching off.

With the route, it flies each waypoint in turn and then lands. The watcher measures, from the camera tracking reports, how close the drone came to each waypoint. Either way, it ends with a RESULT line.

Draw the newest flight from above. The picture is saved next to the flight's record in `simulator/generated_files/flight_records/`.

```bash
docker run --rm --volume "$PWD:/repository" drone-simulator python3 /repository/simulator/plot_flight.py
```

Each flight leaves these files in `simulator/generated_files/`:

| File | What it is |
|---|---|
| `flight_records/flight_*_summary.json` | The referee's summary: touches, checkpoints, finish time and PX4's clock speed. |
| `flight_records/flight_*.csv` | The drone's true position 30 times per second. |
| `flight_records/flight_*_team_nav_log.txt` | Our mission's own log for that flight (only from `run_team_nav.sh`). Every decision it made, with its reason. |
| `last_simulator_log.txt` | Everything the simulator printed. Look here first if something goes wrong. |
| `last_team_nav_log.txt` | Our mission's log from the latest `run_team_nav.sh` run. |
| `px4_flight_logs/` | PX4's own detailed record of everything it did. It is a ULog file, and the Python package `pyulog` can read it. |

Check the clock speed in the summary. It should be close to 1.0, and ours were between 0.92 and 1.01. If it's far off, close other busy programs on the Mac and fly again.

## Which files to read, in order

1. **[`ros_packages/course_simulator/launch/whole_simulator.launch.py`](ros_packages/course_simulator/launch/whole_simulator.launch.py)**. This lists every program that runs, so it gives you the whole picture.
2. **[`test_flights/landing_test.py`](test_flights/landing_test.py)**. This is an example of flight code using the simulator. It reads as a list of steps.
3. **[`ros_packages/course_simulator/course_simulator/pretend_remote_control.py`](ros_packages/course_simulator/course_simulator/pretend_remote_control.py)**. This shows how a run starts: arm, then flip to Position.
4. **The other `pretend_` files**, in any order. They are all short and have the same shape: read the drone's true position from PX4, then publish what a sensor would see from there.
5. **[`ros_packages/course_simulator/course_simulator/referee.py`](ros_packages/course_simulator/course_simulator/referee.py)**. This watches the flight and records touches, checkpoints and times.
6. **[`px4_changes/drone_16_settings.sh`](px4_changes/drone_16_settings.sh)**. This lists every PX4 setting we use, and says which values are measured and which are estimated.

## What each folder is for

| Folder | What is in it |
|---|---|
| `course_model/` | The organizers' 3D model of the course. |
| `organizers_code/` | The organizers' programs, unchanged. See its [README](organizers_code/README.md). |
| `px4_changes/` | Drone 16's settings, the script that starts PX4, and our three small edits to PX4: one shares the drone's true position with the simulator, one lowers the noise on the pretend motion sensor, and one makes the pretend floor solid. |
| `ros_packages/course_simulator/` | Our pretend sensors, pretend remote control and referee. |
| `ros_packages/isaac_ros_visual_slam_interfaces/` | A stand-in for one message definition from the real camera tracking software, so the organizers' code can run without that software. |
| `inside_the_box/` | Small scripts that run inside the Docker box. |
| `test_flights/` | Scripted flights that test the simulator. |
| `generated_files/` | Everything the simulator produces. Git ignores it, and you can delete it at any time. |

## What is real and what is pretend

| Part | In the simulator | On the real drone |
|---|---|---|
| Flight controller software (PX4) | **Real**: the same version (1.16.2), with the settings in `px4_changes/drone_16_settings.sh` | The same, plus a small patch by the organizers |
| Message link to PX4 (MicroXRCEAgent) | **Real**: the same version | The same |
| Organizers' flight interface and position bridge | **Real**: identical files, version 0.3.2 | The same |
| Drone's movement and the air | **Pretend**: PX4's built-in simple physics, called SIH, with a more solid floor | Real physics |
| Motion sensor | **Pretend**: part of SIH, with less noise than SIH's default | A real sensor on a vibrating frame |
| Position from the cameras | **Pretend**: the true position, plus a delay, jitter and slow drift | cuVSLAM, a program that estimates position from camera images |
| Bottom laser | **Pretend**: a ray traced to the course model | A real laser |
| Depth from the cameras | **Pretend**: 64 × 40 rays traced to the course model, with errors added | Depth computed from the OAK-D Pro stereo cameras |
| Remote control | **Pretend**: a service we call | A teammate |
| Referee | **Pretend**: a program that uses the drone's true position | A person |

## What the simulator can't tell us

Please keep these limits in mind when you read a result. A pass here means the logic works, not that the flight will work.

- **There are no camera images.** On the real drone, the camera tracking may lose its position or drift when it sees dark or plain walls. Here it never does, unless we add that deliberately. The simulator can't tell us how well the real tracking will work in the course.
- **A collision doesn't stop the drone.** The referee records the touch, but the pretend drone flies straight through the wall.
- **There is no airflow.** The real drone pushes air off walls and the floor, and that moves it around. The pretend drone doesn't feel this.
- **Only the floor physically exists.** PX4's simple physics assumes a flat floor at height zero, with no walls and no raised platforms.
- **The vibration is an estimate.** We set the pretend motion sensor's noise to the level that drone 16's settings expect. If the real drone vibrates more, it will wander more than the simulated one, which held its position to within about 10 cm. See `px4_changes/lower_simulated_motion_sensor_noise.py`.
- **Touching the floor is simplified.** The simulated legs stop the drone dead, with no bounce or slide.
- **The weight and thrust are estimates.** We haven't weighed the drone or measured its thrust. They are labelled as estimates in `px4_changes/drone_16_settings.sh`.
- **The frame's shape is simplified.** The simple physics treats the drone as a symmetric frame with default balance values, and we didn't copy drone 16's exact motor positions or control tuning. It won't wobble the way drone 16 does.
- **The course may change.** The organizers call this course model preliminary, and the checkpoint positions in `referee.py` are our own draft, because the exact lines haven't been published.
- **The way a run starts is our best prediction.** See section 8 of [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md).

## Why this isn't the organizers' simulator

The organizers have their own, more realistic simulator, built on Isaac Sim. It renders camera images, but it needs a Linux PC with an x86 processor and an RTX 4080 graphics card, so it can't run on an M1 Mac. Ours is the closest we can get on our Mac.

## What we have tested

All of this was run on 2026-11-06, on an M1 Mac.

- **The course model.** `check_course_model.py` passes. The simulator's copy of the course matches five known measurements to within 1.5 cm.
- **The start.** The pretend person arms and flips to Position, and the organizers' real program then hands control to the test flight. The takeoff follows without any stick input.
- **Short flights.** `landing_test.py` climbs to 0.8 m, flies 1 m forward along the first corridor, and lands. It stayed within about 8 cm of its line, peaked at about 0.84 m, and touched nothing. The closest a propeller came to an obstacle was about 13 cm.
- **Landing.** Six flights passed, three with the push-down and three without. The results are in section 7 of [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md).
- **Depth points.** One flight's points were correct: at cruise height, they showed the first wall where the course model says it is, with no invalid values. Nothing uses these points yet.
- **Our mission program** (2026-11-07, on a second Mac with Docker Desktop). `run_team_nav.sh` flew `drone_code/team_nav`, started by its own launch file. It did everything expected:
  - the hand-over;
  - a climb peaking at 0.78 m;
  - a 5 s hold for the missing line input;
  - the landing at medium and then gentle speed, with touchdown confirmed and the push-down at 1 m/s;
  - motors off 30 s after the hand-over;
  - no touches.

  PX4 also kept reporting Offboard after it disarmed, which the mission now expects. PX4's clock ran at only 0.59 of real time on that Mac, so these times are stretched, and the camera tracking reports had short gaps. The first run also found a real bug: `rclpy.parameter_client` doesn't exist in ROS 2 Humble, so the mission program crashed on startup. The camera exposure cap now calls the camera's parameter services directly. The simulator has no camera program, so the cap itself wasn't exercised here.

- **The first route mission** (2026-11-07, same Mac, PX4's clock at 0.90 of real time). This was the earlier 14-waypoint route, at 0.4 m/s with 15°/s turns. We ran `run_team_nav.sh route` twice:
  - **First run:** it reached waypoints 1 to 11, each within 4 cm by camera tracking. Then the simulator's camera tracking reports paused for 0.21 s. That tripped watchdog rule (a), whose limit is 0.2 s, so the drone landed there, with the push-down and the motors off. It was 1.7 m short of the last waypoint.
  - **Second run:** it reached all 14 waypoints, the last within 7 cm, then landed with the push-down. The motors were off 162.8 s after the hand-over, close to the 170 s limit.

  In both runs the referee credited the first corridor, Gate A and the tunnel. It also recorded touches: 2 in the first run, both at the corner of waypoint 9. In the second run there were 6: at waypoint 9's corner again, on the leg from waypoint 3, and on the legs from waypoints 11 and 12. The simulator adds slow drift to camera tracking, while the referee uses the true position, so the late touches may be partly drift. The corner at waypoint 9 needs more room.

- **The current route mission** (2026-11-07, same Mac, PX4's clock at 0.90). This is the 20-waypoint route, which keeps 0.40 m from the walls except at Gate B, flown at 0.5 m/s with 25°/s turns. It flies through waypoints 13 and 14 and stops at the other corners. The watchdog's rule (a) limit is 0.3 s. Two runs:
  - **First run:** all 20 waypoints, the last within 6 cm. Motors off 152.4 s after the hand-over. 3 touches, on the legs after waypoints 16, 17 and 18.
  - **Second run:** all 20 waypoints, the last within 5 cm. Motors off 154.8 s after the hand-over. 3 touches, on the legs after waypoints 10, 16 and 18.

  Both runs reached the first four checkpoints, including the chicane into the landing area, and the watchdog never tripped. Neither met the team's target of 120 s. Of the first run's time, 69 s was flying (the drone trails its carrot, then creeps the last stretch into each of the 17 stop corners), 48 s turning in place, 10 s stopping, 7 s taking off and 11 s landing. Two touch spots repeated in both runs, late in the route, where the simulator's deliberate tracking drift is largest.

We have not tested a full run of the course, because the mission's detectors (the line, the pad and the depth clearances) don't exist yet. We also haven't tested the bottom edge of the window, the boulders, or lost camera tracking. No result here has been compared with a real flight.
