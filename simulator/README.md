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

The simulated course comes from a course map file, the same kind of file the drone's route planner reads. Without `--world`, it is the organizers' drawing. To fly in our phone scan of the real course instead, or in any other map, name the map file first:

```bash
simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json landing_test.py --with-push-down
```

To see what changing one PX4 setting would do, without changing anything on the real drone, name it with `--try-px4-setting`. The flight's output then starts by saying it uses a setting drone 16 doesn't have. For example, this flies the whole course with PX4's laser check made less strict (section 7 of [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) says why):

```bash
simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json --try-px4-setting EKF2_RNG_K_GATE=3 route_test.py
```

The first flight in a new map takes about a minute longer, because the simulator builds a 3D model of the course from the map, and its distance tables, before anything starts. A changed map file counts as a new map. [`ros_packages/course_simulator/course_simulator/world.py`](ros_packages/course_simulator/course_simulator/world.py) explains how this works.

Look at the newest flight in 3D. This makes a web page next to the flight's record in `simulator/generated_files/flight_records/`, and opens it in your browser. You can turn the course with the mouse and play the flight back. See [`flight_viewer/README.md`](flight_viewer/README.md).

```bash
python3 simulator/flight_viewer/make_flight_viewer.py
```

Or draw a flat picture of the newest flight, seen from above. The picture is saved next to the flight's record.

```bash
docker run --rm --volume "$PWD:/repository" drone-simulator python3 /repository/simulator/plot_flight.py
```

Each flight leaves these files in `simulator/generated_files/`:

| File | What it is |
|---|---|
| `flight_records/flight_*_summary.json` | The referee's summary: which map it flew in, touches, checkpoints and the line used for each, finish time and PX4's clock speed. |
| `flight_records/flight_*.csv` | The drone's true position and heading 30 times per second. |
| `last_simulator_log.txt` | Everything the simulator printed. Look here first if something goes wrong. |
| `px4_flight_logs/` | PX4's own detailed record of everything it did. It is a ULog file, and the Python package `pyulog` can read it. |
| `worlds/` | The 3D model and distance tables made for each map other than the organizers' drawing. |

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
| `px4_changes/` | Drone 16's settings, the script that starts PX4, and our four small edits to PX4: one shares the drone's true position with the simulator, one lowers the noise on the pretend motion sensor, one makes the pretend floor solid, and one puts the motors where drone 16's motors are. |
| `ros_packages/course_simulator/` | Our pretend sensors, pretend remote control and referee. |
| `ros_packages/isaac_ros_visual_slam_interfaces/` | A stand-in for one message definition from the real camera tracking software, so the organizers' code can run without that software. |
| `inside_the_box/` | Small scripts that run inside the Docker box. |
| `test_flights/` | Scripted flights that test the simulator. |
| `flight_viewer/` | The tool that shows a flight in 3D, as a web page. |
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
- **The balance point is assumed.** The physics uses drone 16's motor positions, measured from the balance point that the organizers entered into PX4. We don't know whether that is where the drone really balances. If the battery sits further back, the real drone needs different motor power to stay level.
- **How the body resists turning and moving through air is guessed.** These are PX4's default values for a generic drone, not measurements. The control tuning, on the other hand, is drone 16's own, because drone 16 also uses PX4's defaults.
- **The bottom laser is a thin ray.** It gives 5 readings per second, each about 0.1 seconds late, which is what PX4's driver for the real sensor does. PX4's driver says the real sensor sees a cone 27 degrees wide, while the organizers say its beam is about 1 degree wide. If the cone is wide, the real drone notices an edge a little before it is over it.
- **The checkpoint lines are our own reading of the rulebook.** The exact lines haven't been published. The referee takes them from the map file, and `course_map_tools/checkpoint_line_rules.py` says how each is placed.
- **The real course differs from the drawing.** A phone scan of the real course, taken on 2026-11-06, shows some walls up to 73 cm from where the drawing puts them, one wall slanted the other way, one wall missing, and the boulders elsewhere. The simulator uses the drawing unless you choose the scan with `--world`. Some of the scan's walls still need checking against a photo (see [`../course_map_tools/README.md`](../course_map_tools/README.md)).
- **A course made from a map is plainer than the organizers' model.** Walls are plain boxes, the tunnel is a smooth oval tube, and boulders are upright cylinders. There is no window frame and no line on the floor.
- **The way a run starts is our best prediction.** See section 8 of [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md).

## Why this isn't the organizers' simulator

The organizers have their own, more realistic simulator, built on Isaac Sim. It renders camera images, but it needs a Linux PC with an x86 processor and an RTX 4080 graphics card, so it can't run on an M1 Mac. Ours is the closest we can get on our Mac.

## What we have tested

All of this was run on an M1 Mac, on 2026-11-06 unless another date is given.

- **The course model.** `check_course_model.py` passes. The simulator's copy of the course matches five known measurements to within 1.5 cm.
- **The start.** The pretend person arms and flips to Position, and the organizers' real program then hands control to the test flight. The takeoff follows without any stick input.
- **Short flights.** `landing_test.py` climbs to 0.8 m, flies 1 m forward along the first corridor, and lands. It stayed within about 8 cm of its line, peaked at about 0.84 m, and touched nothing. The closest a propeller came to an obstacle was about 13 cm.
- **Landing.** Six flights passed, three with the push-down and three without. The results are in section 7 of [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md).
- **Depth points.** One flight's points were correct: at cruise height, they showed the first wall where the course model says it is, with no invalid values. Nothing uses these points yet.
- **The drone's package.** `drone_code/team_nav` builds here, but this simulator doesn't run its launch file.
- **After the accuracy fixes (2026-11-07).** We added drone 16's motor positions, the laser's real reading rate and position, and the rest of drone 16's laser settings, then flew the landing test again, once with the push-down and once without. The motors stopped 6.0 and 9.7 seconds after touchdown, the same as before, with no touches. While hovering, the front motors ran at 65% power and the back motors at 35%, as drone 16's motor positions predict. The laser gave a reading every 0.2 seconds, and PX4 used it for height for the whole flight. The highest point was 0.89 m.
- **Other worlds (2026-11-07).** The landing test passed in the organizers' drawing, in our scan, and in the scan with every wall trusted. The referee now judges checkpoints by the mission file and the world's map. The landing test crosses no checkpoint lines, so that part hasn't been exercised in a flight yet. The first flight in our scan went wrong: three simulator programs each built the new world's distance tables during the flight, the referee started 40 seconds late, and the drone climbed to 3.6 m and never saw its touchdown. The same flight was normal once the tables existed. The simulator now builds them once, before anything starts, and a flight in a never-used map then passed too.
- **Touches fail the test (2026-11-07).** `run_test_flight.sh` now fails if the referee counts any touch. We checked this by flying the landing test into a wall added across the first corridor: the referee counted one touch, said where, and the test failed.
- **The box is cut off from other boxes (2026-11-07).** One flight went wrong because another box on the Mac (the Gazebo trial) was running its own PX4 at the same time, and the two boxes heard each other's ROS messages. The referee mixed up the two drones. The simulator's box now has no network at all, and the next flight was normal with the Gazebo box still running.

- **The whole course (2026-11-07).** `test_flights/route_test.py` plans a route from our scan map, takes off, follows it through all four checkpoints and the window, and lands on the pad. Three flights with drone 16's settings, each with a better route follower, took 244, 164 and 138 seconds from takeoff to motors off. All three landed on the pad with no touches, and all three failed the height check: PX4 stopped using the bottom laser and its height estimate drifted 29 to 53 cm from the truth. Each time the trigger was the laser's reading jumping (over the window's lower wall, or over a boulder the drifting camera tracking had put under the drone), followed within a few seconds by the drone slowing down. Section 7 of [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) explains PX4's rule and the results.

We have not tested lost camera tracking, or the drone spotting boulders in flight, which isn't written yet. No result here has been compared with a real flight.
