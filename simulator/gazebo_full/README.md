# The full Gazebo simulator: our real flight program, with two camera eyes

**SIMULATOR ONLY.** Nothing in this folder runs on the drone or talks to one. The box has no network at all.

This simulator runs the same software as drone 16 as far as a Mac allows, so the team can watch the real `course_run` fly the course and see the path. Its new part is a pair of **camera pictures**: Gazebo draws what the OAK-D's left and right eyes would see, on the same ROS topics as on the drone, so the team's stereo depth code can be tested on real pictures.

It builds on [`../gazebo_trial/`](../gazebo_trial/README.md), which showed that PX4 flies drone 16's shape in Gazebo on this Mac. Read that README's "What doesn't work" too: everything said there about timing gaps also applies here.

## How to run it

From the top folder of the repository, on the Mac, with Colima running (see [`../README.md`](../README.md)):

```bash
simulator/gazebo_full/fly_course_in_gazebo.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
```

- `--world <map file>` is what is really there: the course Gazebo builds, the referee judges and the picture shows. Without it, the organizers' drawing.
- `--run-mode practice` (the default) or `competition` is given to `course_run` as `TEAM_NAV_RUN_MODE`, as on the drone.
- `--longest-run-seconds N` stops `course_run` with Ctrl+C after N seconds (default 360), so it lands.
- Everything else, or everything after `--`, goes to `course_run` unchanged. The drone's own map is `course_run`'s `--map`; without it, `course_run` uses its own default map, whatever `--world` says. For example, to fly a drone whose map is wrong:

```bash
simulator/gazebo_full/fly_course_in_gazebo.sh --world simulator/generated_files/boulder_surprise_worlds/<world>.json -- --map phone_scan_2026-11-06.json
```

The first run builds the box (`build_gazebo_full_image.sh`, a few minutes if the Gazebo trial's box exists) and our ROS packages. A run then takes about 1 minute to start, plus the flight. Don't run the normal simulator at the same time: Gazebo needs the processors (see the trial's README).

## What a run leaves

Each run has its own folder, `simulator/generated_files/gazebo_full_runs/<date_time>/`:

| File | What it is |
|---|---|
| `summary.txt` | Did it finish, `course_run`'s own verdict, checkpoints, touches and where, times, simulation speed, camera rates, stereo check. Read this first. |
| `path_from_above.png` | The world map's walls, tunnel and boulders, the true path (from Gazebo), the camera tracking path (what the drone believed), the targets `course_run` sent, every touch, and course_run's last live map (squares it believed blocked) and last planned live path. |
| `stereo_check/` | One left and right picture from the launch mark, and `stereo_depth_check.png`: OpenCV StereoSGBM depth next to the true distances, with a table in `stereo_depth_check.txt`. |
| `course_run_records/` | `course_run`'s own records and the flight recorder's, exactly as on the drone. In practice mode the recorder also saves one camera picture a second per eye. |
| `referee/` | The referee's record of the true path (`flight_*.csv`) and its judgement (`*_summary.json`). |
| `flight_program_output.txt` | Everything `course_run` printed, and the start sequence. |
| `simulator_log.txt` | Everything the other programs printed. |
| `px4_and_gazebo/` | PX4's and Gazebo's output, PX4's flight log (ULog), the settings PX4 really used, and the camera program's rate reports. |
| `camera_rate_left.txt`, `camera_rate_right.txt` | `ros2 topic hz` on the two picture topics, measured on the launch mark. |
| `camera_stamps.csv` | The stamp of every picture of the run. `summary.txt` turns it into in-flight picture rates and the left-right stamp gap, next to the depth program's in-flight rate and live navigation's count of depth pictures used. |
| `gazebo_world/` | The world file Gazebo used, and its textures. |

To draw a run again: `docker run --rm --network none -v "$PWD:/repository" drone-simulator-gazebo-full python3 /repository/simulator/gazebo_full/draw_run.py /repository/simulator/generated_files/gazebo_full_runs/<run> /repository/<world map>`.

## What is real and what is pretend

| Part | Here | On drone 16 |
|---|---|---|
| PX4 | **Real**: 1.16.2, drone 16's settings, without the `SIH_*` ones for PX4's own physics | The same |
| Message link (MicroXRCEAgent), `/fmu/in/*`, `/fmu/out/*` | **Real** | The same |
| Organizers' `flight_interface` (`auto_offboard_on_arm`, `target_system` 1) and `px4_odom_bridge` | **Real**: the same files | The same |
| `course_run`, `flight_recorder` and `depth_camera` | **Real**: started as `ros2 run team_nav ...`, unchanged. `depth_camera` runs on the Gazebo pictures and publishes `/team_nav/obstacle_points` | The same, but `depth_camera` has a GPU there; here it uses NumPy, about 0.2 to 0.3 s per picture pair |
| Physics, walls, net | **Gazebo**: a drone with drone 16's motor positions and estimated weight and thrust; walls are solid | Real |
| Camera pictures `/oak/left/image_raw`, `/oak/right/image_raw`, `camera_info` | **Pretend**: drawn by Gazebo, `bgr8`, **512 × 320 at 10 per second**, drone 16's calibration halved, eye spacing 0.07515 m, on the nose at drone 16's camera position | 1024 × 640 at about 25 per second |
| Camera tracking `/visual_slam/tracking/odometry`, `/visual_slam/status` | **Pretend**: the true position from Gazebo, 30 per second, 60 ms late, 5 mm jitter, 1% sideways drift (the normal simulator's `pretend_camera_tracking`) | cuVSLAM, from the pictures |
| Bottom laser | **Pretend**: a ray traced through the world map's model | A real laser |
| Remote control | **Pretend**: arms and flips to Position once `course_run` sends targets (`real_program_test.py` and `pretend_remote_control`) | A teammate |
| Referee | **Pretend**: counts a touch when a propeller comes within 1 cm of the world map's model | A person |

## What does not match the real drone

- **Pictures are half size and slower**: 512 × 320 at 10 per second, not 1024 × 640 at 25. Software drawing on the Mac can't do more without slowing the whole simulation. Depth code must read the size and lens numbers from `camera_info`, not assume them.
- **The walls' look is invented.** Every surface has a random blotchy texture so that stereo matching has something to match. Real walls may be plainer or busier. The net is drawn as a solid sheet; the real one is see-through.
- **The pictures are perfect**: no noise, blur, exposure changes or motion blur. Picture time stamps are this computer's clock when the picture arrives, not when it was taken.
- **Camera tracking doesn't look at the pictures.** It is made from the true position, so it never loses track on a plain wall. cuVSLAM needs an NVIDIA graphics processor, which the Mac lacks.
- **Gazebo's timing gaps.** Gazebo doesn't wait for PX4. On this Mac PX4 sometimes misses motion readings for 20 to 44 ms, which can make the drone dip (see the trial's README). Before blaming flight code for a height problem here, check PX4's log for gaps.
- **The course sits 4 cm lower in Gazebo** than in the referee's and laser's model (the trial's `make_gazebo_world.py` explains why).
- **A course made from a map is plain**: boxes, an oval tube and cylinders. No window frame or floor line.
- **Weight, thrust and inertia are estimates**, as in the normal simulator.
- The ray-traced pretend depth camera of the normal simulator (`/simulator_only/depth_points`) is **not** running here.
- **The depth program is slower than on the drone.** The box has no graphics processor, so `team_nav depth_camera` uses NumPy, which takes about 0.2 to 0.3 s per picture pair and competes with Gazebo for the processors. Check the in-flight depth rate in `summary.txt` before judging live navigation.
- **Walls are solid.** A propeller that touches a wall can flip the drone. In the normal simulator the drone passes through and only the referee notices.

## Files, in reading order

1. [`fly_course_in_gazebo.sh`](fly_course_in_gazebo.sh): the one command.
2. [`whole_stack_in_gazebo.launch.py`](whole_stack_in_gazebo.launch.py): every program that runs, marked real or pretend.
3. [`inside_the_box/start_px4_and_gazebo.sh`](inside_the_box/start_px4_and_gazebo.sh): starts Gazebo, PX4 and the cameras.
4. [`inside_the_box/check_cameras_then_fly.sh`](inside_the_box/check_cameras_then_fly.sh): the recorder, the camera checks, then `course_run`.
5. [`models/drone_16_with_oak_d_stereo/model.sdf`](models/drone_16_with_oak_d_stereo/model.sdf): the two eyes, and where each number comes from.
6. [`pretend_oak_stereo_cameras.py`](pretend_oak_stereo_cameras.py): Gazebo pictures to the real ROS topics.
7. [`make_textured_world.py`](make_textured_world.py): the world from a map, with textures.
8. [`check_stereo_depth.py`](check_stereo_depth.py): the one-off stereo depth check, and [`record_camera_stamps.py`](record_camera_stamps.py), which notes every picture's stamp.
9. [`draw_run.py`](draw_run.py): the path picture and the summary.
10. [`Dockerfile`](Dockerfile) and [`build_gazebo_full_image.sh`](build_gazebo_full_image.sh): the box (the trial's box plus OpenCV).

## What we have tested

All on our M1 Mac on 2026-11-07, with nothing else running. Run folders are in `simulator/generated_files/gazebo_full_runs/`.

- **The whole stack starts and flies** every time (6 runs): PX4, the message link, the organizers' programs, `course_run`, the recorder and the team's `depth_camera`. The pretend person armed and flipped to Position, and `course_run` got control. Simulation speed 0.90 to 0.94.
- **Pictures:** `ros2 topic hz` on the launch mark gave 9.2 to 9.4 pictures per second on each eye. In flight, with `depth_camera` busy, 6.7 per second (run 20261107_193810).
- **Stereo depth is right.** OpenCV StereoSGBM on one pair from the launch mark matched 65% of pixels. Against the world map, the wall to the right (1.14 m) read 1.12 to 1.13 m and the floor (0.36 to 0.37 m) read 0.37 to 0.39 m: median error 1.5 to 3 cm.
- **The left and right stamps must be equal.** At first each eye was stamped when it arrived. In flight the pair then arrived 30 to 40 ms apart (median), more than the 20 ms within which `depth_camera` pairs pictures, so live navigation got only 9 to 16 depth pictures in a whole flight. With one shared stamp per Gazebo moment (`pretend_oak_stereo_cameras.py`), the gap is 0 ms and live navigation used 169 depth pictures in about 50 s (`depth_camera` 3.5 per second in flight, about 0.2 s of calculation each).
- **Flights so far have not reached the pad.** In the sketch world (20261107_193810), the drone crossed the north-west boulder, PX4 stopped using the bottom laser, and the drone truly sank from 0.82 m to the floor while PX4 believed it was at 0.8 m. This is the same as the Gazebo trial's height problem; check PX4's log before blaming flight code. Earlier runs ended with a propeller touching a wall end in the maze, which flips the drone in Gazebo, or with live navigation's 300 s limit.
