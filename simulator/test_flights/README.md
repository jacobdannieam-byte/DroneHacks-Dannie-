# Test flights

Each file here is a scripted flight that runs in the simulator. These flights test the simulator and individual ideas, such as the landing. They are not the competition program, and they must not run on the real drone.

Read them in this order:

| File | What it does |
|---|---|
| `simulator_test_flight.py` | The steps every flight shares: waiting for the simulator, starting, climbing, landing, and sending targets without a break. |
| `landing_test.py` | Climbs to 0.8 m, flies 1 m forward, and lands. Its results are in section 7 of [`../../docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md). |
| `route_test.py` | Plans a route from the drone's map, flies the whole course and lands on the pad. |
| `real_program_test.py` | Runs one of our real drone programs, unchanged, and plays the teammate who arms the drone and flips the switch. |
| `make_fallback_test_worlds.py` | Makes our phone scan with the maze walls changed (one removed, one moved, one removed and one slanted), for the fallback missions' flights. |
| `fallback_flight_report.py` | After a fallback flight: speed, true height, PX4's height and the laser at each wall crossing, touches, and whether it landed on the pad. |
| `make_world_for_the_referee.py` | Copies a practice map, such as the mock gap and window, and adds the rulebook's checkpoint lines as tiny pretend lines in a corner, so the referee can load it. Simulator only; the drone's program still gets the real map. |
| `rise_and_land_test.py` | Runs the rise-and-land program from `drone_code/smoke_flights/` (rise 0.5 m, hold 5 s, land), written by another session. It passed once on 2026-11-07 at about 12:57, with PX4's height within 10 cm (largest error 0.098 m). |

Run them from the top folder of the repository. Each run starts a fresh simulator, flies, and prints the referee's summary.

## The landing test

The first command uses the landing design from the plan. The second leaves out the push-down step, so that we can see how much time that step saves.

```bash
simulator/run_test_flight.sh landing_test.py --with-push-down
```

```bash
simulator/run_test_flight.sh landing_test.py --without-push-down
```

## The route test

The world, which is what is really there, and the drone's map, which is what the drone believes, are chosen separately. They are usually the same file:

```bash
simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json route_test.py --drone-map drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
```

To test a drone whose map is wrong, give a different world. [`../../course_map_tools/make_boulder_surprise_worlds.py`](../../course_map_tools/make_boulder_surprise_worlds.py) makes worlds where the boulders have been moved, added or removed, and prints the command for each.

Two more options choose how the route is planned:

- `--mission` picks the mission file. It defaults to `drone_code/team_nav/missions/dronehacks_2026_side_gap_first.json`, which goes through the side gap into the landing area, as the team chose for the first competition run. Give `drone_code/team_nav/missions/dronehacks_2026.json` to try the window first.
- `--also-avoid-boulders-from` names other maps whose boulders the drone's stops should also keep away from. It defaults to the organizers' drawing. Give the option with no files to use only the drone's map.

While following the route, the test lands at once, where the drone is, if the referee reports a touch, if the drone tilts or turns as if it hit something, or if PX4's height and the bottom laser's disagree over clear floor. `route_test.py`'s header explains each.

## Testing a real drone program

`real_program_test.py` doesn't fly anything itself. It starts a program from `drone_code`, exactly as the drone would run it, waits until that program is sending targets, and then calls the simulator's start service in place of the teammate who arms the drone and flips the switch to Position. It passes if the program ends with exit code 0 (or the code given with `--expected-exit-code`). Give the program's command after `--`, and any environment settings with `--env`:

```bash
simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json real_program_test.py -- /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/course_run
```

```bash
simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json real_program_test.py --env TEAM_NAV_RUN_MODE=practice --env TEAM_NAV_STOP_AFTER_STAGE=climb -- /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/course_run
```

The long path is where the box installs the program. Once `drone_code/team_nav` has a `setup.cfg` that puts its programs in `lib/team_nav`, `ros2 run team_nav course_run` works too. The program's own records go to `simulator/generated_files/team_runs/`, and the test prints which folder and files it wrote. The program sees the real flight_interface, PX4 and the referee, but not the referee's touch messages, so a touch it can't notice by itself shows up only in the referee's summary.

The practice flights (`drone_code/team_nav/team_nav/practice_flights/`, whose README has the results) run the same way. Out and back, in the default world:

```bash
simulator/run_test_flight.sh real_program_test.py --env TEAM_NAV_RUN_MODE=practice -- /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/practice_flight out_and_back --map organizers_drawing.json
```

The mock gap and window, in a world made for the referee first:

```bash
python3 simulator/test_flights/make_world_for_the_referee.py drone_code/team_nav/course_maps/practice_mock_gap_and_window.json
simulator/run_test_flight.sh --world simulator/generated_files/practice_worlds/practice_mock_gap_and_window.json real_program_test.py --env TEAM_NAV_RUN_MODE=practice -- /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/practice_flight mock_gap_and_window --ignore-raw-laser
```

## The fallback missions

`course_run` flies a fallback route high over the walls when `TEAM_NAV_MISSION` names one
(`drone_code/team_nav/team_nav/fallback_run.py`). Fly them as the real drone works, without the raw
laser reading (`--ignore-raw-laser`). The referee notes "went above the wall tops" each time, as it
should: these routes take that penalty on purpose.

```bash
simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json real_program_test.py --env TEAM_NAV_MISSION=fallback_tunnel -- /repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/course_run --ignore-raw-laser
python3 simulator/test_flights/fallback_flight_report.py --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json
```

For a changed maze, make the worlds first and give one as `--world` (the drone keeps the scan as its map):

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 simulator/test_flights/make_fallback_test_worlds.py
```

## What makes a flight fail

`run_test_flight.sh` says TEST FAILED, and stops with an error, if:

- the flight program itself gave up, for example because the drone never reached a target;
- the flight program stopped the run early, after a touch or because PX4's height stopped matching the laser;
- the drone touched anything;
- the drone stayed above a boulder for more than 2 seconds;
- PX4's height estimate strayed more than 10 cm from the true height in flight, or more than 20 cm during the takeoff climb. Time on the floor after touchdown is not counted.

The last two come from the referee's boulder watch (`simulator/ros_packages/course_simulator/course_simulator/boulder_watch.py`), which explains the limits.

## Safety

Each flight first waits for the simulator's start service, which only exists in the simulator. If the service doesn't appear, the flight stops with an error before it sends the drone anything. That is what protects the real drone if someone runs one of these files there by accident.
