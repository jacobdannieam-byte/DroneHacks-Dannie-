# Test flights

Each file here is a scripted flight that runs in the simulator. These flights test the simulator and individual ideas, such as the landing. They are not the competition program, and they must not run on the real drone.

Read them in this order:

| File | What it does |
|---|---|
| `simulator_test_flight.py` | The steps every flight shares: waiting for the simulator, starting, climbing, landing, and sending targets without a break. |
| `landing_test.py` | Climbs to 0.8 m, flies 1 m forward, and lands. Its results are in section 7 of [`../../docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md). |
| `route_test.py` | Plans a route from the drone's map, flies the whole course and lands on the pad. |

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

## What makes a flight fail

`run_test_flight.sh` says TEST FAILED, and stops with an error, if:

- the flight program itself gave up, for example because the drone never reached a target;
- the flight program stopped the run early, after a touch or because PX4's height stopped matching the laser;
- the drone touched anything;
- the drone stayed above a boulder for more than 2 seconds;
- PX4's height estimate strayed more than 10 cm from the true height.

The last two come from the referee's boulder watch (`simulator/ros_packages/course_simulator/course_simulator/boulder_watch.py`), which explains the limits.

## Safety

Each flight first waits for the simulator's start service, which only exists in the simulator. If the service doesn't appear, the flight stops with an error before it sends the drone anything. That is what protects the real drone if someone runs one of these files there by accident.
