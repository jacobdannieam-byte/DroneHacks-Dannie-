# Testing sequences

This file gives each command to test the drone, in order. It starts with simple checks. It ends with the competition run.

Each program writes its records to `~/team_ws/flight_records/` on the drone. Each run gets a new folder. Our recorder runs all the time. Step 11 copies the records to the Mac.

## Definitions

- **The remote control**: a handheld controller with sticks and switches. An organizer holds it, and does the two steps below on it.
- **Arm**: give the motors permission to turn. Nothing flies yet.
- **The hand-over**: the organizers' program gives control to our program. This occurs only when three events occur in this sequence. 1. Our program sends a target. 2. The organizer arms the drone. 3. The organizer moves the mode switch to Position.
- **Position**: one setting of the mode switch on the remote control. Moving the switch to Position, after the drone is armed, gives control to our program. The switch must not already be on Position when the drone is armed.
- **Take control back**: the organizer moves the mode switch out of Position, or uses the kill switch.
- **The Jetson**: the computer on the drone.
- **PX4**: the software on the flight controller.



## Before each session

- Keep the propellers off until step 6.
- Before a flight, put the drone in the net, on the launch mark. Point the nose down the first corridor.

---



## Step 0. Copy the code to the drone (not ready to fly)

Do this step only after you change the code. Type this at the top of the repository:

```bash
drone_code/install_on_drone_16.sh
```

Result: the last line tells you to type `./drone.sh start watch_only` on the drone.

## Step 1. Log in and look (not ready to fly)

On the Mac, type:

```bash
ssh dronehacks@skybeta-16.local
```

Type the drone password. Then, on the Jetson, type:

```bash
cd ~/team_ws
```

```bash
./drone.sh status
```

Make sure that the status shows `PX4 connected: True` and `armed: False`. 

If PX4 is not connected, connect the battery of the flight controller. 

The Jetson alone does not supply power to the flight controller.

For a second check, use the organizers' health check:

```bash
~/skybeta/host/doctor.sh
```



## Step 2. Make sure that nothing can fly (not ready to fly)

```bash
./drone.sh start watch_only
```

Make sure that the status shows `ros2 launch team_nav watch_only.launch.py` and the hand-over
value `False`.

After a restart, the drone starts `watch_only` by itself. Do this step after step 0 too, because
programs that run keep the old code.

## Step 3. Unarmed checks (not ready to fly)

Put the drone on a table. Type:

```bash
./drone.sh hand_test all_table_checks
```

It takes approximately one minute. It shows PASS or FAIL for four checks:


| Check                  | It proves that                                            | If it fails                                                                                        |
| ---------------------- | --------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `report_rates`         | each report arrives often enough                          | The screen shows the slow report.                                                                  |
| `estimator_sources`    | PX4 uses the laser for height and the camera for position | On 2026-11-07, PX4 did not use the laser on the floor. Thus a FAIL is possible. Step 4 tells more. |
| `interface_settings`   | the hand-over is off                                      | Do step 2 again.                                                                                   |
| `tracking_still_drift` | camera tracking does not move for 30 s                    | Make sure that there is sufficient light and that the camera sees detail.                          |




## Step 4. Hand tests (not ready to fly)

Each test tells you what to do. Then it waits for Enter. Do one test at a time:

```bash
./drone.sh hand_test laser_heights
```

```bash
./drone.sh hand_test directions
```

```bash
./drone.sh hand_test touch_tilt
```

```bash
./drone.sh hand_test tunnel_carry
```

```bash
./drone.sh hand_test laser_over_step
```


| Test              | You need                           | It proves that                                                                         |
| ----------------- | ---------------------------------- | -------------------------------------------------------------------------------------- |
| `laser_heights`   | a tape measure                     | the laser height agrees with the tape (within 5 cm)                                    |
| `directions`      | tape marks A, B and C on the floor | forward, left and a left turn have the correct direction                               |
| `touch_tilt`      | nothing                            | our touch detector does not alarm when the drone is still, and alarms past 25° of tilt |
| `tunnel_carry`    | the course                         | camera tracking continues through the tunnel and back                                  |
| `laser_over_step` | a box                              | the width of the laser beam                                                            |


`team_nav/team_nav/hand_tests/README.md` explains each test. It also tells what to do after a FAIL.

**Stop here if a test in step 3 or step 4 failed.** The one exception is `estimator_sources`.

## Step 5. Handover procedure example (not ready to fly)

This step proves that the hand-over works before the drone can fly. Make sure that the propellers are off and that everyone is clear. Once our program has control, it tells the motors to climb, so they speed up. That is why point 5 below moves the switch out of Position straight away.

```bash
./drone.sh start test_flights
```

Then type:

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight out_and_back
```

1. Wait until the program says that it waits for the switch.
2. The organizer arms the drone.
3. The organizer moves the switch to Position.
4. Make sure that the program says that it has control.
5. The organizer moves the switch out of Position and disarms the drone.
6. Push Ctrl+C to stop the program.



## Step 6. First flight - hover and land (ready to fly test)

```bash
./drone.sh start rise_and_land
```

The organizer arms the drone and moves the switch to Position. The drone goes up to a low height. It stays there for a short time. Then it lands by itself.

The result is correct if the drone lands on the mark and the motors stop by themselves. Then type:

```bash
./drone.sh start watch_only
```



## Step 7. Out and back (ready to fly test)

This flight measures how fast the drone follows a target.

```bash
./drone.sh start test_flights
```

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight out_and_back
```

The organizer arms the drone and moves the switch to Position when the program asks. The drone goes up to 0.5 m. It flies 1 m forward and back. Then it lands. It needs 1 m of clear floor in front of the launch mark.

The result is correct if the drone lands where it started. The program shows the delay. We  
expect approximately 1.3 s on the drone. The simulator gave 1.7 s.

## Step 8. The full course in practice mode

course_run uses live navigation. The drone looks with its depth camera, keeps a live map, plans
its path again and again, and turns its nose to where it flies. The map file
(`sketch_2026-11-08.json`) is only a first guess. Read `team_nav/team_nav/live_flight.py` for how
it works.

What you will see:
- After the climb, the drone turns a full circle in four quarter turns. It stops at each quarter
  turn for 1 second. This is correct: it is looking around.
- Before it moves, it turns slowly (30 degrees a second) to face its path.
- It stops and holds when something is close ahead, or when no depth picture comes for 1.5 s.
  Then it plans again.
- If it finds no way forward for 20 s, it lands where it is.

Get the team's approval before the flight. Use Reset world if keyboard RC was used. Hold the
kill switch.

First start test_flights. It now also starts the depth program:

```bash
./drone.sh start test_flights
```

Then start the run:

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh course_run
```

The drone flies the whole course and lands on the pad. The route goes through the side gap, not
the window (`docs/HANDOFF_boulders_and_height.md` says why).

Before the first flight, check that the depth program runs: in the records folder, the newest
`*_depth_camera/depth_camera.csv` must grow by about 8 lines a second, with GPU in the last column.

After the flight, copy the records to the Mac (Step 11). The folder `live_maps` holds what the drone
saw and planned every 2 seconds. To compare with the old way of flying (follow the map route, never
turn), add `--navigation map_route`.

The result is correct if the drone lands on the pad and touches nothing. When the program ends, it shows a number: 0 finished, 1 stopped or failed, 2 did not start, 130 Ctrl+C.

## Step 9. The competition run (fly test)

The competition launch file keeps the hand-over off. Change one line to switch it on.

1. On the Mac, open `drone_code/team_nav/launch/course_run.launch.py`.
2. Change `HAND_OVER_ON = False` to `HAND_OVER_ON = True`.
3. Do step 0.
4. On the Jetson, type:

```bash
./drone.sh start course_run
```

The course program starts in competition mode. The organizer arms the drone and moves the switch to Position when the referee says go.

After the competition, change the line back to `False` on the Mac. Then do step 0 and step 2.

If the drone software restarts, the drone starts `watch_only`. Then type
`./drone.sh start course_run` again.

## Step 10. Make the drone safe after each flight (fly test)

```bash
./drone.sh start watch_only
```



## Step 11. Copy the records to the Mac (after fly test)

Type this at the top of the repository:

```bash
drone_code/sync_telemetry.sh --push
```

The script copies the records to `flight_records_from_drone/skybeta-16/`. Git ignores that folder. Then it packs the records without the camera pictures into `telemetry/skybeta-16/`. It commits only `telemetry/` and pushes it. `telemetry/README.md` explains the folder. To copy only, without a commit, use `drone_code/collect_flight_records.sh`. The flight controller also keeps its own log on its memory card. Get it with QGroundControl (Analyze Tools, Log Download). `COLLECTING_RECORDS.md` explains both.

---



## Watch a launch file

The launch files run in a window with the name `team_launch`. To see it, type on the Jetson:

```bash
docker exec -it skybeta-stack tmux attach -t skybeta
```

- Push Ctrl+B, then W. Select the `team_launch` window.
- To leave without a stop, push Ctrl+B, then D.
- Do not push Ctrl+C in that window. Ctrl+C stops the launch file.



## If a problem occurs

- `./drone.sh start` **says that flight_interface did not stop**: attach to the window (above) and  
look. If necessary, restart the drone software with `docker rm -f skybeta-stack`. It starts  
again in approximately 40 s, with `watch_only`.

