# Testing sequences

This file gives each command to test the drone, in order. It starts with simple checks. It ends with the competition run.

Each program writes its records to `~/team_ws/flight_records/` on the drone. Each run gets a new folder. Our recorder runs all the time. Step 12 copies the records to the Mac.

## Definitions

- **Arm**: give the motors permission to turn. Only a teammate does this, on the remote control.
- **The hand-over**: the organizers' program gives control to our program. This occurs only when three events occur in this sequence. 1. Our program sends a target. 2. A teammate arms the drone. 3. The teammate moves the mode switch to Position.
- **Take control back**: move the mode switch out of Position, or use the kill switch.
- **The Jetson**: the computer on the drone.
- **PX4**: the software on the flight controller.



## Before each session

- One teammate holds the remote control and the kill switch.
- Nobody arms the drone without the approval of the team.
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

## Step 3. Table checks (not ready to fly)

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

## Step 5. Rehearse the hand-over (propellers OFF, team approval)

This step proves that the hand-over operates before the drone can fly. The motors will turn for a
short time. Make sure that the propellers are off. Make sure that all persons are clear.

```bash
./drone.sh start test_flights
```

Type `yes` when it asks. Then type:

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight out_and_back
```

1. Wait until the program says that it waits for the switch.
2. The teammate arms the drone.
3. The teammate moves the switch to Position.
4. Make sure that the program says that it has control.
5. The teammate moves the switch out of Position and disarms the drone.
6. Push Ctrl+C to stop the program.



## Step 6. First flight: hover and land (propellers ON, team approval)

```bash
./drone.sh start rise_and_land
```

Type `yes`. The teammate arms the drone and moves the switch to Position. The drone goes up to a
low height. It stays there for a short time. Then it lands by itself.

The result is correct if the drone lands on the mark and the motors stop by themselves. Then type:

```bash
./drone.sh start watch_only
```



## Step 7. Out and back (team approval)

This flight measures how fast the drone follows a target.

```bash
./drone.sh start test_flights
```

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight out_and_back
```

The teammate arms the drone and moves the switch to Position when the program asks. The drone
goes up to 0.5 m. It flies 1 m forward and back. Then it lands. It needs 1 m of clear floor in
front of the launch mark.

The result is correct if the drone lands where it started. The program shows the delay. We
expect approximately 1.3 s on the drone. The simulator gave 1.7 s.

## Step 8. The mock gap and window (team approval)

First, build the practice course in the net. Measure it. Correct
`team_nav/course_maps/practice_mock_gap_and_window.json` on the Mac. The depth of the walls is
important. Then do step 0 and step 2 again.

With `test_flights` started (step 7), type:

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight mock_gap_and_window
```

The result is correct if the drone goes through the gap and the window, lands on the pad, and
touches nothing.

Watch the height after the window. In the simulator, the box under the window caused an error of
approximately 13 cm in the PX4 height. The competition route uses the side gap to prevent this
problem.

## Step 9. The course, one stage at a time (team approval)

With `test_flights` started, do each run below. The drone flies to the end of the stage that you
name. Then it lands. Put the drone back on the launch mark before each run.

```bash
TEAM_NAV_RUN_MODE=practice TEAM_NAV_STOP_AFTER_STAGE=climb ./drone.sh course_run
```

```bash
TEAM_NAV_RUN_MODE=practice TEAM_NAV_STOP_AFTER_STAGE="first corridor" ./drone.sh course_run
```

```bash
TEAM_NAV_RUN_MODE=practice TEAM_NAV_STOP_AFTER_STAGE=maze ./drone.sh course_run
```

```bash
TEAM_NAV_RUN_MODE=practice TEAM_NAV_STOP_AFTER_STAGE=tunnel ./drone.sh course_run
```

```bash
TEAM_NAV_RUN_MODE=practice TEAM_NAV_STOP_AFTER_STAGE=chicane ./drone.sh course_run
```

Then the full course, in practice mode:

```bash
TEAM_NAV_RUN_MODE=practice ./drone.sh course_run
```

The route goes through the side gap, not the window. `docs/HANDOFF_boulders_and_height.md` tells
why. The program ends with one of these exit codes: 0 finished, 1 stopped or failed, 2 did not
start, 130 Ctrl+C.

## Step 10. The competition run (team approval)

The competition launch file keeps the hand-over off. The team must change one line to switch it on.

1. On the Mac, open `drone_code/team_nav/launch/course_run.launch.py`.
2. Change `HAND_OVER_ON = False` to `HAND_OVER_ON = True`.
3. Do step 0.
4. On the Jetson, type:

```bash
./drone.sh start course_run
```

The course program starts in competition mode. The teammate arms the drone and moves the switch
to Position when the referee says go.

After the competition, change the line back to `False` on the Mac. Then do step 0 and step 2.

If the drone software restarts, the drone starts `watch_only`. Then type
`./drone.sh start course_run` again.

## Step 11. Make the drone safe after each flight

```bash
./drone.sh start watch_only
```



## Step 12. Copy the records to the Mac (on the Mac)

Type this at the top of the repository:

```bash
drone_code/collect_flight_records.sh
```

The records go to `flight_records_from_drone/skybeta-16/`. Git ignores that folder. The flight
controller also keeps its own log on its memory card. Get it with QGroundControl (Analyze Tools,
Log Download). `COLLECTING_RECORDS.md` explains both.

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

- **In the air**: the teammate moves the switch out of Position or uses the kill switch. Our
programs also land by themselves after an error, a touch or Ctrl+C.
- `./drone.sh start` **says that flight_interface did not stop**: attach to the window (above) and
look. If necessary, restart the drone software with `docker rm -f skybeta-stack`. It starts
again in approximately 40 s, with `watch_only`.
- `team_nav` **is not found**: do step 0 again.

