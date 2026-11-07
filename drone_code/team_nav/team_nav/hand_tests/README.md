# Hand tests and table checks (propellers off)

These programs check drone 16's hardware piece by piece, with the **propellers off**, before
any flight. They are steps 1 and 3 to 5 of the test order in section 11 of
[`docs/ARCHITECTURE.md`](../../../../docs/ARCHITECTURE.md).

**They only listen.** Nothing here can arm the drone (give its motors permission to spin), spin
a motor, send a target or change a setting. `test/test_hand_tests_are_read_only.py` scans this
folder and fails if anything that sends ever appears.

Each test prints **PASS** or **FAIL** with one sentence saying why, and saves its readings and
its result (`result.json`) in its own folder in the records folder, named
`..._hand_test_<name>`. `run_log.py` says where the records folder is.

## First: the quick first check

Run the quick first check in [`drone_code/hardware_checks/`](../../../hardware_checks/README.md)
before these. It runs from the Mac over USB, confirms the drone is drone 16, and shows whether
the flight controller, the laser, the cameras and camera tracking send anything at all, the ROS
network number (`ROS_DOMAIN_ID`), whether the organizers' demo is still running, and the free
disk space. The checks here go further, using the same topic names and settings as our flight
code.

On 2026-11-07, at first the flight controller sent nothing: its topics existed but were silent,
most likely because only the Jetson had power. **Connect the drone's battery** (propellers off)
so the flight controller and the laser are powered. If PX4 is silent, every test here stops at
once with: "No PX4 report for N s: is the flight controller powered (battery connected)?"

A later check that day found what drone 16 does and doesn't send, and the tests are built
around it:
- There is **no raw laser reading** (`/fmu/out/distance_sensor` doesn't exist). So the laser
  tests read PX4's `dist_bottom` (its distance to the ground below) and its estimator flags
  `cs_rng_hgt` (laser used for height) and `cs_rng_terrain` (laser used to measure the ground).
  When neither flag is set, `dist_bottom` comes from the barometer and says nothing about the
  laser. If the raw reading ever appears, the tests use it too.
- **On the floor, PX4 was not using the laser** (`cs_rng_hgt` false). `laser_heights` finds out
  whether, and from what height, it does.
- PX4's attitude comes as `vehicle_attitude` (about 100 a second). Our flight code reads
  `vehicle_odometry` instead (`drone_reports.py`); the tests use `vehicle_odometry` if it
  arrives, and otherwise `vehicle_attitude` with `sensor_combined`'s rotation speed.

## How to run them

On the drone, inside the organizers' software container, where ROS runs:

```bash
docker exec -it skybeta-stack bash     # -it: the tests ask you to press Enter
source skybeta_env.sh                  # as the quick first check does
ros2 run team_nav hand_test            # lists the tests
ros2 run team_nav hand_test all_table_checks
ros2 run team_nav hand_test laser_heights
```

We have not yet run these on the drone. If `ros2 run` says the package `team_nav` is not found,
the container's environment doesn't include the team workspace yet; source the team
workspace's `install/setup.bash` inside the container. Press Ctrl+C at any time to stop: the test
is saved as FAIL, "stopped by the person".

## The order to run them

| # | Name | Where | What it proves | Pass rule |
|---|---|---|---|---|
| 1 | `report_rates` | table | Every report the flight code needs arrives often enough, using our flight code's topic names. | Over 10 s, each needed report arrives at least at its lowest rate (`report_rates.py` lists each, with its source) and the last one is under 1.5 s old. The raw laser, the battery and the camera pictures are listed but not required. |
| 2 | `estimator_sources` | table | PX4's estimator uses the laser for height and camera tracking for position and heading, as drone 16's settings `EKF2_HGT_REF = 2` and `EKF2_EV_CTRL = 9` should make it. | Nine estimator flags have the expected values. Expect a FAIL on `cs_rng_hgt` on the real drone, as seen on 2026-11-07; `laser_heights` then says more. |
| 3 | `interface_settings` | table | The organizers' interface won't hand control to software, and their demo isn't running. | `auto_offboard_on_arm` and `allow_mode_request` both read false; no `square_mission` program. |
| 4 | `tracking_still_drift` | table | Camera tracking stays put while the drone sits still. Point the camera at a detailed scene. | Over 30 s: never lost, at least 15 reports a second, moves under 2 cm and turns under 1 degree. |
| 5 | `laser_heights` | hand | Whether the bottom laser works at all, and from what height PX4 uses it. Resting on the floor, then held at 0.10, 0.20, 0.30, 0.50, 0.80 and 1.20 m, it prints at each height whether PX4 is using the laser and what `dist_bottom` says against the tape. | PX4 used the laser at one or more held heights (or the raw reading arrived), and wherever it was in use the height matched the tape within 5 cm, with the drone tilted under 10 degrees. `dist_bottom` is measured from the drone's centre, 3.4 cm above the laser (`EKF2_RNG_POS_Z`); the test allows for that. |
| 6 | `directions` | hand | 1 m forward, 1 m left and a 90 degree left turn are seen the right way round by camera tracking and by PX4. Needs three tape marks: A, B 1 m in front, C 1 m to the left of B. | Tracking moves within 15 cm of the tape; PX4's moves 1 m long within 15 cm and the left move 90 degrees anticlockwise of the forward one; the turn is +90 for tracking and -90 for PX4 within 10 degrees. |
| 7 | `touch_tilt` | hand | The touch detector (`touch_detector.py`) is silent when the drone is held still, and fires when it is tilted past 25 degrees. | No alarm in 5 s still; an alarm when tilted; at least 10 attitude reports a second. |
| 8 | `tunnel_carry` | hand | Camera tracking holds when the drone is carried through the tunnel and back, and measures its drift. | Never lost; no gap over 0.2 s; no jump over 10 cm; back on the start mark within 30 cm and 10 degrees. |
| 9 | `laser_over_step` | hand | How wide the laser's beam is, our biggest known risk (section 7). Needs a box 15 to 40 cm tall. Uses the raw laser if it arrives, otherwise PX4's `dist_bottom`, which only shows the box while PX4 uses the laser and is smoothed, so it gives an upper bound. | The floor and box top are both seen, the step matches the box within 5 cm, and an edge is crossed while tracking measures the speed. The result gives the beam's spot width and angle as a range, and whether PX4 used the laser during the walk. |

`all_table_checks` runs 1 to 4 in order. The hand tests tell the person exactly what to do at
each step, for example "Hold the drone so the bottom laser is 0.50 m above the floor (measure
with a tape), keep it level and still, then press Enter".

Every tolerance is our own choice, and each file says why next to it.

## What to do when one FAILS

The sentence after FAIL says what went wrong. Then:

- **"No PX4 report ..."**: connect the battery. If it is connected, the link between the Jetson
  and the flight controller is down; the quick first check prints the organizers' link advice.
- **`report_rates`**: a report "no program offers" means the program that sends it isn't
  running, or (for PX4 reports) the drone's PX4 doesn't export it. The bottom laser's raw
  reading `/fmu/out/distance_sensor` is not in standard PX4 1.16 (section 12); without it the
  landing and the height check need another plan. A slow report means the Jetson is overloaded
  or the link is poor.
- **`estimator_sources`**: the flag named in the sentence shows which setting or sensor is wrong.
  Check that setting in QGroundControl, and that camera tracking and the laser are running.
- **`interface_settings`**: start `launch/watch_only.launch.py` (see `drone_code/README.md`). In
  `test_flights.launch.py` the hand-over is on, so this check fails there on purpose; it is for
  propellers-off work.
- **`tracking_still_drift`, `tunnel_carry`**: check the lighting and that the camera sees detail,
  not a blank wall or a person. Look at `tracking.csv` to see where it went wrong. Tell the
  team: our plan depends on camera tracking (section 5).
- **`laser_heights`**: if PX4 never used the laser, open QGroundControl's MAVLink Inspector and
  look at `DISTANCE_SENSOR`: no messages means the laser or its wiring or driver is not working;
  messages with sensible distances mean PX4 is refusing them (check `EKF2_RNG_CTRL`,
  `EKF2_HGT_REF`, `EKF2_MIN_RNG`, and whether `cs_rng_fault` was set). Our landing and height
  check depend on the laser, so tell the team at once. If PX4 used it but the height is off,
  check the lens is clean and nothing is under the drone; an error that is the same at every
  height is an offset to record.
- **`directions`**: do not fly. A swapped or reversed direction must be understood first.
- **`touch_tilt`**: if it fires while still, the attitude report is noisy or wrong; if it never
  fires, the attitude report is wrong. Either way the touch check can't be trusted.
- **`laser_over_step`**: a FAIL here means the measurement didn't work; follow the sentence and
  repeat it.

## What these tests can't check

- **PX4's reaction to a laser jump.** PX4 only runs the check that drops the laser while it
  believes the drone is flying (`range_height_control.cpp` in PX4 1.16.2 updates it only when
  "in air"), and a disarmed drone never is. So `laser_over_step` measures the beam that feeds the
  check, and records PX4's flag `cs_rng_hgt`, but only a practice flight over a mock window can
  show what PX4 does.
- **PX4 settings that don't change a flag.** PX4's settings can't be read through the ROS link.
  Check these in QGroundControl and compare with drone 16's settings file of 2026-11-03 (copied
  in `simulator/px4_changes/drone_16_settings.sh`): `EKF2_RNG_K_GATE` (section 7), `EKF2_MIN_RNG`,
  `EKF2_RNG_NOISE`, `EKF2_RNG_A_VMAX`, `EKF2_RNG_A_HMAX`, `EKF2_EV_DELAY`, the landing settings
  (`LNDMC_*`, `COM_DISARM_LAND`, `COM_DISARM_PRFLT`, `MPC_THR_HOVER`), the safety settings
  (failsafes, kill switch, geofence and battery limits, which rule 8.5 protects),
  `UXRCE_DDS_DOM_ID` (must equal the drone's ROS network number, 16) and `UXRCE_DDS_CFG`.
- **Camera calibration.** The lens correction values were all zero on 2026-11-07. Whether the
  pictures are already corrected can't be told from software alone; ask the organizers, or look
  at a picture of a straight edge near a corner of the picture.
- **The ROS network number's uniqueness.** The quick first check prints it (16 on 2026-11-07,
  matching the organizers' drone number). Only the organizers can confirm no other drone uses 16.
- **The motors.** Nothing here spins them. A motor test needs the team and the organizers' rules.

## Files, in reading order

1. `run_hand_test.py`: lists the tests and runs the one you name.
2. `hand_test_tools.py`: the verdict, talking to the person, printing and saving.
3. `drone_listener.py`: listens to the drone in the background (the only file that needs ROS).
4. One file per test, as in the table above. Each holds its instructions and its pass rule; the
   pass rule is a plain function, tested with made-up readings in
   `test/test_hand_tests_decisions.py`.

`simulator/test_flights/hand_tests_in_simulator.py` runs them all in our simulator, on an open
floor, with a pretend person who flies the drone instead of carrying it. Its header gives the
two commands.
