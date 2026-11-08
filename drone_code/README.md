# Code that runs on drone 16

This folder holds all the code that runs on the drone. It also holds the scripts that copy the code
to the drone and copy the records back to the Mac. Simulator code is in `simulator/`. Code in this
folder must never import simulator code.

**To test the drone, use [`TEST_SEQUENCE.md`](TEST_SEQUENCE.md).** It gives each command in order.

## The two computers on the drone

- The **flight controller** (a Pixhawk board with PX4 1.16.2 software) controls the motors and
  keeps the drone level. It flies to the position that it gets. It also controls the arming and
  the failsafes.
- The **Jetson** is a small Linux computer. Our programs run on it. They tell the flight controller
  where to go.

## What runs on the Jetson

The organizers' software runs in a Docker container with the name `skybeta-stack`. It starts
when the Jetson starts. It has these parts:

```
skybeta-stack (the organizers' container)
├── camera tracking (oakd_vslam)      finds the position of the drone from the camera
├── px4_odom_bridge                   sends that position to the flight controller
├── the link to PX4                   PX4 reports appear as ROS topics /fmu/out/...
└── the "competitor" window           builds ~/team_ws/src, then starts our team_nav.launch.py
    ├── flight_interface              the organizers' program. Only it sends commands to PX4.
    └── flight_recorder               our program. It records all reports. It sends nothing.
```

The settings file `/etc/skybeta/drone.env` names `team_nav` and `team_nav.launch.py`. Our
`team_nav.launch.py` starts `watch_only`. Thus, after each restart, no program can fly the drone.

## The folders on the Jetson

| Folder | What it holds |
|---|---|
| `~/team_ws/src/team_nav` | Our main package (from `drone_code/team_nav`). |
| `~/team_ws/src/team_smoke_flights` | Our hover-and-land flight (from `drone_code/smoke_flights`). |
| `~/team_ws/drone.sh` | A link to our helper. Use it for all tests. |
| `~/team_ws/flight_records` | The records of all our programs. |
| `~/archive` | Old code that we moved away. Nothing runs it. |

On 2026-11-07 we moved Soham's `team_nav` from `~/team_ws/src/team_nav` to
`~/archive/20261107_cleanup/soham_team_nav`. The `README.txt` in that folder tells how to put it
back.

## The launch files

A launch file tells which programs start. Only one can run at a time.

| Launch file | Can a program fly the drone? | Use |
|---|---|---|
| `team_nav` | No. It starts `watch_only`. | The drone starts it after each restart. |
| `watch_only` | No. | Checks and hand tests. |
| `test_flights` | Yes, but only a program that you start by hand. | Practice flights. |
| `rise_and_land` | Yes. | The first hover. |
| `course_run` | Only after the team sets `HAND_OVER_ON = True` in the file. | The competition. |

All of them start our recorder.

## The hand-over

The organizers' `flight_interface` gives control to our program only after these three events,
in this sequence:

1. One of our programs sends a target position.
2. A teammate arms the drone on the remote control. "Arm" gives the motors permission to turn.
3. The teammate moves the mode switch to Position.

The teammate can take control back at any time. Move the switch out of Position, or use the kill
switch.

## Our programs (all in `team_nav`)

| Program | Can it fly? | What it does |
|---|---|---|
| `hand_test` | No | Checks with the propellers off. |
| `flight_recorder` | No | Records all ROS reports and the health of the Jetson. |
| `practice_flight` | Yes | Out-and-back, and the mock gap and window. |
| `course_run` | Yes | The competition program. In practice mode, it can stop after a stage. |

A program that flies sends only target positions. It never arms the drone. It never changes the
mode or the settings of PX4. It lands by itself after an error, a touch, a height problem or
Ctrl+C.

## Settings for our programs

Put a setting before the command. Example:
`TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight out_and_back`

| Setting | Values | Meaning |
|---|---|---|
| `TEAM_NAV_RUN_MODE` | `competition` (default) or `practice` | Practice mode records more data and can stop early. |
| `TEAM_NAV_STOP_AFTER_STAGE` | the name of a stage | Practice mode only. The drone lands after that stage. |
| `TEAM_NAV_RECORDS_FOLDER` | a folder | Where the records go. The default is `~/team_ws/flight_records`. |

## The files in this folder

| File or folder | What it is |
|---|---|
| [`TEST_SEQUENCE.md`](TEST_SEQUENCE.md) | The commands for each test, in order. |
| [`install_on_drone_16.sh`](install_on_drone_16.sh) | Mac. Copies our two packages to the drone and builds them. |
| [`collect_flight_records.sh`](collect_flight_records.sh) | Mac. Copies the records from the drone. [`COLLECTING_RECORDS.md`](COLLECTING_RECORDS.md) explains it. |
| [`sync_telemetry.sh`](sync_telemetry.sh) | Mac. Copies the records, packs them into `telemetry/` for Git, and can commit and push them. |
| [`team_nav/`](team_nav/README.md) | Our main package. `team_nav/drone.sh` is the helper. |
| [`smoke_flights/`](smoke_flights/README.md) | The hover-and-land flight. |
| [`hardware_checks/`](hardware_checks/README.md) | A quick check of the drone from the Mac, over the USB cable. |

## Log in

The name of the drone is `skybeta-16.local`. It operates on the venue's "DroneHacks Wifi" and over
the USB cable. Over the cable, the address is also `192.168.55.1`. Type the password yourself. Do
not write it in a file.
