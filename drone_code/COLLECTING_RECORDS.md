# Collecting flight records

There are two kinds of record of a flight, and both are worth keeping:

1. **Our records**, written on the Jetson by our own programs: everything the drone's software said on ROS during the flight, plus how busy the Jetson was. These are CSV and JSON files, easy to read with Python.
2. **PX4's own flight log**, written by the flight controller (the Pixhawk) on its memory card. It holds PX4's internal state hundreds of times a second, including things that never reach ROS, such as the motor outputs. It is one "ULog" file (`.ulg`) per flight.

Neither is used by the drone in a later run. Rule 6.4 forbids feeding data from one run into another; these records are only for us to study and tune by hand.

Nothing on this page has been tried on drone 16 yet. The recorder has only run in our simulator.

## 1. Our records

### What the recorder writes

`flight_recorder` is a separate program (`team_nav/team_nav/flight_recorder.py`). It listens to every ROS topic it can find and writes as it goes, so a crash of the flight program, or of the recorder itself, or a power cut, loses at most about half a second. It has no way to send anything, so it cannot move the drone. Its file header lists exactly what it writes; in short, each run gets a folder such as

```
~/team_ws/flight_records/20261108_101502_competition_flight_recorder/
    events.jsonl          what the recorder decided and noticed, one JSON line each
    session.json          computer, ROS settings, and a fingerprint of the installed code
    topics_seen.json      every topic seen, and how it was recorded
    computer_health.csv   processor, memory, temperatures, free disk, once a second
    topics/*.csv          one file per topic, one row per recorded message
    pictures/...          practice mode only: about one camera picture a second per camera
```

The flight programs write their own folders next to it (`run_log.py`), for example `..._competition_course_run/`.

In **competition mode** most topics are limited to 30 rows a second, and camera pictures are not saved. In **practice mode** every message is recorded, and one picture a second from each camera (`/oak/left/image_raw` and `/oak/right/image_raw`) is saved as a PNG file. A few rare and important topics, such as PX4's status and every program's log messages (`/rosout`), are recorded in full in both modes. The mode comes from `TEAM_NAV_RUN_MODE` (`run_mode.py`); if it is missing, the mode is competition.

In our simulator, competition mode used about 12% of one core of a Mac and 7 MB of disk a minute; practice mode about 27% and up to about 270 MB a minute with pictures. The Jetson's cores are slower, so expect two to three times the processor use there. The recorder writes its own use into `events.jsonl` every 30 seconds ("status" lines), so check those after the first flight.

If the disk gets below 5 GB free the recorder records less, and below 1 GB only a trickle. It never deletes anything.

### Starting it on the drone

The organizers' software runs inside a Docker container called `skybeta-stack`, and our package is built there. Every command below is typed on the Mac. The password is the shared drone password; ssh asks for it, and nobody should save it in a file.

First make sure you are talking to drone 16 (the address is the same for every drone):

```bash
ssh dronehacks@192.168.55.1 hostname     # must print skybeta-16
```

Start the recorder in the background, in practice mode, before the flight program:

```bash
ssh dronehacks@192.168.55.1 "docker exec -d skybeta-stack bash -lc 'source skybeta_env.sh && TEAM_NAV_RUN_MODE=practice ros2 run team_nav flight_recorder'"
```

For competition runs, leave out `TEAM_NAV_RUN_MODE=practice`. Normally you don't start it by hand at all: all three of our launch files (`watch_only`, `test_flights`, `course_run`) start the recorder with the drone's other software, and restart it if it crashes.

Stopping it is optional, since everything is already on disk. To stop it cleanly:

```bash
ssh dronehacks@192.168.55.1 "docker exec skybeta-stack pkill -INT -f lib/team_nav/flight_recorder"
```

We have not checked that `skybeta_env.sh` loads our package's build in that shell; if `ros2 run` says it can't find `team_nav`, ask whoever built the package which setup file to `source`.

### Check that the records will survive a restart

The container has its own private filesystem, which is deleted whenever the container is removed (for example by `docker rm -f skybeta-stack`, which `drone_code/README.md` uses to restart the software). Only folders it borrows from the drone's real disk survive. The recorder checks this as it starts, and its third event says the answer:

```bash
ssh dronehacks@192.168.55.1 "docker exec skybeta-stack bash -lc 'grep -h storage ~/team_ws/flight_records/*_flight_recorder*/events.jsonl | tail -1'"
```

It should say the folder is "on a real disk". If it says "WARNING ... LOST", copy the records off with `--from-inside-container` (below) before restarting anything, and then find out which folder the container borrows: `collect_flight_records.sh` prints the list, and `TEAM_NAV_RECORDS_FOLDER` can point the recorder at a folder inside it.

### Copying them to the Mac

```bash
drone_code/collect_flight_records.sh
```

It checks that the drone is skybeta-16 and refuses otherwise, then copies `~/team_ws/flight_records` from the drone into `flight_records_from_drone/skybeta-16/` at the top of the repository. Git ignores that folder. Running it again copies only what is new. It never deletes anything, on the drone or on the Mac. If the records are only inside the container:

```bash
drone_code/collect_flight_records.sh --from-inside-container
```

### Reading them

Every table is a plain CSV file. For example, PX4's position estimate during a flight:

```python
import pandas
position = pandas.read_csv("flight_records_from_drone/skybeta-16/<run folder>/topics/fmu__out__vehicle_local_position.csv")
print(position[["seconds_since_start", "x", "y", "z", "dist_bottom"]])
```

Each row has `seconds_since_start` (counted from when the recorder started), `received_unix_time_seconds` and `sent_unix_time_seconds` (the Jetson's clock), and then the message's own fields. Nested fields are joined with dots, such as `pose.pose.position.x`, and lists are either one column per value (`q[0]` to `q[3]`) or one column of JSON text.

The Jetson's clock may be wrong if it has not had network time since it was switched on, so folder names can show a strange date. `session.json` also records how long the computer had been on and a fingerprint of the installed code, which tell runs apart reliably.

## 2. PX4's own flight logs

### Where they are

PX4's logger writes ULog files to the microSD card in the Pixhawk 6C, in the card's `log/` folder (PX4 calls it `/fs/microsd/log`), one subfolder per day, or `sess001`, `sess002` and so on when PX4 does not know the date. With PX4's default setting, a log starts when the drone is armed and ends when it is disarmed, so each flight is one file.

Which PX4 logging settings drone 16 uses (the `SDLOG_*` settings) we have not checked. **Do not change them.** Rule 8.5 and our team's rules say we don't change PX4's settings. Looking at them is fine.

### How to get them

The Jetson cannot fetch them. It talks to PX4 through the XRCE-DDS link, which carries PX4's live messages to ROS, not files, and the link's other kind of traffic (MAVLink, which can carry logs) is switched off on that port (the organizers' check lists `MAV_1_CONFIG=0`). Switching it on would mean changing PX4 settings. So the logs come off the Pixhawk itself, in one of two ways. Both only read.

**A. QGroundControl over the Pixhawk's USB port (the easiest).** Do this only on the ground with the drone disarmed, never during a run (rule 5.5 allows a laptop to watch, read-only; after the run is over, it doesn't matter).

1. Install QGroundControl on the Mac from qgroundcontrol.com.
2. Connect a USB cable from the Mac to the Pixhawk's own USB-C port (not the Jetson's). QGroundControl connects by itself.
3. Open **Analyze Tools** (the Q menu at the top left), then **Log Download**, and press **Refresh**. Each line is one flight, with its date (or the time since PX4 started) and size.
4. Tick the flights you want and press **Download**. Choose a folder outside the repository, or in `flight_records_from_drone/`, which git ignores.

Things not to press: **Erase All** in the same window deletes every log on the card, and if QGroundControl offers to update the Pixhawk's firmware or change any setting, say no. Downloading only reads.

**B. The memory card.** With the drone switched off, take the microSD card out of the Pixhawk, read its `log/` folder on the Mac with a card reader, and put it back firmly. This is quick for many logs, but the card is small and easy to lose or to leave half-inserted, so do it only when the drone is not about to fly, and check it boots normally afterwards.

A command-line way also exists (MAVProxy's `log list` and `log download` commands over the same USB port), but we have not tried it, so we don't recommend it for the competition day. We have not written a script for PX4 logs, because we could not test one without the drone.

### Reading them

- **pyulog**, a Python package (`pip install pyulog`): `ulog_info file.ulg` lists what is inside, and `ulog2csv file.ulg` turns every message type into CSV files.
- **PlotJuggler**, a desktop program, opens `.ulg` files directly for plotting.
- **PX4 Flight Review** (logs.px4.io) makes a full report from a log in the browser. An upload is stored on their server, so choose the option that keeps it private, or use the two tools above instead.

PX4's messages in our CSV files carry PX4's own `timestamp` column (microseconds since PX4 started), as PX4's log does, which should let the two be lined up. We have not checked that the two clocks agree on drone 16; if they don't, line them up by a clear moment such as arming.

## What has not been tested

- The recorder has never run on drone 16, only in our simulator on a Mac. Its processor use on the Jetson, which has slower cores, is estimated, not measured.
- Starting it with `ros2 run` inside `skybeta-stack`, and whether the records folder there is really on the borrowed disk.
- `collect_flight_records.sh` has only been tried with a pretend `ssh` that never connects anywhere: it refused a drone with the wrong name and stopped when the folder was missing.
- Getting PX4's logs off drone 16 in any of the ways above.
