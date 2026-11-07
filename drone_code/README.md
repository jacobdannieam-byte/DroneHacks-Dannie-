# Code that runs on the real drone

Everything in `team_nav/` is copied to drone 16 and runs there. Simulator code lives in
`simulator/` and must never be imported from here. The scripts in this folder run on the Mac.

## What is here

`team_nav` is our team's ROS package. It replaces the organizers' demo, which would climb to 1.5 m
and fly a 5 m square into the course walls as soon as someone armed the drone and flipped the
switch to Position. Its launch file starts exactly one copy of the organizers' flight interface
with the hand-over to software switched on, our mission, and our helper programs. Nothing moves
until the pilot arms the drone and flips the switch to Position. See
[`team_nav/README.md`](team_nav/README.md).

The scripts, all run on the Mac. Each takes the drone's address as its first argument, or from
`DRONE_HOST`: `skybeta-16.local` over the DroneHacks Wi-Fi, or `192.168.55.1` (the default) over
USB. Each first checks that the computer it reaches calls itself `skybeta-16`, because logging in
to another team's drone gets us disqualified.

| Script | What it does |
|---|---|
| [`install_on_drone_16.sh`](install_on_drone_16.sh) | Copies `team_nav` into `~/team_ws/src` on the drone. |
| [`restart_drone_16.sh`](restart_drone_16.sh) | Restarts the drone's software (the "stack"), waits for our mission to start, then runs the check. |
| [`check_drone_16.sh`](check_drone_16.sh) | Prints one line per check and ends with **READY TO FLY** or **NOT READY** and the reasons. It only reads. |
| [`drone_16_check_on_drone.sh`](drone_16_check_on_drone.sh) | The checks themselves; `check_drone_16.sh` runs them on the drone. |
| [`drone_16_common.sh`](drone_16_common.sh) | Shared by the scripts: the address, the name check, one reused ssh connection. |

None of them reads, sets or resets a PX4 parameter; the organizers require PX4's default parameters.

## Start-of-flight checklist

Do these in order, every flight.

1. **Put the drone on the takeoff spot first**, before anything else. Camera tracking's position
   starts at (0, 0, 0) wherever the drone is when the stack starts. Every target is held at least
   0.3 m above that point, so a drone carried to the net afterwards climbs to 0.3 m above the old
   start point (on 2026-11-07 that was 2.4 m above the floor).
2. **Unplug the Pixhawk USB cable.** PX4 will not arm while it is plugged in.
3. Run the restart, and wait for **READY TO FLY**:
   ```bash
   drone_code/restart_drone_16.sh skybeta-16.local
   ```
   If it says NOT READY, fix the first reason and run `drone_code/check_drone_16.sh skybeta-16.local`
   again.
4. **Don't move the drone after this.**
5. The pilot arms with the mode switch on **Altitude**, throttle low.
6. Within a few seconds (PX4 disarms 10 s after arming if it has not taken off), the pilot switches
   to **Position**, throttle still low. The drone lifts off at once.
7. If nothing happens, switch out of Position and back in. Each arming gets one attempt at the
   hand-over; if it still does not start, disarm, then run the check again.

The pilot takes the drone back by moving a stick (raise the throttle towards centre), as the
organizers' guide says.

## How to install it on the drone

Run these from the top of the repository.

Step 1. Copy the package onto the drone:

```bash
drone_code/install_on_drone_16.sh skybeta-16.local
```

Step 2, once only. Tell the drone to start our package instead of the demo. This changes a
protected file, so it asks for the shared drone password. Please type it yourself; it must never
be saved in any file.

```bash
ssh -t dronehacks@skybeta-16.local "sudo sed -i 's/^COMPETITOR_PKG=.*/COMPETITOR_PKG=team_nav/; s/^COMPETITOR_LAUNCH=.*/COMPETITOR_LAUNCH=team_nav.launch.py/' /etc/skybeta/drone.env"
```

To fly the route mission instead of the course stages, use
`COMPETITOR_LAUNCH=team_nav_route.launch.py` in that command. That file is identical to
`team_nav.launch.py`, except that it starts the route mission by default.

Step 3. Restart and check, with the drone on the takeoff spot:

```bash
drone_code/restart_drone_16.sh skybeta-16.local
```

At every start the drone builds the whole of `~/team_ws`. If any package in it fails to build,
nothing of ours starts and there is no hand-over; the check says so on its "last team_ws build"
line.

## Tests for the scripts

`tests/` runs `drone_16_check_on_drone.sh` against stand-in `docker` and `ros2` commands playing a
healthy drone and broken ones, and the Mac-side scripts against a stand-in `ssh`:

```bash
python3 -m pytest drone_code/tests
```

They have not been run against the real drone yet.
