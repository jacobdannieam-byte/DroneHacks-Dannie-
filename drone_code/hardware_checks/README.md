# Hardware checks for drone 16

These checks make sure every part of drone 16 works, **without flying**. They only look and
listen. They never arm the drone, spin a motor, send a target or change a setting.

## How to run them

Connect the Mac to the drone's computer (the Jetson) with a USB cable. Then run this from the top
of the repository:

```bash
drone_code/hardware_checks/check_drone_16.sh
```

It takes about a minute. Everything it prints is also saved in `results/`, which Git ignores.

To check the flight controller, the bottom laser and the battery, the drone's battery must be
connected, because they get their power from it. **Take the propellers off first**, and don't
touch the remote control's arm switch.

## What it checks

1. **That this is drone 16.** Every drone answers on the same USB address, and logging in to
   another team's drone gets us disqualified. So the script checks the drone's identity key and
   its name, and stops if either is wrong.
2. **The drone's computer and the organizers' software.** It runs the organizers' own health check
   (`host/doctor.sh`). It shows which program the drone starts, which programs are running, the
   ROS network number, whether the flight controller is talking to the computer, the free disk
   space and the temperatures.
3. **Every sensor.** It listens to each sensor for 8 seconds. Each line says PASS if data
   arrived and FAIL if not, with the messages per second and the reading that matters. For
   example, it shows the laser's distance, the battery voltage, the camera picture size and the
   camera tracking position.

## Files, in reading order

1. [`check_drone_16.sh`](check_drone_16.sh) runs on the Mac and does the three steps above.
2. [`on_the_drone/look_at_the_drone.sh`](on_the_drone/look_at_the_drone.sh) runs on the drone.
   It covers step 2.
3. [`on_the_drone/listen_to_every_sensor.py`](on_the_drone/listen_to_every_sensor.py) runs
   inside the organizers' software on the drone. It covers step 3.

## Still to add

These are the hand tests from section 11 of `docs/ARCHITECTURE.md`. Each would tell the person
what to do and then check the result:

- holding the drone at known heights, to check the bottom laser;
- moving it 1 m forward and 1 m left and turning it 90°, to check the directions;
- carrying it through the tunnel, to check that camera tracking keeps working.
