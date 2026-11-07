# Code copied from the organizers

Nothing in this folder was written by our team, and we have not changed any of it. We keep it unchanged so that the simulator runs the same code as the drone, and so that we can compare it with the original at any time. Please do not reformat or edit these files. If you want different behaviour, change our own code instead.

## Where it came from

The DroneHacks organizers wrote these files as part of the Sky Beta drone software, which is installed on each drone in `~/skybeta`. On 2026-11-06 we compared our copy with the copy on drone 16 using file fingerprints (SHA-256), and every file was identical. If the organizers release a new version, compare again and copy the new files in.

| File or folder | What it does |
|---|---|
| `VERSION` | The version we copied: 0.3.2. |
| `skybeta_flight_interface/` | The program that sits between our flight code and PX4 (the flight controller's software). It accepts our targets, applies the safety limits, and decides when to hand control to our code. |
| `px4_odom_bridge.py` | The program that passes camera-based position estimates on to PX4. |

## Why the simulator uses it

These two programs decide whether a flight is allowed to start and what happens to the targets our code sends. If the simulator used a rewritten copy, a mistake in our rewrite could make a test pass that would fail on the real drone. Running the organizers' own code avoids that.

Our own code never replaces these programs on the drone. On the real drone, the same programs already run, and our team doesn't modify them.
