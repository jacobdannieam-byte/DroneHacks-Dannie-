# Test flights for the real drone

This folder is a small ROS package, `team_smoke_flights`, holding our first real test flights.
It's kept separate from `team_nav`, the competition program, so that testing one never
changes the other.

For now it has one flight, **rise and land**: rise 0.5 m straight up, hold for 5 seconds,
come back down, and land. PX4 then switches the motors off. It passed in the 3D rendered sim on
2026-11-07 (`simulator/run_test_flight.sh rise_and_land_test.py`). It has never flown on the
real drone.

## How it starts, and how to stop it

The program never arms the drone and never changes its flight mode. A teammate arms the drone on
the remote control, then flips the mode switch to Position. Only then does the organizers'
software hand control to the program. Until that moment the drone stays on the ground, whatever
the program is doing.

To stop it at any time:
- **flip the mode switch**, which takes control back from the program;
- or use **the kill switch**, which stops the motors at once.

Every flight is recorded, one line per tenth of a second, in `~/team_ws/flight_logs/` on the
drone.

## Files, in reading order

1. [`team_smoke_flights/rise_and_land.py`](team_smoke_flights/rise_and_land.py) is the flight.
2. [`team_smoke_flights/telemetry_recorder.py`](team_smoke_flights/telemetry_recorder.py) writes
   the recording.
3. [`launch/rise_and_land.launch.py`](launch/rise_and_land.launch.py) is what starts on the drone:
   the organizers' flight interface, and the flight.

## Put it on the drone and fly it

`drone_code/install_on_drone_16.sh` copies this package to the drone, with our main package. Then
`drone_code/TEST_SEQUENCE.md`, step 6, gives the commands for this flight:

```bash
./ours.sh start rise_and_land
```
