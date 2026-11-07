# How we set up PX4 for the simulator

Everything in this folder is for the simulator only. None of it ever goes on the real drone.

PX4 is the flight controller's software. The simulator runs the real PX4 program, so this folder holds only the small differences between how PX4 runs here and how it runs on drone 16.

## Read these first

1. [`start_px4.sh`](start_px4.sh) starts PX4 each time the simulator starts.
2. [`drone_16_settings.sh`](drone_16_settings.sh) lists the PX4 settings we use. It says which values were read from drone 16 and which are estimates.

## Our three edits to PX4

The simulator's build runs these scripts once, on a copy of PX4's source code. Each one explains its reason at the top. Together they are the only changes we make to PX4's code, so read them if you want to know how far the simulated PX4 differs from the real one.

| Script | What it changes | Why |
|---|---|---|
| [`let_simulator_read_true_position.py`](let_simulator_read_true_position.py) | PX4 shares the drone's true position. | The pretend sensors and the referee need to know where the drone really is. |
| [`lower_simulated_motion_sensor_noise.py`](lower_simulated_motion_sensor_noise.py) | The pretend motion sensor is less noisy. | The default noise made the simulated drone wander about 0.5 m while hovering. |
| [`make_simulated_floor_solid.py`](make_simulated_floor_solid.py) | The drone can't sink into the floor. | The default floor let the drone sink about 13 cm and tilt. |
