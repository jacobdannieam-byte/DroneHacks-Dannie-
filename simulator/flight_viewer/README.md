# The 3D flight viewer

This folder makes a web page that shows one simulated flight in 3D. You can turn the course around with the mouse, and play the flight back to see exactly where the drone went and how close it came to things. It is for the simulator only, because it uses the drone's true position, which only the simulator knows.

## How to use it

Run a test flight first (see [`../README.md`](../README.md)). Then, from the top folder of the repository, run this on the Mac. It needs only Python, with no extra libraries.

```bash
python3 simulator/flight_viewer/make_flight_viewer.py
```

It uses the newest flight, saves the page next to that flight's record in `simulator/generated_files/flight_records/`, and opens it in your web browser. To view an older flight, put the path of its `.csv` record after the command. Records made before 2026-11-07 can't be shown, because they don't say which way the drone was facing.

The page needs an internet connection, because it loads the 3D drawing library, three.js, from the internet.

## What the page shows

- **The course**, in the colours of the organizers' model. Obstacles are see-through at first, so that you can see the drone inside the maze and the tunnel. The safety net around the course is hidden at first, because it would block the view. A box in the panel turns each of these on or off.
- **The drone's true path.** Its colour shows the height above the floor, as the key in the panel explains.
- **A green dot** where the record starts, and **red dots** where a propeller touched an obstacle. The red dots follow the referee's rule, so there is one dot for each touch the referee counted.
- **The drone itself**, which you can play back along the path at different speeds. Its four propellers are orange circles that turn red while they touch something, and a cone on the front shows which way it faces.
- **Readings for each moment**: the time since takeoff, the height, the distance from the nearest propeller to the nearest obstacle, and whether the motors are armed, PX4 thinks the drone has landed, and our program is flying it.

## Limits

- The drone is drawn level. The record stores which way it faces, but not how it tilts.
- The propeller size is assumed to be 7 inches, as it is in `referee.py`. Nobody has confirmed it.
- The drone's body, arms and nose are drawn at sizes we chose. Only the motor positions and the propellers match the real drone.

## Files, in reading order

1. [`make_flight_viewer.py`](make_flight_viewer.py) reads the course model and the flight record, and writes the page.
2. [`viewer_page.html`](viewer_page.html) holds the page's drawing code. The first script fills it with data. If you open this file directly, it only shows a message, because it has no data in it.
