# The pretend world around the drone

This ROS package is everything that pretends to be the physical world in the simulator: the sensors, the person holding the remote control, and the referee. It is for the simulator only. Nothing on the real drone may use it.

Start with [`launch/whole_simulator.launch.py`](launch/whole_simulator.launch.py), which lists every program that runs in the simulator. Then read the files in `course_simulator/`, in this order:

| File | What it does |
|---|---|
| `pretend_remote_control.py` | Plays the teammate with the remote: it arms the motors and flips the switch to Position. |
| `true_drone_state.py` | Tells the other programs where the drone truly is on the course. |
| `pretend_camera_tracking.py` | Reports the drone's position the way the camera tracking does, with added delay, jitter and drift. |
| `pretend_bottom_laser.py` | Reports the distance to whatever is below the drone. |
| `pretend_depth_camera.py` | Reports the distance to whatever is in front of the drone, as a set of points. |
| `referee.py` | Watches the flight, and records touches, checkpoints and times. The checkpoints come from the mission file and the world's map. |
| `boulder_watch.py` | Used by the referee. Records each time the drone is above a boulder, and how far PX4's height estimate strays from the true height. |
| `world.py` | Decides which course the simulator flies in: the organizers' drawing, or any course map file named with `--world`. |
| `course_mesh_from_map.py` | Makes a 3D model of the course from a map file, for any world other than the organizers' drawing. |
| `course_model.py` | Answers questions about the course, such as how far a ray travels before it hits something. |
| `paths.py` | Says where the simulator's files are inside the box. |
