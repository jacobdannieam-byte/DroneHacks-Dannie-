# Five-target controller validation

This branch starts from `classical-dannie-attemptone` at `a6251d7` and adds the
`five_waypoints` practice flight. The previous `full_course` entry remains available.

The five stage endpoints are First corridor, Maze exit, Tunnel exit, Chicane exit,
and Landing pad. Intermediate corners keep straight segments out of walls. The
controller stops, changes its heading target gradually, waits for measured settling,
then advances. Only the short first-corridor retreat travels backward.

Depth must be recent, have a matching tracked pose, and face the next travel direction.
Nearby forward obstacles pause progress. Wider final chicane corners leave room for
horizontal tracking error. Existing camera-height feedback corrects
small height offsets, capped here at 0.15 m. It does not repair the PX4 estimator.

## Tests

64 tests passed in the ROS Humble/PX4 message simulator image:

```bash
python3 -m pytest test/test_five_waypoints.py test/test_full_course.py \
  test/test_first_maze_leg.py test/test_flight_steps.py test/test_live_navigation.py -q
```

Coverage includes swept turn clearance, checkpoint/pad clearance, bounded retreat,
shortest-angle yaw, waiting for measured alignment and arrival, depth freshness and
view direction, persistent obstacle stops, stamped depth transforms, height limits, and
final-chicane clearance with a 25 cm northward offset plus 3 cm east/west error.

## Gazebo trials on 2026-11-08

All trials used the sketch map, practice mode, normal sensor/contact checks, and the
existing PX4 parameters. WSL used an isolated internal Docker network and UDP-only DDS
transport, as in the previous full-course trial. Other simulator workloads shared the host.
Failed records were retained under `physical-gazebo-evidence/attemptone-five-waypoints`
in the parent workspace; large camera recordings are not committed to Git.

| Run | Outcome and resulting change |
|---|---|
| `20261108_102606` | Stopped in the maze for height divergence, 0 contacts, peak error 0.464 m. Cause not conclusively isolated; do not claim the rerun fixed the estimator. |
| `20261108_103116` | Crossed all four checkpoint lines but contacted a wall near the chicane exit; recovery was unstable and the owned SITL container was stopped. About 18 cm of horizontal route offset motivated connecting the existing wall-alignment code. |
| `20261108_103918` | Wall alignment active; 3 checkpoints, 0 contacts. Timed out at the tunnel endpoint because camera height remained roughly 9 cm below target; landed locally. Connected the existing bounded height controller. Peak height error including recovery was 0.186 m. |
| `20261108_104629` | Height feedback active: peak flight height error 0.060 m. Wall alignment caused a growing lateral correction approaching the chicane wall, leading to contact. Removed wall matching from this mode and widened the final chicane corners; four checkpoint crossings here do not constitute a successful flight. |
| `20261108_105556` | Final version: exit code 0, all four checkpoints, landed on pad, 0 contacts/penalties, no wall-height or course-boundary violations. Peak height error 0.050 m; motors off 314.47 s after takeoff. Runtime source hashes matched the branch's flight/mission files. |

The final trial passed the referee checks. Checkpoint times were 17.70 s (First corridor),
119.53 s (Maze), 167.83 s (Tunnel), and 270.60 s (Chicane). The turns and settling make
this version slower than the previous fixed-heading flight. This is one successful final
trial after the failures above, not a reliability study.

## Limits

Gazebo uses a simulated tracking feed with added noise, delay and drift. It does not run
real cuVSLAM or validate its behavior during yaw, texture loss or motion blur. Stereo depth
does use the rendered camera images. A completed simulation is not hardware qualification.
The reverse retreat has no rear camera obstacle detection. This is a surveyed waypoint
controller, not a planner that finds a new route around a persistent obstruction.
