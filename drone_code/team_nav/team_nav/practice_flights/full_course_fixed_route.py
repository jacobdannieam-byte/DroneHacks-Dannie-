"""full_course's fixed route on real_course_2026-11-08.json, checked against the map before flight.

REAL DRONE CODE, run on the ground before anything is sent. Uses the participant map only.

The nose points north the whole flight and the drone flies at CRUISE_HEIGHT_METRES. The route is
the team's (2026-11-08):

  1. From the launch mark, a short step west, then north along the west net, so the right-hand
     propellers keep clear of the start diagonal wall.
  2. North-east to the first checkpoint's turn-back point (north 2.515 m, full_course_route.py).
  3. East along the north side of the course, past Gate A, to above the tunnel's entrance.
  4. South straight through the middle of the tunnel to below its exit.
  5. Through the chicane and Gate B to the landing pad: the points of the 22-leg A* route that flew
     in Gazebo run 20261108_132229 (tag gateb-143s), with three of them widened (below).

Widened legs. The Gazebo check fails a run whose true propellers come within 0.08 m of anything,
and these legs failed it with a 2 degree launch heading error or 10 cm of tracking drift, so each
was widened (moved where the worst of those errors still leaves the most room on the map):
  - The two west-net points: the team gave x = -2.73 m (0.069 m from the net). At -2.70 m run
    20261108_142045 (2 degrees) came 0.074 m from the net; at -2.67 m the worst case keeps 0.098 m.
  - The two chicane points after (0.90, -1.70): (0.60, -1.50) and (0.50, -1.20) passed 0.025 m from
    C1's south end in run 20261108_142045. Now (0.30, -1.70) and (0.40, -1.38): worst case 0.127 m.
  - The Gate B point: (-0.60, -1.00) passed 0.068 m from A1's south end in run 20261108_142447
    (drift). Now (-0.65, -1.25): worst case 0.147 m.

What it does not do: plan. If the map changes, check_route (full_course_route.py) refuses the
route before flight rather than finding a new one.
"""
import numpy as np

from team_nav.practice_flights.full_course_route import CRUISE_HEIGHT_METRES, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES

# The team's route (east, north) in metres, except the widened points (see above).
LAUNCH_EAST_NORTH_METRES = (-2.5103, 0.9167)
WEST_NET_EAST_METRES = -2.67
ROUTE_EAST_NORTH_METRES = (
    LAUNCH_EAST_NORTH_METRES,
    (WEST_NET_EAST_METRES, 0.92),
    (WEST_NET_EAST_METRES, 1.60),
    (-2.51, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES),
    (2.30, 2.45),
    (2.30, -0.80),
    # From the 22-leg route of Gazebo run 20261108_132229.
    (1.30, -1.60),
    (0.90, -1.70),
    (0.30, -1.70),      # Widened, was (0.60, -1.50).
    (0.40, -1.38),      # Widened, was (0.50, -1.20).
    (0.60, -0.60),
    (0.60, -0.40),
    (0.50, -0.20),
    (0.30, -0.10),
    (0.00, -0.10),
    (-0.20, -0.20),
    (-0.65, -1.25),     # Widened, was (-0.60, -1.00).
    (-1.20, -1.35),
    (-2.0422, -0.7925),
)


def fixed_route_metres():
    """The route as an N x 3 array of (east, north, height), launch mark first. full_course checks
    it against the map with check_route before flying it."""
    points = np.array(ROUTE_EAST_NORTH_METRES, dtype=float)
    return np.column_stack((points, np.full(len(points), CRUISE_HEIGHT_METRES)))
