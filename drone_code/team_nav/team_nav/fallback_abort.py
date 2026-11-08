"""What a fallback mission does when its run must stop: fly to a landing spot if the drone still
knows where it is, otherwise descend where it is.

REAL DRONE CODE, also used in the simulator and the tests. fallback_run.py calls it from
FallbackRun.land_after_a_problem. course_run's own abort (land where the drone is) is unchanged.

The team's rule (review of 2026-11-08)
--------------------------------------
  - Position healthy (position_health.py: camera tracking fresh, no jump, no PX4 reset): fly at the
    current height, at ABORT_SPEED_METRES_PER_SECOND, to above the landing pad if the straight line
    there is clear at that height; otherwise to the nearest open floor that a straight line reaches;
    then land with the normal landing. If position is lost on the way, stop there and descend.
  - Position lost: never move sideways. The normal landing descends in place (it sends only
    downward speeds).
Up at 2.3 m nothing but the net is in the way, so the pad is almost always reachable. Low down, in
the tunnel or near a wall, it often is not.

"Clear at that height" means the drone's centre keeps drone_dimensions.py's 0.35 m from everything
that stands between the drone's lowest and highest points, all along the line. "Open floor" means
nothing on the floor (walls, tunnel, boulders) is under the bottom laser there, allowing for
camera tracking's drift (high_path_rules.py).

What it does not do: decide that the run must stop (run_safety.py), or land (flight_steps.py).
"""
import numpy as np

from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.drone_dimensions import HALF_HEIGHT_NEEDED_METRES, REQUIRED_ROOM_METRES
from team_nav.high_path_rules import open_floor_clearances

# Our own choices. The abort flies at the course run's usual cruise speed, slower than the
# fallback's 1.0 m/s, because something has already gone wrong.
ABORT_SPEED_METRES_PER_SECOND = 0.5
# Open floor is looked for on a grid this fine, no further than this from the drone.
OPEN_FLOOR_SEARCH_SPACING_METRES = 0.1
OPEN_FLOOR_SEARCH_REACH_METRES = 2.0
# The straight line to a landing spot is checked at points this far apart.
LINE_CHECK_SPACING_METRES = 0.02
# Time allowed for the flight: the distance at a slow 0.3 m/s, plus 10 s to settle.
SLOWEST_ABORT_SPEED_METRES_PER_SECOND = 0.3
EXTRA_ABORT_SECONDS = 10.0


def choose_landing_spot(course_map, drone_metres):
    """(east, north) to fly to at the drone's current height, and why; or (None, why) if none."""
    drone = np.asarray(drone_metres, dtype=float)
    height_metres = float(drone[2])
    blocking = shapes_blocking_between_heights(course_map, height_metres - HALF_HEIGHT_NEEDED_METRES,
                                               height_metres + HALF_HEIGHT_NEEDED_METRES)
    pad = np.asarray(course_map.landing_pad_centre_metres, dtype=float)
    if line_is_clear(blocking, drone[:2], pad):
        return pad, "the landing pad, which a straight line reaches at this height"
    spot = nearest_reachable_open_floor(course_map, blocking, drone[:2], height_metres)
    if spot is None:
        return None, "no open floor within reach of a clear straight line"
    return spot, "the nearest open floor that a clear straight line reaches (the pad is blocked at this height)"


def line_is_clear(blocking, start_metres, end_metres):
    """True if the drone's centre keeps REQUIRED_ROOM_METRES from every blocking shape all along."""
    length_metres = float(np.linalg.norm(end_metres - start_metres))
    steps = max(2, int(np.ceil(length_metres / LINE_CHECK_SPACING_METRES)))
    points = start_metres + np.linspace(0.0, 1.0, steps)[:, np.newaxis] * (end_metres - start_metres)
    room_metres, _ = distance_to_shapes_metres(points, blocking)
    return bool(room_metres.min() >= REQUIRED_ROOM_METRES)


def nearest_reachable_open_floor(course_map, blocking, drone_xy, height_metres):
    best, best_distance = None, np.inf
    offsets = np.arange(-OPEN_FLOOR_SEARCH_REACH_METRES, OPEN_FLOOR_SEARCH_REACH_METRES + 1e-9, OPEN_FLOOR_SEARCH_SPACING_METRES)
    for east_offset in offsets:
        for north_offset in offsets:
            spot = drone_xy + np.array([east_offset, north_offset])
            distance = float(np.hypot(east_offset, north_offset))
            if distance > OPEN_FLOOR_SEARCH_REACH_METRES or distance >= best_distance:
                continue
            if min(margin for _, margin in open_floor_clearances(course_map, spot, height_metres)) < 0.0:
                continue
            if line_is_clear(blocking, drone_xy, spot):
                best, best_distance = spot, distance
    return best


def longest_abort_seconds(start_metres, spot_metres):
    distance_metres = float(np.linalg.norm(np.asarray(spot_metres)[:2] - np.asarray(start_metres)[:2]))
    return distance_metres / SLOWEST_ABORT_SPEED_METRES_PER_SECOND + EXTRA_ABORT_SECONDS
