"""The bottom laser rule for the route mission: which waypoints must be flown through, never stopped
at, so that PX4 keeps trusting the bottom laser.

How it fits: mission.py calls waypoints_to_fly_through once, when a route mission is made, and
hands the answer to route_flight.py, which flies those waypoints through whatever the turn. It needs
the course map (Kenny's course_map.py) and where the route starts on it (start_frame.py). It never
talks to ROS.

Why (from the team, 2026-11-07): PX4 takes the drone's height mainly from the bottom laser (PX4
setting EKF2_HGT_REF = 2 on drone 16; we never change PX4 parameters). When the laser's reading jumps,
over the window's sill or a boulder, PX4 can reject the laser if the drone then slows below about
0.32 m/s within 6 to 10 s, and its height then drifts by 0.3 to 0.5 m. The simulator showed the same:
after a boulder, the drone crossed the window about 0.35 m too high and clipped its top.

The rule: after a leg that crosses a window (an opening in a wall), changes height, or passes over a
boulder, every waypoint within 8 s of flight at the route speed must be flown through at that
speed, never stopped at. The 8 s are counted from the end of that leg, which covers the whole 6 to
10 s after the crossing itself. The last waypoint is always a stop, because the landing starts there;
that is reported.

"Passes over a boulder" means the leg comes within the boulder's radius plus the laser's footprint:
the laser sees a cone below it, not a single point (estimated below).
"""
import math
from dataclasses import dataclass

import numpy as np

from .course_map import distance_to_line_piece_metres

# Estimated: the bottom laser (a VL53L1X on drone 16) sees a cone about 27 degrees wide, so at
# cruise height (0.8 m) it sees a circle about 0.8 x tan(13.5 deg) = 0.19 m in radius.
LASER_FOOTPRINT_RADIUS_METRES = 0.2


@dataclass
class LegEvent:
    leg: int        # the leg into this waypoint (0-based); leg 0 starts at the takeoff spot
    reason: str


def legs_that_upset_the_laser(course_map, start_frame, waypoints, heights_above_floor_metres, cruise_height_metres):
    """Every leg that crosses a window, changes height, or passes over a boulder, with why.
    waypoints are (forward, left) from the takeoff spot; heights are each leg's height or None."""
    corners = [(0.0, 0.0)] + [tuple(waypoint[:2]) for waypoint in waypoints]
    on_course = [np.asarray(start_frame.course_from_start_frame((forward, left, 0.0))[:2]) for forward, left in corners]
    leg_heights = [cruise_height_metres if height is None else height for height in heights_above_floor_metres]
    events = []
    for leg in range(len(waypoints)):
        start, end = on_course[leg], on_course[leg + 1]
        reasons = []
        height_before = cruise_height_metres if leg == 0 else leg_heights[leg - 1]
        if abs(leg_heights[leg] - height_before) > 1e-9:
            reasons.append(f"changes height from {height_before:.2f} to {leg_heights[leg]:.2f} m")
        reasons += openings_crossed(course_map, start, end)
        reasons += boulders_passed_over(course_map, start, end)
        if reasons:
            events.append(LegEvent(leg, ", ".join(reasons)))
    return events


def openings_crossed(course_map, start, end):
    crossed = []
    for wall in course_map.walls:
        along = crossing_along_wall_metres(start, end, np.asarray(wall.start_metres)[:2], np.asarray(wall.end_metres)[:2])
        if along is None:
            continue
        for opening in wall.openings:
            if opening.start_along_wall_metres <= along <= opening.end_along_wall_metres:
                crossed.append(f"crosses the {opening.name} in {wall.name}")
    return crossed


def crossing_along_wall_metres(start, end, wall_start, wall_end):
    """Where the leg from start to end crosses the wall's centre line, as metres along the wall, or None."""
    leg, wall = end - start, wall_end - wall_start
    denominator = leg[0] * wall[1] - leg[1] * wall[0]
    if abs(denominator) < 1e-12:
        return None                                  # parallel
    to_wall = wall_start - start
    on_leg = (to_wall[0] * wall[1] - to_wall[1] * wall[0]) / denominator
    on_wall = (to_wall[0] * leg[1] - to_wall[1] * leg[0]) / denominator
    if not (0.0 <= on_leg <= 1.0 and 0.0 <= on_wall <= 1.0):
        return None
    return on_wall * float(np.linalg.norm(wall))


def boulders_passed_over(course_map, start, end):
    passed = []
    for obstacle in course_map.low_obstacles:
        centre = np.asarray(obstacle.centre_metres, float)[:2].reshape(1, 2)
        distance = float(distance_to_line_piece_metres(centre, start, end)[0])
        if distance <= obstacle.radius_metres + LASER_FOOTPRINT_RADIUS_METRES:
            passed.append(f"passes over {obstacle.name}")
    return passed


def waypoints_to_fly_through(waypoints, events, speed_metres_per_second, seconds):
    """({waypoint index: why it must be flown through}, a note if the last waypoint is affected, or
    None). A waypoint counts if the path from the end of an upsetting leg to it is at most seconds of
    flight. Each waypoint keeps the first reason found."""
    reach_metres = speed_metres_per_second * seconds
    last = len(waypoints) - 1
    must, last_note = {}, None
    for event in events:
        travelled = 0.0
        index = event.leg
        while index <= last and travelled <= reach_metres + 1e-9:
            why = f"within {seconds:g} s after the leg into waypoint {event.leg + 1}, which {event.reason}"
            if index == last:
                last_note = last_note or f"waypoint {index + 1} is the last, so the drone still stops there ({why})"
            else:
                must.setdefault(index, why)
            if index < last:
                travelled += math.dist(waypoints[index][:2], waypoints[index + 1][:2])
            index += 1
    return must, last_note
