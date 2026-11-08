"""Plans the tunnel_first mission's route on the drone's map, and checks it before anything flies.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. tunnel_first_run.py calls plan_tunnel_first_route once, before the start.

The route (the team's design, 2026-11-08; positions are (east, north) on the course map)
--------------------------------------------------------------------------------------
  1. Climb to CLIMB_HEIGHT_METRES above the launch mark (course_run's tested climb).
  2. Fly FORWARD_METRES north at that height, to the "high climb point".
  3. Climb there to CRUISE_HEIGHT_METRES.
  4. Fly at that height to S, above open floor before the tunnel's north entrance, by the shortest
     path that keeps the bottom laser's rules (high_path_rules.py), turning only where the drone
     stays NET_DISTANCE_METRES from the net.
  5. Turn 180 degrees at S (heading_turn.py), descend at S to TUNNEL_HEIGHT_METRES.
  6. GO: fly straight south along x = GO_LINE_EAST_METRES at TUNNEL_SPEED_METRES_PER_SECOND,
     through the tunnel, to GO_END_NORTH_METRES, and land there (or fly on to the pad).

Why S and the GO line are where they are (checked on every map of the real course, 2026-11-08)
---------------------------------------------------------------------------------------------
The team asked for S = (2.15, 2.40) and a line at x = 2.15. Rechecked from the map files:
  - North 2.40 is 0.65 m from the north net, not 0.9 m. S is moved to north 2.14 (0.91 m).
  - At x = 2.15 the propeller tips clear the drawing's tunnel entrance by 0.235 m, under the
    0.25 m asked for. At x = 2.17 the least clearance inside the tunnel is 0.255 m on the drawing
    and 0.31 m on the phone scan (tunnel centre lines at x = 2.31 and 2.07). S is then 0.88 m from
    the east net, 2 cm short of 0.9 m; we chose tunnel clearance over those 2 cm.
  - S is over open floor at 2.4 m, 1.5 m and 0.9 m, with nothing within 0.6 m, on the drawing,
    the new-start map and the phone scan. On the team's hand sketch, a sketched boulder (about
    15 cm accuracy) is 0.47 m from S; it is 0.53 m tall, so the drone at S (lowest point 0.8 m)
    can't touch it.
  - The GO line ends at north -0.95, 0.63 m past the drawing's tunnel exit and 0.73 m past the
    scan's; there the propeller tips are 0.22 m from the scan's C1 chicane wall.

The high leg's one allowed rule break
-------------------------------------
On the phone scan no path from the high climb point to S keeps every laser rule: it must either
cross the M3 maze divider more than 10 degrees off square, or turn while the scan's "Low object
2" (0.36 m tall, unknown, maybe a boulder) is under the laser. At 2.4 m PX4 has already stopped
using the laser at the first wall it crossed, so a low object under it changes nothing; that one
kind of break is allowed and written in the records. Any other break refuses the start.

What this file does not do: fly, or plan anything for the other missions.
"""
import os
from dataclasses import dataclass, field

import numpy as np

from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.drone_dimensions import DRONE_RADIUS_METRES, REQUIRED_ROOM_METRES
from team_nav.fallback_routes import FallbackRouteError, rounded_corners, shortest_high_path
from team_nav.high_path_rules import _distances_to_everything, check_high_path, open_floor_clearances


class TunnelFirstRouteError(ValueError):
    """The route can't be flown on this map; the message says why and what to do."""

# The team's choices (2026-11-08).
CLIMB_HEIGHT_METRES = 0.70
FORWARD_METRES = 0.8
CRUISE_HEIGHT_METRES = 2.4
TUNNEL_HEIGHT_METRES = 0.90
NET_DISTANCE_METRES = 0.9
TURNING_CIRCLE_METRES = 0.6
TUNNEL_SPEED_METRES_PER_SECOND = 0.25
# The team's choices, corrected from the map files (see above).
DEFAULT_S_NORTH_METRES = 2.14
S_EAST_METRES = 2.17
# The team may move S north for one run with this setting (drone.sh passes it), within these limits.
# Why: the LOOK step (tunnel_look.py) needs both of the tunnel's inside edges in the picture. The
# camera sees 78 degrees across (512 px at fx 317 in Gazebo; the same angle on drone 16), so the
# 1.47 m opening fits only from about 1.0 m away. At north 2.14 the camera is 0.70 m from the phone
# scan's tunnel mouth and 0.80 m from the drawing's, too close: both edges are cut off and the
# look is always "unsure" (the drone then flies the map's line, as in Phase 1). At north 2.40 the
# camera is 0.97 m and 1.07 m away, but S is then only 0.65 m from the north net.
S_NORTH_SETTING = "TEAM_NAV_S_NORTH_METRES"
LOWEST_S_NORTH_METRES = 2.0
HIGHEST_S_NORTH_METRES = 2.45


def s_north_from_environment():
    text = os.environ.get(S_NORTH_SETTING, "").strip()
    if not text:
        return DEFAULT_S_NORTH_METRES
    try:
        north_metres = float(text)
    except ValueError:
        raise TunnelFirstRouteError(f"{S_NORTH_SETTING} is '{text}', not a number of metres. Leave it out for "
                                    f"{DEFAULT_S_NORTH_METRES} m.")
    if not LOWEST_S_NORTH_METRES <= north_metres <= HIGHEST_S_NORTH_METRES:
        raise TunnelFirstRouteError(f"{S_NORTH_SETTING} is {north_metres} m, outside {LOWEST_S_NORTH_METRES} to "
                                    f"{HIGHEST_S_NORTH_METRES} m.")
    return north_metres


S_METRES = (S_EAST_METRES, DEFAULT_S_NORTH_METRES)

# The team's choice (2026-11-08): after the descent at S, before the LOOK, the drone backs straight
# north (nose still south, no turn) by this much, so the camera is about 1 m from the tunnel's
# mouth and both inside edges fit in the picture. At S (north 2.14) the camera is 0.70 m from the
# phone scan's mouth and 0.80 m from the drawing's; 0.26 m back makes it 0.96 m and 1.06 m. The turn
# stays at S, 0.91 m from the north net.
LOOK_BACKUP_SETTING = "TEAM_NAV_LOOK_BACKUP_METRES"
DEFAULT_LOOK_BACKUP_METRES = 0.26
LARGEST_LOOK_BACKUP_METRES = 0.5
LOOK_BACKUP_SPEED_METRES_PER_SECOND = 0.10
LOOK_BACKUP_SETTLE_SECONDS = 1.0
# The team's choice: the backup is refused if the drone's centre would come closer than this to the
# north net (at 0.62 m its propeller tips are 0.32 m from the net).
LEAST_LOOK_POINT_NET_METRES = 0.62


def look_backup_from_environment():
    text = os.environ.get(LOOK_BACKUP_SETTING, "").strip()
    if not text:
        return DEFAULT_LOOK_BACKUP_METRES
    try:
        backup_metres = float(text)
    except ValueError:
        raise TunnelFirstRouteError(f"{LOOK_BACKUP_SETTING} is '{text}', not a number of metres. Leave it out for "
                                    f"{DEFAULT_LOOK_BACKUP_METRES} m.")
    if not 0.0 <= backup_metres <= LARGEST_LOOK_BACKUP_METRES:
        raise TunnelFirstRouteError(f"{LOOK_BACKUP_SETTING} is {backup_metres} m, outside 0 to {LARGEST_LOOK_BACKUP_METRES} m.")
    return backup_metres
GO_LINE_EAST_METRES = 2.17
GO_END_NORTH_METRES = -0.95
# The team's choices for the descent at S: the target slides at the fast speed until it is this
# far above the tunnel height, then at the slow speed.
DESCENT_FAST_METRES_PER_SECOND = 0.7
DESCENT_SLOW_METRES_PER_SECOND = 0.2
DESCENT_SLOW_PART_METRES = 0.3
# Our own choices: the high leg is flown like the fallback routes (fallback_routes.py), the short
# moves near the launch mark at the climb speed.
HIGH_SPEED_METRES_PER_SECOND = 1.0
CLIMB_SPEED_METRES_PER_SECOND = 0.5
# Our own choice: the least room between the propeller tips and the tunnel's inside, at the
# tunnel height, on the drone's map. Less than this refuses the start.
LEAST_TUNNEL_CLEARANCE_METRES = 0.20
# The one kind of high-leg rule break allowed (see above): its sentence starts like this, and it
# names a low obstacle.
ALLOWED_BREAK_START = "The path turns with "

# Stage names, which TEAM_NAV_STOP_AFTER_STAGE may name.
SETTLE_STAGE = "settle"
FORWARD_STAGE = "forward and high climb"
HIGH_LEG_STAGE = "to S"
TURN_STAGE = "turn"
DESCENT_STAGE = "descent"
TUNNEL_STAGE = "tunnel"




@dataclass
class TunnelFirstRoute:
    map_name: str
    launch_metres: np.ndarray
    high_climb_point_metres: np.ndarray
    high_leg_metres: list                     # (east, north) points, S last.
    s_metres: np.ndarray
    go_end_metres: np.ndarray
    look_point_metres: np.ndarray = None      # Where the LOOK happens: S moved north by the backup.
    high_leg_breaks: list = field(default_factory=list)
    high_leg_cautions: list = field(default_factory=list)
    notes: list = field(default_factory=list)  # Sentences for the records: the checks' results.

    def describe(self):
        lines = [f"tunnel_first on the map '{self.map_name}':",
                 f"  climb to {CLIMB_HEIGHT_METRES:.2f} m at the launch mark ({self.launch_metres[0]:+.2f}, {self.launch_metres[1]:+.2f});",
                 f"  {FORWARD_METRES:.1f} m north to ({self.high_climb_point_metres[0]:+.2f}, {self.high_climb_point_metres[1]:+.2f}), "
                 f"climb to {CRUISE_HEIGHT_METRES:.1f} m;",
                 f"  high leg of {len(self.high_leg_metres)} points to S ({self.s_metres[0]:+.2f}, {self.s_metres[1]:+.2f});",
                 f"  turn 180 degrees left, descend to {TUNNEL_HEIGHT_METRES:.2f} m;",
                 f"  back {self.look_point_metres[1] - self.s_metres[1]:.2f} m north to "
                 f"({self.look_point_metres[0]:+.2f}, {self.look_point_metres[1]:+.2f}) and LOOK;",
                 f"  GO south at {TUNNEL_SPEED_METRES_PER_SECOND:.2f} m/s to ({self.go_end_metres[0]:+.2f}, {self.go_end_metres[1]:+.2f})."]
        lines += [f"  Allowed rule break: {text}" for text in self.high_leg_breaks]
        lines += [f"  Caution: {text}" for text in self.high_leg_cautions]
        lines += [f"  Check: {text}" for text in self.notes]
        return "\n".join(lines)


def plan_tunnel_first_route(course_map):
    """Plans and checks the route on this map. Raises TunnelFirstRouteError if it can't be flown."""
    launch = np.asarray(course_map.launch_mark_centre_metres, dtype=float)
    high_climb_point = launch + np.array([0.0, FORWARD_METRES])
    s_point = np.array([S_EAST_METRES, s_north_from_environment()])
    go_end = np.array([GO_LINE_EAST_METRES, GO_END_NORTH_METRES])
    route = TunnelFirstRoute(course_map.name, launch, high_climb_point, [], s_point, go_end)
    # Every mission takes off from the launch mark, so something under the laser there is noted, not refused.
    check_open_floor(course_map, route, "the climb at the launch mark", launch, [CLIMB_HEIGHT_METRES], refuse=False)
    check_open_floor(course_map, route, "the high climb point", high_climb_point, [CLIMB_HEIGHT_METRES, CRUISE_HEIGHT_METRES])
    check_s(course_map, route)
    plan_the_look_backup(course_map, route)
    plan_the_high_leg(course_map, route)
    check_the_go_line(course_map, route)
    return route


def check_open_floor(course_map, route, what, point_metres, heights_metres, refuse=True):
    """Refuses (or, with refuse=False, notes) if a wall, the tunnel or a boulder is under the laser there."""
    for height_metres in heights_metres:
        name, margin_metres = min(open_floor_clearances(course_map, point_metres, height_metres), key=lambda pair: pair[1])
        if margin_metres < 0.0 and not refuse:
            route.notes.append(f"{what} at {height_metres:.2f} m: {name} is under the laser ({margin_metres:.2f} m).")
            continue
        if margin_metres < 0.0:
            raise TunnelFirstRouteError(
                f"On this map, {name} would be under the bottom laser at {what} ({point_metres[0]:+.2f}, "
                f"{point_metres[1]:+.2f}) at {height_metres:.2f} m, and the drone changes height there. "
                f"Check the map file, or fly another mission.")
        route.notes.append(f"{what} at {height_metres:.2f} m: open floor, nearest {name} with {margin_metres:.2f} m to spare.")


def check_s(course_map, route):
    """S must be over open floor while the drone turns and descends there, with a clear circle
    around it. A low obstacle in the circle or under the laser is only noted (see the file header)."""
    s_point = route.s_metres
    for height_metres in (CRUISE_HEIGHT_METRES, 1.5, TUNNEL_HEIGHT_METRES):
        for name, margin_metres in open_floor_clearances(course_map, s_point, height_metres):
            if margin_metres < 0.0:
                _refuse_or_note_at_s(course_map, route, name, f"under the laser at {height_metres:.1f} m")
    for name, distance, _top in _distances_to_everything(course_map, s_point[np.newaxis, :]):
        if float(distance[0]) < TURNING_CIRCLE_METRES:
            _refuse_or_note_at_s(course_map, route, name, f"{float(distance[0]):.2f} m from S, inside the "
                                                          f"{TURNING_CIRCLE_METRES:.1f} m turning circle")
    least_net_metres = float(np.min(np.r_[s_point - course_map.lowest_corner_metres, course_map.highest_corner_metres - s_point]))
    route.notes.append(f"S is {least_net_metres:.2f} m from the nearest net.")


def _refuse_or_note_at_s(course_map, route, name, what):
    low_obstacle_names = [obstacle.name for obstacle in course_map.low_obstacles]
    if name in low_obstacle_names:
        route.notes.append(f"At S, {name} is {what}; it is a low obstacle, below the drone's lowest point there.")
        return
    raise TunnelFirstRouteError(f"On this map, {name} is {what}, at S ({route.s_metres[0]:+.2f}, {route.s_metres[1]:+.2f}). "
                                f"The drone turns and descends there. Check the map file, or fly another mission.")


def plan_the_look_backup(course_map, route):
    """The LOOK point, north of S by the backup. Refuses the start if its centre would be closer than
    LEAST_LOOK_POINT_NET_METRES to the north net, or if anything solid at the tunnel height comes within
    the drone's needed room on the way."""
    backup_metres = look_backup_from_environment()
    look_point = route.s_metres + np.array([0.0, backup_metres])
    north_net_metres = float(course_map.highest_corner_metres[1] - look_point[1])
    if north_net_metres < LEAST_LOOK_POINT_NET_METRES:
        raise TunnelFirstRouteError(
            f"Backing {backup_metres:.2f} m north from S for the LOOK would put the drone's centre {north_net_metres:.2f} m from "
            f"the north net on this map, closer than {LEAST_LOOK_POINT_NET_METRES:.2f} m. Use a smaller "
            f"{LOOK_BACKUP_SETTING}, or 0 to look from S.")
    north_values = np.linspace(route.s_metres[1], look_point[1], 20)
    points = np.c_[np.full_like(north_values, look_point[0]), north_values]
    shapes = [shape for shape in shapes_blocking_between_heights(course_map, TUNNEL_HEIGHT_METRES - 0.10, TUNNEL_HEIGHT_METRES + 0.10)
              if shape.name != "the safety net"]
    distances, names = distance_to_shapes_metres(points, shapes)
    closest = int(np.argmin(distances))
    if distances[closest] < REQUIRED_ROOM_METRES:
        raise TunnelFirstRouteError(f"Backing north for the LOOK would bring the drone within {distances[closest]:.2f} m of "
                                    f"{names[closest]}. Use a smaller {LOOK_BACKUP_SETTING}.")
    route.look_point_metres = look_point
    route.notes.append(f"LOOK point {backup_metres:.2f} m north of S: {north_net_metres:.2f} m from the north net; nearest "
                       f"thing at the tunnel height {names[closest]}, {distances[closest]:.2f} m.")


def plan_the_high_leg(course_map, route):
    turn_box = (np.asarray(course_map.lowest_corner_metres) + NET_DISTANCE_METRES,
                np.asarray(course_map.highest_corner_metres) - NET_DISTANCE_METRES)
    path, check = shortest_high_path(course_map, route.high_climb_point_metres, route.s_metres,
                                     CRUISE_HEIGHT_METRES, turn_box)
    if _forbidden_breaks(course_map, check.rule_breaks):
        path, check = _best_leg_with_one_corner(course_map, route, (path, check))
    forbidden = _forbidden_breaks(course_map, check.rule_breaks)
    if forbidden:
        raise TunnelFirstRouteError("No high path to S keeps the laser rules on this map: " + " ".join(forbidden)
                                    + " Check the map file, or fly another mission.")
    route.high_leg_metres = [np.asarray(point, dtype=float) for point in path]
    route.high_leg_breaks = list(check.rule_breaks)
    route.high_leg_cautions = list(check.cautions)


def _forbidden_breaks(course_map, rule_breaks):
    low_obstacle_names = [obstacle.name for obstacle in course_map.low_obstacles]
    forbidden = []
    for text in rule_breaks:
        allowed = text.startswith(ALLOWED_BREAK_START) and any(f"with {name} under" in text for name in low_obstacle_names)
        if not allowed:
            forbidden.append(text)
    return forbidden


def _best_leg_with_one_corner(course_map, route, best):
    """East along the high climb point's line first, then one corner, then to S: the path the
    search misses on the phone scan, because its turn points must be clear of low obstacles."""
    start, goal = route.high_climb_point_metres, route.s_metres
    best_cost = (len(_forbidden_breaks(course_map, best[1].rule_breaks)), best[1].cost())
    for corner_north in np.arange(start[1], goal[1] + 1e-9, 0.05):
        for corner_east in np.arange(start[0] + 0.5, goal[0], 0.1):
            try:
                path = rounded_corners([start, [corner_east, corner_north], goal])
            except FallbackRouteError:
                continue
            check = check_high_path(course_map, path, CRUISE_HEIGHT_METRES)
            cost = (len(_forbidden_breaks(course_map, check.rule_breaks)), check.cost())
            if cost < best_cost:
                best, best_cost = (path, check), cost
    return best


def check_the_go_line(course_map, route):
    """Refuses if the propeller tips would pass the tunnel's inside, or anything else, with less
    than LEAST_TUNNEL_CLEARANCE_METRES on the way from S to the end, at the tunnel height."""
    north_values = np.arange(route.s_metres[1], route.go_end_metres[1] - 1e-9, -0.01)
    points = np.c_[np.full_like(north_values, GO_LINE_EAST_METRES), north_values]
    lowest_metres, highest_metres = TUNNEL_HEIGHT_METRES - 0.10, TUNNEL_HEIGHT_METRES + 0.10
    distances, names = distance_to_shapes_metres(points, shapes_blocking_between_heights(course_map, lowest_metres, highest_metres))
    closest = int(np.argmin(distances))
    clearance_metres = float(distances[closest]) - DRONE_RADIUS_METRES
    where = f"at north {north_values[closest]:+.2f} ({names[closest]})"
    if clearance_metres < LEAST_TUNNEL_CLEARANCE_METRES:
        raise TunnelFirstRouteError(
            f"On this map the propeller tips would pass only {clearance_metres:.2f} m from something {where} on the GO "
            f"line at x = {GO_LINE_EAST_METRES:.2f}, less than {LEAST_TUNNEL_CLEARANCE_METRES:.2f} m. Check the map file.")
    route.notes.append(f"GO line: least propeller clearance {clearance_metres:.2f} m, {where}.")
    check_open_floor(course_map, route, "the end of the GO line", route.go_end_metres, [TUNNEL_HEIGHT_METRES])
