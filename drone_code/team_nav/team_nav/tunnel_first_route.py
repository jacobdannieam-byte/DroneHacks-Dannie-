"""Plans the tunnel_first mission's route on the drone's map, and checks it before anything flies.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. tunnel_first_run.py calls plan_tunnel_first_route once, before the start.

The route (the team's spec, 2026-11-08; positions are (east, north) on the course map)
------------------------------------------------------------------------------------
  1. Climb to CLIMB_HEIGHT_METRES above the launch mark (course_run's tested climb).
  2. Fly FORWARD_METRES north at that height, to the "high climb point", and climb there to the
     cruise height (TEAM_NAV_FALLBACK_CRUISE_METRES, default DEFAULT_CRUISE_HEIGHT_METRES).
  3. Optionally (TEAM_NAV_CROSS_CORRIDOR_END=on) fly north at cruise to CORRIDOR_END_NORTH_METRES,
     across the end-of-first-corridor line.
  4. Fly at cruise to S at HIGH_SPEED_METRES_PER_SECOND, by the shortest path that keeps the bottom
     laser's rules (high_path_rules.py), turning only NET_DISTANCE_METRES or more from the net.
  5. Descend at S to TUNNEL_HEIGHT_METRES; the height gate and the turn happen there.
  6. Back north by TEAM_NAV_LOOK_BACKUP_METRES to the LOOK point; LOOK.
  7. GO south along the tunnel to GO_END_NORTH_METRES, about 0.5 m past the exit; land there, or
     (TEAM_NAV_AFTER_TUNNEL=pad) slide to the pad spot, climb, and fly over the chicane to the pad.

Why S and the GO line are where they are (checked on every map of the real course, 2026-11-08)
---------------------------------------------------------------------------------------------
  - S = (2.17, 2.14): 0.91 m from the north net, 0.88 m from the east net (2 cm short of 0.9 m:
    we chose tunnel clearance). At x = 2.17 the least propeller clearance inside the tunnel is
    0.255 m on the drawing and 0.31 m on the phone scan (centre lines at x = 2.31 and 2.07).
  - S is over open floor at 2.4, 1.5 and 0.9 m with nothing within its 0.6 m turning circle on the
    drawing, the new-start map and the phone scan. On the team's hand sketch a sketched boulder
    (about 15 cm accuracy) is 0.47 m from S; it is 0.53 m tall, below the drone's lowest point
    (0.8 m) during the turn at 0.90 m. That is noted, not refused.
  - The LOOK point is 0.26 m north of S: the camera is then about 1 m from the mouth, and the drone's
    centre 0.65 m from the north net (the limit is LEAST_LOOK_POINT_NET_METRES).
  - The GO line ends at north -0.95, 0.63 m past the drawing's tunnel exit and 0.73 m past the
    scan's.

The high leg's one allowed rule break
-------------------------------------
On the phone scan no path to S keeps every laser rule: it must cross the M3 maze divider more than
10 degrees off square, or turn while the scan's "Low object 2" (0.36 m tall) is under the laser. At
cruise PX4 has already stopped using the laser at the first wall, so a low object under it changes
nothing; that one kind of break is allowed and written in the records. Any other break refuses.

What this file does not do: fly, or plan anything for the other missions.
"""
import os
from dataclasses import dataclass, field

import numpy as np

from team_nav.course_map import distance_to_line_piece_metres, distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.drone_dimensions import DRONE_RADIUS_METRES, REQUIRED_ROOM_METRES
from team_nav.fallback_routes import FallbackRouteError, cruise_height_from_environment, rounded_corners, shortest_high_path
from team_nav.high_path_rules import _distances_to_everything, check_high_path, distance_to_tunnel_metres, open_floor_clearances


class TunnelFirstRouteError(ValueError):
    """The route can't be flown on this map; the message says why and what to do."""


# The team's choices (2026-11-08).
CLIMB_HEIGHT_METRES = 0.70
FORWARD_METRES = 0.8
DEFAULT_CRUISE_HEIGHT_METRES = 2.4
CRUISE_SETTING = "TEAM_NAV_FALLBACK_CRUISE_METRES"
TUNNEL_HEIGHT_METRES = 0.90
NET_DISTANCE_METRES = 0.9
TURNING_CIRCLE_METRES = 0.6
TUNNEL_SPEED_METRES_PER_SECOND = 0.25
HIGH_SPEED_METRES_PER_SECOND = 0.6
# The team's choices, corrected from the map files (see above).
S_METRES = (2.17, 2.14)
GO_LINE_EAST_METRES = 2.17
GO_END_NORTH_METRES = -0.95
# The team's choice: with TEAM_NAV_CROSS_CORRIDOR_END=on the drone first flies north to here. The
# team asked for 2.35 "at least 0.7 m from the north net"; the net is at north 3.048, so 2.34.
CORRIDOR_END_SETTING = "TEAM_NAV_CROSS_CORRIDOR_END"
CORRIDOR_END_NORTH_METRES = 2.34
# The team's choices for the descent at S: the target slides at the fast speed down to the slow
# part's top, then at the slow speed. No dip below the tunnel height.
DESCENT_FAST_METRES_PER_SECOND = 0.7
DESCENT_SLOW_FROM_METRES = 1.2
DESCENT_SLOW_METRES_PER_SECOND = 0.15
# The team's choices for the backup before the LOOK.
LOOK_BACKUP_SETTING = "TEAM_NAV_LOOK_BACKUP_METRES"
DEFAULT_LOOK_BACKUP_METRES = 0.26
LARGEST_LOOK_BACKUP_METRES = 0.5
LOOK_BACKUP_SPEED_METRES_PER_SECOND = 0.10
LOOK_BACKUP_SETTLE_SECONDS = 1.0
LEAST_LOOK_POINT_NET_METRES = 0.62
# The team's choice for TEAM_NAV_AFTER_TUNNEL=pad: the climb spot past the exit should keep this
# far from every wall (a target, not a refusal).
PAD_SPOT_WANTED_WALL_METRES = 0.6
# Our own choices: the short moves near the launch mark and the climbs at 0.5 m/s, the landing's
# medium speed (flight_steps.py); the pad spot is searched this far around the end of the GO line.
CLIMB_SPEED_METRES_PER_SECOND = 0.5
PAD_SPOT_SEARCH_EAST_METRES = (-0.5, 0.6)
PAD_SPOT_SEARCH_NORTH_METRES = (-0.3, 0.4)
PAD_SPOT_SEARCH_STEP_METRES = 0.05
# Our own choice: the least room between the propeller tips and anything on the GO line.
LEAST_TUNNEL_CLEARANCE_METRES = 0.20
# The one kind of high-leg rule break allowed (see above).
ALLOWED_BREAK_START = "The path turns with "

# Stage names, which TEAM_NAV_STOP_AFTER_STAGE may name.
SETTLE_STAGE = "settle"
FORWARD_STAGE = "forward and high climb"
CORRIDOR_STAGE = "corridor end"
HIGH_LEG_STAGE = "to S"
DESCENT_STAGE = "descent"
GATE_STAGE = "gate"
TURN_STAGE = "turn"
LOOK_STAGE = "look"
TUNNEL_STAGE = "tunnel"
PAD_STAGE = "to the pad"


def setting_is_on(name):
    return os.environ.get(name, "").strip().lower() in ("on", "1", "true", "yes")


def cruise_from_environment():
    """TEAM_NAV_FALLBACK_CRUISE_METRES if set (2.2 to 2.5 m, fallback_routes.py checks it), else 2.4."""
    if not os.environ.get(CRUISE_SETTING, "").strip():
        return DEFAULT_CRUISE_HEIGHT_METRES
    return cruise_height_from_environment()


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


@dataclass
class TunnelFirstRoute:
    map_name: str
    cruise_height_metres: float
    launch_metres: np.ndarray
    high_climb_point_metres: np.ndarray
    s_metres: np.ndarray
    go_end_metres: np.ndarray
    corridor_end_metres: object = None        # (east, north), or None when the option is off.
    high_leg_metres: list = field(default_factory=list)   # (east, north) points, S last.
    look_point_metres: np.ndarray = None
    pad_spot_metres: object = None            # With TEAM_NAV_AFTER_TUNNEL=pad: the climb spot past the exit.
    pad_leg_metres: list = field(default_factory=list)    # From the pad spot to the pad, at cruise.
    high_leg_breaks: list = field(default_factory=list)
    high_leg_cautions: list = field(default_factory=list)
    notes: list = field(default_factory=list)

    def describe(self):
        lines = [f"tunnel_first on the map '{self.map_name}', cruise {self.cruise_height_metres:.2f} m:",
                 f"  climb to {CLIMB_HEIGHT_METRES:.2f} m at the launch mark ({self.launch_metres[0]:+.2f}, {self.launch_metres[1]:+.2f});",
                 f"  {FORWARD_METRES:.1f} m north to ({self.high_climb_point_metres[0]:+.2f}, {self.high_climb_point_metres[1]:+.2f}), climb;"]
        if self.corridor_end_metres is not None:
            lines.append(f"  north to the corridor end ({self.corridor_end_metres[0]:+.2f}, {self.corridor_end_metres[1]:+.2f});")
        lines += [f"  high leg of {len(self.high_leg_metres)} points at {HIGH_SPEED_METRES_PER_SECOND} m/s to S "
                  f"({self.s_metres[0]:+.2f}, {self.s_metres[1]:+.2f});",
                  f"  descend to {TUNNEL_HEIGHT_METRES:.2f} m, height gate, turn 180 degrees left;",
                  f"  back to the LOOK point ({self.look_point_metres[0]:+.2f}, {self.look_point_metres[1]:+.2f}), LOOK;",
                  f"  GO south at {TUNNEL_SPEED_METRES_PER_SECOND:.2f} m/s to ({self.go_end_metres[0]:+.2f}, {self.go_end_metres[1]:+.2f})."]
        if self.pad_spot_metres is not None:
            lines.append(f"  slide to ({self.pad_spot_metres[0]:+.2f}, {self.pad_spot_metres[1]:+.2f}), climb, "
                         f"{len(self.pad_leg_metres)} points to the pad.")
        lines += [f"  Allowed rule break: {text}" for text in self.high_leg_breaks]
        lines += [f"  Caution: {text}" for text in self.high_leg_cautions]
        lines += [f"  Check: {text}" for text in self.notes]
        return "\n".join(lines)


def plan_tunnel_first_route(course_map, after_tunnel_to_pad=False):
    """Plans and checks the route on this map. Raises TunnelFirstRouteError if it can't be flown."""
    cruise = cruise_from_environment()
    launch = np.asarray(course_map.launch_mark_centre_metres, dtype=float)
    high_climb_point = launch + np.array([0.0, FORWARD_METRES])
    route = TunnelFirstRoute(course_map.name, cruise, launch, high_climb_point, np.asarray(S_METRES, dtype=float),
                             np.array([GO_LINE_EAST_METRES, GO_END_NORTH_METRES]))
    # Every mission takes off from the launch mark, so something under the laser there is noted, not refused.
    check_open_floor(course_map, route, "the climb at the launch mark", launch, [CLIMB_HEIGHT_METRES], refuse=False)
    check_open_floor(course_map, route, "the high climb point", high_climb_point, [CLIMB_HEIGHT_METRES, cruise])
    if setting_is_on(CORRIDOR_END_SETTING):
        plan_the_corridor_end(course_map, route)
    check_s(course_map, route)
    plan_the_look_backup(course_map, route)
    plan_the_high_leg(course_map, route)
    check_the_go_line(course_map, route)
    if after_tunnel_to_pad:
        plan_the_way_to_the_pad(course_map, route)
    return route


def check_open_floor(course_map, route, what, point_metres, heights_metres, refuse=True, low_obstacles_only_noted=False):
    """Refuses (or, with refuse=False, notes) if a wall, the tunnel or a boulder is under the laser there.
    With low_obstacles_only_noted, a boulder is noted rather than refused."""
    low_names = [obstacle.name for obstacle in course_map.low_obstacles]
    for height_metres in heights_metres:
        for name, margin_metres in open_floor_clearances(course_map, point_metres, height_metres):
            if margin_metres >= 0.0:
                continue
            if not refuse or (low_obstacles_only_noted and name in low_names):
                route.notes.append(f"{what} at {height_metres:.2f} m: {name} is under the laser ({margin_metres:.2f} m).")
                continue
            raise TunnelFirstRouteError(
                f"On this map, {name} would be under the bottom laser at {what} ({point_metres[0]:+.2f}, "
                f"{point_metres[1]:+.2f}) at {height_metres:.2f} m, and the drone stops or changes height there. "
                f"Check the map file, or fly another mission.")
    route.notes.append(f"{what}: open floor at {', '.join(f'{height:.2f}' for height in heights_metres)} m.")


def plan_the_corridor_end(course_map, route):
    corridor_end = np.array([route.high_climb_point_metres[0], CORRIDOR_END_NORTH_METRES])
    check = check_high_path(course_map, [route.high_climb_point_metres, corridor_end], route.cruise_height_metres)
    forbidden = _forbidden_breaks(course_map, check.rule_breaks)
    if forbidden:
        raise TunnelFirstRouteError("The leg north to the corridor end breaks the laser rules on this map: " + " ".join(forbidden))
    check_open_floor(course_map, route, "the corridor end", corridor_end, [route.cruise_height_metres], low_obstacles_only_noted=True)
    route.corridor_end_metres = corridor_end
    route.notes.append(f"Corridor end {course_map.highest_corner_metres[1] - corridor_end[1]:.2f} m from the north net.")


def check_s(course_map, route):
    """S must be over open floor where the drone stops at cruise, descends and turns at 0.90 m, with
    nothing in the 0.6 m turning circle. A low obstacle there is only noted (see the file header)."""
    for height_metres in (route.cruise_height_metres, DESCENT_SLOW_FROM_METRES, TUNNEL_HEIGHT_METRES):
        check_open_floor(course_map, route, "S", route.s_metres, [height_metres], low_obstacles_only_noted=True)
    for name, distance, _top in _distances_to_everything(course_map, route.s_metres[np.newaxis, :]):
        if float(distance[0]) >= TURNING_CIRCLE_METRES:
            continue
        what = f"{name} is {float(distance[0]):.2f} m from S, inside the {TURNING_CIRCLE_METRES:.1f} m turning circle"
        if name in [obstacle.name for obstacle in course_map.low_obstacles]:
            route.notes.append(f"{what}; it is a low obstacle, below the drone's lowest point during the turn.")
            continue
        raise TunnelFirstRouteError(f"On this map, {what}. The drone turns there. Check the map file.")
    net_metres = float(np.min(np.r_[route.s_metres - course_map.lowest_corner_metres, course_map.highest_corner_metres - route.s_metres]))
    if net_metres < TURNING_CIRCLE_METRES:
        raise TunnelFirstRouteError(f"S is {net_metres:.2f} m from the net, inside the turning circle.")
    route.notes.append(f"S is {net_metres:.2f} m from the nearest net.")


def plan_the_look_backup(course_map, route):
    """The LOOK point, north of S by the backup. Refuses the start if the drone's centre would come
    closer than LEAST_LOOK_POINT_NET_METRES to the north net, or near anything solid on the way."""
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
    check_open_floor(course_map, route, "the LOOK point", look_point, [TUNNEL_HEIGHT_METRES], low_obstacles_only_noted=True)
    route.look_point_metres = look_point
    route.notes.append(f"LOOK point {backup_metres:.2f} m north of S: {north_net_metres:.2f} m from the north net.")


def plan_the_high_leg(course_map, route):
    start = route.corridor_end_metres if route.corridor_end_metres is not None else route.high_climb_point_metres
    turn_box = (np.asarray(course_map.lowest_corner_metres) + NET_DISTANCE_METRES,
                np.asarray(course_map.highest_corner_metres) - NET_DISTANCE_METRES)
    path, check = shortest_high_path(course_map, start, route.s_metres, route.cruise_height_metres, turn_box)
    if _forbidden_breaks(course_map, check.rule_breaks):
        path, check = _best_leg_with_one_corner(course_map, start, route.s_metres, route.cruise_height_metres, (path, check))
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


def _best_leg_with_one_corner(course_map, start, goal, height_metres, best):
    """Along the start's line first, then one corner, then to the goal: the path the search misses on
    the phone scan, because its turn points must be clear of low obstacles."""
    best_cost = (len(_forbidden_breaks(course_map, best[1].rule_breaks)), best[1].cost())
    lowest_north, highest_north = sorted([start[1], goal[1]])
    lowest_east, highest_east = sorted([start[0], goal[0]])
    for corner_north in np.arange(lowest_north, highest_north + 1e-9, 0.05):
        for corner_east in np.arange(lowest_east + 0.3, highest_east - 0.1, 0.1):
            try:
                path = rounded_corners([start, [corner_east, corner_north], goal])
            except FallbackRouteError:
                continue
            check = check_high_path(course_map, path, height_metres)
            cost = (len(_forbidden_breaks(course_map, check.rule_breaks)), check.cost())
            if cost < best_cost:
                best, best_cost = (path, check), cost
    return best


def check_the_go_line(course_map, route):
    """Refuses if the propeller tips would pass anything with less than LEAST_TUNNEL_CLEARANCE_METRES
    on the way from the LOOK point to the end, at the tunnel height, and checks the landing spot."""
    north_values = np.arange(route.look_point_metres[1], route.go_end_metres[1] - 1e-9, -0.01)
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
    check_open_floor(course_map, route, "the landing past the tunnel", route.go_end_metres, [TUNNEL_HEIGHT_METRES])


def plan_the_way_to_the_pad(course_map, route):
    """TEAM_NAV_AFTER_TUNNEL=pad: the spot near the end of the GO line with the most room from every
    wall (and the tunnel), reachable straight at the tunnel height and open floor up to cruise for the
    straight climb; then the shortest path that keeps the laser rules from there to the pad."""
    cruise = route.cruise_height_metres
    spot, room = choose_pad_spot(course_map, route.go_end_metres, cruise)
    if spot is None:
        raise TunnelFirstRouteError("With TEAM_NAV_AFTER_TUNNEL=pad there is no open floor near the end of the GO line to climb "
                                    "from on this map. Use TEAM_NAV_AFTER_TUNNEL=land.")
    path, check = shortest_high_path(course_map, spot, course_map.landing_pad_centre_metres, cruise)
    if _forbidden_breaks(course_map, check.rule_breaks):
        path, check = _best_leg_with_one_corner(course_map, spot, np.asarray(course_map.landing_pad_centre_metres), cruise,
                                                (path, check))
    forbidden = _forbidden_breaks(course_map, check.rule_breaks)
    if forbidden:
        raise TunnelFirstRouteError("With TEAM_NAV_AFTER_TUNNEL=pad, the way from the tunnel to the pad breaks the laser rules: "
                                    + " ".join(forbidden) + " Use TEAM_NAV_AFTER_TUNNEL=land.")
    route.pad_spot_metres = spot
    route.pad_leg_metres = [np.asarray(point, dtype=float) for point in path]
    route.high_leg_breaks += [f"(to the pad) {text}" for text in check.rule_breaks]
    route.high_leg_cautions += [f"(to the pad) {text}" for text in check.cautions]
    wanted = "" if room >= PAD_SPOT_WANTED_WALL_METRES else f", short of the {PAD_SPOT_WANTED_WALL_METRES:.1f} m wanted"
    route.notes.append(f"Pad spot ({spot[0]:+.2f}, {spot[1]:+.2f}): {room:.2f} m from the nearest wall or the tunnel{wanted}.")


def choose_pad_spot(course_map, go_end_metres, cruise_metres):
    """(spot, room) with the most room from walls and the tunnel, or (None, 0)."""
    go_end = np.asarray(go_end_metres, dtype=float)
    blocking = [shape for shape in shapes_blocking_between_heights(course_map, TUNNEL_HEIGHT_METRES - 0.2, TUNNEL_HEIGHT_METRES + 0.2)
                if shape.name != "the safety net"]
    best, best_room = None, 0.0
    for east in np.arange(*PAD_SPOT_SEARCH_EAST_METRES, PAD_SPOT_SEARCH_STEP_METRES):
        for north in np.arange(*PAD_SPOT_SEARCH_NORTH_METRES, PAD_SPOT_SEARCH_STEP_METRES):
            spot = go_end + np.array([east, north])
            room = _room_from_walls_and_tunnel(course_map, spot)
            if room <= best_room or not _straight_line_is_clear(blocking, go_end, spot):
                continue
            if not _open_floor_all_the_way_up(course_map, spot, cruise_metres):
                continue
            best, best_room = spot, room
    return best, best_room


def _room_from_walls_and_tunnel(course_map, spot):
    point = spot[np.newaxis, :]
    rooms = [distance_to_line_piece_metres(point, wall.start_metres, wall.end_metres)[0] - wall.thickness_metres / 2.0
             for wall in course_map.walls]
    rooms += [distance_to_tunnel_metres(point, tube)[0] for tube in course_map.tubes]
    net = float(np.min(np.r_[spot - course_map.lowest_corner_metres, course_map.highest_corner_metres - spot]))
    return min(rooms + [net])


def _straight_line_is_clear(blocking, start, end):
    points = start + np.linspace(0.0, 1.0, 20)[:, np.newaxis] * (end - start)
    distances, _ = distance_to_shapes_metres(points, blocking)
    return bool(distances.min() >= REQUIRED_ROOM_METRES)


def _open_floor_all_the_way_up(course_map, spot, cruise_metres):
    """No wall or tunnel under the laser at any height of the straight climb (boulders allowed: the
    climb passes over them only with the laser, never the body)."""
    low_names = [obstacle.name for obstacle in course_map.low_obstacles]
    for height_metres in (TUNNEL_HEIGHT_METRES, 1.5, cruise_metres):
        for name, margin_metres in open_floor_clearances(course_map, spot, height_metres):
            if margin_metres < 0.0 and name not in low_names:
                return False
    return True
