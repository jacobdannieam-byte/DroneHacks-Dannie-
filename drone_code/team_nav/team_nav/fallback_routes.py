"""Plans the two fallback routes, which fly high over the walls (2.3 m) to make sure of the landing
points even if the drone collects penalties on the way.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. fallback_run.py calls plan_fallback_route once, before anything is sent to the drone, and flies
the result with the route follower (route_follower.py) and the landing from flight_steps.py.

The two routes (the team's design, 2026-11-07)
-----------------------------------------------
fallback_pad:
  climb to 2.3 m on the launch mark; fly the shortest path to above the landing pad that keeps the
  bottom laser's rules (high_path_rules.py); then land with the normal landing.
fallback_tunnel:
  climb to 2.3 m on the launch mark; fly north up the first corridor and east along a line
  NORTH_LEG_FROM_NET_METRES from the north net, over the maze; turn south onto the tunnel's centre
  line and stop TUNNEL_APPROACH_METRES before its north entrance; descend to the tunnel's centre
  height; fly straight through at 0.5 m/s; stop TUNNEL_EXIT_CLEAR_METRES past the exit and climb
  back to 2.3 m; fly over the chicane to above the pad by the shortest path that keeps the rules;
  land.

Why the tunnel route goes north first
-------------------------------------
The maze walls may change (one removed, one slanted), so the route over the maze must not depend
on where they are. A line close to the north net, running east, crosses every maze wall that
reaches the net square-on, wherever it stands, and passes over a wall that does not reach that
far without touching it. It also gives the drone a long run-up before the first wall, and lets it
arrive on the tunnel's centre line already heading into the tunnel.

How the turns work
------------------
Every turn is an arc made of straight pieces ARC_PIECE_METRES long that each turn by at most
ARC_STEP_DEGREES. The route follower only stops at waypoints where the route turns by more than
10 degrees, or next to a very short leg (stopping_spots.py), so it flies through an arc at full
speed. The drone stops only at
the climbs, the descents and the pad, all over open floor.

The cruise height can be changed for one run with TEAM_NAV_FALLBACK_CRUISE_METRES (drone.sh passes
it into the container); without it the route flies at DEFAULT_HIGH_HEIGHT_METRES.

What it does not do: fly, or change anything about the normal course route (route_planner.py).
"""
import math
import os
from dataclasses import dataclass, field

import numpy as np

from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES, REQUIRED_ROOM_METRES
from team_nav.high_path_rules import check_high_path, open_floor_clearances

FALLBACK_PAD = "fallback_pad"
FALLBACK_TUNNEL = "fallback_tunnel"
FALLBACK_ROUTES = (FALLBACK_PAD, FALLBACK_TUNNEL)

# The team's choice (2026-11-07): above every wall top (1.73 to 1.87 m) by more than the drone's
# half height, and below the net.
DEFAULT_HIGH_HEIGHT_METRES = 2.3
# The team's choice (2026-11-08): the cruise height may be set for one run with this environment
# setting, but only within these limits. The lowest keeps the drone's underside 0.23 m over the
# tallest (1.87 m) wall; the highest keeps the route's highest target (cruise + 0.15 m) at 2.65 m.
CRUISE_HEIGHT_SETTING = "TEAM_NAV_FALLBACK_CRUISE_METRES"
LOWEST_ALLOWED_CRUISE_METRES = 2.2
HIGHEST_ALLOWED_CRUISE_METRES = 2.5


def cruise_height_from_environment():
    """TEAM_NAV_FALLBACK_CRUISE_METRES if it is set, else DEFAULT_HIGH_HEIGHT_METRES."""
    text = os.environ.get(CRUISE_HEIGHT_SETTING, "").strip()
    if not text:
        return DEFAULT_HIGH_HEIGHT_METRES
    try:
        height_metres = float(text)
    except ValueError:
        raise ValueError(f"{CRUISE_HEIGHT_SETTING} is '{text}', which is not a number of metres. "
                         f"Set it to a height such as 2.4, or leave it unset for {DEFAULT_HIGH_HEIGHT_METRES} m.")
    if not LOWEST_ALLOWED_CRUISE_METRES <= height_metres <= HIGHEST_ALLOWED_CRUISE_METRES:
        raise ValueError(f"{CRUISE_HEIGHT_SETTING} is {height_metres} m, outside the allowed "
                         f"{LOWEST_ALLOWED_CRUISE_METRES} to {HIGHEST_ALLOWED_CRUISE_METRES} m. Choose a height in that range.")
    return height_metres


HIGH_HEIGHT_METRES = cruise_height_from_environment()
# Measured: drone 16's flight_interface clamped targets to 3.00 m above the start point in the
# logs of 2026-11-07 (the organizers' max_altitude_m). From the team: the net roof is 3.05 m high.
FLIGHT_INTERFACE_CEILING_METRES = 3.00
NET_ROOF_HEIGHT_METRES = 3.05
# Our own choice: run_safety.py refuses any target higher than this during a fallback route. It
# replaces the rulebook's "below the wall tops" limit, which these routes break on purpose.
HIGHEST_TARGET_HEIGHT_METRES = HIGH_HEIGHT_METRES + 0.15

# Our own choices for the route follower. Up high the target slides at 1.0 m/s and may lead the
# drone by 1.0 m; PX4 then flies at about 0.77 m/s (docs/ARCHITECTURE.md, section 9), so walls are
# crossed above 0.5 m/s. The team asked for 0.5 m/s through the tunnel. Climbs and descents slide
# at 0.5 m/s, the landing's medium speed (flight_steps.py), so PX4 never overshoots towards the roof.
HIGH_SPEED_METRES_PER_SECOND = 1.0
TUNNEL_SPEED_METRES_PER_SECOND = 0.5
CLIMB_SPEED_METRES_PER_SECOND = 0.5
TARGET_LEAD_METRES = 1.0

# Our own choices for the turns. The route follower flies straight through a waypoint only if the
# route turns there by 10 degrees or less and both legs beside it are at least 0.05 m long
# (stopping_spots.py). So an arc is made of pieces ARC_PIECE_METRES long, each turning at most
# ARC_STEP_DEGREES. A 90 degree turn then has a radius of about 0.5 m.
ARC_PIECE_METRES = 0.08
ARC_STEP_DEGREES = 9.0
# Our own choice: a little more than stopping_spots.py's SHORTEST_LEG_WITH_A_DIRECTION_METRES (0.05).
SHORTEST_PIECE_METRES = 0.06

# The team's choice: the descent point is about 0.6 m before the tunnel's entrance. Our own choice:
# the climb point is as far past the exit, so the whole drone (0.30 m radius) is clear of the tube.
TUNNEL_APPROACH_METRES = 0.6
TUNNEL_EXIT_CLEAR_METRES = 0.6
# Our own choice: the east leg over the maze runs this far from the north net, the drone's 0.35 m
# of room plus 0.10 m for camera tracking's drift.
NORTH_LEG_FROM_NET_METRES = REQUIRED_ROOM_METRES + 0.10

# Our own choice: turn points for the searched parts are tried on a grid this fine.
TURN_SEARCH_SPACING_METRES = 0.1
# Our own choice: with two turns, only this many of the best first and last turn points are paired.
BEST_TURN_POINTS_TO_PAIR = 40

# Stage names (fallback_run.py), which practice mode can stop after.
HIGH_CLIMB_STAGE = "high climb"
OVER_THE_MAZE_STAGE = "over the maze"
TUNNEL_STAGE = "tunnel"


class FallbackRouteError(ValueError):
    """The map does not allow a fallback route. The message says why, in full sentences."""


@dataclass
class FallbackRoute:
    name: str
    map_name: str
    waypoints_metres: np.ndarray         # N x 3: metres east, metres north, height above the floor.
    leg_speeds_metres_per_second: list   # The speed of the leg into each waypoint (the first is unused).
    stages: list                         # (stage name, number of the waypoint where it ends)
    path_checks: list                    # (what the part is, high_path_rules.PathCheck)
    height_change_places: list           # (what happens there, (east, north), clearances)
    notes: list = field(default_factory=list)

    def length_metres(self):
        return float(np.linalg.norm(np.diff(self.waypoints_metres, axis=0), axis=1).sum())

    def rule_breaks(self):
        return [text for _, check in self.path_checks for text in check.rule_breaks]

    def describe(self):
        lines = [f"Fallback route '{self.name}' on the map '{self.map_name}': {self.length_metres():.1f} m, "
                 f"{len(self.waypoints_metres)} waypoints, at {HIGH_HEIGHT_METRES:.1f} m over the walls."]
        lines += [f"  {note}" for note in self.notes]
        for part_name, check in self.path_checks:
            lines.append(f"  {part_name}: {check.length_metres:.2f} m.")
            lines += [f"    It {crossing.describe()}." for crossing in check.crossings]
            lines += [f"    RULE BROKEN: {text}" for text in check.rule_breaks]
            lines += [f"    Caution: {text}" for text in check.cautions]
        for what, point, clearances in self.height_change_places:
            name, margin_metres = min(clearances, key=lambda pair: pair[1])
            verdict = "open floor" if margin_metres >= 0 else "CLOSE TO SOMETHING"
            lines.append(f"  {what} at ({point[0]:+.2f}, {point[1]:+.2f}): {verdict}; the nearest thing is {name}, "
                         f"{margin_metres:+.2f} m beyond the room the laser needs there.")
        lines.append("Waypoints (metres east, metres north, height above the floor, speed into it in m/s):")
        for number, waypoint in enumerate(self.waypoints_metres):
            speed = self.leg_speeds_metres_per_second[number]
            lines.append(f"  {number + 1:2d}: ({waypoint[0]:+.2f}, {waypoint[1]:+.2f}, {waypoint[2]:.2f}) {speed:.1f}")
        return "\n".join(lines)


def plan_fallback_route(course_map, route_name):
    """Returns the FallbackRoute called route_name (FALLBACK_PAD or FALLBACK_TUNNEL) on this map,
    starting from the drone hovering at cruise height above the launch mark. Raises
    FallbackRouteError if the map does not allow it."""
    check_heights_fit_under_the_roof()
    if route_name == FALLBACK_PAD:
        return _plan_pad_route(course_map)
    if route_name == FALLBACK_TUNNEL:
        return _plan_tunnel_route(course_map)
    raise FallbackRouteError(f"There is no fallback route called '{route_name}'. Choose one of: "
                             f"{', '.join(FALLBACK_ROUTES)}.")


def check_heights_fit_under_the_roof():
    if HIGHEST_TARGET_HEIGHT_METRES >= FLIGHT_INTERFACE_CEILING_METRES:
        raise FallbackRouteError("The fallback routes' highest target is not below flight_interface's 3.00 m ceiling.")
    if HIGH_HEIGHT_METRES + DRONE_HALF_HEIGHT_METRES >= NET_ROOF_HEIGHT_METRES - REQUIRED_ROOM_METRES:
        raise FallbackRouteError("At the fallback routes' height the drone would come too close to the net roof.")


# ---- The two routes ------------------------------------------------------------------------------

class _RouteBuilder:
    """Collects waypoints, the speed into each, and where each stage ends."""

    def __init__(self, start_metres):
        self.waypoints = [np.asarray(start_metres, dtype=float)]
        self.speeds = [0.0]
        self.stages = []

    def go_to(self, east_north_metres, height_metres, speed_metres_per_second):
        point = np.array([east_north_metres[0], east_north_metres[1], height_metres], dtype=float)
        if np.allclose(point, self.waypoints[-1]):
            return
        self.waypoints.append(point)
        self.speeds.append(speed_metres_per_second)

    def follow(self, path_points_metres, height_metres, speed_metres_per_second):
        for point in path_points_metres:
            self.go_to(point, height_metres, speed_metres_per_second)

    def end_stage(self, name):
        self.stages.append((name, len(self.waypoints) - 1))


def _plan_pad_route(course_map):
    launch = np.asarray(course_map.launch_mark_centre_metres, dtype=float)
    pad = np.asarray(course_map.landing_pad_centre_metres, dtype=float)
    builder, places = _start_with_the_high_climb(course_map, launch)
    path, check = shortest_high_path(course_map, launch, pad)
    builder.follow(path, HIGH_HEIGHT_METRES, HIGH_SPEED_METRES_PER_SECOND)
    places.append(("Descent to land on the pad", pad, open_floor_clearances(course_map, pad, HIGH_HEIGHT_METRES)))
    return FallbackRoute(FALLBACK_PAD, course_map.name, np.array(builder.waypoints), builder.speeds, builder.stages,
                         [("Launch mark to the pad", check)], places)


def _plan_tunnel_route(course_map):
    launch = np.asarray(course_map.launch_mark_centre_metres, dtype=float)
    pad = np.asarray(course_map.landing_pad_centre_metres, dtype=float)
    tunnel = _tunnel_geometry(course_map)
    builder, places = _start_with_the_high_climb(course_map, launch)

    maze_path = rounded_corners(_maze_corners(course_map, launch, tunnel))
    maze_check = check_high_path(course_map, maze_path, HIGH_HEIGHT_METRES)
    builder.follow(maze_path, HIGH_HEIGHT_METRES, HIGH_SPEED_METRES_PER_SECOND)
    builder.end_stage(OVER_THE_MAZE_STAGE)

    builder.go_to(tunnel.descent_point, tunnel.centre_height_metres, CLIMB_SPEED_METRES_PER_SECOND)
    builder.go_to(tunnel.climb_point, tunnel.centre_height_metres, TUNNEL_SPEED_METRES_PER_SECOND)
    builder.end_stage(TUNNEL_STAGE)
    builder.go_to(tunnel.climb_point, HIGH_HEIGHT_METRES, CLIMB_SPEED_METRES_PER_SECOND)

    chicane_path, chicane_check = shortest_high_path(course_map, tunnel.climb_point, pad)
    builder.follow(chicane_path, HIGH_HEIGHT_METRES, HIGH_SPEED_METRES_PER_SECOND)
    for what, point in [("Descent to the tunnel", tunnel.descent_point), ("Climb after the tunnel", tunnel.climb_point),
                        ("Descent to land on the pad", pad)]:
        places.append((what, point, open_floor_clearances(course_map, point, HIGH_HEIGHT_METRES)))
    notes = [f"The tunnel is flown at {tunnel.centre_height_metres:.2f} m, its centre height, at "
             f"{TUNNEL_SPEED_METRES_PER_SECOND:.1f} m/s."]
    return FallbackRoute(FALLBACK_TUNNEL, course_map.name, np.array(builder.waypoints), builder.speeds, builder.stages,
                         [("Over the maze to the tunnel", maze_check), ("Tunnel exit over the chicane to the pad",
                                                                         chicane_check)], places, notes)


def _start_with_the_high_climb(course_map, launch):
    """The route starts where flight_steps.climb_above_launch leaves the drone: at the mission's
    cruise height above the launch mark. Its first leg climbs straight up to HIGH_HEIGHT_METRES."""
    builder = _RouteBuilder((launch[0], launch[1], CRUISE_HEIGHT_AT_THE_START_METRES))
    builder.go_to(launch, HIGH_HEIGHT_METRES, CLIMB_SPEED_METRES_PER_SECOND)
    builder.end_stage(HIGH_CLIMB_STAGE)
    places = [("Climb on the launch mark", launch, open_floor_clearances(course_map, launch, HIGH_HEIGHT_METRES))]
    return builder, places


# From both mission files (missions/*.json): the drone takes off to 0.8 m, as in the normal course
# run, before a fallback route begins. fallback_run.py checks that the mission says the same.
CRUISE_HEIGHT_AT_THE_START_METRES = 0.8


# ---- The tunnel and the maze line ----------------------------------------------------------------

@dataclass
class _TunnelGeometry:
    along: np.ndarray               # Unit vector from the entrance to the exit, seen from above.
    entrance: np.ndarray
    descent_point: np.ndarray       # On the centre line, TUNNEL_APPROACH_METRES before the entrance.
    climb_point: np.ndarray         # On the centre line, TUNNEL_EXIT_CLEAR_METRES past the exit.
    centre_height_metres: float


def _tunnel_geometry(course_map):
    if len(course_map.tubes) != 1:
        raise FallbackRouteError(f"The map '{course_map.name}' has {len(course_map.tubes)} tunnels; fallback_tunnel "
                                 f"needs exactly one.")
    tube = course_map.tubes[0]
    entrance, exit_centre = np.asarray(tube.entrance_centre_metres), np.asarray(tube.exit_centre_metres)
    along = (exit_centre - entrance) / np.linalg.norm(exit_centre - entrance)
    # Our own choice: the route is built for a tunnel entered from the north, as on the course.
    if along[1] > -0.9:
        raise FallbackRouteError(f"The {tube.name} does not run from north to south on the map '{course_map.name}', "
                                 f"which fallback_tunnel's route over the maze is built for.")
    return _TunnelGeometry(along, entrance, entrance - along * TUNNEL_APPROACH_METRES,
                           exit_centre + along * TUNNEL_EXIT_CLEAR_METRES, tube.outside_height_metres / 2.0)


def _maze_corners(course_map, launch, tunnel):
    """Launch mark, north up the corridor, east near the north net, south down the tunnel's line."""
    north_leg_metres = course_map.highest_corner_metres[1] - NORTH_LEG_FROM_NET_METRES
    up_the_corridor = np.array([launch[0], north_leg_metres])
    # Where the east line meets the tunnel's centre line, extended north.
    steps_back = (north_leg_metres - tunnel.entrance[1]) / -tunnel.along[1]
    above_the_tunnel_line = tunnel.entrance - tunnel.along * steps_back
    return [launch, up_the_corridor, above_the_tunnel_line, tunnel.descent_point]


# ---- The searched parts --------------------------------------------------------------------------

def shortest_high_path(course_map, start_metres, goal_metres):
    """The shortest path from start to goal (both stops) with the fewest rule breaks, then the
    fewest cautions: a straight line, else one turn, else two turns. Returns (path points, check)."""
    start, goal = np.asarray(start_metres, dtype=float)[:2], np.asarray(goal_metres, dtype=float)[:2]
    best = _best_path(course_map, [[start, goal]])
    if not best[1].rule_breaks and not best[1].cautions:
        return best
    candidates = _turn_point_candidates(course_map)
    best = min([best, _best_path(course_map, [[start, turn, goal] for turn in candidates])], key=_path_cost)
    if not best[1].rule_breaks:
        return best
    two_turns = _two_turn_corners(course_map, start, goal, candidates)
    return min([best, _best_path(course_map, two_turns)], key=_path_cost)


def _two_turn_corners(course_map, start, goal, candidates):
    """Pairs of the turn points whose own legs from the start and to the goal are best."""
    def leg_cost(corners):
        return check_high_path(course_map, corners, HIGH_HEIGHT_METRES).cost()

    first_turns = sorted(candidates, key=lambda turn: leg_cost([start, turn]))[:BEST_TURN_POINTS_TO_PAIR]
    last_turns = sorted(candidates, key=lambda turn: leg_cost([turn, goal]))[:BEST_TURN_POINTS_TO_PAIR]
    pairs = []
    for first in first_turns:
        for last in last_turns:
            if not np.allclose(first, last):
                pairs.append([start, first, last, goal])
    return pairs


def _best_path(course_map, corner_lists):
    """(path points, check) of the corner list whose rounded path is best."""
    best = None
    for corners in corner_lists:
        try:
            path = rounded_corners(corners)
        except FallbackRouteError:
            continue
        candidate = (path, check_high_path(course_map, path, HIGH_HEIGHT_METRES))
        if best is None or _path_cost(candidate) < _path_cost(best):
            best = candidate
    if best is None:
        raise FallbackRouteError("No path with room for its turns was found.")
    return best


def _path_cost(path_and_check):
    return path_and_check[1].cost()


def _turn_point_candidates(course_map):
    """Grid points over open floor, with room for an arc, where a turn could be."""
    lowest = course_map.lowest_corner_metres + REQUIRED_ROOM_METRES
    highest = course_map.highest_corner_metres - REQUIRED_ROOM_METRES
    candidates = []
    for east in np.arange(lowest[0], highest[0] + 1e-9, TURN_SEARCH_SPACING_METRES):
        for north in np.arange(lowest[1], highest[1] + 1e-9, TURN_SEARCH_SPACING_METRES):
            point = np.array([east, north])
            clearances = open_floor_clearances(course_map, point, HIGH_HEIGHT_METRES)
            if min(margin for _, margin in clearances) >= 0.0:
                candidates.append(point)
    return candidates


# ---- Arcs ----------------------------------------------------------------------------------------

def rounded_corners(corners_metres):
    """The path through these (east, north) corners, with every corner that turns by more than
    ARC_STEP_DEGREES replaced by an arc of pieces ARC_PIECE_METRES long. Raises FallbackRouteError
    if a leg is too short for the arcs at its ends."""
    corners = [np.asarray(corner, dtype=float)[:2] for corner in corners_metres]
    path = [corners[0]]
    room_used_metres = 0.0
    for number in range(1, len(corners) - 1):
        arc, cut_metres = _arc_at_corner(corners[number - 1], corners[number], corners[number + 1])
        if room_used_metres + cut_metres > np.linalg.norm(corners[number] - corners[number - 1]) + 1e-9:
            raise FallbackRouteError(f"The leg into the corner at ({corners[number][0]:+.2f}, {corners[number][1]:+.2f}) "
                                     f"is too short for the arcs at its ends.")
        path += arc
        room_used_metres = cut_metres
    if room_used_metres > np.linalg.norm(corners[-1] - corners[-2]) + 1e-9:
        raise FallbackRouteError("The last leg is too short for the arc before it.")
    path.append(corners[-1])
    _refuse_tiny_pieces(path)
    return path


def _refuse_tiny_pieces(path):
    """A piece shorter than the follower's shortest leg with a direction would make it stop there."""
    for start, end in zip(path[:-1], path[1:]):
        if 1e-9 < np.linalg.norm(end - start) < SHORTEST_PIECE_METRES:
            raise FallbackRouteError(f"The path has a piece only {np.linalg.norm(end - start):.3f} m long near "
                                     f"({start[0]:+.2f}, {start[1]:+.2f}), where the drone would stop.")


def _arc_at_corner(before, corner, after):
    """The arc replacing one corner, and how far before and after the corner it starts and ends."""
    arriving = (corner - before) / np.linalg.norm(corner - before)
    leaving = (after - corner) / np.linalg.norm(after - corner)
    turn_radians = math.acos(float(np.clip(arriving @ leaving, -1.0, 1.0)))
    if math.degrees(turn_radians) <= ARC_STEP_DEGREES:
        return [corner], 0.0
    pieces = int(math.ceil(math.degrees(turn_radians) / ARC_STEP_DEGREES))
    step_radians = turn_radians / pieces
    radius_metres = ARC_PIECE_METRES / (2.0 * math.sin(step_radians / 2.0))
    cut_metres = radius_metres * math.tan(turn_radians / 2.0)
    turns_left = arriving[0] * leaving[1] - arriving[1] * leaving[0] > 0
    sign = 1.0 if turns_left else -1.0
    arc_start = corner - arriving * cut_metres
    centre = arc_start + sign * radius_metres * np.array([-arriving[1], arriving[0]])
    from_centre = arc_start - centre
    arc = []
    for piece in range(pieces + 1):
        angle = sign * step_radians * piece
        cosine, sine = math.cos(angle), math.sin(angle)
        arc.append(centre + np.array([cosine * from_centre[0] - sine * from_centre[1],
                                      sine * from_centre[0] + cosine * from_centre[1]]))
    return arc, cut_metres
