"""Plans full_course's route with Aditya's A* planner, for a drone whose nose stays north.

REAL DRONE CODE, run on the ground before anything is sent. Uses the participant map only.

Aditya's planner (astar_route.py) turns the nose to face each leg. full_course never turns:
the nose points north the whole flight. So this file gives the same search a different
geometry, NorthFacingGeometry, which checks every point with the airframe facing north and
asks for at least PROPELLER_CLEARANCE_METRES between the propellers and anything on the map.
"Turning" is then free, because the airframe doesn't rotate. It also treats the window as
closed (map_with_every_opening_closed says why), so the chicane is entered through Gate B.

The route
---------
  1. Straight north from the launch mark to the first checkpoint's turn-back point (north
     2.515 m, full_course_route.py). The drone turns back as soon as the whole drone is past the
     line, so this point is a target, not a hover point. It is closer to the top wall than
     PROPELLER_CLEARANCE_METRES (0.086 m there), which is why A* starts from BACK_OFF_POINT instead.
  2. A* from BACK_OFF_POINT to each remaining goal in missions/real_course_five_goals.json, in
     order, then fewer_points removes the points that a shortcut with as much room can replace.

Unlike Aditya's planner, the chicane leg does not use ChicaneErrorMargin (room for the drone
being 0.25 m further north than planned). On real_course_2026-11-08.json the passage over the
north end of C2b leaves only a 15 cm band for the drone's centre with 0.12 m of propeller
clearance on both sides (the barrier and the sketched wall to the tunnel are north of it), so a
0.25 m margin cannot fit. A*'s preference for room keeps the path in the middle of the band.

What it does not do: replan in flight, or check the route against anything but the map.
"""
import copy
import json
import math

import numpy as np

from team_nav.practice_flights.astar_geometry import (
    LASER_MARGIN_METRES, OUTER_RADIUS_METRES, TRANSLATION_SAMPLE_METRES, RouteGeometry,
)
from team_nav.practice_flights.astar_route import search_route
from team_nav.practice_flights.full_course_route import CRUISE_HEIGHT_METRES, MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES

# The team's requirement of 2026-11-08: the propellers keep at least this far from anything.
PROPELLER_CLEARANCE_METRES = 0.12
NORTH_RADIANS = math.pi / 2
# Our own choice: 0.115 m south of the turn-back point, where the propellers have 0.20 m to the top
# wall on the sketch, so A* can start there with full clearance.
BACK_OFF_NORTH_METRES = 2.40
MISSION_FILE_NAME = 'real_course_five_goals.json'
# Our own choice. The first full run on real_course_2026-11-08.json (Gazebo run 20261108_131638) had
# 25 legs and took 153 s, over the team's 150 s limit. Giving up at most 2 cm of room removes 3 points.
ROOM_GIVEN_UP_FOR_A_SHORTCUT_METRES = 0.02


class NorthFacingGeometry:
    """RouteGeometry's checks, always with the nose north and PROPELLER_CLEARANCE_METRES of room."""

    def __init__(self, course_map):
        self.inner = RouteGeometry(map_with_every_opening_closed(course_map), CRUISE_HEIGHT_METRES)
        self.course_map = course_map

    def centre_clearance(self, points):
        return self.inner.centre_clearance(points)

    def airframe_is_clear(self, centres):
        centres = np.atleast_2d(centres)
        # Far from everything, the whole airframe (radius 0.30 m) is clear whatever its shape.
        tight = self.centre_clearance(centres) < OUTER_RADIUS_METRES + PROPELLER_CLEARANCE_METRES
        if not np.any(tight):
            return True
        return bool(np.all(self.inner.footprint_clearance(centres[tight], NORTH_RADIANS) >= PROPELLER_CLEARANCE_METRES))

    def turn_clear(self, point, old_heading, new_heading):
        # The nose never turns, so only the point itself must be clear.
        return self.airframe_is_clear(point)

    def segment_clear(self, start, end, heading):
        count = max(2, int(math.ceil(np.linalg.norm(end - start) / TRANSLATION_SAMPLE_METRES)) + 1)
        centres = np.linspace(start, end, count)
        if not self.airframe_is_clear(centres):
            return False
        # As Aditya's planner: keep the bottom laser off boulders, so it keeps seeing the floor.
        for obstacle in self.course_map.low_obstacles:
            if np.min(np.linalg.norm(centres - obstacle.centre_metres, axis=1)) < obstacle.radius_metres + LASER_MARGIN_METRES:
                return False
        return True


def fewer_points(geometry, path):
    """The same path with as many points removed as possible, keeping its room.

    The drone stops at every point (full_course's arrival rule), so each point costs about 3.5 s.
    A shortcut from one point to a later one is taken only if the airframe keeps
    PROPELLER_CLEARANCE_METRES along it, and the centre keeps as much room as on the stretch of A*
    path it replaces, less at most ROOM_GIVEN_UP_FOR_A_SHORTCUT_METRES."""
    result = [path[0]]
    index = 0
    while index < len(path) - 1:
        chosen = index + 1
        for candidate in range(len(path) - 1, index + 1, -1):
            if _shortcut_keeps_its_room(geometry, path[index:candidate + 1]):
                chosen = candidate
                break
        result.append(path[chosen])
        index = chosen
    return result


def _shortcut_keeps_its_room(geometry, stretch):
    start, end = stretch[0], stretch[-1]
    if not geometry.segment_clear(start, end, NORTH_RADIANS):
        return False
    room_before_metres = math.inf
    for first, second in zip(stretch, stretch[1:]):
        room_before_metres = min(room_before_metres, float(geometry.centre_clearance(np.linspace(first, second, 20)).min()))
    shortcut_room_metres = float(geometry.centre_clearance(np.linspace(start, end, 40)).min())
    return shortcut_room_metres >= room_before_metres - ROOM_GIVEN_UP_FOR_A_SHORTCUT_METRES


def map_with_every_opening_closed(course_map):
    """A copy of the map in which no wall has an opening, so the route never uses the window.

    The organizers' model puts the window 0.32 to 1.20 m high, so at 0.75 m A* would fly through it.
    Our phone scan of the built course puts it 0.81 to 1.56 m high, where a drone at 0.75 m would hit
    the wall below it. Until a photo settles it, the route goes through Gate B instead."""
    closed = copy.deepcopy(course_map)
    for wall in closed.walls:
        wall.openings = []
    return closed


def load_mission(path):
    with open(path, encoding='utf-8') as source:
        return json.load(source)


def plan_north_facing_route(course_map, mission):
    """The route as an N x 3 array of (east, north, height), launch mark first. Raises
    ValueError, with a sentence saying why, if A* finds no route."""
    geometry = NorthFacingGeometry(course_map)
    launch = np.array(mission['start_metres'][:2], dtype=float)
    turn_back = np.array([launch[0], MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES])
    back_off = np.array([launch[0], BACK_OFF_NORTH_METRES])
    points = [launch, turn_back]
    start = back_off
    for goal in mission['goals'][1:]:
        target = np.array(goal['point_metres'][:2], dtype=float)
        path, _ = search_route(geometry, start, target, NORTH_RADIANS)
        points.extend(fewer_points(geometry, path)[1:])
        start = target
    route = np.array(points)
    return np.column_stack((route, np.full(len(route), CRUISE_HEIGHT_METRES)))
