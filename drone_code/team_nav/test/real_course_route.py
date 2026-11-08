"""The real-course map and full_course's A* route on it, planned once and shared by the tests.

Planning takes about 10 seconds, so the tests reuse one plan instead of planning again.
"""
import functools
import os

from our_files import MISSIONS_FOLDER, load_our_map
from team_nav.practice_flights.full_course_route import check_route
from team_nav.practice_flights.north_facing_astar_route import MISSION_FILE_NAME, load_mission, plan_north_facing_route

REAL_COURSE_MAP = 'real_course_2026-11-08'


def real_course_mission():
    return load_mission(os.path.join(MISSIONS_FOLDER, MISSION_FILE_NAME))


@functools.lru_cache(maxsize=1)
def _planned_route():
    return check_route(load_our_map(REAL_COURSE_MAP), plan_north_facing_route(load_our_map(REAL_COURSE_MAP),
                                                                             real_course_mission()))


def real_course_map_and_route():
    """(a fresh copy of the map, a copy of the planned route)."""
    return load_our_map(REAL_COURSE_MAP), _planned_route().copy()
