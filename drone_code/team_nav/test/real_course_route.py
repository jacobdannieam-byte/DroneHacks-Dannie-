"""The real-course map and full_course's fixed route on it, checked, for the tests."""
from our_files import load_our_map
from team_nav.practice_flights.full_course_fixed_route import fixed_route_metres
from team_nav.practice_flights.full_course_route import check_route

REAL_COURSE_MAP = 'real_course_2026-11-08'


def real_course_map_and_route():
    """(a fresh copy of the map, the checked route)."""
    course_map = load_our_map(REAL_COURSE_MAP)
    return course_map, check_route(course_map, fixed_route_metres())
