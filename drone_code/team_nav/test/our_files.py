"""Loads the map and mission files kept in this package, for the tests.

The tests use the same files the drone uses, so that a mistake in a file shows up as a failing
test rather than during a flight.
"""
import os

from team_nav.course_map import load_course_map
from team_nav.mission import load_mission

PACKAGE_FOLDER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS_FOLDER = os.path.join(PACKAGE_FOLDER, "course_maps")
MISSIONS_FOLDER = os.path.join(PACKAGE_FOLDER, "missions")

# The three maps we keep, by the name of their file without ".json".
DRAWING_MAP = "organizers_drawing"
SCAN_MAP = "phone_scan_2026-11-06"
SCAN_MAP_TRUSTING_EVERY_WALL = "phone_scan_2026-11-06_trusting_every_wall"
ALL_OUR_MAPS = (DRAWING_MAP, SCAN_MAP, SCAN_MAP_TRUSTING_EVERY_WALL)


def load_our_map(map_name):
    return load_course_map(os.path.join(MAPS_FOLDER, f"{map_name}.json"))


def load_rulebook_mission():
    """The mission that tries the window first, for later competition runs."""
    return load_mission(os.path.join(MISSIONS_FOLDER, "dronehacks_2026.json"))


def load_side_gap_first_mission():
    """The mission the team chose for the first competition run."""
    return load_mission(os.path.join(MISSIONS_FOLDER, "dronehacks_2026_side_gap_first.json"))


def boulders_from_every_map():
    """Every boulder in every map we keep. The same boulder may appear more than once."""
    boulders = []
    for map_name in ALL_OUR_MAPS:
        boulders += load_our_map(map_name).low_obstacles
    return boulders
