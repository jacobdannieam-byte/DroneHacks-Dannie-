"""Tests height_check.py, which stops the run when PX4's height and the bottom laser disagree.

The heights here are made up. The places are on our scan map: the middle of the landing pad,
which is open floor, and the middle of a boulder.

How to run these tests is in the package's README.
"""
import numpy as np

from team_nav.course_map import LowObstacle
from team_nav.height_check import HEIGHT_GAP_LIMIT_METRES, HEIGHT_GAP_LONGEST_SECONDS, HeightCheck, laser_height_above_launch_metres

from our_files import SCAN_MAP, load_our_map

CRUISE_HEIGHT_METRES = 0.8
SAMPLES_PER_SECOND = 10
# Our own choice: a gap clearly bigger than the limit.
CLEARLY_TOO_BIG_GAP_METRES = HEIGHT_GAP_LIMIT_METRES + 0.1


def feed(check, place, gap_metres, seconds, start_seconds=0.0):
    """Gives the check SAMPLES_PER_SECOND samples a second, with PX4's height that much above
    the laser's. Returns the first problem it reports, or None."""
    for sample in range(int(seconds * SAMPLES_PER_SECOND) + 1):
        now_seconds = start_seconds + sample / SAMPLES_PER_SECOND
        problem = check.check(now_seconds, place, CRUISE_HEIGHT_METRES + gap_metres, CRUISE_HEIGHT_METRES)
        if problem is not None:
            return problem
    return None


def middle_of_the_pad():
    return load_our_map(SCAN_MAP).landing_pad_centre_metres


def test_the_middle_of_the_pad_is_clear_floor_and_a_boulder_is_not():
    scan = load_our_map(SCAN_MAP)
    check = HeightCheck(scan)

    assert check.laser_sees_clear_floor(scan.landing_pad_centre_metres)
    assert not check.laser_sees_clear_floor(scan.low_obstacles[0].centre_metres)


def test_heights_that_agree_are_fine():
    check = HeightCheck(load_our_map(SCAN_MAP))

    assert feed(check, middle_of_the_pad(), 0.05, seconds=10.0) is None


def test_a_short_gap_is_ignored():
    check = HeightCheck(load_our_map(SCAN_MAP))

    assert feed(check, middle_of_the_pad(), CLEARLY_TOO_BIG_GAP_METRES, seconds=HEIGHT_GAP_LONGEST_SECONDS / 2.0) is None
    assert feed(check, middle_of_the_pad(), 0.0, seconds=0.1, start_seconds=1.0) is None
    assert feed(check, middle_of_the_pad(), CLEARLY_TOO_BIG_GAP_METRES, seconds=HEIGHT_GAP_LONGEST_SECONDS / 2.0, start_seconds=2.0) is None


def test_a_lasting_gap_over_clear_floor_stops_the_run():
    check = HeightCheck(load_our_map(SCAN_MAP))

    problem = feed(check, middle_of_the_pad(), -CLEARLY_TOO_BIG_GAP_METRES, seconds=2.0 * HEIGHT_GAP_LONGEST_SECONDS)

    assert problem is not None
    assert "can't be trusted" in problem


def test_a_gap_above_a_boulder_is_ignored():
    scan = load_our_map(SCAN_MAP)
    check = HeightCheck(scan)

    assert feed(check, scan.low_obstacles[0].centre_metres, CLEARLY_TOO_BIG_GAP_METRES, seconds=10.0) is None


def test_a_boulder_from_another_map_also_counts():
    scan = load_our_map(SCAN_MAP)
    somewhere_else = LowObstacle("boulder from another map", np.asarray(scan.landing_pad_centre_metres), 0.2, 0.5, "made up for a test")
    check = HeightCheck(scan, other_possible_boulders=[somewhere_else])

    assert not check.laser_sees_clear_floor(scan.landing_pad_centre_metres)


def test_laser_height_allows_for_tilt():
    # Tilted 60 degrees, a 1 m slanted reading is 0.5 m straight down.
    assert np.isclose(laser_height_above_launch_metres(1.0, 0.1, 60.0), 0.4)
