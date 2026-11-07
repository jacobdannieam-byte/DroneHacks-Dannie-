"""Tests touch_detector.py, which stops the run when the drone tilts or spins as if it hit
something.

How to run these tests is in the package's README.
"""
import math

import numpy as np

from team_nav.touch_detector import (
    LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND, LARGEST_NORMAL_TILT_DEGREES, TOUCH_SIGNS_SHORTEST_SECONDS,
    TouchDetector, rotation_degrees_per_second, tilt_degrees,
)

# Like the largest seen in simulator flights on 2026-11-07.
NORMAL_TILT_DEGREES = 3.0
NORMAL_ROTATION_DEGREES_PER_SECOND = 10.0
SECONDS_BETWEEN_REPORTS = 0.01


def feed(detector, tilt, rotation, seconds, start_seconds=0.0):
    """Gives the detector a report every SECONDS_BETWEEN_REPORTS. Returns the first touch it
    reports, or None."""
    for report in range(int(round(seconds / SECONDS_BETWEEN_REPORTS)) + 1):
        touch = detector.check(start_seconds + report * SECONDS_BETWEEN_REPORTS, tilt, rotation)
        if touch is not None:
            return touch
    return None


def test_normal_flight_is_not_a_touch():
    assert feed(TouchDetector(), NORMAL_TILT_DEGREES, NORMAL_ROTATION_DEGREES_PER_SECOND, seconds=60.0) is None


def test_a_single_odd_report_is_not_a_touch():
    detector = TouchDetector()

    assert detector.check(0.0, NORMAL_TILT_DEGREES, LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND * 2.0) is None
    assert detector.check(0.01, NORMAL_TILT_DEGREES, NORMAL_ROTATION_DEGREES_PER_SECOND) is None
    assert detector.check(TOUCH_SIGNS_SHORTEST_SECONDS * 2.0, NORMAL_TILT_DEGREES, NORMAL_ROTATION_DEGREES_PER_SECOND) is None


def test_a_lasting_sudden_spin_is_a_touch():
    touch = feed(TouchDetector(), NORMAL_TILT_DEGREES, LARGEST_NORMAL_ROTATION_DEGREES_PER_SECOND * 2.0,
                 seconds=TOUCH_SIGNS_SHORTEST_SECONDS * 2.0)

    assert touch is not None
    assert "hit something" in touch


def test_a_lasting_big_tilt_is_a_touch():
    touch = feed(TouchDetector(), LARGEST_NORMAL_TILT_DEGREES + 10.0, NORMAL_ROTATION_DEGREES_PER_SECOND,
                 seconds=TOUCH_SIGNS_SHORTEST_SECONDS * 2.0)

    assert touch is not None


def test_tilt_from_a_quaternion():
    # A quaternion for a 30 degree roll: (cos 15°, sin 15°, 0, 0).
    half_angle = math.radians(15.0)
    assert np.isclose(tilt_degrees((math.cos(half_angle), math.sin(half_angle), 0.0, 0.0)), 30.0)
    # Turning on the spot doesn't tilt the drone.
    assert np.isclose(tilt_degrees((math.cos(half_angle), 0.0, 0.0, math.sin(half_angle))), 0.0)


def test_rotation_speed_from_angular_velocity():
    assert np.isclose(rotation_degrees_per_second((0.0, 0.0, math.pi)), 180.0)
