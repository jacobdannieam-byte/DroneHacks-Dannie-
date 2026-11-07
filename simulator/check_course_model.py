"""Checks that the simulator's picture of the course matches the course's known measurements.

SIMULATOR ONLY. It needs only Python, numpy and scipy, not ROS, so it runs directly on the Mac:

    python3 simulator/check_course_model.py

The first run takes about a minute, because it prepares the course tables. Each check below
shoots one ray at a place where we know the right answer from the organizers' model. For example,
a boulder is 0.533 m tall, so a ray going down from 0.8 m above one should travel 0.267 m.
"""
import importlib.util
import os
import sys

import numpy as np

FOLDER_OF_THIS_SCRIPT = os.path.dirname(os.path.abspath(__file__))
COURSE_MODEL_FILE = os.path.join(FOLDER_OF_THIS_SCRIPT, "course_model", "dronehacks-course-v3.obj")
SAVED_TABLES_FOLDER = os.path.join(FOLDER_OF_THIS_SCRIPT, "generated_files", "course_tables")
COURSE_MODEL_CODE = os.path.join(
    FOLDER_OF_THIS_SCRIPT, "ros_packages", "course_simulator", "course_simulator", "course_model.py"
)

# The course tables are accurate to about 1 cm, so we allow that much error, plus a little.
LARGEST_ALLOWED_ERROR_METRES = 0.015

LOOKING_DOWN = (0.0, 0.0, -1.0)
LOOKING_ALONG_PLUS_X = (1.0, 0.0, 0.0)
LOOKING_ALONG_MINUS_Y = (0.0, -1.0, 0.0)

# Each check: what it tests, where the ray starts, which way it goes, the expected distance. An
# expected distance of infinity means the ray should find nothing within the longest range.
CHECKS = (
    ("Down at the launch mark, over bare floor", (-2.5145, 0.914, 0.8), LOOKING_DOWN, 0.8),
    ("Down over the top of boulder 1, which is 0.533 m tall", (2.0115, -2.4685, 0.8), LOOKING_DOWN, 0.267),
    ("Down at the window's bottom edge, which is 0.30 m tall", (-0.471, -2.272, 0.8), LOOKING_DOWN, 0.5),
    ("From the launch mark to the first maze divider, at x = -1.981", (-2.5145, 0.914, 0.8), LOOKING_ALONG_PLUS_X, 0.5335),
    ("Above the walls, with nothing to hit before the far side", (-2.9, 2.9, 2.5), LOOKING_ALONG_PLUS_X, np.inf),
)


def load_course_model_code():
    """Loads course_model.py directly, because importing it the usual way needs ROS installed."""
    specification = importlib.util.spec_from_file_location("course_model", COURSE_MODEL_CODE)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


def main():
    course_model_code = load_course_model_code()
    course = course_model_code.CourseModel(COURSE_MODEL_FILE, SAVED_TABLES_FOLDER)

    all_passed = True
    for description, ray_start, ray_direction, expected_metres in CHECKS:
        measured_metres = float(course.cast_rays(
            np.array([ray_start]), np.array([ray_direction]), longest_range_metres=5.0
        )[0])
        if np.isinf(expected_metres):
            passed = np.isinf(measured_metres)
        else:
            passed = abs(measured_metres - expected_metres) < LARGEST_ALLOWED_ERROR_METRES
        all_passed = all_passed and passed
        print(f"{'PASS' if passed else 'FAIL'}  {description}: expected {expected_metres:.3f} m, "
              f"measured {measured_metres:.3f} m")

    sys.exit(0 if all_passed else 1)


if __name__ == "__main__":
    main()
