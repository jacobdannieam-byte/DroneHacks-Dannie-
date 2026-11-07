"""Tells ROS what this package contains: our code, the launch files, and the map and mission
files the route planner reads.

The programs below can be started with "ros2 run team_nav <name>":
  course_run         the competition program: flies the whole obstacle course (course_run.py).
  flight_recorder    records everything the drone reports, as a separate program, so the record
                     survives a crash of the flight program (flight_recorder.py).
  hand_test          runs one hardware smoke test with the propellers off, for example
                     "ros2 run team_nav hand_test laser_heights" (hand_tests/). The quick
                     first check of drone 16 is drone_code/hardware_checks/, run from the Mac.
  practice_flight    runs one short practice flight, for example "out_and_back"
                     (practice_flights/). So far these have flown only in our simulators.
"""
from glob import glob

from setuptools import setup

PACKAGE_NAME = "team_nav"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=[PACKAGE_NAME, f"{PACKAGE_NAME}.hand_tests", f"{PACKAGE_NAME}.practice_flights"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/course_maps", glob("course_maps/*.json")),
        (f"share/{PACKAGE_NAME}/missions", glob("missions/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team drone 16",
    maintainer_email="team@example.com",
    description="Our team's code for the real drone: the course program, the recorder, and the smoke tests.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "course_run = team_nav.course_run:main",
            "flight_recorder = team_nav.flight_recorder:main",
            "hand_test = team_nav.hand_tests.run_hand_test:main",
            "practice_flight = team_nav.practice_flights.run_practice_flight:main",
        ],
    },
)
