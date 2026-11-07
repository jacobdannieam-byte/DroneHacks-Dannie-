"""Installs the team's real-drone test flights as a ROS package called team_smoke_flights."""
from glob import glob

from setuptools import setup

PACKAGE_NAME = "team_smoke_flights"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Team Quiet Sparrow's first real-drone test flights.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "rise_and_land = team_smoke_flights.rise_and_land:main",
        ],
    },
)
