"""Tells ROS what this package contains: the mission program (team_nav/mission_node.py), which
`ros2 run team_nav mission` starts, and the launch file."""
from setuptools import setup

PACKAGE_NAME = "team_nav"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/team_nav.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Team drone 16",
    maintainer_email="team@example.com",
    description="Our team's code for the real drone: the mission program and its launch file.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "mission = team_nav.mission_node:main",
        ],
    },
)
