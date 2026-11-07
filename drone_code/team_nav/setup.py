"""Tells ROS what this package contains: the mission program (team_nav/mission_node.py), which
`ros2 run team_nav mission` starts, the launch files, the route mission's waypoints, and the course
maps the drift correction measures against."""
from setuptools import setup

PACKAGE_NAME = "team_nav"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/team_nav.launch.py", "launch/team_nav_route.launch.py"]),
        (f"share/{PACKAGE_NAME}/missions", ["missions/route_waypoints.json", "missions/route_waypoints_drawing.json",
                                             "missions/route_waypoints_scan.json"]),
        (f"share/{PACKAGE_NAME}/course_maps", ["course_maps/organizers_drawing.json",
                                                "course_maps/phone_scan_2026-11-06.json"]),
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
            "corner_count = team_nav.corner_count_node:main",
            "depth_points = team_nav.depth_points_node:main",
            "depth_check = team_nav.depth_check_node:main",
        ],
    },
)
