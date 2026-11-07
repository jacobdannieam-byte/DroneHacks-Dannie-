"""Tells ROS what this package contains. There are no programs yet, only the launch file."""
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
    description="Our team's code for the real drone. For now it starts no mission.",
    license="Apache-2.0",
)
