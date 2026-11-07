"""Tells ROS what this package contains, and which programs it can run. SIMULATOR ONLY."""
from setuptools import setup

PACKAGE_NAME = "course_simulator"

setup(
    name=PACKAGE_NAME,
    version="0.0.1",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/whole_simulator.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DroneHacks team",
    maintainer_email="team@example.com",
    description="Pretend sensors and a referee for flying the obstacle course on a laptop.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pretend_camera_tracking = course_simulator.pretend_camera_tracking:main",
            "pretend_bottom_laser = course_simulator.pretend_bottom_laser:main",
            "pretend_depth_camera = course_simulator.pretend_depth_camera:main",
            "pretend_remote_control = course_simulator.pretend_remote_control:main",
            "referee = course_simulator.referee:main",
        ],
    },
)
