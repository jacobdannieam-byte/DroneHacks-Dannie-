# team_nav

This is our team's ROS package for the real drone. For now it does nothing on purpose: it replaces the organizers' demo program, so that the drone can't fly the demo's 5 m square into the course walls.

Read [`launch/team_nav.launch.py`](launch/team_nav.launch.py) first. It is the only file with any behaviour, and it says what starts on the drone and why that is safe.

The other files are what ROS needs in order to build a package. `package.xml` and `setup.py` describe the package, `resource/team_nav` lets ROS find it, and `team_nav/__init__.py` is empty for now. The mission program will go in that `team_nav/` folder.

To install this on the drone, see [`../README.md`](../README.md).
