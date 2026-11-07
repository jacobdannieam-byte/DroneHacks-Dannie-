"""What starts on the real drone for the route mission: exactly team_nav.launch.py, except that the
mission launch argument defaults to route instead of stages.

Why a second file: on the drone, the only things we can choose are COMPETITOR_PKG and
COMPETITOR_LAUNCH in /etc/skybeta/drone.env (see drone_code/README.md); we cannot add launch
arguments there. So switching the drone to the route mission means pointing COMPETITOR_LAUNCH at
this file:

    COMPETITOR_LAUNCH=team_nav_route.launch.py

This file loads team_nav.launch.py from the same folder and calls it with a different default, so
the two can never differ in anything else. Every launch argument of team_nav.launch.py works here
too, and mission:=stages still overrides the default.
"""
import importlib.util
import os

MAIN_LAUNCH_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'team_nav.launch.py')


def load_main_launch_file():
    specification = importlib.util.spec_from_file_location('team_nav_launch', MAIN_LAUNCH_FILE)
    main_launch_file = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(main_launch_file)
    return main_launch_file


def generate_launch_description():
    return load_main_launch_file().generate_launch_description(mission_default='route')
