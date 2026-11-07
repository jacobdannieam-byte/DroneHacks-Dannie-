"""Practice flight "mock_gap_and_window": flies the competition program over a small practice
course built in the net, through a gap between two walls and a window with a bottom edge, and
lands on a pad.

REAL DRONE CODE, also flown unchanged in the simulator. Start it with
"ros2 run team_nav practice_flight mock_gap_and_window" (run_practice_flight.py).

Why it is so short
------------------
It is the competition program, course_run.py, with a different map and mission:
  - the map is course_maps/practice_mock_gap_and_window.json, the practice course's layout
    (README.md in this folder says how to build it and what to measure);
  - the mission is missions/practice_mock_gap_and_window.json: cross the gap, then the window;
  - no boulders are borrowed from other maps. course_run.py normally also avoids the organizers'
    drawing's boulders, but those are in the competition course, not in the practice net.
So the route planner, the route follower, the safety checks, the landing and the records are
exactly the competition run's. If the practice goes well, it is evidence for the real run.

Any other course_run option may be added after the flight's name, for example
--ignore-raw-laser, --start-heading-degrees 90, or --map <a measured copy of the map>. A later
option replaces the one given here. In practice mode, TEAM_NAV_STOP_AFTER_STAGE may be "climb",
"mock gap" or "mock window" (the stages are named after the mission's checkpoints).

What this file does not do: anything course_run.py does not already do.
"""
from team_nav import course_run

FLIGHT_NAME = "mock_gap_and_window"

# The practice course's files, in team_nav's course_maps and missions folders.
MAP_FILE = "practice_mock_gap_and_window.json"
MISSION_FILE = "practice_mock_gap_and_window.json"


def course_run_words(command_line_words):
    """The command line for course_run: our map and mission, no borrowed boulders, then
    whatever the person added, which may replace any of these."""
    return ["--map", MAP_FILE, "--mission", MISSION_FILE, "--other-boulder-maps", ""] + list(command_line_words)


def run(command_line_words, run_mode, log):
    """Plans the route on the practice course and flies it. Returns course_run's exit code."""
    arguments = course_run.parse_arguments(course_run_words(command_line_words))
    return course_run.plan_then_fly(arguments, run_mode, log)
