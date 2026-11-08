"""Starts one practice flight on drone 16.

REAL DRONE CODE, also run unchanged in the simulator. Start it on the drone, where ROS runs:

    ros2 run team_nav practice_flight                          lists the flights
    ros2 run team_nav practice_flight out_and_back [options]   runs one flight

Practice flights fly only in practice mode: set TEAM_NAV_RUN_MODE=practice first. That keeps the
records honest, because every records folder is named after the mode (run_log.py), and a practice
flight must never be filed as a competition run.

This file only chooses a flight, checks the mode, opens the records and makes "stop the program"
land the drone. Each flight's own file says what it flies and what it measures; README.md in this
folder says how to run each one and what to look at afterwards.

Exit codes, the same as course_run.py's: 0 finished with the motors off; 1 stopped early or
failed; 2 refused to start (unknown flight, wrong mode, bad settings); 130 Ctrl+C.
"""
import signal
import sys

from team_nav.course_run import EXIT_REFUSED_TO_START, treat_terminate_as_ctrl_c
from team_nav.practice_flights import first_maze_leg, full_course, mock_gap_and_window, out_and_back
from team_nav.run_log import RunLog
from team_nav.run_mode import RUN_MODE_SETTING, read_run_mode

# (name, the flight's file, what it proves). In the order to fly them (docs/ARCHITECTURE.md, section 11).
FLIGHTS = [
    (full_course.FLIGHT_NAME, full_course,
     "checked sketch-map route through the first corridor, maze, tunnel and chicane to the landing pad"),
    (first_maze_leg.FLIGHT_NAME, first_maze_leg,
     "flies Corridor 1 to North = +2.18 m, turns 90 degrees East into the maze to East = -0.50 m, and lands to bank Top 3 qualification points"),
    (out_and_back.FLIGHT_NAME, out_and_back,
     "from a low hover, 1 m forward as a step and back as a slide: measures the response delay and trailing distance"),
    (mock_gap_and_window.FLIGHT_NAME, mock_gap_and_window,
     "the competition program on the practice course: through a gap, through a window over a 0.4 m box, land on a pad"),
]


def main(arguments=None):
    if arguments is None:
        arguments = sys.argv[1:]
    # "ros2 run" may add ROS's own arguments after --ros-args. They are not ours.
    if "--ros-args" in arguments:
        arguments = arguments[:arguments.index("--ros-args")]
    if not arguments:
        print_the_list()
        return 0
    flight = flight_called(arguments[0])
    if flight is None:
        print(f"There is no practice flight called '{arguments[0]}'.\n")
        print_the_list()
        return EXIT_REFUSED_TO_START
    run_mode = practice_mode_or_none()
    if run_mode is None:
        return EXIT_REFUSED_TO_START
    return run_flight(arguments[0], flight, arguments[1:], run_mode)


def flight_called(name):
    for flight_name, flight, _ in FLIGHTS:
        if flight_name == name:
            return flight
    return None


def practice_mode_or_none():
    """The run mode, or None after saying why the flight can't start."""
    try:
        run_mode = read_run_mode()
    except ValueError as problem:
        print(f"REFUSING TO START: {problem}", flush=True)
        return None
    if not run_mode.is_practice:
        print(f"REFUSING TO START: practice flights only run in practice mode, so that their records are never "
              f"filed as a competition run. Set {RUN_MODE_SETTING}=practice and start again.", flush=True)
        return None
    return run_mode


def run_flight(name, flight, flight_words, run_mode):
    # As in course_run.py: "ros2 launch" and other tools stop a program with SIGTERM, which would
    # otherwise end it at once, without landing or closing the records.
    signal.signal(signal.SIGTERM, treat_terminate_as_ctrl_c)
    log = RunLog(f"practice_{name}", run_mode)
    try:
        exit_code = flight.run(flight_words, run_mode, log)
        log.event("exit", f"practice flight {name} is ending with exit code {exit_code}.", exit_code=exit_code)
    finally:
        log.close()
    return exit_code


def print_the_list():
    print(f"Usage: {RUN_MODE_SETTING}=practice ros2 run team_nav practice_flight <name> [options]")
    print("Add --help after the name to see its options. README.md in practice_flights/ explains each flight.\n")
    for name, _, description in FLIGHTS:
        print(f"  {name:22} {description}")


if __name__ == "__main__":
    sys.exit(main())
