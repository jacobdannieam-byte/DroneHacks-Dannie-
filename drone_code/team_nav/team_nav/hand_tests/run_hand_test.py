"""Runs one hand test or table check on drone 16, with the propellers off.

REAL DRONE CODE. Start it on the drone, where ROS runs (see README.md):

    ros2 run team_nav hand_test                    lists the tests
    ros2 run team_nav hand_test laser_heights      runs one test
    ros2 run team_nav hand_test all_table_checks   runs the four table checks in order

Every test only listens to the drone: nothing here can arm it, spin a motor or send a target.
Each test prints PASS or FAIL with one sentence saying why, and saves its readings and result
in its own folder in the records folder (run_log.py says where), named hand_test_<name>.

This file only chooses and runs tests. Each test's file holds its instructions and its pass
rule; drone_listener.py does the listening; hand_test_tools.py holds the shared pieces.
"""
import sys

from team_nav.hand_tests import (
    directions, estimator_sources, interface_settings, laser_heights, laser_over_step, report_rates,
    touch_tilt, tracking_still_drift, tunnel_carry, tunnel_look,
)
from team_nav.hand_tests.hand_test_tools import Person, failed, report_verdict
from team_nav.run_log import RunLog
from team_nav.run_mode import read_run_mode

ALL_TABLE_CHECKS = "all_table_checks"

# (name, the test's file, what it proves). In the order to run them: the table checks first,
# then the hand tests (docs/ARCHITECTURE.md, section 11).
TABLE_CHECKS = [
    ("report_rates", report_rates, "every report the flight code needs arrives often enough (10 s)"),
    ("estimator_sources", estimator_sources, "PX4 uses the laser for height and camera tracking for position"),
    ("interface_settings", interface_settings, "the organizers' interface won't hand control to software; no demo"),
    ("tracking_still_drift", tracking_still_drift, "camera tracking stays put while the drone sits still (30 s)"),
]
HAND_TESTS = [
    ("laser_heights", laser_heights, "the bottom laser matches a tape measure at heights up to 1.2 m"),
    ("directions", directions, "1 m forward, 1 m left and a 90 degree left turn are seen the right way round"),
    ("touch_tilt", touch_tilt, "the touch detector is silent when still and fires when tilted past 25 degrees"),
    ("tunnel_carry", tunnel_carry, "camera tracking holds when carried through the tunnel and back"),
    ("laser_over_step", laser_over_step, "how wide the laser's beam is, from carrying it over a box edge"),
    ("tunnel_look", tunnel_look, "prints where the camera sees the tunnel's centre, twice a second (Ctrl+C stops)"),
]


def main(arguments=None):
    if arguments is None:
        arguments = sys.argv[1:]
    # "ros2 run" may add ROS's own arguments after --ros-args. They are not test names.
    if "--ros-args" in arguments:
        arguments = arguments[:arguments.index("--ros-args")]
    if len(arguments) != 1:
        print_the_list()
        return 0 if not arguments else 2
    names = names_to_run(arguments[0])
    if names is None:
        print(f"There is no test called '{arguments[0]}'.\n")
        print_the_list()
        return 2

    print("PROPELLERS OFF. These tests only listen to the drone: they never arm it, spin a motor or send a target.")
    from team_nav.hand_tests.drone_listener import DroneListener  # Needs ROS, so only loaded to run a test.
    listener = DroneListener()
    try:
        verdicts = run_tests(names, listener, Person())
    finally:
        listener.close()
    print_summary(verdicts)
    return 0 if all(verdict.passed for _, verdict in verdicts) else 1


def names_to_run(name):
    """The list of test names this argument means, or None if it means nothing."""
    if name == ALL_TABLE_CHECKS:
        return [test_name for test_name, _, _ in TABLE_CHECKS]
    for test_name, _, _ in TABLE_CHECKS + HAND_TESTS:
        if test_name == name:
            return [name]
    return None


def run_tests(names, listener, person):
    """Runs each named test in turn. Returns (name, Verdict) pairs."""
    test_files = {test_name: test_file for test_name, test_file, _ in TABLE_CHECKS + HAND_TESTS}
    verdicts = []
    for name in names:
        print(f"\n===== {name} =====", flush=True)
        log = RunLog(f"hand_test_{name}", read_run_mode(),
                     mode_sentence="HAND TEST, propellers off: this program only listens and never flies.")
        stopped_by_the_person = False
        try:
            verdict = test_files[name].run(listener, person, log)
        except KeyboardInterrupt:
            verdict = failed("The person stopped the test with Ctrl+C before it finished.")
            stopped_by_the_person = True
        report_verdict(name, verdict, log)
        log.close()
        verdicts.append((name, verdict))
        if stopped_by_the_person:
            break
    return verdicts


def print_the_list():
    print("Usage: ros2 run team_nav hand_test <name>    (propellers off; README.md in hand_tests/ explains each)\n")
    print("Table checks, drone on the table:")
    for name, _, description in TABLE_CHECKS:
        print(f"  {name:22} {description}")
    print(f"  {ALL_TABLE_CHECKS:22} runs the four table checks above, in order")
    print("\nHand tests, a person moves the drone:")
    for name, _, description in HAND_TESTS:
        print(f"  {name:22} {description}")


def print_summary(verdicts):
    if len(verdicts) < 2:
        return
    print("\n===== Summary =====")
    for name, verdict in verdicts:
        print(f"{'PASS' if verdict.passed else 'FAIL'}  {name}")


if __name__ == "__main__":
    sys.exit(main())
