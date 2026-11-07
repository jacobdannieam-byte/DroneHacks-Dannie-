"""Checks that the hand tests (team_nav/hand_tests/) can never command the drone.

The hand tests are run on the real drone by people holding it, with the propellers off. They
must only listen. This test reads their source code, and the shared DroneReports they listen
through, and fails if it finds anything that sends: a publisher, a PX4 input topic, the
organizers' target topics or hold service, a PX4 command, the word for giving the motors
permission to spin, or a service that changes settings.

How to run these tests is in the package's README.
"""
import os

PACKAGE_FOLDER = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HAND_TESTS_FOLDER = os.path.join(PACKAGE_FOLDER, "team_nav", "hand_tests")
DRONE_REPORTS_FILE = os.path.join(PACKAGE_FOLDER, "team_nav", "drone_reports.py")

FORBIDDEN_TEXT = [
    "create_publisher",      # A ROS publisher sends messages.
    "/fmu/in",               # PX4's input topics.
    "/skybeta/setpoint",     # The organizers' target topics.
    "/skybeta/hold",         # The organizers' hold service.
    "request_offboard",      # The organizers' service that asks for software control.
    "VehicleCommand",        # PX4's command message, which can arm and change modes.
    "arming",                # Any arm command or arming-state logic.
    "create_service",        # A service lets other programs make this one act.
    "SetParameters",         # Changing another program's settings.
    "set_parameters",
    "Trigger",               # The message type of the organizers' and simulator's action services.
]


def files_to_scan():
    paths = [DRONE_REPORTS_FILE]
    for file_name in sorted(os.listdir(HAND_TESTS_FOLDER)):
        if file_name.endswith(".py"):
            paths.append(os.path.join(HAND_TESTS_FOLDER, file_name))
    return paths


def test_the_hand_tests_folder_has_python_files_to_scan():
    assert len(files_to_scan()) > 5


def test_no_hand_test_file_contains_anything_that_sends():
    findings = []
    for path in files_to_scan():
        with open(path, encoding="utf-8") as file:
            for line_number, line in enumerate(file, start=1):
                for text in FORBIDDEN_TEXT:
                    if text in line:
                        findings.append(f"{os.path.basename(path)} line {line_number} contains '{text}'")
    assert findings == [], (
        "The hand tests must only listen to the drone, but these lines look like they send something: "
        + "; ".join(findings)
    )
