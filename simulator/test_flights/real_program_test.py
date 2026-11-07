"""Runs one of our REAL drone programs, unchanged, in the 3D rendered sim, and plays the teammate's
part at the start. SIMULATOR TEST CODE. Do not run this on the real drone.

The program under test is any command, for example "ros2 run team_nav course_run". This file:
  1. Checks that it is inside the simulator: the service /simulator_only/start_run must exist.
     If it doesn't, it stops before starting anything.
  2. Starts the program as a separate process, with the environment settings given, and copies
     its output to the screen, each line marked with "|".
  3. Waits until the program is sending targets (on /skybeta/setpoint/pose or /twist), as it
     would on the real drone before the start.
  4. Calls /simulator_only/start_run. This stands in for the teammate who arms the drone and flips
     the switch to Position (docs/ARCHITECTURE.md, section 8). The organizers' real
     flight_interface then hands control to the program.
  5. Waits for the program to end, and says TEST PASSED if its exit code is the one expected
     (0 unless told otherwise), and where the program's records are.

Run it from the top of the repository, with "--" before the program's command:

    simulator/run_test_flight.sh --world drone_code/team_nav/course_maps/phone_scan_2026-11-06.json \\
        real_program_test.py -- ros2 run team_nav course_run

    simulator/run_test_flight.sh real_program_test.py \\
        --env TEAM_NAV_RUN_MODE=practice --env TEAM_NAV_STOP_AFTER_STAGE=climb -- ros2 run team_nav course_run

Unless --env sets it, TEAM_NAV_RECORDS_FOLDER is set to simulator/generated_files/team_runs, so the
program's records stay out of Git. run_test_flight.sh then prints the referee's summary.
"""
import argparse
import os
import signal
import subprocess
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from std_srvs.srv import Trigger

# Our own choices.
RECORDS_FOLDER_IN_THE_BOX = "/repository/simulator/generated_files/team_runs"
LONGEST_WAIT_FOR_SIMULATOR_SECONDS = 180.0
# The program first plans its route and waits for the drone to be ready, which can take minutes
# while the simulator starts.
LONGEST_WAIT_FOR_TARGETS_SECONDS = 300.0
# About 2 seconds of targets at 10 per second: clearly a steady stream, not a single message.
TARGETS_BEFORE_PRESSING_START = 20
DEFAULT_LONGEST_RUN_SECONDS = 400.0
# How long the program gets to land and finish after we interrupt it, before it is killed.
SECONDS_TO_FINISH_AFTER_INTERRUPT = 60.0


class TestFailed(Exception):
    """The message says what went wrong, in full sentences."""


class PretendTeammate(Node):
    """Counts the program's targets and presses the simulator's start button. SIMULATOR ONLY."""

    def __init__(self):
        super().__init__("real_program_test_simulator_only")
        self.targets_seen = 0
        self.create_subscription(PoseStamped, "/skybeta/setpoint/pose", self.on_target, 10)
        self.create_subscription(TwistStamped, "/skybeta/setpoint/twist", self.on_target, 10)
        self.start_service = self.create_client(Trigger, "/simulator_only/start_run")

    def on_target(self, _message):
        self.targets_seen += 1


def parse_arguments():
    parser = argparse.ArgumentParser(description="Run a real drone program in the simulator and press start.")
    parser.add_argument("--env", action="append", default=[], metavar="NAME=VALUE",
                        help="An environment setting for the program. May be repeated.")
    parser.add_argument("--longest-run-seconds", type=float, default=DEFAULT_LONGEST_RUN_SECONDS,
                        help="How long the program may run after start is pressed.")
    parser.add_argument("--expected-exit-code", type=int, default=0)
    parser.add_argument("program", nargs=argparse.REMAINDER,
                        help="'--' and then the program's command, for example -- ros2 run team_nav course_run")
    arguments = parser.parse_args()
    if arguments.program[:1] == ["--"]:
        arguments.program = arguments.program[1:]
    if not arguments.program:
        parser.error("Give the program's command after '--', for example: -- ros2 run team_nav course_run")
    return arguments


def program_environment(env_settings):
    environment = dict(os.environ)
    environment["TEAM_NAV_RECORDS_FOLDER"] = RECORDS_FOLDER_IN_THE_BOX
    for setting in env_settings:
        if "=" not in setting:
            raise TestFailed(f"--env needs NAME=VALUE, but got '{setting}'.")
        name, value = setting.split("=", 1)
        environment[name] = value
    return environment


def start_program(command, environment):
    print(f"Starting the program: {' '.join(command)}", flush=True)
    program = subprocess.Popen(command, env=environment, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1)
    threading.Thread(target=copy_output_to_screen, args=(program,), daemon=True).start()
    return program


def copy_output_to_screen(program):
    for line in program.stdout:
        print(f"  | {line}", end="", flush=True)


def spin_until(teammate, is_finished, longest_wait_seconds, what_we_are_waiting_for):
    end_seconds = time.monotonic() + longest_wait_seconds
    while not is_finished():
        if time.monotonic() > end_seconds:
            raise TestFailed(f"Waited {longest_wait_seconds:.0f} s for {what_we_are_waiting_for}, and it did not happen.")
        rclpy.spin_once(teammate, timeout_sec=0.1)


def refuse_to_run_outside_the_simulator(teammate):
    # This service exists only in the simulator, so this file can never start a real flight.
    if not teammate.start_service.wait_for_service(timeout_sec=LONGEST_WAIT_FOR_SIMULATOR_SECONDS):
        raise TestFailed("The service /simulator_only/start_run did not appear, so this is not the simulator. "
                         "Nothing was started. Use simulator/run_test_flight.sh.")


def press_start_when_targets_flow(teammate, program):
    def targets_flow_or_program_ended():
        return teammate.targets_seen >= TARGETS_BEFORE_PRESSING_START or program.poll() is not None

    spin_until(teammate, targets_flow_or_program_ended, LONGEST_WAIT_FOR_TARGETS_SECONDS, "the program to send targets")
    if program.poll() is not None:
        raise TestFailed(f"The program ended with exit code {program.returncode} before it sent any targets.")
    print("[real_program_test] The program is sending targets. Pressing start, in place of a teammate arming "
          "the drone and flipping the switch to Position.", flush=True)
    reply = teammate.start_service.call_async(Trigger.Request())
    spin_until(teammate, reply.done, 10.0, "the start service to reply")
    if not reply.result().success:
        raise TestFailed(f"The simulator refused to start: {reply.result().message}")


def wait_for_program_to_end(teammate, program, longest_run_seconds):
    try:
        spin_until(teammate, lambda: program.poll() is not None, longest_run_seconds, "the program to finish")
    except TestFailed:
        print("[real_program_test] The program is taking too long. Interrupting it (Ctrl+C).", flush=True)
        program.send_signal(signal.SIGINT)
        try:
            program.wait(timeout=SECONDS_TO_FINISH_AFTER_INTERRUPT)
        except subprocess.TimeoutExpired:
            program.kill()
        raise
    return program.returncode


def show_newest_records_folder(start_wall_seconds, records_folder):
    if not os.path.isdir(records_folder):
        print(f"[real_program_test] No records folder at {records_folder}.", flush=True)
        return
    new_folders = []
    for name in sorted(os.listdir(records_folder)):
        path = os.path.join(records_folder, name)
        if os.path.isdir(path) and os.path.getmtime(path) >= start_wall_seconds:
            new_folders.append(path)
    for path in new_folders:
        print(f"[real_program_test] Records written: {path}: {', '.join(sorted(os.listdir(path)))}", flush=True)


def run_test(arguments):
    """Returns the program's exit code."""
    environment = program_environment(arguments.env)
    teammate = PretendTeammate()
    refuse_to_run_outside_the_simulator(teammate)
    start_wall_seconds = time.time()
    program = start_program(arguments.program, environment)
    try:
        press_start_when_targets_flow(teammate, program)
        exit_code = wait_for_program_to_end(teammate, program, arguments.longest_run_seconds)
    finally:
        if program.poll() is None:
            program.kill()
        time.sleep(0.5)  # Let the last lines of output reach the screen.
        show_newest_records_folder(start_wall_seconds, environment["TEAM_NAV_RECORDS_FOLDER"])
    return exit_code


def main():
    arguments = parse_arguments()
    rclpy.init()
    try:
        exit_code = run_test(arguments)
    except TestFailed as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        sys.exit(1)
    if exit_code != arguments.expected_exit_code:
        print(f"TEST FAILED: the program ended with exit code {exit_code}, not {arguments.expected_exit_code}. "
              f"Its output above says why.", flush=True)
        sys.exit(1)
    print(f"TEST PASSED: the program ended with exit code {exit_code}, as expected.", flush=True)


if __name__ == "__main__":
    main()
