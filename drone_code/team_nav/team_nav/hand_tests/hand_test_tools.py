"""The pieces every hand test shares: a test's verdict, telling the person what to do, printing
PASS or FAIL, and saving the result. Also two small angle calculations several tests need.

REAL DRONE CODE, and plain Python: no ROS. That keeps every test's pass-or-fail decision
testable on a laptop with made-up readings (test/test_hand_tests*.py).

How the pieces fit
------------------
Each test file has a run(listener, person, log) function. It tells the person what to do through
a Person, reads the drone through the listener (drone_listener.py), writes its readings into the
log (run_log.py), and returns a Verdict. run_hand_test.py prints and saves that Verdict with
report_verdict below.

What this file does not do: listen to the drone, or decide any test's result.
"""
import json
import math
from dataclasses import dataclass, field


# Our own choice. PX4's status report normally arrives several times a second, so this many
# seconds of silence means the link to the flight controller is down.
LONGEST_PX4_SILENCE_SECONDS = 3.0


@dataclass
class Verdict:
    """The result of one test: PASS or FAIL, one sentence saying why, and the numbers behind it."""
    passed: bool
    reason: str
    measurements: dict = field(default_factory=dict)


def passed(reason, **measurements):
    return Verdict(True, reason, measurements)


def failed(reason, **measurements):
    return Verdict(False, reason, measurements)


def combine(verdicts):
    """One verdict from several parts: PASS only if every part passed. The reason lists the
    failed parts' reasons, or every part's reason if all passed."""
    failed_parts = []
    for verdict in verdicts:
        if not verdict.passed:
            failed_parts.append(verdict)
    shown = failed_parts if failed_parts else verdicts
    reason = " ".join(verdict.reason for verdict in shown)
    measurements = {}
    for verdict in verdicts:
        measurements.update(verdict.measurements)
    return Verdict(not failed_parts, reason, measurements)


def px4_silence_problem(seconds_since_last_px4_status):
    """A FAIL verdict if PX4 (the flight controller's software) has gone quiet, else None. Every
    test that reads the flight controller checks this first, so that a dead link gives one clear
    sentence instead of a confusing list of missing readings."""
    if seconds_since_last_px4_status <= LONGEST_PX4_SILENCE_SECONDS:
        return None
    if math.isinf(seconds_since_last_px4_status):
        silence_text = "has arrived since the test started"
    else:
        silence_text = f"for {seconds_since_last_px4_status:.0f} s"
    return failed(
        f"No PX4 report {silence_text}: is the flight controller powered (battery connected)? "
        f"The Jetson alone does not power it. If it is powered, run the quick first check "
        f"(drone_code/hardware_checks/README.md), which looks at the link itself."
    )


class Person:
    """The teammate holding the drone, reached through the keyboard. Tests only talk to the
    person through these methods, so the simulator and the unit tests can stand in for them."""

    def do_this_then_press_enter(self, instruction):
        print(f"\n>>> {instruction}", flush=True)
        input("    Press Enter when done (Ctrl+C stops the test). ")

    def type_a_number(self, question):
        """Asks until the person types a number."""
        while True:
            answer = input(f"\n>>> {question} ").strip()
            try:
                return float(answer)
            except ValueError:
                print(f"    '{answer}' is not a number. Type it like 0.30.", flush=True)

    def tell(self, text):
        print(f"    {text}", flush=True)


def report_verdict(test_name, verdict, log):
    """Prints PASS or FAIL with the reason, and saves both in the run's records."""
    word = "PASS" if verdict.passed else "FAIL"
    print(f"\n{word}  {test_name}: {verdict.reason}", flush=True)
    log.event("result", f"{word}: {verdict.reason}", test=test_name, passed=verdict.passed,
              measurements=verdict.measurements)
    result = {"test": test_name, "result": word, "reason": verdict.reason, "measurements": verdict.measurements}
    log.save_text_file("result.json", json.dumps(result, indent=2, default=str) + "\n")
    print(f"      Records saved in {log.folder}", flush=True)


# ---- Angles -----------------------------------------------------------------------------------

def heading_change_degrees(heading_before_degrees, heading_after_degrees):
    """How far the heading turned, from -180 to +180 degrees, taking the short way round."""
    change = (heading_after_degrees - heading_before_degrees) % 360.0
    if change > 180.0:
        change -= 360.0
    return change


def yaw_degrees_from_quaternion(w, x, y, z):
    """Which way the nose points, as an angle about the vertical axis, in degrees. A quaternion is
    the four-number way ROS and PX4 store an orientation. In ROS's directions (x forward, y left,
    z up) a turn to the left makes this angle larger."""
    return math.degrees(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
