"""Hand test: the touch detector stays silent while the drone is held still, and fires when the
drone is tilted far over.

REAL DRONE CODE, run with the PROPELLERS OFF. It only listens to the drone.

What it proves
--------------
The real drone has no touch sensor. Our flight code guesses that it has hit something when it
tilts or turns much more than in normal flight (touch_detector.py), and then lands. That depends
on PX4's attitude report: which way the drone is tilted, and how fast it is turning. "PX4" is
the flight controller's software. The flight code reads it from PX4's vehicle_odometry report;
if drone 16 doesn't send that, this test uses vehicle_attitude and sensor_combined instead
(drone_listener.py), and the flight code will need the same change. This test feeds the real attitude reports into the real touch
detector, with the drone in a person's hands:
  - held still and level, the detector must stay silent (no false alarm);
  - tilted past TILT_TO_ASK_FOR_DEGREES for a second, it must fire.
This proves that the attitude report arrives, means what we think it means, and that the
thresholds behave as intended. It can't prove the thresholds are right for real flight, where
the drone wobbles more; practice flights check that.
"""
from dataclasses import dataclass

from team_nav.hand_tests.hand_test_tools import combine, failed, passed, px4_silence_problem
from team_nav.touch_detector import LARGEST_NORMAL_TILT_DEGREES, TouchDetector

# Our own choices: long enough to see any false alarm, and long enough to tilt and hold.
SECONDS_HELD_STILL = 5.0
SECONDS_ALLOWED_FOR_THE_TILT = 10.0
# Our own choice: 5 degrees past the detector's limit (20 degrees), so a person's rough tilt
# clearly crosses it.
TILT_TO_ASK_FOR_DEGREES = LARGEST_NORMAL_TILT_DEGREES + 5.0
# Our own choice. PX4's attitude report normally arrives dozens of times a second (about 100 a
# second on drone 16 on 2026-11-07). The test polls every POLLING_PERIOD_SECONDS, so it sees at
# most 100 a second.
FEWEST_ATTITUDE_REPORTS_PER_SECOND = 10.0
# Our own choice: how often the test looks for a new attitude report.
POLLING_PERIOD_SECONDS = 0.01


@dataclass
class DetectorRun:
    report_count: int
    seconds: float
    largest_tilt_degrees: float
    largest_rotation_degrees_per_second: float
    alarm_count: int
    first_alarm: str   # The detector's sentence, or "" if it never fired.


def run_detector_over(attitude_samples, seconds):
    """Feeds (time in seconds, tilt in degrees, rotation in degrees per second) samples, oldest
    first, into a fresh touch detector, and counts its alarms."""
    detector = TouchDetector()
    largest_tilt = 0.0
    largest_rotation = 0.0
    alarm_count = 0
    first_alarm = ""
    for time_seconds, tilt, rotation in attitude_samples:
        largest_tilt = max(largest_tilt, tilt)
        largest_rotation = max(largest_rotation, rotation)
        alarm = detector.check(time_seconds, tilt, rotation)
        if alarm is not None:
            alarm_count += 1
            first_alarm = first_alarm or alarm
    return DetectorRun(len(attitude_samples), seconds, largest_tilt, largest_rotation, alarm_count, first_alarm)


def judge_enough_reports(name, detector_run):
    rate = detector_run.report_count / detector_run.seconds
    if rate < FEWEST_ATTITUDE_REPORTS_PER_SECOND:
        return failed(f"While {name}, PX4's attitude report arrived only {rate:.1f} times a second; the touch "
                      f"detector needs at least {FEWEST_ATTITUDE_REPORTS_PER_SECOND:.0f}.")
    return passed(f"Attitude reports arrived {rate:.0f} times a second.")


def judge_touch_tilt(held_still, tilted):
    parts = [judge_enough_reports("held still", held_still), judge_enough_reports("tilted", tilted)]
    if held_still.alarm_count > 0:
        parts.append(failed(f"Held still, the touch detector fired anyway (a false alarm): {held_still.first_alarm}"))
    else:
        parts.append(passed(f"Held still (at most {held_still.largest_tilt_degrees:.1f} degrees tilt), it stayed silent."))
    if tilted.largest_tilt_degrees < TILT_TO_ASK_FOR_DEGREES:
        parts.append(failed(f"The drone was only tilted {tilted.largest_tilt_degrees:.0f} degrees; tilt it past "
                            f"{TILT_TO_ASK_FOR_DEGREES:.0f} degrees and run the test again. If you did, PX4's "
                            f"attitude report is wrong."))
    elif tilted.alarm_count == 0:
        parts.append(failed(f"Tilted {tilted.largest_tilt_degrees:.0f} degrees, the touch detector never fired."))
    else:
        parts.append(passed(f"Tilted {tilted.largest_tilt_degrees:.0f} degrees, it fired."))
    verdict = combine(parts)
    verdict.measurements.update(
        still_largest_tilt_degrees=round(held_still.largest_tilt_degrees, 2),
        still_largest_rotation_degrees_per_second=round(held_still.largest_rotation_degrees_per_second, 1),
        tilted_largest_tilt_degrees=round(tilted.largest_tilt_degrees, 2),
        tilted_largest_rotation_degrees_per_second=round(tilted.largest_rotation_degrees_per_second, 1),
        tilted_alarm_count=tilted.alarm_count,
    )
    return verdict


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    problem = px4_silence_problem(listener.reports.seconds_since("px4_status"))
    if problem:
        return problem
    person.do_this_then_press_enter(
        f"Hold the drone level and still, or leave it standing on the table. After you press Enter, keep it still "
        f"for {SECONDS_HELD_STILL:.0f} seconds.")
    held_still = run_detector_over(collect_attitude(listener, log, "held still", SECONDS_HELD_STILL), SECONDS_HELD_STILL)
    person.do_this_then_press_enter(
        f"Next: after you press Enter, within {SECONDS_ALLOWED_FOR_THE_TILT:.0f} seconds, tilt the drone more than "
        f"{TILT_TO_ASK_FOR_DEGREES:.0f} degrees to one side (about a quarter of the way to vertical), hold it "
        f"there for 2 seconds, then bring it back level.")
    tilted = run_detector_over(collect_attitude(listener, log, "tilted", SECONDS_ALLOWED_FOR_THE_TILT),
                               SECONDS_ALLOWED_FOR_THE_TILT)
    return judge_touch_tilt(held_still, tilted)


def collect_attitude(listener, log, phase_name, seconds):
    """Every new PX4 attitude report for this many seconds, as (time, tilt, rotation) samples."""
    samples = []
    last_arrival = None
    end_seconds = listener.now() + seconds
    while listener.now() < end_seconds:
        listener.wait(POLLING_PERIOD_SECONDS)
        attitude = listener.attitude_now()
        if attitude is None or attitude[0] == last_arrival:
            continue
        last_arrival = attitude[0]
        samples.append(attitude)
        log.sample("attitude", phase=phase_name, tilt_degrees=round(attitude[1], 2),
                   rotation_degrees_per_second=round(attitude[2], 2))
    return samples
