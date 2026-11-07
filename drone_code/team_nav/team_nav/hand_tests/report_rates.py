"""Table check: every report our flight code needs arrives, often enough, and is still arriving.

REAL DRONE CODE, run on the table with the PROPELLERS OFF. It only listens to the drone.

What it proves
--------------
Our flight code reads PX4's reports, the organizers' summary and camera tracking many times a
second. "PX4" is the flight controller's software; "camera tracking" is the organizers'
program that works out the drone's position from the front cameras. If a report is missing or
slow, the flight code holds the drone or works from old information. This check listens for
LISTENING_SECONDS and, for each report, counts the messages and checks that the last one is
recent. It differs from the quick first check (drone_code/hardware_checks/), which runs from the
Mac and only asks whether anything arrives: this one uses the same topic names and message
settings as our flight code, and checks rates against what the flight code needs.

PASS means every report marked "needed" arrived at least at its lowest rate, and the last one
arrived within FRESHEST_SECONDS of the end. Reports not marked "needed" are listed but do not
fail the check.

What it does not do: check the readings' values. The other table checks and hand tests do that.
"""
import math
from dataclasses import dataclass

from team_nav.hand_tests.hand_test_tools import failed, passed

# Our own choice: long enough to count reports that come about once a second.
LISTENING_SECONDS = 10.0
# Our own choice. Reports sent about once a second (PX4's land detector and estimator flags)
# can be up to a second old at any moment, so this allows for that plus a little delay.
FRESHEST_SECONDS = 1.5


@dataclass(frozen=True)
class ExpectedReport:
    description: str
    topic: str                     # A topic name, or "px4:<our name>" for a PX4 report whose
                                   # exact name drone_listener.py works out on this computer.
    lowest_per_second: float
    needed: bool


EXPECTED_REPORTS = [
    # From the organizers' code: flight_interface.py's default state_rate_hz is 5. Measured 5.0
    # per second on drone 16 on 2026-11-07.
    ExpectedReport("Organizers' summary (/skybeta/state)", "/skybeta/state", 4.0, True),
    # Our own choice. The organizers' code says vehicle_status comes about 5 times a second.
    ExpectedReport("PX4 status", "px4:px4_status", 1.0, True),
    # Our own choice. The flight code sends 10 targets a second, so it needs at least as many
    # position reports.
    ExpectedReport("PX4 position estimate", "px4:px4_position", 10.0, True),
    # Our own choice. The touch detector needs a few attitude reports within its 0.05 s window.
    # Drone 16 sent vehicle_attitude about 100 times a second on 2026-11-07. Our flight code reads
    # vehicle_odometry instead (drone_reports.py), which may not exist on the drone.
    ExpectedReport("PX4 attitude", "px4:px4_attitude", 20.0, True),
    ExpectedReport("PX4 odometry, which the flight code reads for attitude", "px4:px4_odometry", 20.0, False),
    # From PX4 source code: the land detector publishes at least once a second (LandDetector.cpp).
    ExpectedReport("PX4 landed report", "px4:px4_land_detected", 0.5, True),
    # From PX4 source code: EKF2 publishes its status flags at least once a second (EKF2.cpp).
    ExpectedReport("PX4 estimator flags", "px4:estimator_flags", 0.5, True),
    # Our own choice. Section 7 of docs/ARCHITECTURE.md says the real laser reads about 5 times a
    # second. Drone 16 did NOT send it on 2026-11-07, so it is not marked needed here; the landing
    # and the height check then have to work from PX4's dist_bottom instead.
    ExpectedReport("Bottom laser, raw reading", "px4:laser", 3.0, False),
    # Our own choice. Useful, but no flight decision uses it.
    ExpectedReport("Battery", "px4:battery", 0.5, False),
    # Our own choice, about half of the 28.9 per second measured on drone 16 on 2026-11-07.
    ExpectedReport("Camera tracking position", "/visual_slam/tracking/odometry", 15.0, True),
    ExpectedReport("Camera tracking status", "/visual_slam/status", 15.0, False),
    # Our own choice, about half of the 22 to 24 per second measured on drone 16 on 2026-11-07.
    ExpectedReport("Left camera pictures", "/oak/left/image_raw", 10.0, False),
    ExpectedReport("Right camera pictures", "/oak/right/image_raw", 10.0, False),
    # Our own choice, about a third of the 30 per second measured on drone 16 on 2026-11-07.
    ExpectedReport("Left camera lens settings", "/oak/left/camera_info", 10.0, False),
]


@dataclass
class ReportCount:
    report: ExpectedReport
    topic_name: str
    exists: bool
    per_second: float
    seconds_since_last: float


def judge_one_report(count):
    """'ok', or a short sentence saying what is wrong with this report."""
    name = f"{count.report.description} ({count.topic_name})"
    if not count.exists:
        return f"{name}: no program offers this topic"
    if count.per_second == 0.0:
        return f"{name}: the topic exists but nothing arrived"
    if count.per_second < count.report.lowest_per_second:
        return f"{name}: {count.per_second:.1f} per second, below {count.report.lowest_per_second:g}"
    if count.seconds_since_last > FRESHEST_SECONDS:
        return f"{name}: the last one is {count.seconds_since_last:.1f} s old"
    return "ok"


def judge_report_rates(counts):
    """The check's verdict from every report's count."""
    px4_needed = [count for count in counts if count.report.topic.startswith("px4:") and count.report.needed]
    if px4_needed and all(count.per_second == 0.0 for count in px4_needed):
        return failed(f"No PX4 report for {LISTENING_SECONDS:.0f} s: is the flight controller powered (battery "
                      f"connected)? The Jetson alone does not power it. If it is, run the quick first check "
                      f"(drone_code/hardware_checks/README.md), which looks at the link itself.",
                      rates=table_of_rates(counts))
    problems = []
    for count in counts:
        problem = judge_one_report(count)
        if problem != "ok" and count.report.needed:
            problems.append(problem)
    if problems:
        return failed("Reports the flight code needs are missing or slow: " + "; ".join(problems) + ".",
                      rates=table_of_rates(counts))
    return passed(f"Every report the flight code needs arrived often enough over {LISTENING_SECONDS:.0f} s.",
                  rates=table_of_rates(counts))


def table_of_rates(counts):
    rows = []
    for count in counts:
        rows.append({"report": count.report.description, "topic": count.topic_name, "needed": count.report.needed,
                     "per_second": round(count.per_second, 2), "verdict": judge_one_report(count)})
    return rows


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    person.tell(f"Listening to every report for {LISTENING_SECONDS:.0f} seconds. Leave the drone on the table.")
    topic_names = [topic_name_of(report, listener) for report in EXPECTED_REPORTS]
    raw_counts = listener.count_messages(topic_names, LISTENING_SECONDS)
    counts = []
    for report, topic_name in zip(EXPECTED_REPORTS, topic_names):
        raw = raw_counts[topic_name]
        counts.append(ReportCount(report, topic_name, raw.exists, raw.message_count / LISTENING_SECONDS,
                                  raw.seconds_since_last))
    for count in counts:
        verdict_text = judge_one_report(count)
        person.tell(f"{'ok  ' if verdict_text == 'ok' else 'LOOK'}  {count.per_second:6.1f}/s  "
                    f"{'needed ' if count.report.needed else 'useful '} {count.report.description} ({count.topic_name})")
        log.sample("report_rates", topic=count.topic_name, needed=count.report.needed,
                   per_second=round(count.per_second, 2),
                   seconds_since_last=None if math.isinf(count.seconds_since_last) else round(count.seconds_since_last, 2))
    return judge_report_rates(counts)


def topic_name_of(report, listener):
    if report.topic.startswith("px4:"):
        return listener.px4_topic_names[report.topic[len("px4:"):]]
    return report.topic
