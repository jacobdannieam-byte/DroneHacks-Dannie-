"""Table check: camera tracking's position stays put while the drone sits still for 30 seconds.

REAL DRONE CODE, run on the table with the PROPELLERS OFF. It only listens to the drone.

What it proves
--------------
Camera tracking is the organizers' program (cuVSLAM) that works out how the drone has moved by
following details in the front cameras' pictures. It is the drone's only source of sideways
position. A drone that isn't moving sees the same picture all the time, so its tracked position
should not change by more than noise. If it wanders while still, it will wander in flight too,
and the drone will fly off its route by that much. It also needs something to look at: point
the camera at a detailed scene (the course, a shelf), not a blank wall.

Each report jitters by a few millimetres, so the test compares averages: the average position of
each second against the average of the first second. That shows a slow wander, not jitter.

PASS means that over STILL_SECONDS tracking never said it was lost, reported often enough, and
its averaged position and heading never moved further than the limits below from where they
started.
"""
import math

from team_nav.hand_tests.hand_test_tools import failed, heading_change_degrees, passed

# Our own choice, from the task: long enough to see a slow wander.
STILL_SECONDS = 30.0
# Our own choice: each average covers this long, about 30 reports.
AVERAGING_SECONDS = 1.0
# Our own choice. Noise in a still picture should move the position by well under a centimetre.
# Two centimetres in 30 s would be 4 cm a minute, a wander our route margins (5 cm spare) can't
# absorb over a run.
LARGEST_STILL_MOVEMENT_METRES = 0.02
# Our own choice: the same reasoning for heading. One degree is 9 cm sideways error after 5 m.
LARGEST_STILL_TURN_DEGREES = 1.0
# Our own choice, about half of the 28.9 per second measured on drone 16 on 2026-11-07.
FEWEST_REPORTS_PER_SECOND = 15.0
# From the organizers' code (px4_odom_bridge.py): the status value that means tracking is lost.
TRACKING_LOST_STATE = 2


def judge_still_drift(tracking_reports, tracking_states, seconds):
    """tracking_reports are drone_listener.py's TrackingReport objects, oldest first;
    tracking_states are the vo_state values from tracking's status reports."""
    rate = len(tracking_reports) / seconds
    if rate < FEWEST_REPORTS_PER_SECOND:
        return failed(f"Camera tracking reported only {rate:.1f} times a second, below {FEWEST_REPORTS_PER_SECOND:g}. "
                      f"Check it is running (ros2 topic hz /visual_slam/tracking/odometry).")
    if TRACKING_LOST_STATE in tracking_states:
        return failed("Camera tracking said it was lost while the drone sat still. Point the camera at a detailed "
                      "scene, not a blank wall, and check the lighting.")
    averages = one_second_averages(tracking_reports)
    first = averages[0]
    largest_movement = 0.0
    largest_turn = 0.0
    for average in averages:
        largest_movement = max(largest_movement, math.dist(first[:3], average[:3]))
        largest_turn = max(largest_turn, abs(heading_change_degrees(first[3], average[3])))
    measurements = {"largest_movement_metres": round(largest_movement, 4), "largest_turn_degrees": round(largest_turn, 3),
                    "reports_per_second": round(rate, 1)}
    if largest_movement > LARGEST_STILL_MOVEMENT_METRES or largest_turn > LARGEST_STILL_TURN_DEGREES:
        return failed(f"Sitting still for {seconds:.0f} s, camera tracking's averaged position wandered up to "
                      f"{largest_movement * 100:.1f} cm and its heading {largest_turn:.2f} degrees (limits "
                      f"{LARGEST_STILL_MOVEMENT_METRES * 100:.0f} cm and {LARGEST_STILL_TURN_DEGREES:g} degree). Make sure "
                      f"nothing moved the drone and the camera sees a detailed, still scene.", **measurements)
    return passed(f"Sitting still for {seconds:.0f} s, camera tracking moved at most {largest_movement * 100:.1f} cm and "
                  f"{largest_turn:.2f} degrees.", **measurements)


def one_second_averages(tracking_reports):
    """(forward, left, up, heading) averaged over each AVERAGING_SECONDS, oldest first. Headings
    are averaged as changes from the first report, so a heading near +-180 degrees can't wrap."""
    start_seconds = tracking_reports[0].arrival_seconds
    reference_heading = tracking_reports[0].heading_degrees
    groups = {}
    for report in tracking_reports:
        group_number = int((report.arrival_seconds - start_seconds) / AVERAGING_SECONDS)
        groups.setdefault(group_number, []).append(report)
    averages = []
    for group_number in sorted(groups):
        group = groups[group_number]
        count = len(group)
        averages.append((
            sum(report.forward_metres for report in group) / count,
            sum(report.left_metres for report in group) / count,
            sum(report.up_metres for report in group) / count,
            reference_heading + sum(heading_change_degrees(reference_heading, report.heading_degrees)
                                    for report in group) / count,
        ))
    return averages


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    person.tell(f"Camera tracking still test: for the next {STILL_SECONDS:.0f} seconds don't touch the drone or the "
                f"table, and keep people out of the camera's view.")
    start_seconds = listener.now()
    listener.wait(STILL_SECONDS)
    tracking_reports = listener.tracking_since(start_seconds)
    for report in tracking_reports:
        log.sample("tracking", forward_metres=report.forward_metres, left_metres=report.left_metres,
                   up_metres=report.up_metres, heading_degrees=round(report.heading_degrees, 3))
    if not tracking_reports:
        return failed("No camera tracking report arrived. Check that tracking is running "
                      "(ros2 topic hz /visual_slam/tracking/odometry).")
    return judge_still_drift(tracking_reports, listener.tracking_states_since(start_seconds), STILL_SECONDS)
