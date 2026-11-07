"""Hand test: carry the drone through the tunnel and back, and check that camera tracking keeps
working and comes back to where it started.

REAL DRONE CODE, run with the PROPELLERS OFF. It only listens to the drone.

What it proves
--------------
Camera tracking is the organizers' program (cuVSLAM) that works out how the drone has moved by
following details in the front cameras' pictures. It is the drone's only source of sideways
position. In the tunnel the walls are close and may be plain, which is where it is most likely
to lose track (docs/ARCHITECTURE.md, section 5). If it loses track in flight, PX4 no longer
knows where the drone is and the automatic flight ends.

This test carries the drone along the flight path by hand and checks four things:
  - tracking never says it is lost (its status report's vo_state is never 2);
  - its reports never stop for longer than our flight code tolerates;
  - its position never jumps, which is what a quiet reset looks like;
  - after the round trip, back on the start mark, it is close to where it started. The
    difference is the drift over the distance carried, which section 12 lists as unknown.
"""
import math
from dataclasses import dataclass

from team_nav.hand_tests.hand_test_tools import combine, failed, heading_change_degrees, passed

# From the organizers' code (px4_odom_bridge.py): the status value that means tracking is lost.
TRACKING_LOST_STATE = 2

# From our plan (docs/ARCHITECTURE.md, section 11): the flight code holds if position updates
# stop for more than 0.2 s. A longer gap here means the flight would have stopped there.
LONGEST_GAP_BETWEEN_REPORTS_SECONDS = 0.2
# Our own choice. Carried at a slow walk (under 1 m/s) and about 30 reports a second, the drone
# moves about 3 cm between reports. A step of more than 10 cm is a jump, not a movement.
LARGEST_STEP_BETWEEN_REPORTS_METRES = 0.10
# Our own choice. Section 5 guesses drift at about 1% of the distance flown. The round trip is
# roughly 10 to 20 m, so 1% is 10 to 20 cm. More than this is clearly worse than we planned for.
LARGEST_RETURN_ERROR_METRES = 0.30
# Our own choice. Setting the drone back on its mark by eye is good to about 5 degrees.
LARGEST_RETURN_HEADING_ERROR_DEGREES = 10.0
# Our own choice: at least this many reports, or nothing was really tested.
FEWEST_REPORTS = 30


@dataclass
class CarrySummary:
    report_count: int
    longest_gap_seconds: float
    largest_step_metres: float
    distance_carried_metres: float
    return_error_metres: float
    return_heading_error_degrees: float
    times_reported_lost: int


def summarise_carry(tracking_reports, tracking_states):
    """Boils the whole carry down to the numbers the pass rule needs. tracking_reports are
    TrackingReport objects (drone_listener.py), oldest first; tracking_states are vo_state values."""
    longest_gap = 0.0
    largest_step = 0.0
    distance_carried = 0.0
    for earlier, later in zip(tracking_reports, tracking_reports[1:]):
        longest_gap = max(longest_gap, later.arrival_seconds - earlier.arrival_seconds)
        step = math.dist((earlier.forward_metres, earlier.left_metres, earlier.up_metres),
                         (later.forward_metres, later.left_metres, later.up_metres))
        largest_step = max(largest_step, step)
        distance_carried += step
    first, last = tracking_reports[0], tracking_reports[-1]
    return CarrySummary(
        report_count=len(tracking_reports),
        longest_gap_seconds=longest_gap,
        largest_step_metres=largest_step,
        distance_carried_metres=distance_carried,
        return_error_metres=math.dist((first.forward_metres, first.left_metres), (last.forward_metres, last.left_metres)),
        return_heading_error_degrees=heading_change_degrees(first.heading_degrees, last.heading_degrees),
        times_reported_lost=sum(1 for state in tracking_states if state == TRACKING_LOST_STATE),
    )


def judge_carry(summary):
    if summary.report_count < FEWEST_REPORTS:
        return failed(f"Only {summary.report_count} camera tracking reports arrived during the carry. Check that "
                      f"tracking is running (ros2 topic hz /visual_slam/tracking/odometry).")
    parts = []
    if summary.times_reported_lost > 0:
        parts.append(failed(f"Camera tracking said it was lost in {summary.times_reported_lost} status reports."))
    if summary.longest_gap_seconds > LONGEST_GAP_BETWEEN_REPORTS_SECONDS:
        parts.append(failed(f"Tracking reports stopped for {summary.longest_gap_seconds:.2f} s once, longer than the "
                            f"{LONGEST_GAP_BETWEEN_REPORTS_SECONDS} s after which the flight code holds the drone."))
    if summary.largest_step_metres > LARGEST_STEP_BETWEEN_REPORTS_METRES:
        parts.append(failed(f"The tracked position jumped {summary.largest_step_metres:.2f} m between two reports, "
                            f"which looks like tracking restarted."))
    if summary.return_error_metres > LARGEST_RETURN_ERROR_METRES:
        parts.append(failed(f"Back on the start mark, tracking was {summary.return_error_metres:.2f} m from where it "
                            f"started, after {summary.distance_carried_metres:.1f} m carried."))
    if abs(summary.return_heading_error_degrees) > LARGEST_RETURN_HEADING_ERROR_DEGREES:
        parts.append(failed(f"Back on the start mark, tracking's heading was {summary.return_heading_error_degrees:+.0f} "
                            f"degrees off."))
    measurements = {name: round(value, 3) for name, value in summary.__dict__.items()}
    if parts:
        verdict = combine(parts)
        verdict.measurements.update(measurements)
        return verdict
    return passed(f"Camera tracking held through the tunnel: never lost, no gap or jump, and back on the mark it was "
                  f"{summary.return_error_metres:.2f} m and {summary.return_heading_error_degrees:+.1f} degrees off after "
                  f"{summary.distance_carried_metres:.1f} m carried.", **measurements)


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    person.tell("Put a tape mark S on the floor a little before the tunnel's entrance. You will carry the drone "
                "through the tunnel and back. Hold it at about 0.8 m (its flying height), level, with its nose "
                "(the camera) pointing the way you walk, at a slow walk. Keep your body behind the drone so "
                "the camera sees the course, not you.")
    person.do_this_then_press_enter("Set the drone on mark S, nose pointing into the tunnel, and let go.")
    start_seconds = listener.now()
    person.do_this_then_press_enter(
        "Now pick it up and carry it through the whole tunnel. At the far end, turn round slowly and carry it back "
        "through the tunnel. Set it on mark S again, nose pointing into the tunnel exactly as at the start, and let go.")
    listener.wait(1.0)   # Our own choice: let the last reports arrive after the drone is set down.
    tracking_reports = listener.tracking_since(start_seconds)
    tracking_states = listener.tracking_states_since(start_seconds)
    for report in tracking_reports:
        log.sample("tracking", forward_metres=report.forward_metres, left_metres=report.left_metres,
                   up_metres=report.up_metres, heading_degrees=round(report.heading_degrees, 2))
    if not listener.can_read_tracking_status():
        person.tell("Note: tracking's status messages could not be read here, so 'never lost' was not checked.")
    if len(tracking_reports) < 2:
        return failed("No camera tracking reports arrived during the carry. Check that tracking is running "
                      "(ros2 topic hz /visual_slam/tracking/odometry).")
    return judge_carry(summarise_carry(tracking_reports, tracking_states))
