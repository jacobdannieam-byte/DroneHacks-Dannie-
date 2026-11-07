"""Hand test: move the drone 1 m forward, 1 m left, and turn it 90 degrees left, and check that
camera tracking and PX4 both see the right directions.

REAL DRONE CODE, run with the PROPELLERS OFF. It only listens to the drone.

What it proves
--------------
Our flight code steers by camera tracking's position: metres forward, left and up, measured
along the way the drone's nose pointed when tracking started (start_frame.py). If any of these
directions were swapped or reversed, for example if "left" really meant "right", the drone
would fly into a wall on its first turn. This test catches that on the floor.

It also checks PX4, the flight controller's software, which gets the same tracking through the
organizers' bridge. PX4 uses north-east-down directions and measures its heading clockwise when
seen from above, so a left turn makes PX4's heading SMALLER and camera tracking's heading
LARGER. PX4's north is not tied to tracking's forward (the bridge lets PX4 choose its own
reference), so for PX4 the test checks only distances and the angle between the two moves.

The moves
---------
Before starting, put three tape marks on the floor: A, then B 1.00 m in front of A, then C 1.00 m
to the left of B (left when facing from A to B). The person sets the drone on A facing B, then
on B, then on C, all with the nose pointing the same way, and finally turns it 90 degrees to the
left on C. At each mark the test averages the readings for a second.

Every displacement is measured in the drone's own directions at mark A, so it doesn't matter
which way the drone faced when tracking started.
"""
import math
import statistics
from dataclasses import dataclass

from team_nav.hand_tests.hand_test_tools import (
    combine, failed, heading_change_degrees, passed, px4_silence_problem,
)

# Our own choice: the tape distances. 1 m is long enough that a swapped direction is obvious.
STEP_METRES = 1.0
TURN_DEGREES = 90.0

# Our own choice. Placing a drone on a tape mark by hand is good to about 5 cm, and tracking may
# drift a few cm over 2 m. A swapped or reversed direction gives an error of 1 m or more.
LARGEST_POSITION_ERROR_METRES = 0.15
# Our own choice. The camera is about 10 cm in front of the drone's centre (the organizers'
# camera_x of 0.1053 m), so turning by hand about the centre moves it up to about 15 cm, plus
# about 5 cm for placing it by hand and a few cm of tracking drift. In the simulator a turn moved
# it 8 to 21 cm. A wrong direction shows up in the heading checks, not here.
LARGEST_POSITION_CHANGE_DURING_TURN_METRES = 0.25
# Our own choice. Turning by eye against tape lines is good to about 5 degrees.
LARGEST_HEADING_ERROR_DEGREES = 10.0
# Our own choice. The angle between two 1 m moves, each placed to about 5 cm, is good to about 6
# degrees; a mirrored direction gives 180 degrees.
LARGEST_ANGLE_ERROR_BETWEEN_MOVES_DEGREES = 15.0

# Our own choices: settle for a moment after the person lets go, then average.
SECONDS_TO_SETTLE = 1.0
SECONDS_TO_AVERAGE = 1.0


@dataclass
class Snapshot:
    """Where tracking and PX4 believe the drone is, at one mark."""
    tracking_forward_metres: float   # In the directions tracking started with.
    tracking_left_metres: float
    tracking_heading_degrees: float  # Grows when the drone turns left.
    px4_north_metres: float          # PX4's own directions; north is PX4's reference, not a compass.
    px4_east_metres: float
    px4_heading_degrees: float       # Grows when the drone turns right.


def move_in_drone_directions(snapshot_before, snapshot_after, heading_at_start_degrees):
    """The tracked move from one snapshot to the next, as (forward, left) metres along the way
    the nose pointed at mark A."""
    forward_change = snapshot_after.tracking_forward_metres - snapshot_before.tracking_forward_metres
    left_change = snapshot_after.tracking_left_metres - snapshot_before.tracking_left_metres
    heading_radians = math.radians(heading_at_start_degrees)
    forward_metres = forward_change * math.cos(heading_radians) + left_change * math.sin(heading_radians)
    left_metres = -forward_change * math.sin(heading_radians) + left_change * math.cos(heading_radians)
    return forward_metres, left_metres


def px4_move(snapshot_before, snapshot_after):
    """PX4's view of a move: its length in metres, and its compass-style direction in degrees
    (clockwise from PX4's north)."""
    north_change = snapshot_after.px4_north_metres - snapshot_before.px4_north_metres
    east_change = snapshot_after.px4_east_metres - snapshot_before.px4_east_metres
    return math.hypot(north_change, east_change), math.degrees(math.atan2(east_change, north_change))


def judge_tracked_step(name, forward_metres, left_metres, expected_forward_metres, expected_left_metres):
    forward_error = forward_metres - expected_forward_metres
    left_error = left_metres - expected_left_metres
    if max(abs(forward_error), abs(left_error)) > LARGEST_POSITION_ERROR_METRES:
        return failed(f"Camera tracking saw the {name} move as {forward_metres:+.2f} m forward and {left_metres:+.2f} m "
                      f"left, but it was {expected_forward_metres:+.2f} m forward and {expected_left_metres:+.2f} m left.")
    return passed(f"Tracking saw the {name} move as {forward_metres:+.2f} m forward, {left_metres:+.2f} m left.")


def judge_px4_step(name, px4_length_metres):
    if abs(px4_length_metres - STEP_METRES) > LARGEST_POSITION_ERROR_METRES:
        return failed(f"PX4 saw the {name} move as {px4_length_metres:.2f} m long instead of {STEP_METRES:.2f} m.")
    return passed(f"PX4 saw it as {px4_length_metres:.2f} m.")


def judge_angle_between_moves(forward_direction_degrees, left_direction_degrees):
    """In PX4's directions, angles grow clockwise, so the left move must point 90 degrees
    anticlockwise of the forward move: its direction is 90 degrees SMALLER."""
    turn_between_moves = heading_change_degrees(forward_direction_degrees, left_direction_degrees)
    if abs(turn_between_moves + TURN_DEGREES) > LARGEST_ANGLE_ERROR_BETWEEN_MOVES_DEGREES:
        return failed(f"In PX4's directions the left move pointed {turn_between_moves:+.0f} degrees from the forward "
                      f"move, not -90 degrees (PX4 measures clockwise, so left is -90). Left and right may be swapped.")
    return passed(f"PX4 saw the left move {turn_between_moves:+.0f} degrees from the forward move.")


def judge_turn(snapshot_before, snapshot_after):
    tracking_turn = heading_change_degrees(snapshot_before.tracking_heading_degrees, snapshot_after.tracking_heading_degrees)
    px4_turn = heading_change_degrees(snapshot_before.px4_heading_degrees, snapshot_after.px4_heading_degrees)
    position_change = math.hypot(snapshot_after.tracking_forward_metres - snapshot_before.tracking_forward_metres,
                                 snapshot_after.tracking_left_metres - snapshot_before.tracking_left_metres)
    parts = []
    if abs(tracking_turn - TURN_DEGREES) > LARGEST_HEADING_ERROR_DEGREES:
        parts.append(failed(f"Camera tracking saw the 90 degree left turn as {tracking_turn:+.0f} degrees "
                            f"(it should be +90: tracking's heading grows when turning left)."))
    else:
        parts.append(passed(f"Tracking saw the left turn as {tracking_turn:+.0f} degrees."))
    if abs(px4_turn + TURN_DEGREES) > LARGEST_HEADING_ERROR_DEGREES:
        parts.append(failed(f"PX4 saw the 90 degree left turn as {px4_turn:+.0f} degrees "
                            f"(it should be -90: PX4's heading grows when turning right)."))
    else:
        parts.append(passed(f"PX4 saw it as {px4_turn:+.0f} degrees."))
    if position_change > LARGEST_POSITION_CHANGE_DURING_TURN_METRES:
        parts.append(failed(f"Turning on the spot moved the tracked position by {position_change:.2f} m."))
    return combine(parts)


def judge_directions(at_a, at_b, at_c, after_turn):
    """The whole test's verdict from the four snapshots."""
    heading_at_a = at_a.tracking_heading_degrees
    forward_move = move_in_drone_directions(at_a, at_b, heading_at_a)
    left_move = move_in_drone_directions(at_b, at_c, heading_at_a)
    px4_forward_length, px4_forward_direction = px4_move(at_a, at_b)
    px4_left_length, px4_left_direction = px4_move(at_b, at_c)
    verdict = combine([
        judge_tracked_step("forward", forward_move[0], forward_move[1], STEP_METRES, 0.0),
        judge_tracked_step("left", left_move[0], left_move[1], 0.0, STEP_METRES),
        judge_px4_step("forward", px4_forward_length),
        judge_px4_step("left", px4_left_length),
        judge_angle_between_moves(px4_forward_direction, px4_left_direction),
        judge_turn(at_c, after_turn),
    ])
    verdict.measurements.update(
        tracked_forward_move_metres=[round(value, 3) for value in forward_move],
        tracked_left_move_metres=[round(value, 3) for value in left_move],
        px4_forward_move_length_metres=round(px4_forward_length, 3),
        px4_left_move_length_metres=round(px4_left_length, 3),
    )
    if verdict.passed:
        verdict.reason = "Camera tracking and PX4 both saw forward, left and a left turn the right way round. " + verdict.reason
    return verdict


# ---- Running it on the drone -------------------------------------------------------------------

def run(listener, person, log):
    problem = px4_silence_problem(listener.reports.seconds_since("px4_status"))
    if problem:
        return problem
    person.tell("Before starting, put three tape marks on the floor: A; B exactly 1.00 m in front of A; and C "
                "exactly 1.00 m to the left of B (left when you stand at A looking at B).")
    instructions = [
        ("A", "Set the drone on mark A, its nose pointing exactly towards mark B, and let go."),
        ("B", "Pick the drone up and set it on mark B, nose still pointing the same way as before, and let go."),
        ("C", "Pick the drone up and set it on mark C (1 m to its left), nose still pointing the same way, and let go."),
        ("C turned", "Turn the drone on mark C by 90 degrees to the LEFT (anticlockwise seen from above), "
                     "keeping its centre on the mark, and let go."),
    ]
    snapshots = []
    for mark_name, instruction in instructions:
        person.do_this_then_press_enter(instruction)
        snapshot = take_snapshot(listener)
        if snapshot is None:
            return failed(f"At mark {mark_name}, camera tracking or PX4's position was missing. Check that camera "
                          f"tracking is running (ros2 topic hz /visual_slam/tracking/odometry).")
        log.event("mark", f"Readings at mark {mark_name}.", mark=mark_name, snapshot=snapshot.__dict__)
        snapshots.append(snapshot)
    return judge_directions(*snapshots)


def take_snapshot(listener):
    """Averages the readings for SECONDS_TO_AVERAGE after letting the drone settle. None if
    tracking or PX4's position is missing."""
    listener.wait(SECONDS_TO_SETTLE)
    start_seconds = listener.now()
    listener.wait(SECONDS_TO_AVERAGE)
    tracking = listener.tracking_since(start_seconds)
    px4_position = listener.reports.latest.get("px4_position")
    if not tracking or px4_position is None or not px4_position.xy_valid:
        return None
    return Snapshot(
        tracking_forward_metres=statistics.mean(report.forward_metres for report in tracking),
        tracking_left_metres=statistics.mean(report.left_metres for report in tracking),
        # The heading of the last report, because averaging angles near +-180 degrees goes wrong.
        tracking_heading_degrees=tracking[-1].heading_degrees,
        px4_north_metres=float(px4_position.x),
        px4_east_metres=float(px4_position.y),
        px4_heading_degrees=math.degrees(float(px4_position.heading)),
    )
