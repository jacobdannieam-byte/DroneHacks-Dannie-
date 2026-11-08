"""The gentle 180 degree turn of the tunnel_first mission, and the heading guard that watches PX4's
heading against camera tracking's for the rest of the flight.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. tunnel_first_run.py feeds it readings and sends the headings it asks for.

Words
-----
"Heading" (or "yaw") is the way the drone's nose points, seen from above. PX4 (the flight
controller's software) keeps its own heading estimate; camera tracking keeps another. They should
move together. The "heading split" is PX4's heading minus camera tracking's.

Why a slow ramp (from our real flight logs, telemetry/skybeta-16/flight_records)
--------------------------------------------------------------------------------
A one-step 90 degree heading command made PX4's heading split from camera tracking's by 36
degrees. A gentle 165 degree turn was fine. So the turn's commanded heading moves only
STEP_DEGREES every STEP_SECONDS (10 degrees a second), starting from PX4's own present heading,
so the very first command is no step at all.

The turn's rules (the team's design, 2026-11-08)
-----------------------------------------------
  - Always counter-clockwise (to the left), by exactly TURN_DEGREES, measured on the commanded
    heading, so it can never take the short way round the other side.
  - The ramp pauses while PX4's heading lags the command by more than LARGEST_LAG_DEGREES, or
    while the heading split has drifted more than LARGEST_SPLIT_CHANGE_DEGREES from its value
    before the turn. When it resumes it moves one step again: it never catches up with a jump.
  - Done when the command has reached the end and, for SETTLED_SECONDS in a row, the drone moves
    slower than SETTLED_SPEED_METRES_PER_SECOND, turns slower than SETTLED_YAW_RATE_DEGREES_PER_SECOND,
    PX4's heading is within SETTLED_YAW_ERROR_DEGREES of the command, and the split is back within
    SETTLED_SPLIT_CHANGE_DEGREES of its value before the turn.

The heading guard (outside the turn)
------------------------------------
If the heading split differs from its value on the launch mark by more than
GUARD_LIMIT_DEGREES for more than GUARD_SECONDS_BEFORE_HOLD, the drone holds still. If that lasts
GUARD_SECONDS_BEFORE_LANDING, the run stops and lands. It compares with the launch mark's value,
not with zero, because PX4's and camera tracking's headings may be measured from different zero
directions; only a change between them means one has gone wrong.

What this file does not do: send anything, or decide what the drone does after an abort.
"""
import math

# The team's choices (2026-11-08) for the turn.
TURN_DEGREES = 180.0
STEP_DEGREES = 0.5
STEP_SECONDS = 0.05
LARGEST_LAG_DEGREES = 5.0
LARGEST_SPLIT_CHANGE_DEGREES = 5.0
SETTLED_SECONDS = 1.0
SETTLED_SPEED_METRES_PER_SECOND = 0.05
SETTLED_YAW_RATE_DEGREES_PER_SECOND = 3.0
SETTLED_YAW_ERROR_DEGREES = 3.0
SETTLED_SPLIT_CHANGE_DEGREES = 3.0
# Our own choices: a turn that has not finished in this long is stopped (the ramp alone takes 18 s).
LONGEST_TURN_SECONDS = 60.0
# The team's choices (2026-11-08) for the heading guard.
GUARD_LIMIT_DEGREES = 10.0
GUARD_SECONDS_BEFORE_HOLD = 0.5
GUARD_SECONDS_BEFORE_LANDING = 3.0


def wrapped_degrees(angle_degrees):
    """The same angle, between -180 and 180 degrees."""
    return math.degrees(math.atan2(math.sin(math.radians(angle_degrees)), math.cos(math.radians(angle_degrees))))


def odom_heading_from_px4_heading(px4_heading_radians):
    """PX4's heading (vehicle_local_position.heading: from north, clockwise, "NED") written the way
    camera tracking and flight_interface write headings (from the x axis toward y, "ENU").
    From the organizers' code (frames.yaw_enu_to_ned): heading_ned = pi/2 - heading_enu, its own
    inverse."""
    return math.atan2(math.sin(math.pi / 2.0 - px4_heading_radians), math.cos(math.pi / 2.0 - px4_heading_radians))


class TurnRamp:
    """The commanded heading during the turn, one step at a time."""

    def __init__(self, start_heading_degrees):
        """start_heading_degrees is PX4's present heading, in camera tracking's way of writing it."""
        self.start_heading_degrees = start_heading_degrees
        self.commanded_turn_degrees = 0.0
        self.paused_because = None

    def next_command_degrees(self, px4_turn_so_far_degrees, split_change_degrees):
        """One step: moves the command on by STEP_DEGREES unless it must pause, and returns the
        commanded heading (not wrapped). px4_turn_so_far_degrees is how far PX4's heading has turned
        to the left since the start; split_change_degrees is the heading split's change since then."""
        self.paused_because = self._reason_to_pause(px4_turn_so_far_degrees, split_change_degrees)
        if self.paused_because is None:
            self.commanded_turn_degrees = min(self.commanded_turn_degrees + STEP_DEGREES, TURN_DEGREES)
        return self.start_heading_degrees + self.commanded_turn_degrees

    def command_has_reached_the_end(self):
        return self.commanded_turn_degrees >= TURN_DEGREES

    def final_heading_degrees(self):
        return self.start_heading_degrees + TURN_DEGREES

    def _reason_to_pause(self, px4_turn_so_far_degrees, split_change_degrees):
        lag_degrees = self.commanded_turn_degrees - px4_turn_so_far_degrees
        if lag_degrees > LARGEST_LAG_DEGREES:
            return f"PX4's heading lags the command by {lag_degrees:.1f} degrees"
        if abs(split_change_degrees) > LARGEST_SPLIT_CHANGE_DEGREES:
            return f"the heading split has changed by {split_change_degrees:+.1f} degrees"
        return None


class TurnTracker:
    """How far a heading has turned since a start, counting whole turns (headings wrap at 180)."""

    def __init__(self, start_heading_degrees):
        self.last_heading_degrees = start_heading_degrees
        self.turned_degrees = 0.0

    def update(self, heading_degrees):
        self.turned_degrees += wrapped_degrees(heading_degrees - self.last_heading_degrees)
        self.last_heading_degrees = heading_degrees
        return self.turned_degrees


class SettledTimer:
    """True once a condition has held for SETTLED_SECONDS in a row."""

    def __init__(self, seconds_needed=SETTLED_SECONDS):
        self.seconds_needed = seconds_needed
        self.held_since_seconds = None

    def update(self, now_seconds, condition_holds):
        if not condition_holds:
            self.held_since_seconds = None
            return False
        if self.held_since_seconds is None:
            self.held_since_seconds = now_seconds
        return now_seconds - self.held_since_seconds >= self.seconds_needed


def turn_is_settled(speed_metres_per_second, yaw_rate_degrees_per_second, yaw_error_degrees, split_change_degrees):
    return (speed_metres_per_second < SETTLED_SPEED_METRES_PER_SECOND
            and abs(yaw_rate_degrees_per_second) < SETTLED_YAW_RATE_DEGREES_PER_SECOND
            and abs(yaw_error_degrees) < SETTLED_YAW_ERROR_DEGREES
            and abs(split_change_degrees) < SETTLED_SPLIT_CHANGE_DEGREES)


# The heading guard's answers.
GUARD_OK = "carry on"
GUARD_HOLD = "hold still"
GUARD_LAND = "stop the run and land"


class HeadingGuard:
    """Outside the turn: holds, then lands, if the heading split moves away from its launch value."""

    def __init__(self, launch_split_degrees):
        self.launch_split_degrees = launch_split_degrees
        self.off_since_seconds = None

    def check(self, now_seconds, split_degrees):
        """Returns GUARD_OK, GUARD_HOLD or GUARD_LAND. A missing split (None) proves nothing."""
        if split_degrees is None or abs(wrapped_degrees(split_degrees - self.launch_split_degrees)) <= GUARD_LIMIT_DEGREES:
            self.off_since_seconds = None
            return GUARD_OK
        if self.off_since_seconds is None:
            self.off_since_seconds = now_seconds
        seconds_off = now_seconds - self.off_since_seconds
        if seconds_off > GUARD_SECONDS_BEFORE_HOLD + GUARD_SECONDS_BEFORE_LANDING:
            return GUARD_LAND
        if seconds_off > GUARD_SECONDS_BEFORE_HOLD:
            return GUARD_HOLD
        return GUARD_OK

    def change_degrees(self, split_degrees):
        return wrapped_degrees(split_degrees - self.launch_split_degrees)
