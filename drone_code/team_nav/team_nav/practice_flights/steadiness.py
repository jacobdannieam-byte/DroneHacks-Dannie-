"""Decides when the drone has really arrived: close to its target, nearly still, and staying so.

REAL DRONE CODE. Plain arithmetic with no ROS, so the tests can drive it with a pretend clock.

full_course.py uses it twice:
  - after the climb, to wait until the drone hovers steadily over the launch mark;
  - at the end of each leg of the route, to decide that the drone has arrived at the waypoint.

A rule says how close and how slow the drone must be, and for how long without a break. A
single close, slow reading is not enough, because the drone can pass through its target
while swinging past it. The condition must hold for the whole of the rule's time.

This file does not read the drone or send targets. The caller passes in the readings each cycle.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class SteadinessRule:
    """How close to the target and how slow the drone must be, and for how long.
    Use math.inf for a limit that does not apply."""
    largest_horizontal_error_metres: float     # Sideways distance from the target, seen from above.
    largest_height_error_metres: float         # Difference in height from the target.
    largest_distance_metres: float             # Straight-line distance from the target.
    largest_speed_metres_per_second: float     # Speed in any direction, which must stay below this.
    seconds_held: float                        # How long all of this must hold without a break.

    def is_met(self, position_metres, target_metres, speed_metres_per_second):
        """True if this one reading meets the rule. A missing reading never meets it."""
        if position_metres is None or speed_metres_per_second is None:
            return False
        error = np.asarray(position_metres, dtype=float) - np.asarray(target_metres, dtype=float)
        horizontal_error_metres = math.hypot(error[0], error[1])
        return (horizontal_error_metres <= self.largest_horizontal_error_metres
                and abs(error[2]) <= self.largest_height_error_metres
                and float(np.linalg.norm(error)) <= self.largest_distance_metres
                and speed_metres_per_second < self.largest_speed_metres_per_second)


class SteadinessWatch:
    """Remembers since when the readings have met a rule. Give it every cycle's readings."""

    def __init__(self, rule):
        self.rule = rule
        self.met_since_seconds = None

    def reset(self):
        self.met_since_seconds = None

    def update(self, now_seconds, position_metres, target_metres, speed_metres_per_second):
        """Adds this cycle's readings. Returns True once the rule has held for its whole time."""
        if not self.rule.is_met(position_metres, target_metres, speed_metres_per_second):
            self.met_since_seconds = None
            return False
        if self.met_since_seconds is None:
            self.met_since_seconds = now_seconds
        return now_seconds - self.met_since_seconds >= self.rule.seconds_held
