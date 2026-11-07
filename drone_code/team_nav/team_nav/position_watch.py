"""Watches the camera tracking position reports for two things: sudden jumps and flying speed.

How it fits: mission.py hands it every position report, and once per step asks whether the position
jumped since the previous step. It also keeps the fastest horizontal speed flown, which mission.py
logs. It never talks to ROS and never reads a clock.

Why jumps matter: the drone cannot really move half a metre in one step (a twentieth of a second).
A jump that big means camera tracking has suddenly re-estimated where it is, so every position
target worked out before the jump now points somewhere else.

Why speed matters: PX4 puts no useful limit on horizontal speed when flying to a position target
(its setting MPC_XY_VEL_MAX is 12 m/s on drone 16, from the organizer code audit). Only our carrot,
kept at most 0.5 m ahead, keeps the drone slow, so we log the fastest speed actually flown to check.
"""
import math

# Our own choice: horizontal speed is measured over at least this much position history, so that
# the noise in single reports does not look like speed.
SPEED_WINDOW_SECONDS = 0.2
# Our own choice: more than three times our cruise speed. After a gap in the reports, a jump is
# only counted if it is bigger than the drone could have flown at this speed during the gap.
FASTEST_BELIEVABLE_SPEED_METRES_PER_SECOND = 1.5


class PositionWatch:
    def __init__(self, jump_limit_metres, step_seconds):
        self.jump_limit_metres = jump_limit_metres
        self.step_seconds = step_seconds
        self.reset()

    def reset(self):
        """Forget everything from the previous run."""
        self.recent_reports = []                 # [(seconds, x, y)]
        self.peak_horizontal_speed_metres_per_second = 0.0
        self.position_at_last_step = None
        self.last_step_seconds = None

    def observe(self, now_seconds, position):
        self.recent_reports.append((now_seconds, position[0], position[1]))
        oldest_kept = now_seconds - 2 * SPEED_WINDOW_SECONDS
        self.recent_reports = [report for report in self.recent_reports if report[0] >= oldest_kept]
        older = [report for report in self.recent_reports if report[0] <= now_seconds - SPEED_WINDOW_SECONDS]
        if not older:
            return
        then_seconds, then_x, then_y = older[-1]
        speed = math.hypot(position[0] - then_x, position[1] - then_y) / (now_seconds - then_seconds)
        self.peak_horizontal_speed_metres_per_second = max(self.peak_horizontal_speed_metres_per_second, speed)

    def jump_since_last_step(self, now_seconds, position):
        """The jump in metres if the position moved further than believable since the last step,
        else None. Call it once per step, only with a fresh position."""
        previous, previous_seconds = self.position_at_last_step, self.last_step_seconds
        self.position_at_last_step, self.last_step_seconds = tuple(position), now_seconds
        if previous is None:
            return None
        # Between two steps in a row the limit is exact. After a gap in the reports, allow for the
        # distance the drone could really have flown during the extra time.
        extra_seconds = max(0.0, now_seconds - previous_seconds - self.step_seconds)
        allowed = self.jump_limit_metres + FASTEST_BELIEVABLE_SPEED_METRES_PER_SECOND * extra_seconds
        jump = math.dist(previous, position)
        return jump if jump > allowed else None
