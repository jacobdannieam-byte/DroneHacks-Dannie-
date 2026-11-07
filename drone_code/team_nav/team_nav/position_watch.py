"""Watches the camera tracking position reports for three things: freezing, sudden jumps, and
flying speed.

How it fits: mission.py hands it every position report, asks whether tracking has frozen, and once
per step asks whether the position jumped since the previous step. It also keeps the fastest
horizontal speed flown, which mission.py logs. It never talks to ROS and never reads a clock.

What camera tracking is on drone 16 (measured on the drone): cuVSLAM runs with enable_imu_fusion
false, so it is pure stereo visual odometry. It uses no motion sensor (IMU) to carry its estimate
through darkness or a blank view. It also runs with enable_localization_n_mapping false, so there is
no loop closure: drift is never corrected by the tracker itself. So it can freeze, jump, or slowly
drift, and nothing inside it will notice.

Why freezing matters: on drone 16 (2026-11-07), after tracking died, its reports kept arriving 30
times a second with a bit-identical position for many seconds. The reports look fresh, but they
say nothing. Real tracking always varies a little from one report to the next, even when the drone
is still, so a long run of identical positions means frozen.

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
    def __init__(self, jump_limit_metres, step_seconds, frozen_after_identical_reports):
        self.jump_limit_metres = jump_limit_metres
        self.step_seconds = step_seconds
        self.frozen_after_identical_reports = frozen_after_identical_reports
        # These describe the live report stream, not a run, so reset() leaves them alone.
        self.last_report = None
        self.identical_reports_in_a_row = 0
        self.reset()

    def reset(self):
        """Forget everything from the previous run."""
        self.recent_reports = []                 # [(seconds, x, y)]
        self.peak_horizontal_speed_metres_per_second = 0.0
        self.position_at_last_step = None
        self.last_step_seconds = None

    def observe(self, now_seconds, position):
        if self.last_report is not None and tuple(position) == self.last_report:   # bit-identical
            self.identical_reports_in_a_row += 1
        else:
            self.identical_reports_in_a_row = 1
        self.last_report = tuple(position)
        self.recent_reports.append((now_seconds, position[0], position[1]))
        oldest_kept = now_seconds - 2 * SPEED_WINDOW_SECONDS
        self.recent_reports = [report for report in self.recent_reports if report[0] >= oldest_kept]
        older = [report for report in self.recent_reports if report[0] <= now_seconds - SPEED_WINDOW_SECONDS]
        if not older:
            return
        then_seconds, then_x, then_y = older[-1]
        speed = math.hypot(position[0] - then_x, position[1] - then_y) / (now_seconds - then_seconds)
        self.peak_horizontal_speed_metres_per_second = max(self.peak_horizontal_speed_metres_per_second, speed)

    def tracking_frozen(self):
        """True after more than frozen_after_identical_reports identical positions in a row."""
        return self.identical_reports_in_a_row > self.frozen_after_identical_reports

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
