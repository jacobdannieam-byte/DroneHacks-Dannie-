"""Watches the camera tracking position reports for two things: freezing, and flying speed.

How it fits: mission.py hands it every position report and asks whether tracking has frozen (to
refuse a start, and to hold while hovering). It also keeps the current and fastest horizontal
speed flown: mission.py logs the fastest, and the route mission waits at each corner until the
current speed is low. Sudden jumps are the position watchdog's job (position_watchdog.py). It never
talks to ROS and never reads a clock.

What camera tracking is on drone 16 (measured on the drone): cuVSLAM runs with enable_imu_fusion
false, so it is pure stereo visual odometry. It uses no motion sensor (IMU) to carry its estimate
through darkness or a blank view. It also runs with enable_localization_n_mapping false, so there is
no loop closure: drift is never corrected by the tracker itself. So it can freeze, jump, or slowly
drift, and nothing inside it will notice.

Why freezing matters: on drone 16 (2026-11-07), after tracking died, its reports kept arriving 30
times a second with a bit-identical position for many seconds. The reports look fresh, but they
say nothing. Real tracking always varies a little from one report to the next, even when the drone
is still, so a long run of identical positions means frozen.

Why speed matters: PX4 puts no useful limit on horizontal speed when flying to a position target
(its setting MPC_XY_VEL_MAX is 12 m/s on drone 16, from the organizer code audit). Only our carrot,
kept at most 0.5 m ahead, keeps the drone slow, so we log the fastest speed actually flown to check.
"""
import math

# Our own choice: horizontal speed is measured over at least this much position history, so that
# the noise in single reports does not look like speed.
SPEED_WINDOW_SECONDS = 0.2


class PositionWatch:
    def __init__(self, frozen_after_identical_reports):
        self.frozen_after_identical_reports = frozen_after_identical_reports
        # These describe the live report stream, not a run, so reset() leaves them alone.
        self.last_report = None
        self.identical_reports_in_a_row = 0
        self.reset()

    def reset(self):
        """Forget everything from the previous run."""
        self.recent_reports = []                 # [(seconds, x, y)]
        self.peak_horizontal_speed_metres_per_second = 0.0
        self.latest_horizontal_speed_metres_per_second = None

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
        self.latest_horizontal_speed_metres_per_second = speed
        self.peak_horizontal_speed_metres_per_second = max(self.peak_horizontal_speed_metres_per_second, speed)

    def tracking_frozen(self):
        """True after more than frozen_after_identical_reports identical positions in a row."""
        return self.identical_reports_in_a_row > self.frozen_after_identical_reports
