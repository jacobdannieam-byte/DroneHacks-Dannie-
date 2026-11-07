"""The position watchdog: for the whole flight, it watches for signs that the drone's position can no
longer be trusted, and says which rule tripped.

How it fits: mission.py hands it every camera tracking report and every PX4 position report, calls
start_watching() at takeoff, and asks check() once per step. The first rule to trip is latched;
mission.py then lands at once, using the normal landing (landing.py). It never talks to ROS and
never reads a clock.

The rules, lettered as the team named them (the team's list has no rule (d)):
  (a) No new camera tracking report for 0.3 s, judged by when the reports arrive. (The simulator's
      reports pause for 0.21 s now and then; PX4 itself gives up on them after about 0.4 s.)
  (b) A camera tracking report moved further from the one before than 1.5 m/s times the time
      between them plus 0.05 m. The time between them comes from the reports' own timestamps,
      so a late delivery does not make a real move look like a jump.
  (c) Position and orientation bit-identical for 0.5 s while a target asks the drone to move.
      Real camera tracking always varies a little between reports; drone 16's, after tracking
      died, repeated one position exactly for many seconds (2026-11-07).
  (e) PX4 reset its horizontal position or heading estimate after takeoff: xy_reset_counter or
      heading_reset_counter in its position report changed. Every target worked out before a
      reset points somewhere else afterwards.
Rule (f), PX4 leaving Offboard, is checked in mission.py, because it means stop, not land.

What it deliberately does not use: cuVSLAM's own status (vo_state on /visual_slam/status). On drone
16 it stayed at 1 ("tracking") while the camera was covered, so mission_node.py only logs it.

Every threshold is a setting in mission_settings.py, and a launch argument of the same name.
"""
import math


class PositionWatchdog:
    def __init__(self, settings):
        self.settings = settings
        self.watching = False
        self.tripped = None                      # (rule letter, reason), latched until the next takeoff
        self.last_report_received_seconds = None
        self.last_report_stamp_seconds = None
        self.last_report_position = None
        self.last_pose = None                    # (position, orientation), compared exactly
        self.pose_unchanged_since_seconds = None
        self.motion_requested_since_seconds = None
        self.latest_reset_counters = None        # (xy, heading) from PX4's latest position report
        self.reset_counters_at_takeoff = None

    # ------------------------------------------------------------------ calls from mission.py
    def start_watching(self, now_seconds):
        """At takeoff. The reset counters PX4 reports now are the ones that must not change."""
        self.watching = True
        self.tripped = None
        self.motion_requested_since_seconds = None
        self.reset_counters_at_takeoff = self.latest_reset_counters

    def stop_watching(self):
        self.watching = False

    def observe_report(self, received_seconds, stamp_seconds, position, orientation):
        """A camera tracking report: when it arrived, its own timestamp, position and orientation."""
        if stamp_seconds is None or stamp_seconds <= 0.0:
            stamp_seconds = received_seconds     # no timestamp in the report: the best we can do
        position = tuple(position)
        if self.watching and self.last_report_position is not None:
            self.check_for_jump(stamp_seconds, position)
        self.last_report_received_seconds = received_seconds
        self.last_report_stamp_seconds = stamp_seconds
        self.last_report_position = position
        pose = (position, tuple(orientation))
        if pose != self.last_pose:
            self.last_pose = pose
            self.pose_unchanged_since_seconds = received_seconds

    def observe_reset_counters(self, xy_reset_counter, heading_reset_counter):
        counters = (xy_reset_counter, heading_reset_counter)
        self.latest_reset_counters = counters
        if not self.watching:
            return
        if self.reset_counters_at_takeoff is None:
            self.reset_counters_at_takeoff = counters    # no report had arrived at takeoff
            return
        if counters != self.reset_counters_at_takeoff:
            before = self.reset_counters_at_takeoff
            self.trip("e", f"PX4 reset its position or heading estimate after takeoff (xy_reset_counter "
                           f"{before[0]} -> {counters[0]}, heading_reset_counter {before[1]} -> {counters[1]})")

    def check(self, now_seconds, motion_requested):
        """Rules (a) and (c), which depend on the time now. Returns the latched trip, or None."""
        if not self.watching or self.tripped is not None:
            return self.tripped
        self.check_reports_still_arriving(now_seconds)
        self.check_for_frozen_tracking(now_seconds, motion_requested)
        return self.tripped

    # ------------------------------------------------------------------ the rules
    def trip(self, rule, reason):
        if self.tripped is None:
            self.tripped = (rule, reason)

    def check_for_jump(self, stamp_seconds, position):
        settings = self.settings
        seconds_between = max(0.0, stamp_seconds - self.last_report_stamp_seconds)
        allowed = settings.watchdog_jump_speed_metres_per_second * seconds_between + settings.watchdog_jump_margin_metres
        moved = math.dist(position, self.last_report_position)
        if moved > allowed:
            self.trip("b", f"camera tracking moved {moved:.2f} m in {seconds_between:.3f} s between two reports "
                           f"(at most {allowed:.2f} m is believable)")

    def check_reports_still_arriving(self, now_seconds):
        limit = self.settings.watchdog_odometry_timeout_seconds
        last = self.last_report_received_seconds
        if last is None:
            self.trip("a", "no camera tracking report has arrived")
        elif now_seconds - last > limit:
            self.trip("a", f"no new camera tracking report for {now_seconds - last:.2f} s (limit {limit:g} s)")

    def check_for_frozen_tracking(self, now_seconds, motion_requested):
        if not motion_requested:
            self.motion_requested_since_seconds = None
            return
        if self.motion_requested_since_seconds is None:
            self.motion_requested_since_seconds = now_seconds
        if self.pose_unchanged_since_seconds is None:
            return
        frozen_since = max(self.pose_unchanged_since_seconds, self.motion_requested_since_seconds)
        limit = self.settings.watchdog_frozen_seconds
        if now_seconds - frozen_since >= limit - 1e-9:
            self.trip("c", f"camera tracking position and orientation bit-identical for "
                           f"{now_seconds - frozen_since:.2f} s while a target asks the drone to move (limit {limit:g} s)")
