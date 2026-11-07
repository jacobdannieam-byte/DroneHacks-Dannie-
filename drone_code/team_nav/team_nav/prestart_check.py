"""Before the start: is camera tracking healthy enough to hand the drone over, and is the drone
where the camera tracking frame began?

How it fits: mission.py hands this every camera tracking report and every corner count from
corner_count_node.py while the drone waits on the launch mark, and asks readiness() each step. If
Offboard begins while it is not ready, mission.py refuses the start. Either way, mission.py keeps
publishing a hold target at the drone's position whenever reports are arriving: the organizers'
flight interface only hands control over while it hears a target at least once a second. This
file never talks to ROS and never reads a clock.

"Camera tracking" is cuVSLAM, the program that works out the drone's position from the two
front cameras. Its "frame" starts where the drone was when the drone's software (the "stack")
started: (0, 0, 0), with +x the way the nose pointed then.

Why the old check had to go (drone 16, 2026-11-07): a drone sitting still on the floor gives
bit-identical positions, which is normal, but our old rule called 11 identical positions in a row
"frozen". Each tiny change in the report reset the count, so the mission flapped between ready and
not ready once a second, and while "not ready" it sent no target, so the hand-over never happened.

Not ready only when (from the team, 2026-11-07):
  * the reports' own timestamps have stopped advancing for 0.5 s (the reports are stale copies), or
  * the position and orientation have been bit-identical for more than 3 s AND the left camera is
    blind: corner_count_node.py counts fewer than prestart_blind_below_corners FAST corners in it.
    ("FAST corners" are the small high-contrast spots camera tracking follows; a covered lens or a
    blank view has almost none.) A missing corner count cannot show blindness, so on its own it
    never blocks; the in-flight watchdog's rule (c) still catches tracking that is frozen while
    the drone is asked to move.
Once ready, it stays ready for at least 2 s, so one odd report cannot make it flap.

The origin check only warns: the organizers' code holds every target at least 0.3 m above the
frame's origin, so a drone carried to the net after the stack started climbs to 0.3 m above
wherever the stack started, which can be far above the floor. Only restarting the stack on the
launch mark fixes that.
"""
import math


class PrestartCheck:
    def __init__(self, settings):
        self.settings = settings
        self.last_stamp_seconds = None
        self.stamp_advanced_at_seconds = None    # when a report with a newer timestamp arrived
        self.last_pose = None
        self.pose_changed_at_seconds = None      # when the position or orientation last changed
        self.corner_count = None
        self.corner_count_seconds = None
        self.ready = False
        self.ready_since_seconds = None

    def observe_report(self, received_seconds, stamp_seconds, pose):
        """A camera tracking report: when it arrived, its own timestamp (None if it had none), and
        its position and orientation, compared exactly."""
        if stamp_seconds is None:
            stamp_seconds = received_seconds     # no timestamp: treat each report as new
        if self.last_stamp_seconds is None or stamp_seconds > self.last_stamp_seconds:
            self.last_stamp_seconds = stamp_seconds
            self.stamp_advanced_at_seconds = received_seconds
        pose = tuple(pose)
        if pose != self.last_pose:
            self.last_pose = pose
            self.pose_changed_at_seconds = received_seconds

    def observe_corner_count(self, received_seconds, corner_count):
        self.corner_count = int(corner_count)
        self.corner_count_seconds = received_seconds

    def readiness(self, now_seconds):
        """(ready, reason). Not ready keeps the drone from starting; it never stops the hold target."""
        problem = self.problem(now_seconds)
        if self.ready and now_seconds - self.ready_since_seconds < self.settings.prestart_ready_hold_seconds:
            return True, "ready"
        if problem is not None:
            self.ready = False
            return False, problem
        if not self.ready:
            self.ready = True
            self.ready_since_seconds = now_seconds
        return True, "ready"

    def problem(self, now_seconds):
        settings = self.settings
        if self.stamp_advanced_at_seconds is None:
            return "no camera tracking report has arrived"
        # The reasons hold no running numbers: mission.py logs each change of reason, and a number
        # that changes every step would write a line every step.
        if now_seconds - self.stamp_advanced_at_seconds > settings.prestart_stamp_stall_seconds:
            return (f"camera tracking timestamps have stopped advancing (for over "
                    f"{settings.prestart_stamp_stall_seconds:g} s)")
        identical_for = now_seconds - self.pose_changed_at_seconds
        if identical_for > settings.prestart_identical_pose_seconds and self.camera_blind(now_seconds):
            return (f"camera tracking has not changed for over {settings.prestart_identical_pose_seconds:g} s and the "
                    f"left camera is blind (fewer than {settings.prestart_blind_below_corners} corners)")
        return None

    def camera_blind(self, now_seconds):
        """True only when a fresh corner count says so."""
        if self.corner_count_seconds is None:
            return False
        if now_seconds - self.corner_count_seconds > self.settings.corner_count_max_age_seconds:
            return False
        return self.corner_count < self.settings.prestart_blind_below_corners

    def corner_count_is_fresh(self, now_seconds):
        return (self.corner_count_seconds is not None
                and now_seconds - self.corner_count_seconds <= self.settings.corner_count_max_age_seconds)


def origin_warning(position, settings):
    """None if the drone is near the camera tracking frame's origin; otherwise what is wrong and what
    to do, as one log line."""
    horizontal = math.hypot(position[0], position[1])
    vertical = abs(position[2])
    if horizontal <= settings.origin_check_horizontal_metres and vertical <= settings.origin_check_vertical_metres:
        return None
    return (f"THE DRONE WAS MOVED AFTER THE STACK STARTED: camera tracking puts it {horizontal:.2f} m sideways "
            f"and {position[2]:+.2f} m up from where the stack started (limits {settings.origin_check_horizontal_metres:g} m "
            f"and {settings.origin_check_vertical_metres:g} m). Targets are held at least 0.3 m above that start point, "
            "so restart the stack with the drone on the takeoff spot (drone_code/restart_drone_16.sh) before flying.")
