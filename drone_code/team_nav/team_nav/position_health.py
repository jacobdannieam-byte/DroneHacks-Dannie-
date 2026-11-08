"""Decides whether the drone still knows where it is sideways, for the fallback missions' abort.

REAL DRONE CODE, also used in the simulator and the tests. It is plain arithmetic, with no ROS or
PX4. fallback_run.py feeds it every control cycle (10 times a second) and asks it, when a run must
stop, whether the drone may still fly sideways to a landing spot (fallback_abort.py) or must
descend where it is.

Position counts as LOST when any of these happens:
  - camera tracking has been silent for more than CAMERA_TRACKING_LONGEST_SILENCE_SECONDS, or says
    it has lost track (run_safety.py uses the same 0.5 s limit);
  - camera tracking's position jumps: between two of its reports it moves further than the drone
    can fly in that time plus JUMP_ALLOWANCE_METRES;
  - PX4 resets its position estimate. PX4 counts each reset in its local position report
    (xy_reset_counter, z_reset_counter); a change after takeoff means PX4's idea of where the drone
    is has just been moved.
A jump or a reset is remembered for the rest of the run: after one, nothing proves that the
position is right again. A silence only counts while it lasts.

What it does not do: act on its answer, or check height (height_hold.py does that).
"""
import numpy as np

from team_nav.run_safety import CAMERA_TRACKING_LONGEST_SILENCE_SECONDS

# From the organizers' code: flight_interface caps speed targets at 1.5 m/s (max_speed_mps), so
# the drone cannot honestly move faster than this between two reports.
FASTEST_HONEST_SPEED_METRES_PER_SECOND = 1.5
# Our own choice, the same as height_hold.py's limit on a camera height step: on top of the
# distance the drone can fly, a position may move this much between reports before it is a jump.
JUMP_ALLOWANCE_METRES = 0.15


class PositionHealth:

    def __init__(self):
        self.last_position_metres = None
        self.last_arrival_seconds = None
        self.reset_counts_at_start = None
        self.lasting_problem = None        # A sentence once a jump or a reset has happened.
        self.current_problem = None

    def observe(self, arrival_seconds, position_metres, age_seconds, says_lost, px4_reset_counts):
        """One control cycle's readings. arrival_seconds is when camera tracking's latest report
        arrived; px4_reset_counts is (xy, z) from PX4's local position report, or None."""
        self._watch_for_a_jump(arrival_seconds, position_metres)
        self._watch_for_a_px4_reset(px4_reset_counts)
        self.current_problem = self._silence_or_lost(age_seconds, says_lost)

    def problem(self):
        """None while the position is healthy, else a sentence saying why it is lost."""
        return self.lasting_problem or self.current_problem

    def _watch_for_a_jump(self, arrival_seconds, position_metres):
        if position_metres is None or arrival_seconds is None:
            return
        position_metres = np.asarray(position_metres, dtype=float)
        is_a_new_report = self.last_arrival_seconds is None or arrival_seconds > self.last_arrival_seconds
        if not is_a_new_report:
            return
        if self.last_position_metres is not None:
            seconds_between = arrival_seconds - self.last_arrival_seconds
            step_metres = float(np.linalg.norm(position_metres - self.last_position_metres))
            allowed_metres = FASTEST_HONEST_SPEED_METRES_PER_SECOND * seconds_between + JUMP_ALLOWANCE_METRES
            if step_metres > allowed_metres and self.lasting_problem is None:
                self.lasting_problem = (f"Camera tracking's position jumped {step_metres:.2f} m in {seconds_between:.2f} s, "
                                        f"more than the drone can fly, so its position can't be trusted.")
        self.last_position_metres = position_metres
        self.last_arrival_seconds = arrival_seconds

    def _watch_for_a_px4_reset(self, px4_reset_counts):
        if px4_reset_counts is None or None in px4_reset_counts:
            return
        if self.reset_counts_at_start is None:
            self.reset_counts_at_start = tuple(px4_reset_counts)
            return
        if tuple(px4_reset_counts) != self.reset_counts_at_start and self.lasting_problem is None:
            self.lasting_problem = ("PX4 reset its position estimate during the flight (its reset counters changed), "
                                    "so its idea of where the drone is has just moved.")

    @staticmethod
    def _silence_or_lost(age_seconds, says_lost):
        if says_lost:
            return "Camera tracking reports that it has lost track."
        if age_seconds > CAMERA_TRACKING_LONGEST_SILENCE_SECONDS:
            return (f"Camera tracking has been silent for {age_seconds:.2f} s, more than "
                    f"{CAMERA_TRACKING_LONGEST_SILENCE_SECONDS:.1f} s.")
        return None
