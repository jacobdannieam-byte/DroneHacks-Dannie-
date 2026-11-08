"""The tunnel_first mission's 180 degree turn at S, at the tunnel height, one 0.05 s step at a time.

REAL DRONE CODE, also used in the simulators. tunnel_first_run.py runs it after the height gate;
heading_turn.py holds the arithmetic (the ramp, the pauses, the "settled" test).

What a step does: runs the safety checks; stops the run on a PX4 heading, position or attitude reset,
or if PX4 stops using camera tracking's heading for more than LONGEST_HEADING_FUSION_LOSS_SECONDS;
moves the commanded heading on by at most 0.5 degrees (or pauses); holds the drone above S at the
tunnel height; and once the command has reached the end, waits until the drone has settled.

What it does not do: decide when to turn, or what happens after.
"""
import math

import numpy as np

from team_nav.flight_steps import RunMustStop
from team_nav.heading_turn import (LONGEST_TURN_SECONDS, STEP_SECONDS, SettledTimer, TurnRamp, TurnTracker,
                                   turn_is_settled, wrapped_degrees)
from team_nav.tunnel_first_readings import (heading_split_degrees, px4_heading_degrees, px4_reset_counters,
                                            px4_speed_metres_per_second, px4_uses_camera_heading,
                                            yaw_rate_degrees_per_second)

# The team's choice (2026-11-08): PX4 not using camera tracking's heading for longer than this stops the turn.
LONGEST_HEADING_FUSION_LOSS_SECONDS = 1.0


class TurnInProgress:

    def __init__(self, run, hold_point_metres):
        self.run, self.link, self.watch = run, run.link, run.watch
        start_heading = px4_heading_degrees(self.link)
        split = heading_split_degrees(self.link)
        if start_heading is None or split is None:
            raise RunMustStop("PX4's or camera tracking's heading is not being reported, so the turn can't be watched. Landing.")
        self.ramp = TurnRamp(start_heading)
        self.px4_turn = TurnTracker(start_heading)
        self.split_before_degrees = split
        self.counters_before = px4_reset_counters(self.link)
        self.camera_heading_was_used = px4_uses_camera_heading(self.link)
        if not self.camera_heading_was_used:
            self.link.log.event("warning", f"PX4 is not using camera tracking's heading before the turn (flag "
                                           f"{self.camera_heading_was_used}), so its loss can't be watched during the turn.")
        self.camera_heading_lost_since = None
        self.settled = SettledTimer()
        self.finished = False
        self.last_pause = None
        self.start_seconds = self.link.now_seconds()
        self.hold_point = np.asarray(hold_point_metres, dtype=float)
        self.link.log.event("turn", f"Turn begins at {self.hold_point[2]:.2f} m: PX4 heading {start_heading:.1f} degrees, heading "
                                    f"split {split:+.2f} degrees, reset counters {self.counters_before}.")

    def seconds(self):
        return self.link.now_seconds() - self.start_seconds

    def is_done(self):
        return self.finished

    def one_step(self):
        if self.seconds() > LONGEST_TURN_SECONDS:
            raise RunMustStop(f"The turn did not finish within {LONGEST_TURN_SECONDS:.0f} s. Landing.")
        must_hold = self.watch.check()
        self.stop_if_px4_reset_or_lost_the_camera_heading()
        heading = px4_heading_degrees(self.link)
        split = heading_split_degrees(self.link)
        if heading is None or split is None:
            raise RunMustStop("PX4's or camera tracking's heading stopped being reported during the turn. Landing.")
        turned = self.px4_turn.update(heading)
        split_change = wrapped_degrees(split - self.split_before_degrees)
        if must_hold:
            self.link.targets.resend_last()
        else:
            command = self.next_command(turned, split_change)
            target = self.run.height_hold_correction_target(self.hold_point)
            heading_on_course = self.run.plan.start_frame.course_heading_from_start_frame(math.radians(command))
            self.watch.send_target_on_course(target, heading_on_course_radians=heading_on_course)
        if self.ramp.command_has_reached_the_end():
            yaw_error = self.ramp.final_heading_degrees() - (self.ramp.start_heading_degrees + turned)
            settled = turn_is_settled(px4_speed_metres_per_second(self.link), yaw_rate_degrees_per_second(self.link),
                                      yaw_error, split_change)
            self.finished = self.settled.update(self.link.now_seconds(), settled)
        self.link.record_drone_condition()
        self.link.wait_one_cycle(STEP_SECONDS)

    def next_command(self, turned_degrees, split_change_degrees):
        command = self.ramp.next_command_degrees(turned_degrees, split_change_degrees)
        if self.ramp.paused_because != self.last_pause:
            what = f"paused: {self.ramp.paused_because}" if self.ramp.paused_because else "resumed"
            self.link.log.event("turn", f"Turn ramp {what} (commanded {self.ramp.commanded_turn_degrees:.1f} of 180 degrees, "
                                        f"PX4 has turned {turned_degrees:.1f}).")
            self.last_pause = self.ramp.paused_because
        return command

    def stop_if_px4_reset_or_lost_the_camera_heading(self):
        counters = px4_reset_counters(self.link)
        names = ["heading", "sideways position", "attitude"]
        for name, before, now in zip(names, self.counters_before, counters):
            if before is not None and now is not None and now != before:
                raise RunMustStop(f"PX4 reset its {name} estimate during the turn (its reset counter went from {before} to "
                                  f"{now}), so its heading or position just jumped. Landing.")
        if not self.camera_heading_was_used:
            return
        if px4_uses_camera_heading(self.link):
            self.camera_heading_lost_since = None
            return
        now_seconds = self.link.now_seconds()
        if self.camera_heading_lost_since is None:
            self.camera_heading_lost_since = now_seconds
        if now_seconds - self.camera_heading_lost_since > LONGEST_HEADING_FUSION_LOSS_SECONDS:
            raise RunMustStop(f"PX4 has not used camera tracking's heading for more than "
                              f"{LONGEST_HEADING_FUSION_LOSS_SECONDS:.0f} s during the turn, so its heading can drift. Landing.")
