"""The tunnel_first mission's safety watch: fallback_run's (course_run's checks plus position health),
with the heading guard, the height watch, and a height record every cycle.

REAL DRONE CODE, also used in the simulators. tunnel_first_run.py makes it; every flight step calls
its check() once per cycle.

  - Heading guard (heading_turn.py), off during the turn: holds, then stops the run, if PX4's heading
    and camera tracking's drift apart.
  - Height watch (tunnel_height.py), whenever camera tracking puts the drone above 1.5 m: warns, then
    stops the run with HeightWatchStop. tunnel_first_run.py then comes down by the watch's rules.
  - The height hold's deliberate correction counts in run_safety's camera height comparison, as in
    Kenny's live navigation (Gazebo, 2026-11-08: PX4 drifting 0.33 m high in the tunnel had stopped
    runs although the height hold kept the true height).
  - Every cycle: one row of height_watch.csv (camera height, PX4 height, the disagreement, PX4's
    laser and height flags and reset counters), and the numbers for the after-run height report.

What it does not do: move the drone.
"""
from team_nav.fallback_run import FallbackSafetyWatch
from team_nav.flight_steps import RunMustStop
from team_nav.heading_turn import GUARD_HOLD, GUARD_LAND, GUARD_OK, HeadingGuard
from team_nav.tunnel_first_readings import (camera_height_metres, heading_split_degrees, px4_estimator_flag,
                                            px4_height_metres, px4_position_field)
from team_nav.tunnel_height import WATCH_STOP, WATCH_WARN, HeightRecord, HeightWatch, disagreement_metres

# PX4's estimator flags and local position fields written in height_watch.csv (the team's list).
LOGGED_FLAGS = ["cs_rng_hgt", "cs_rng_kin_consistent", "cs_rng_fault", "cs_rng_stuck", "reject_hagl", "cs_baro_hgt",
                "cs_ev_hgt", "cs_ev_yaw"]
LOGGED_POSITION_FIELDS = ["z_valid", "v_z_valid", "z_reset_counter", "delta_z", "dist_bottom_reset_counter",
                          "delta_dist_bottom", "epv", "dist_bottom"]
COUNTERS_IN_THE_REPORT = ["z_reset_counter", "dist_bottom_reset_counter", "xy_reset_counter", "heading_reset_counter"]


class HeightWatchStop(RunMustStop):
    """The height watch stopped the run; the drone comes down by the watch's rules."""


class TunnelFirstSafetyWatch(FallbackSafetyWatch):

    def __init__(self, link, run_safety, start_frame):
        super().__init__(link, run_safety, start_frame)
        self.height_hold = None          # Set by TunnelFirstRun once the flight begins.
        self.heading_guard = None
        self.heading_guard_is_on = False
        self.last_guard_verdict = GUARD_OK
        self.height_baseline_metres = None   # b0, set once the drone has settled after takeoff.
        self.height_watch = None
        self.height_record = HeightRecord()
        self.last_watch_verdict = None
        self.flight_start_seconds = None

    # ---- Starting the guards ------------------------------------------------------------------------

    def start_the_heading_guard(self, launch_split_degrees):
        if launch_split_degrees is None:
            self.link.log.event("warning", "PX4's or camera tracking's heading isn't reported at the start, so the heading "
                                           "guard is off for this run.")
            return
        self.heading_guard = HeadingGuard(launch_split_degrees)
        self.heading_guard_is_on = True
        self.link.log.event("heading_guard", f"Heading split at the start: {launch_split_degrees:+.2f} degrees (PX4's heading "
                                             f"minus camera tracking's). The guard watches for a change of more than 10.")

    def start_the_height_watch(self, baseline_metres):
        self.height_baseline_metres = baseline_metres
        self.height_watch = HeightWatch(px4_position_field(self.link, "z_reset_counter"))
        self.link.log.event("height", f"Height baseline b0 = {baseline_metres:+.3f} m (PX4's height minus camera tracking's, "
                                      f"steady above the launch mark). The height watch is on above 1.5 m.",
                            height_baseline_metres=round(baseline_metres, 4))

    def disagreement(self):
        return disagreement_metres(px4_height_metres(self.link), camera_height_metres(self.link), self.height_baseline_metres)

    # ---- Every cycle ----------------------------------------------------------------------------------

    def _gather_condition(self):
        condition = super()._gather_condition()
        if self.height_hold is not None:
            condition.height_correction_metres += self.height_hold.correction_metres
        return condition

    def check(self):
        must_hold = super().check()
        self.record_heights()
        self.check_the_height_watch()
        return self.check_the_heading_guard() or must_hold

    def check_the_heading_guard(self):
        if self.heading_guard is None or not self.heading_guard_is_on:
            return False
        split = heading_split_degrees(self.link)
        verdict = self.heading_guard.check(self.link.now_seconds(), split)
        if verdict != self.last_guard_verdict:
            change = None if split is None else round(self.heading_guard.change_degrees(split), 1)
            self.link.log.event("heading_guard", f"Heading guard: {verdict} (the split has changed by {change} degrees).")
            self.last_guard_verdict = verdict
        if verdict == GUARD_LAND:
            raise RunMustStop("PX4's heading and camera tracking's have stayed more than 10 degrees apart (compared with the "
                              "start) for over 3.5 s, so one of them is wrong. Landing.")
        return verdict == GUARD_HOLD

    def check_the_height_watch(self):
        if self.height_watch is None:
            return
        verdict = self.height_watch.update(self.link.now_seconds(), camera_height_metres(self.link), self.disagreement(),
                                           px4_position_field(self.link, "z_reset_counter"),
                                           px4_position_field(self.link, "z_valid"), px4_position_field(self.link, "v_z_valid"))
        if verdict != self.last_watch_verdict and verdict == WATCH_WARN:
            self.link.log.event("height_watch", f"HEIGHT WARNING: {self.height_watch.reason}.")
        self.last_watch_verdict = verdict
        if verdict == WATCH_STOP:
            raise HeightWatchStop(f"The height watch stopped the run: {self.height_watch.reason}.")

    def record_heights(self):
        link = self.link
        values = {name: px4_estimator_flag(link, name) for name in LOGGED_FLAGS}
        values.update({name: px4_position_field(link, name) for name in LOGGED_POSITION_FIELDS})
        camera, px4, disagreement = camera_height_metres(link), px4_height_metres(link), self.disagreement()
        link.log.sample("height_watch", stage=link.stage, camera_height_metres=camera, px4_height_metres=px4,
                        height_baseline_metres=self.height_baseline_metres, disagreement_metres=disagreement, **values)
        if self.flight_start_seconds is None:
            return
        counters = {name: px4_position_field(link, name) for name in COUNTERS_IN_THE_REPORT}
        self.height_record.update(link.now_seconds() - self.flight_start_seconds, link.stage, camera if camera is not None else 0.0,
                                  values["cs_rng_hgt"], disagreement, counters)
