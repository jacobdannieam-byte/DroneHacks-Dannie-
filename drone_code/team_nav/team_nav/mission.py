"""The whole flight, from waiting on the launch mark until the motors switch off.

How it fits: mission_node.py passes in every report it hears (camera tracking position, the
organizers' flight state, PX4's position report with the bottom laser distance, PX4's status) and
calls step twenty times a second. step returns what to publish this time: a position target, a
velocity command, or nothing. Between takeoff and landing it flies either the course stages
(course_stages.py, mission:=stages) or a list of waypoints (route_flight.py, mission:=route). The
landing is landing.py; position_watchdog.py decides when the position can no longer be trusted;
position_watch.py spots frozen tracking before a start and measures speed; drift_correction.py
decides, at route corners, how far camera tracking has drifted. This file never talks to ROS and
never reads a clock, so the tests can run it on a laptop.

Words used here:
* "Offboard" is the PX4 mode in which the drone follows targets from a computer. The organizers'
  code asks PX4 for it once per arming, when the pilot flips the switch to Position after arming
  on the ground (docs/ARCHITECTURE.md, section 8). It reports the result as offboard_active.
* "Arming" gives the motors permission to spin. PX4 disarms by itself after it decides it landed.
* Positions are in the camera tracking frame (ENU: x East, y North, z Up, metres). Heading is
  anticlockwise from East, in radians. Velocity commands use the same world directions, whatever
  way the nose points.

Phases:
  waiting_for_start  On the launch mark. While camera tracking reports arrive, we send a hold target
                     at the drone's position 10 times a second, because the organizers' code only
                     hands control over while it hears a target at least once a second; it has no
                     effect before that. prestart_check.py says whether a start would be accepted, and
                     the position is logged every 5 s, with a warning if the drone is far from where
                     the stack started. We also record the laser's floor reading (section 7, step 1).
  taking_off         From the first step with Offboard active: climb to cruise height.
  flying_course      course_stages.py decides (mission:=stages).
  flying_route       route_flight.py decides (mission:=route).
  landing            landing.py decides.
  start_refused      Offboard began while prestart_check.py said not ready. A gentle descent is sent,
                     so the drone comes back down until PX4 switches the motors off.
  finished           PX4 switched the motors off after landing.
  stopped            Control is not ours any more. Nothing is published until the drone is disarmed
                     on the ground, after which a new run can start (all run state is cleared then).

Every start re-anchors: home, the ground height, the heading and the laser's floor reading are
recorded afresh from where the drone sits at that moment, so each attempt in a block starts from its
own launch position without restarting the drone's software.

A trap in the organizers' code (from the organizer code audit): when a velocity command expires, or
when /skybeta/hold is called, the drone flies back to the LAST POSITION TARGET it was sent, wherever
that was. So before the first velocity command of a landing we send one position target at the
drone's current position, and then keep the velocity commands coming every step until the motors
are off.

Safety rules (each one is a test in test/test_mission.py):
* Control is taken only when Offboard becomes active after we saw it inactive, with PX4 reporting
  the drone on the ground.
* If Offboard stops being active, nothing more is published: we never fight the pilot or PX4.
* The climb starts on the first step. If the drone has not left the ground 10 s after the start,
  we stop, because PX4 switches the motors off then anyway.
* The position watchdog (position_watchdog.py) runs from takeoff until the motors stop. When one
  of its rules trips - (a) camera tracking reports stopped for 0.5 s, (b) a jump between two
  reports, (c) tracking frozen while a target asks for motion, (e) PX4 reset its position or
  heading estimate - the drone lands at once, and the log names the rule. During a landing a trip
  is only logged, because the landing is already the answer.
* Watchdog rule (f): once PX4's own status has shown Offboard, a status that is no longer Offboard
  means stop sending targets, and the log says so.
* Nothing advances unless camera tracking reports and PX4's position reports are each freshly
  received, and the flight state is fresh and says the link to PX4 is up. (The flight state repeats
  old values, so it cannot vouch for the other two.) Otherwise hold, and land if that lasts 5 s.
* Before the start, a drone standing still may report bit-identical positions; that is normal.
  prestart_check.py says not ready only if the reports' timestamps stop advancing, or the position
  has not changed for over 3 s while the left camera is blind. In flight, more than 10 identical
  positions in a row means tracking has frozen: while hovering that is a hold, then a landing
  after 5 s.
* Every position target stays within 0.5 m of the drone, sideways.
* 170 s after the start the landing begins. Nothing stops a landing except the motors switching
  off or Offboard ending.
* Every decision has a reason. Each change of phase or reason is logged, and so is each new peak
  horizontal speed.

The drift correction (route mission, drift_correction:=true): each time the route has stopped at a
corner, before it turns, the depth straight ahead is compared with the course map's wall straight
ahead, and the result (a correction, or why there is none) is logged. The total correction is
added to camera tracking's position before the route sees it, and taken off every target the route
asks for, so the drone flies where the route really is. Camera tracking, PX4's estimate, the
watchdog and the landing never see the correction. It is cleared at every start.
"""
import math
from dataclasses import dataclass, field

from .course_stages import Course, world_to_body, wrap_angle
from .drift_correction import decide_correction
from .laser_rule import legs_that_upset_the_laser, waypoints_to_fly_through
from .landing import Landing
from .mission_inputs import MissionInputs, is_finite_number
from .mission_settings import MissionSettings
from .position_watch import PositionWatch
from .position_watchdog import PositionWatchdog
from .prestart_check import PrestartCheck, origin_warning
from .route_flight import RouteFlight

# Our own choice: after a pause in the steps (a slow computer), move the carrot as if at most this
# much time had passed, so it never jumps ahead.
LONGEST_STEP_SECONDS = 0.25
# Our own choice: log the peak horizontal speed each time it rises by this much.
PEAK_SPEED_LOG_STEP_METRES_PER_SECOND = 0.1
FLYING_PHASES = ("taking_off", "flying_course", "flying_route", "landing")
MISSION_KINDS = ("stages", "route")
# From PX4's VehicleStatus message: the number of the Offboard mode in nav_state.
PX4_NAV_STATE_OFFBOARD = 14


@dataclass
class Command:
    """What mission_node.py publishes this step. kind is "position", "velocity" or "nothing".
    position_before_velocity, when set, is a position target (x, y, z, heading) to publish just
    before the velocity command, so the organizers' code falls back to it and nowhere else."""
    kind: str
    position: tuple | None = None             # (x, y, z) metres
    heading_radians: float | None = None
    velocity: tuple | None = None             # (east, north, up) metres per second, world directions
    position_before_velocity: tuple | None = None
    reason: str = ""
    log_lines: list = field(default_factory=list)


class Mission:
    def __init__(self, settings=None, vision_check=True, mission_kind="stages", route_waypoints=None,
                 map_ranging=None):
        """vision_check=False is for the simulator only (see fly_course); mission_node.py refuses it
        anywhere else. mission_kind is "stages" or "route"; a route needs its waypoints, each
        (forward, left) or (forward, left, height) in metres from the takeoff spot. map_ranging
        (drift_correction.MapRanging) holds the course map and where the route starts on it: the
        drift correction measures walls with it, and the route's bottom laser rule (laser_rule.py)
        finds the windows and boulders in it."""
        if mission_kind not in MISSION_KINDS:
            raise ValueError(f"The mission must be one of {', '.join(MISSION_KINDS)}, not {mission_kind!r}. "
                             "Fix the mission launch argument.")
        if mission_kind == "route" and not route_waypoints:
            raise ValueError("The route mission needs waypoints, and none were given. Check the route file.")
        self.settings = settings or MissionSettings()
        if self.settings.drift_correction and map_ranging is None:
            raise ValueError("The drift correction needs a course map, and none was given. Check the course_map_file "
                             "parameter, or launch with drift_correction:=false.")
        self.map_ranging = map_ranging
        self.route_must_fly_through, self.route_laser_notes = self.work_out_the_laser_rule(mission_kind, route_waypoints)
        self.vision_check = vision_check
        self.mission_kind = mission_kind
        self.route_waypoints = route_waypoints
        self.inputs = MissionInputs()            # detector reports; all missing until detectors exist
        self.position_watch = PositionWatch(self.settings.frozen_tracking_identical_reports)
        # The watchdog follows the report streams across runs; start_watching() restarts its rules.
        self.watchdog = PositionWatchdog(self.settings)
        # Like the watchdog, the pre-start check follows the report streams across runs.
        self.prestart = PrestartCheck(self.settings)
        self.last_prestart_target_seconds = None
        self.last_position_log_seconds = None
        self.nav_state = None                    # PX4's own mode, from its status report
        self.phase = "waiting_for_start"
        self.reason = ""
        self.history = []                        # [(seconds, phase, reason)] at every change
        self.pending_log_lines = []
        self.position = None                     # latest camera tracking report, and when it arrived
        self.heading_radians = None
        self.position_seconds = None
        self.offboard_active = None              # latest flight state report, and when it arrived
        self.armed = None
        self.landed = None
        self.px4_link = None
        self.flight_state_seconds = None
        self.px4_report_seconds = None           # when PX4's position report last arrived
        self.laser_metres = None                 # latest usable bottom laser distance in it
        self.laser_seconds = None
        self.depth_ahead_metres = None           # latest depth straight ahead (None: no good reading)
        self.depth_ahead_seconds = None          # when it arrived
        self.floor_laser_readings = []           # [(seconds, distance)] while on the launch mark
        self.seen_offboard_inactive = False
        self.last_step_seconds = None
        self.reset_run_state()

    def reset_run_state(self):
        """Forget everything worked out during a previous run: the course's memory (where the pad
        is, what space was seen free, how many sightings were counted), the route's progress, the
        drift correction, the landing, the watchdog's trip, the timers, the position watch and the
        detector reports."""
        self.course = Course(self.settings)
        self.route = None
        if self.mission_kind == "route":
            self.route = RouteFlight(self.settings, self.route_waypoints, self.route_must_fly_through)
        self.landing = None
        self.position_watch.reset()
        self.inputs.clear()
        self.watchdog_trip_logged = False
        self.offboard_seen_in_px4_status = False
        self.last_command = None
        self.flight_time_limit_reached = False
        self.left_the_ground = False
        self.waiting_for_reports_since_seconds = None
        self.current_setpoint = None             # (x, y, z, heading) last position target sent
        self.last_published_kind = None
        self.peak_speed_logged_metres_per_second = 0.0
        self.start_seconds = None
        self.ground_height_metres = None
        self.home = None                         # (x, y) of the launch mark
        self.start_heading_radians = None
        self.floor_reading_metres = None
        self.position_correction_metres = (0.0, 0.0)   # (x, y) added to camera tracking's position

    def work_out_the_laser_rule(self, mission_kind, route_waypoints):
        """({waypoint index: why}, [log lines]): the waypoints the route must fly through (laser_rule.py)."""
        if mission_kind != "route":
            return {}, []
        if self.map_ranging is None:
            return {}, ["laser rule: off, because no course map was given"]
        places = [tuple(waypoint[:2]) for waypoint in route_waypoints]
        heights = [waypoint[2] if len(waypoint) == 3 else None for waypoint in route_waypoints]
        events = legs_that_upset_the_laser(self.map_ranging.course_map, self.map_ranging.start_frame, places, heights,
                                           self.settings.cruise_height_metres)
        must, last_note = waypoints_to_fly_through(places, events, self.settings.route_speed_metres_per_second,
                                                   self.settings.route_laser_rule_seconds)
        notes = [f"laser rule: {last_note}"] if last_note else []
        return must, notes

    def log_the_laser_rule(self, now_seconds):
        """At the route's start: each stop the laser rule turns into a fly-through, and why."""
        for index, why in sorted(self.route_must_fly_through.items()):
            if self.route.flies_through_by_the_turn(index):
                continue                             # flown through anyway: the rule changes nothing
            text = f"laser rule: waypoint {index + 1} is flown through at {self.settings.route_speed_metres_per_second:g} m/s, not stopped at ({why})"
            if self.route.leg_heights_metres[index + 1] != self.route.leg_heights_metres[index]:
                text += "; the height changes while flying the next leg"
            self.log(now_seconds, text)
        for note in self.route_laser_notes:
            self.log(now_seconds, note)

    def odometry_text(self):
        x, y, z = self.position
        return f"odometry ({x:.2f}, {y:.2f}, {z:.2f}) m, heading {math.degrees(self.heading_radians):.0f} deg"

    # ------------------------------------------------------------------ reports from mission_node.py
    def observe_position(self, now_seconds, position, heading_radians, stamp_seconds=None, orientation=None):
        """A camera tracking report: when it arrived, and its own timestamp and orientation when known."""
        self.position = (float(position[0]), float(position[1]), float(position[2]))
        self.heading_radians = float(heading_radians)
        self.position_seconds = now_seconds
        self.position_watch.observe(now_seconds, self.position)
        if orientation is None:
            orientation = (self.heading_radians,)
        self.watchdog.observe_report(now_seconds, stamp_seconds, self.position, orientation)
        self.prestart.observe_report(now_seconds, stamp_seconds, self.position + tuple(orientation))

    def observe_corner_count(self, now_seconds, corner_count):
        """How many FAST corners corner_count_node.py found in the left camera's picture."""
        self.prestart.observe_corner_count(now_seconds, corner_count)

    def observe_flight_state(self, now_seconds, offboard_active, armed, landed, px4_link=None):
        """Each value is True, False, or None when the report did not say."""
        self.offboard_active = offboard_active if isinstance(offboard_active, bool) else None
        self.armed = armed if isinstance(armed, bool) else None
        self.landed = landed if isinstance(landed, bool) else None
        self.px4_link = px4_link if isinstance(px4_link, bool) else None
        self.flight_state_seconds = now_seconds

    def observe_px4_status(self, now_seconds, nav_state):
        """PX4's own status report: we use only nav_state, its current mode."""
        self.nav_state = int(nav_state) if nav_state is not None else None

    def observe_px4_position_report(self, now_seconds, laser_distance_metres, xy_reset_counter=None,
                                    heading_reset_counter=None):
        """PX4's position report arrived. We use its bottom laser distance (dist_bottom) and the
        counters it raises each time it resets its position or heading estimate."""
        self.px4_report_seconds = now_seconds
        if xy_reset_counter is not None and heading_reset_counter is not None:
            self.watchdog.observe_reset_counters(int(xy_reset_counter), int(heading_reset_counter))
        if not is_finite_number(laser_distance_metres):
            return
        self.laser_metres = float(laser_distance_metres)
        self.laser_seconds = now_seconds
        if self.phase == "waiting_for_start" and self.landed is True:
            self.floor_laser_readings.append((now_seconds, self.laser_metres))
            oldest_kept = now_seconds - self.settings.floor_reading_window_seconds
            self.floor_laser_readings = [entry for entry in self.floor_laser_readings if entry[0] >= oldest_kept]

    def observe_depth_ahead(self, now_seconds, distance_metres):
        """The median depth straight ahead (depth_ahead.py), or None when there was no good reading."""
        self.depth_ahead_metres = distance_metres if is_finite_number(distance_metres) else None
        self.depth_ahead_seconds = now_seconds

    # ------------------------------------------------------------------ one step
    def step(self, now_seconds):
        if self.last_step_seconds is None:
            seconds_since_last_step = 0.0
        else:
            seconds_since_last_step = min(max(now_seconds - self.last_step_seconds, 0.0), LONGEST_STEP_SECONDS)
        self.last_step_seconds = now_seconds
        command = self.decide(now_seconds, seconds_since_last_step)
        self.note(now_seconds, command.reason)
        self.log_new_peak_speed()
        if command.kind != "nothing":
            self.last_published_kind = command.kind
            self.last_command = command
        command.log_lines = self.pending_log_lines
        self.pending_log_lines = []
        return command

    def decide(self, now_seconds, seconds_since_last_step):
        if self.phase in ("finished", "stopped"):
            if not self.disarmed_on_the_ground(now_seconds):
                return Command("nothing", reason=self.reason)
            self.phase = "waiting_for_start"
            self.seen_offboard_inactive = False
            self.floor_laser_readings = []
            self.reset_run_state()
            self.log(now_seconds, "disarmed on the ground: ready for another run")
        if self.phase == "waiting_for_start":
            return self.wait_for_start(now_seconds)
        ended = self.check_control_is_still_ours(now_seconds)
        if ended is not None:
            return ended
        if self.phase == "start_refused":
            return self.keep_on_the_ground()
        self.check_the_watchdog(now_seconds)
        self.check_flight_time(now_seconds)
        if self.phase == "taking_off":
            return self.take_off(now_seconds)
        if self.phase == "flying_course":
            return self.fly_course(now_seconds, seconds_since_last_step)
        if self.phase == "flying_route":
            return self.fly_route(now_seconds, seconds_since_last_step)
        return self.land(now_seconds)

    def note(self, now_seconds, reason):
        if reason == self.reason and self.history and self.history[-1][1] == self.phase:
            return
        self.reason = reason
        self.history.append((now_seconds, self.phase, reason))
        self.pending_log_lines.append(f"{self.phase}: {reason}")

    def log(self, now_seconds, text):
        """A log line that is not this step's reason."""
        self.history.append((now_seconds, self.phase, text))
        self.pending_log_lines.append(f"{self.phase}: {text}")

    def log_new_peak_speed(self):
        peak = self.position_watch.peak_horizontal_speed_metres_per_second
        if self.phase in FLYING_PHASES and peak >= self.peak_speed_logged_metres_per_second + PEAK_SPEED_LOG_STEP_METRES_PER_SECOND:
            self.peak_speed_logged_metres_per_second = peak
            self.pending_log_lines.append(f"{self.phase}: peak horizontal speed so far {peak:.2f} m/s")

    def end_flight(self, now_seconds, phase, reason):
        flew = self.start_seconds is not None
        self.phase = phase
        self.watchdog.stop_watching()
        if flew:
            peak = self.position_watch.peak_horizontal_speed_metres_per_second
            reason = f"{reason} (peak horizontal speed {peak:.2f} m/s)"
        return Command("nothing", reason=reason)

    def stop(self, now_seconds, reason):
        return self.end_flight(now_seconds, "stopped", reason)

    # ------------------------------------------------------------------ freshness
    def received_within(self, received_seconds, now_seconds, max_age_seconds):
        return received_seconds is not None and now_seconds - received_seconds <= max_age_seconds

    def position_is_fresh(self, now_seconds):
        return self.received_within(self.position_seconds, now_seconds, self.settings.position_report_max_age_seconds)

    def position_is_usable(self, now_seconds):
        """Freshly received, and not frozen."""
        return self.position_is_fresh(now_seconds) and not self.position_watch.tracking_frozen()

    def laser_is_fresh(self, now_seconds):
        return self.received_within(self.laser_seconds, now_seconds, self.settings.laser_report_max_age_seconds)

    def flight_state_is_fresh(self, now_seconds):
        return self.received_within(self.flight_state_seconds, now_seconds, self.settings.flight_state_max_age_seconds)

    def reports_stopped(self, now_seconds):
        """Why the drone may not advance, or None. Each source is judged by when IT last arrived."""
        settings = self.settings
        if not self.position_is_fresh(now_seconds):
            return "camera tracking position reports stopped"
        if self.position_watch.tracking_frozen():
            return f"camera tracking frozen (the same position more than {settings.frozen_tracking_identical_reports} times in a row)"
        if not self.received_within(self.px4_report_seconds, now_seconds, settings.px4_report_max_age_seconds):
            return "PX4 position reports stopped"
        if not self.flight_state_is_fresh(now_seconds):
            return "flight state reports stopped"
        if self.px4_link is not True:
            return "the flight state says the link to PX4 is down"
        return None

    def disarmed_on_the_ground(self, now_seconds):
        return self.flight_state_is_fresh(now_seconds) and self.armed is False and self.landed is True

    # ------------------------------------------------------------------ before the start
    def wait_for_start(self, now_seconds):
        if self.flight_state_is_fresh(now_seconds):
            if self.offboard_active is False:
                self.seen_offboard_inactive = True
            elif self.offboard_active is True and self.armed is not True:
                # After PX4 disarms, its mode can stay at Offboard until the pilot moves the mode
                # switch. That is not a start, and no target is sent to a drone in that state.
                return Command("nothing", reason="PX4 still reports Offboard while disarmed: waiting for it "
                                                 "to end before the next run")
            elif self.offboard_active is True:
                return self.try_to_start(now_seconds)
        if not self.received_within(self.position_seconds, now_seconds, self.settings.prestart_odometry_max_age_seconds):
            return Command("nothing", reason="waiting for position reports from camera tracking")
        self.log_position_while_waiting(now_seconds)
        ready, why = self.prestart.readiness(now_seconds)
        if ready:
            reason = "ready: waiting for Offboard (the pilot's switch to Position after arming)"
        else:
            reason = f"not ready: {why}; a start now would be refused (still sending the hold target)"
        if not self.time_for_a_prestart_target(now_seconds):
            return Command("nothing", reason=reason)
        x, y, z = self.position
        # A target at the drone, raised to the lowest height the organizers' code allows (it would
        # raise it anyway, and log a warning each time). It does nothing until Offboard starts, and
        # then it lifts the drone off at once.
        target = (x, y, z + self.settings.lowest_position_target_metres)
        return Command("position", position=target, heading_radians=self.heading_radians, reason=reason)

    def time_for_a_prestart_target(self, now_seconds):
        """The hold target goes out prestart_setpoints_per_second times a second, not at every step."""
        period = 1.0 / self.settings.prestart_setpoints_per_second
        last = self.last_prestart_target_seconds
        if last is not None and now_seconds - last < period - 1e-6:
            return False
        self.last_prestart_target_seconds = now_seconds
        return True

    def log_position_while_waiting(self, now_seconds):
        last = self.last_position_log_seconds
        if last is not None and now_seconds - last < self.settings.prestart_position_log_seconds:
            return
        self.last_position_log_seconds = now_seconds
        x, y, z = self.position
        if self.prestart.corner_count_is_fresh(now_seconds):
            corners = f"{self.prestart.corner_count} corners in the left camera"
        else:
            corners = "no corner count from corner_count_node"
        self.log(now_seconds, f"waiting: camera tracking at ({x:.2f}, {y:.2f}, {z:.2f}) m, heading "
                              f"{math.degrees(self.heading_radians):.0f} deg; {corners}")
        warning = origin_warning(self.position, self.settings)
        if warning is not None:
            self.log(now_seconds, warning)

    def try_to_start(self, now_seconds):
        if not self.seen_offboard_inactive:
            return self.stop(now_seconds, "Offboard was already active when this program first heard the flight "
                                          "state, so the drone may be flying: not taking control")
        if self.landed is not True:
            return self.stop(now_seconds, "Offboard became active but PX4 does not report the drone on the "
                                          "ground: not taking control")
        if self.position is None:
            return self.stop(now_seconds, "Offboard became active but no position report has ever arrived "
                                          "from camera tracking: cannot take off")
        ready, why = self.prestart.readiness(now_seconds)
        if not ready:
            return self.refuse_start(now_seconds, why)
        floor_reading = self.median_floor_reading()
        self.reset_run_state()
        self.watchdog.start_watching(now_seconds)
        self.start_seconds = now_seconds
        self.ground_height_metres = self.position[2]
        self.home = (self.position[0], self.position[1])
        self.start_heading_radians = self.heading_radians
        self.floor_reading_metres = floor_reading
        self.current_setpoint = (self.home[0], self.home[1],
                                 self.ground_height_metres + self.settings.lowest_position_target_metres,
                                 self.start_heading_radians)
        self.phase = "taking_off"
        floor_text = "none recorded" if floor_reading is None else f"{floor_reading:.2f} m"
        self.log(now_seconds, f"new run: home ({self.home[0]:.2f}, {self.home[1]:.2f}), ground height "
                              f"{self.ground_height_metres:.2f} m, heading {math.degrees(self.heading_radians):.0f} deg, "
                              f"laser floor reading {floor_text}")
        self.log(now_seconds, f"hand-over: {self.odometry_text()} (the start heading)")
        warning = origin_warning(self.position, self.settings)
        if warning is not None:
            self.log(now_seconds, warning)
        return self.take_off(now_seconds)

    def refuse_start(self, now_seconds, why):
        """Offboard has begun, but the pre-start check says camera tracking cannot be trusted: no
        mission. The organizers' code is already flying to our last target, just above the drone,
        so keep commanding a gentle descent until PX4 switches the motors off."""
        self.phase = "start_refused"
        self.log(now_seconds, f"refused to start: {why}")
        return self.keep_on_the_ground()

    def keep_on_the_ground(self):
        descent = self.settings.gentle_descent_speed_metres_per_second
        return Command("velocity", velocity=(0.0, 0.0, -descent),
                       reason="start refused: keeping the drone on the ground until PX4 switches the motors off")

    def median_floor_reading(self):
        readings = sorted(distance for _, distance in self.floor_laser_readings)
        if not readings:
            return None
        return readings[len(readings) // 2]

    # ------------------------------------------------------------------ rules checked every step
    def check_control_is_still_ours(self, now_seconds):
        if self.phase == "start_refused" and self.armed is False:
            return self.stop(now_seconds, "PX4 switched the motors off after the refused start")
        if self.armed is False:
            if self.phase == "landing":
                return self.end_flight(now_seconds, "finished", "landed: PX4 switched the motors off")
            return self.stop(now_seconds, f"the motors were switched off while {self.phase.replace('_', ' ')} "
                                          "(kill switch or PX4): not publishing any more")
        if self.nav_state == PX4_NAV_STATE_OFFBOARD:
            self.offboard_seen_in_px4_status = True
        elif self.offboard_seen_in_px4_status and self.nav_state is not None:
            return self.stop(now_seconds, f"watchdog rule (f): PX4 left Offboard (its status says nav_state "
                                          f"{self.nav_state}; Offboard is {PX4_NAV_STATE_OFFBOARD}): stopped sending setpoints")
        if self.offboard_active is False:
            return self.stop(now_seconds, "PX4 left Offboard, so the pilot or a PX4 safety action has the drone: "
                                          "not publishing any more")
        return None

    def check_the_watchdog(self, now_seconds):
        """Land at once when a watchdog rule trips; during a landing, only say so."""
        tripped = self.watchdog.check(now_seconds, self.target_asks_for_motion())
        if tripped is None or self.watchdog_trip_logged:
            return
        self.watchdog_trip_logged = True
        rule, why = tripped
        if self.phase == "landing":
            self.log(now_seconds, f"watchdog rule ({rule}) tripped during the landing: {why}; the landing goes on")
            return
        self.course.drop_world_goals()
        self.begin_landing(now_seconds, f"watchdog rule ({rule}) tripped: {why}; landing now")

    def target_asks_for_motion(self):
        """Whether the last target sent asks the drone to move: a velocity, a position some way off,
        or a heading some way round (watchdog rule c)."""
        settings, command = self.settings, self.last_command
        if command is None or self.position is None:
            return False
        if command.kind == "velocity":
            speed = math.sqrt(sum(value * value for value in command.velocity))
            return speed > settings.watchdog_motion_request_metres_per_second
        distance = math.dist(command.position, self.position)
        turn = abs(wrap_angle(command.heading_radians - self.heading_radians))
        return (distance > settings.watchdog_motion_request_metres
                or turn > math.radians(settings.watchdog_motion_request_degrees))

    def check_flight_time(self, now_seconds):
        """Only ever starts the landing; it never interrupts one."""
        if self.phase == "landing" or self.flight_time_limit_reached:
            return
        limit = self.settings.flight_time_limit_seconds
        if now_seconds - self.start_seconds >= limit:
            self.flight_time_limit_reached = True
            self.begin_landing(now_seconds, f"flight time limit of {limit:g} s reached, landing")

    def hold_while_reports_stopped(self, now_seconds):
        """None while every report is fresh; otherwise a hold, or the landing after 5 s."""
        why = self.reports_stopped(now_seconds)
        if why is None:
            self.waiting_for_reports_since_seconds = None
            return None
        if self.waiting_for_reports_since_seconds is None:
            self.waiting_for_reports_since_seconds = now_seconds
        longest = self.settings.longest_wait_for_input_seconds
        if now_seconds - self.waiting_for_reports_since_seconds >= longest:
            self.begin_landing(now_seconds, f"{why} for {longest:g} s, landing")
            return self.land(now_seconds)
        return self.position_command(now_seconds, self.current_setpoint, f"{why}, holding")

    # ------------------------------------------------------------------ the phases
    def take_off(self, now_seconds):
        settings = self.settings
        elapsed = now_seconds - self.start_seconds
        held = self.hold_while_reports_stopped(now_seconds)
        if held is not None:
            return held
        height = self.position[2] - self.ground_height_metres
        self.left_the_ground = self.left_the_ground or height >= settings.left_ground_height_metres
        if not self.left_the_ground and elapsed > settings.leave_ground_deadline_seconds:
            return self.stop(now_seconds, f"did not leave the ground within {settings.leave_ground_deadline_seconds:g} s; "
                                          "PX4 switches the motors off by itself (COM_DISARM_PRFLT): not publishing any more")
        if height >= settings.cruise_height_metres - settings.cruise_height_tolerance_metres:
            cruise = (self.home[0], self.home[1], self.ground_height_metres + settings.cruise_height_metres,
                      self.start_heading_radians)
            if self.route is not None:
                self.route.start(now_seconds, cruise, self.ground_height_metres)
                self.phase = "flying_route"
                self.log_the_laser_rule(now_seconds)
            else:
                self.course.start(now_seconds, cruise)
                self.phase = "flying_course"
            return self.position_command(now_seconds, cruise, f"at cruise height {settings.cruise_height_metres:g} m")
        if elapsed > settings.takeoff_time_limit_seconds:
            self.begin_landing(now_seconds, f"did not reach cruise height within {settings.takeoff_time_limit_seconds:g} s, landing")
            return self.land(now_seconds)
        climbed = settings.lowest_position_target_metres + settings.climb_speed_metres_per_second * elapsed
        target = (self.home[0], self.home[1], self.ground_height_metres + min(settings.cruise_height_metres, climbed),
                  self.start_heading_radians)
        return self.position_command(now_seconds, target, f"climbing to {settings.cruise_height_metres:g} m")

    def fly_course(self, now_seconds, seconds_since_last_step):
        held = self.hold_while_reports_stopped(now_seconds)
        if held is not None:
            return held
        if not self.vision_check:
            # SIMULATOR ONLY: the simulator has no camera images, so nothing reports camera
            # tracking health, and every stage would hold for it. Treat it as healthy.
            self.inputs.set_vision(now_seconds, True)
        if self.laser_is_fresh(now_seconds):
            floor = self.floor_reading_metres
            if floor is None:
                floor = self.settings.laser_floor_reading_if_unrecorded_metres
            self.inputs.set_height(self.laser_seconds, self.laser_metres - floor)
        decision = self.course.step(now_seconds, seconds_since_last_step, self.position, self.heading_radians,
                                    self.inputs)
        if decision.land:
            self.current_setpoint = decision.setpoint
            self.begin_landing(now_seconds, f"{decision.stage}: {decision.reason}")
            return self.land(now_seconds)
        return self.position_command(now_seconds, decision.setpoint, f"{decision.stage}: {decision.reason}")

    def fly_route(self, now_seconds, seconds_since_last_step):
        held = self.hold_while_reports_stopped(now_seconds)
        if held is not None:
            return held
        decision = self.route.step(now_seconds, seconds_since_last_step, self.corrected_position(),
                                   self.heading_radians, self.position_watch.latest_horizontal_speed_metres_per_second)
        if decision.reason.startswith("reached waypoint"):
            self.log(now_seconds, f"at waypoint {self.route.reached} of {len(self.route.waypoints)}: {self.odometry_text()}")
        if decision.stopped_at_corner and self.settings.drift_correction:
            self.correct_drift(now_seconds)
        setpoint = self.uncorrected_setpoint(decision.setpoint)
        if decision.complete:
            self.current_setpoint = setpoint
            self.begin_landing(now_seconds, f"route: {decision.reason}")
            return self.land(now_seconds)
        return self.position_command(now_seconds, setpoint, f"route: {decision.reason}")

    # ------------------------------------------------------------------ the drift correction
    def corrected_position(self):
        correction_x, correction_y = self.position_correction_metres
        x, y, z = self.position
        return (x + correction_x, y + correction_y, z)

    def uncorrected_setpoint(self, setpoint):
        """A route target, in the corrected positions the route sees, as a camera tracking target."""
        correction_x, correction_y = self.position_correction_metres
        return (setpoint[0] - correction_x, setpoint[1] - correction_y, setpoint[2], setpoint[3])

    def correct_drift(self, now_seconds):
        """Stopped at a corner, not yet turned: compare the depth straight ahead with the map, and
        move the position estimate along the heading if they agree closely enough."""
        settings = self.settings
        measured = self.depth_ahead_metres
        if not self.received_within(self.depth_ahead_seconds, now_seconds, settings.drift_depth_max_age_seconds):
            measured = None
        x, y, z = self.corrected_position()
        forward, left = world_to_body(self.home, self.start_heading_radians, (x, y))
        heading_from_start = self.heading_radians - self.start_heading_radians
        map_metres, wall_name = self.map_ranging.distance_ahead(forward, left, heading_from_start,
                                                               z - self.ground_height_metres)
        decision = decide_correction(measured, map_metres, wall_name, settings)
        where = self.route.corner_name()
        if decision.shift_metres is None:
            self.log(now_seconds, f"drift correction at {where}: none, {decision.reason}")
            return
        correction_x, correction_y = self.position_correction_metres
        correction_x += decision.shift_metres * math.cos(self.heading_radians)
        correction_y += decision.shift_metres * math.sin(self.heading_radians)
        self.position_correction_metres = (correction_x, correction_y)
        self.log(now_seconds, f"drift correction at {where}: {decision.reason}; total correction now "
                              f"({correction_x:+.2f}, {correction_y:+.2f}) m")

    def begin_landing(self, now_seconds, why):
        self.log(now_seconds, why)
        setpoint = self.current_setpoint
        if setpoint is None:
            setpoint = (self.position[0], self.position[1], self.position[2], self.heading_radians)
        self.landing = Landing(self.settings, self.floor_reading_metres, self.ground_height_metres)
        self.landing.begin(now_seconds, self.position, setpoint)
        self.phase = "landing"

    def land(self, now_seconds):
        position = self.position if self.position_is_usable(now_seconds) else None
        laser = self.laser_metres if self.laser_is_fresh(now_seconds) else None
        landing_command = self.landing.step(now_seconds, position, laser)
        if landing_command.hold_position is not None:
            return self.position_command(now_seconds, landing_command.hold_position, landing_command.reason)
        command = Command("velocity", velocity=landing_command.velocity, reason=landing_command.reason)
        if self.last_published_kind != "velocity":
            # The trap: if the velocity commands ever stop, the drone flies to the last position
            # target. Make that the place it is now.
            x, y, z = self.position
            command.position_before_velocity = (x, y, z, self.heading_radians)
            self.current_setpoint = command.position_before_velocity
        return command

    def position_command(self, now_seconds, setpoint, reason):
        target = self.keep_close_to_the_drone(now_seconds, setpoint)
        self.current_setpoint = target
        return Command("position", position=tuple(target[:3]), heading_radians=target[3], reason=reason)

    def keep_close_to_the_drone(self, now_seconds, setpoint):
        """PX4 flies to a position target as fast as 12 m/s, so a target is never sent further than
        the carrot's 0.5 m from the drone, sideways."""
        if not self.position_is_usable(now_seconds):
            return tuple(setpoint)
        east, north = setpoint[0] - self.position[0], setpoint[1] - self.position[1]
        distance = math.hypot(east, north)
        largest = self.settings.carrot_lead_metres
        if distance <= largest:
            return tuple(setpoint)
        scale = largest / distance
        return (self.position[0] + east * scale, self.position[1] + north * scale, setpoint[2], setpoint[3])
