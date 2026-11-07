"""The whole flight, from waiting on the launch mark until the motors switch off.

How it fits: mission_node.py passes in every report it hears (camera tracking position, the
organizers' flight state, PX4's position report with the bottom laser distance) and calls step
twenty times a second. step returns what to publish this time: a position target, a velocity
command, or nothing. The course is flown by course_stages.py, the landing by landing.py, and
position_watch.py spots position jumps and measures speed. This file never talks to ROS and never
reads a clock, so the tests can run it on a laptop.

Words used here:
* "Offboard" is the PX4 mode in which the drone follows targets from a computer. The organizers'
  code asks PX4 for it once per arming, when the pilot flips the switch to Position after arming
  on the ground (docs/ARCHITECTURE.md, section 8). It reports the result as offboard_active.
* "Arming" gives the motors permission to spin. PX4 disarms by itself after it decides it landed.
* Positions are in the camera tracking frame (ENU: x East, y North, z Up, metres). Heading is
  anticlockwise from East, in radians. Velocity commands use the same world directions, whatever
  way the nose points.

Phases:
  waiting_for_start  On the launch mark. We already send a target just above the drone, because the
                     organizers' code only hands control over while it is hearing targets; it has no
                     effect before that. We also record the laser's floor reading (section 7, step 1).
  taking_off         From the first step with Offboard active: climb to cruise height.
  flying_course      course_stages.py decides.
  landing            landing.py decides.
  faulted            The position jumped. Only a zero velocity is sent; the pilot recovers the drone.
                     (If this program then died, the organizers' code would fly back to the last
                     position target, which the jump has moved. The team accepts this: the pilot
                     recovers the drone.)
  start_refused      Offboard began while camera tracking was frozen. A gentle descent is sent, so
                     the drone stays on the ground until PX4 switches the motors off.
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
* Nothing advances unless camera tracking reports and PX4's position reports are each freshly
  received, and the flight state is fresh and says the link to PX4 is up. (The flight state repeats
  old values, so it cannot vouch for the other two.) Otherwise hold, and land if that lasts 5 s.
* Camera tracking that repeats a bit-identical position more than 10 times in a row has frozen, and
  counts as unhealthy, like reports that stopped. While it is frozen before a start, no target is
  sent, so control cannot be handed over; the reason is logged.
* Every position target stays within 0.5 m of the drone, sideways.
* A position jump of more than 0.5 m between steps latches a fault: stop advancing, send nothing
  but a zero velocity, and leave the recovery to the pilot.
* 170 s after the start the landing begins. Nothing stops a landing except the motors switching
  off or Offboard ending.
* Every decision has a reason. Each change of phase or reason is logged, and so is each new peak
  horizontal speed.
"""
import math
from dataclasses import dataclass, field

from .course_stages import Course
from .landing import Landing
from .mission_inputs import MissionInputs, is_finite_number
from .mission_settings import MissionSettings
from .position_watch import PositionWatch

# Our own choice: after a pause in the steps (a slow computer), move the carrot as if at most this
# much time had passed, so it never jumps ahead.
LONGEST_STEP_SECONDS = 0.25
# Our own choice: log the peak horizontal speed each time it rises by this much.
PEAK_SPEED_LOG_STEP_METRES_PER_SECOND = 0.1
FLYING_PHASES = ("taking_off", "flying_course", "landing", "faulted")


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
    def __init__(self, settings=None):
        self.settings = settings or MissionSettings()
        self.inputs = MissionInputs()            # detector reports; all missing until detectors exist
        self.position_watch = PositionWatch(self.settings.position_jump_limit_metres,
                                            1.0 / self.settings.steps_per_second,
                                            self.settings.frozen_tracking_identical_reports)
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
        self.floor_laser_readings = []           # [(seconds, distance)] while on the launch mark
        self.seen_offboard_inactive = False
        self.last_step_seconds = None
        self.reset_run_state()

    def reset_run_state(self):
        """Forget everything worked out during a previous run: the course's memory (where the pad
        is, what space was seen free, how many sightings were counted), the landing, the fault, the
        timers, the position watch and the detector reports. There is no map correction yet; when
        there is, it must be cleared here too."""
        self.course = Course(self.settings)
        self.landing = None
        self.position_watch.reset()
        self.inputs.clear()
        self.fault = None
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

    # ------------------------------------------------------------------ reports from mission_node.py
    def observe_position(self, now_seconds, position, heading_radians):
        self.position = (float(position[0]), float(position[1]), float(position[2]))
        self.heading_radians = float(heading_radians)
        self.position_seconds = now_seconds
        self.position_watch.observe(now_seconds, self.position)

    def observe_flight_state(self, now_seconds, offboard_active, armed, landed, px4_link=None):
        """Each value is True, False, or None when the report did not say."""
        self.offboard_active = offboard_active if isinstance(offboard_active, bool) else None
        self.armed = armed if isinstance(armed, bool) else None
        self.landed = landed if isinstance(landed, bool) else None
        self.px4_link = px4_link if isinstance(px4_link, bool) else None
        self.flight_state_seconds = now_seconds

    def observe_px4_position_report(self, now_seconds, laser_distance_metres):
        """PX4's position report arrived; we use only its bottom laser distance (dist_bottom)."""
        self.px4_report_seconds = now_seconds
        if not is_finite_number(laser_distance_metres):
            return
        self.laser_metres = float(laser_distance_metres)
        self.laser_seconds = now_seconds
        if self.phase == "waiting_for_start" and self.landed is True:
            self.floor_laser_readings.append((now_seconds, self.laser_metres))
            oldest_kept = now_seconds - self.settings.floor_reading_window_seconds
            self.floor_laser_readings = [entry for entry in self.floor_laser_readings if entry[0] >= oldest_kept]

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
        if self.phase != "faulted":
            self.check_for_position_jump(now_seconds)
        if self.phase == "faulted":
            return Command("velocity", velocity=(0.0, 0.0, 0.0),
                           reason=f"{self.fault}: zero velocity only, waiting for the pilot to take over")
        self.check_flight_time(now_seconds)
        if self.phase == "taking_off":
            return self.take_off(now_seconds)
        if self.phase == "flying_course":
            return self.fly_course(now_seconds, seconds_since_last_step)
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

    def frozen_tracking_reason(self):
        repeats = self.position_watch.identical_reports_in_a_row
        return f"camera tracking frozen (the same position {repeats} times in a row)"

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
        if not self.position_is_fresh(now_seconds):
            return Command("nothing", reason="waiting for position reports from camera tracking")
        if self.position_watch.tracking_frozen():
            # Without a recent target the organizers' code will not hand over control.
            return Command("nothing", reason="not ready: camera tracking is frozen (its position is not changing "
                                             "at all), so no target is sent and control cannot be handed over")
        x, y, z = self.position
        # A target just above the drone: the lowest the organizers' code allows. It does nothing
        # until Offboard starts, and then it lifts the drone off at once.
        target = (x, y, z + self.settings.lowest_position_target_metres)
        return Command("position", position=target, heading_radians=self.heading_radians,
                       reason="ready: waiting for Offboard (the pilot's switch to Position after arming)")

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
        if self.position_watch.tracking_frozen():
            return self.refuse_start(now_seconds)
        floor_reading = self.median_floor_reading()
        self.reset_run_state()
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
        return self.take_off(now_seconds)

    def refuse_start(self, now_seconds):
        """Offboard has begun, but camera tracking is frozen: no mission. The organizers' code is
        already flying to our last target, just above the drone, so keep commanding a gentle
        descent until PX4 switches the motors off. On the floor this cannot move the drone."""
        self.phase = "start_refused"
        self.log(now_seconds, f"refused to start: {self.frozen_tracking_reason()}, so the position is not "
                              "changing at all")
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
        if self.offboard_active is False:
            return self.stop(now_seconds, "PX4 left Offboard, so the pilot or a PX4 safety action has the drone: "
                                          "not publishing any more")
        return None

    def check_for_position_jump(self, now_seconds):
        if not self.position_is_usable(now_seconds):
            return
        if self.phase == "landing" and self.landing.touched_down:
            return                               # on the floor, pushing down: position no longer used
        jump = self.position_watch.jump_since_last_step(now_seconds, self.position)
        if jump is None:
            return
        self.fault = f"position jumped {jump:.2f} m between steps"
        self.course.drop_world_goals()
        self.log(now_seconds, f"{self.fault}: fault latched; no more targets, recovery is up to the pilot")
        self.phase = "faulted"

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
