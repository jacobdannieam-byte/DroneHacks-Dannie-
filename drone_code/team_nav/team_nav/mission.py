"""The whole flight, from waiting on the launch mark until the motors switch off.

How it fits: mission_node.py passes in every report it hears (camera tracking position, the
organizers' flight state, the bottom laser) and calls step ten times a second. step returns what to
publish this time: a position target, a velocity command, or nothing. The course is flown by
course_stages.py and the landing by landing.py. This file never talks to ROS and never reads a
clock, so the tests can run it on a laptop.

Words used here:
* "Offboard" is the PX4 mode in which the drone follows targets from a computer. The organizers'
  code asks PX4 for it once per arming, when the pilot flips the switch to Position after arming
  on the ground (docs/ARCHITECTURE.md, section 8). It reports the result as offboard_active.
* "Arming" gives the motors permission to spin. PX4 disarms by itself after it decides it landed.
* Positions are in the camera tracking frame (ENU: x East, y North, z Up, metres). Heading is
  anticlockwise from East, in radians.

Phases:
  waiting_for_start  On the launch mark. We already send a takeoff target, because the organizers'
                     code only hands control over while it is hearing targets (auto_offboard.py);
                     the target has no effect before that. We also record the laser's floor
                     reading, which the landing uses (section 7, step 1).
  taking_off         From the first step with Offboard active: climb to cruise height.
  flying_course      course_stages.py decides.
  landing            landing.py decides.
  finished           PX4 switched the motors off after landing.
  stopped            Control is not ours any more. Nothing is published again in this run.

Safety rules (each one is a test in test/test_mission.py):
* Control is taken only when Offboard becomes active after we saw it inactive, with PX4 reporting
  the drone on the ground. Otherwise nothing is ever published after that.
* If Offboard stops being active, nothing more is published: we never fight the pilot or PX4.
* The climb starts on the first step. If the drone has not left the ground 10 s after the start,
  we stop, because PX4 switches the motors off then anyway.
* Position reports older than 0.2 s: hold, and land if that lasts 5 s.
* A position jump of more than 0.5 m between steps latches a fault: every position goal is dropped,
  the drone lands where it is with no sideways corrections, and nothing resumes.
* 170 s after the start the drone lands, whatever it is doing.
* Every decision has a reason. Each change of phase or reason is logged.
"""
import math
from dataclasses import dataclass, field

from .course_stages import Course
from .landing import Landing
from .mission_inputs import MissionInputs, is_finite_number
from .mission_settings import MissionSettings

# From the organizers' code: velocity commands are capped at 1.5 m/s, so position reports that
# resume after a gap may differ by at most this speed times the gap.
FASTEST_POSSIBLE_SPEED_METRES_PER_SECOND = 1.5
# Our own choice: after a pause in the steps (a slow computer), move the carrot as if at most this
# much time had passed, so it never jumps ahead.
LONGEST_STEP_SECONDS = 0.25


@dataclass
class Command:
    """What mission_node.py publishes this step. kind is "position", "velocity" or "nothing"."""
    kind: str
    position: tuple | None = None             # (x, y, z) metres
    heading_radians: float | None = None
    velocity: tuple | None = None             # (east, north, up) metres per second
    reason: str = ""
    log_lines: list = field(default_factory=list)


class Mission:
    def __init__(self, settings=None):
        self.settings = settings or MissionSettings()
        self.inputs = MissionInputs()            # detector reports; all missing until detectors exist
        self.course = Course(self.settings)
        self.landing = None
        self.phase = "waiting_for_start"
        self.reason = ""
        self.history = []                        # [(seconds, phase, reason)] at every change
        self.pending_log_lines = []
        self.position = None                     # latest camera tracking report
        self.heading_radians = None
        self.position_seconds = None
        self.offboard_active = None              # latest flight state report
        self.armed = None
        self.landed = None
        self.flight_state_seconds = None
        self.laser_metres = None                 # latest bottom laser report
        self.laser_seconds = None
        self.floor_laser_readings = []           # [(seconds, distance)] while on the launch mark
        self.seen_offboard_inactive = False
        self.start_seconds = None
        self.ground_height_metres = None
        self.home = None                         # (x, y) of the launch mark
        self.start_heading_radians = None
        self.floor_reading_metres = None
        self.left_the_ground = False
        self.current_setpoint = None             # (x, y, z, heading) last position target sent
        self.position_wait_since_seconds = None
        self.position_at_last_step = None
        self.position_at_last_step_seconds = None
        self.fault = None
        self.flight_time_limit_reached = False
        self.last_step_seconds = None

    # ------------------------------------------------------------------ reports from mission_node.py
    def observe_position(self, now_seconds, position, heading_radians):
        self.position = (float(position[0]), float(position[1]), float(position[2]))
        self.heading_radians = float(heading_radians)
        self.position_seconds = now_seconds

    def observe_flight_state(self, now_seconds, offboard_active, armed, landed):
        """Each value is True, False, or None when the report did not say."""
        self.offboard_active = offboard_active if isinstance(offboard_active, bool) else None
        self.armed = armed if isinstance(armed, bool) else None
        self.landed = landed if isinstance(landed, bool) else None
        self.flight_state_seconds = now_seconds

    def observe_laser(self, now_seconds, distance_metres):
        if not is_finite_number(distance_metres):
            return
        self.laser_metres = float(distance_metres)
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
        command.log_lines = self.pending_log_lines
        self.pending_log_lines = []
        return command

    def decide(self, now_seconds, seconds_since_last_step):
        if self.phase in ("finished", "stopped"):
            return Command("nothing", reason=self.reason)
        if self.phase == "waiting_for_start":
            return self.wait_for_start(now_seconds)
        ended = self.check_control_is_still_ours(now_seconds)
        if ended is not None:
            return ended
        self.check_for_position_jump(now_seconds)
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

    def stop(self, now_seconds, reason):
        self.phase = "stopped"
        return Command("nothing", reason=reason)

    # ------------------------------------------------------------------ freshness
    def position_is_fresh(self, now_seconds):
        return (self.position_seconds is not None
                and now_seconds - self.position_seconds <= self.settings.position_report_max_age_seconds)

    def laser_is_fresh(self, now_seconds):
        return (self.laser_seconds is not None
                and now_seconds - self.laser_seconds <= self.settings.laser_report_max_age_seconds)

    def flight_state_is_fresh(self, now_seconds):
        return (self.flight_state_seconds is not None
                and now_seconds - self.flight_state_seconds <= self.settings.flight_state_max_age_seconds)

    # ------------------------------------------------------------------ before the start
    def wait_for_start(self, now_seconds):
        if self.flight_state_is_fresh(now_seconds):
            if self.offboard_active is False:
                self.seen_offboard_inactive = True
            elif self.offboard_active is True:
                return self.try_to_start(now_seconds)
        if not self.position_is_fresh(now_seconds):
            return Command("nothing", reason="waiting for position reports from camera tracking")
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
        self.start_seconds = now_seconds
        self.ground_height_metres = self.position[2]
        self.home = (self.position[0], self.position[1])
        self.start_heading_radians = self.heading_radians
        self.floor_reading_metres = self.median_floor_reading()
        self.current_setpoint = (self.home[0], self.home[1],
                                 self.ground_height_metres + self.settings.lowest_position_target_metres,
                                 self.start_heading_radians)
        self.phase = "taking_off"
        return self.take_off(now_seconds)

    def median_floor_reading(self):
        readings = sorted(distance for _, distance in self.floor_laser_readings)
        if not readings:
            return None
        return readings[len(readings) // 2]

    # ------------------------------------------------------------------ rules checked every step
    def check_control_is_still_ours(self, now_seconds):
        if self.armed is False:
            if self.phase == "landing":
                self.phase = "finished"
                return Command("nothing", reason="landed: PX4 switched the motors off")
            return self.stop(now_seconds, f"the motors were switched off while {self.phase.replace('_', ' ')} "
                                          "(kill switch or PX4): not publishing any more")
        if self.offboard_active is False:
            return self.stop(now_seconds, "PX4 left Offboard, so the pilot or a PX4 safety action has the drone: "
                                          "not publishing any more")
        return None

    def check_for_position_jump(self, now_seconds):
        if self.fault is not None or not self.position_is_fresh(now_seconds):
            return
        previous, previous_seconds = self.position_at_last_step, self.position_at_last_step_seconds
        self.position_at_last_step, self.position_at_last_step_seconds = self.position, now_seconds
        if previous is None:
            return
        # Between two steps in a row the limit is exact; after a gap in the reports, allow for the
        # distance the drone could really have flown during the extra time.
        extra_seconds = max(0.0, now_seconds - previous_seconds - 1.0 / self.settings.steps_per_second)
        allowed = self.settings.position_jump_limit_metres + FASTEST_POSSIBLE_SPEED_METRES_PER_SECOND * extra_seconds
        jump = math.dist(previous, self.position)
        if jump <= allowed:
            return
        self.fault = f"position jumped {jump:.2f} m between steps"
        self.course.drop_world_goals()
        if self.phase == "landing":
            self.landing.drop_sideways_goal()
            self.history.append((now_seconds, self.phase, f"{self.fault}: sideways corrections dropped"))
            self.pending_log_lines.append(f"landing: {self.fault}: sideways corrections dropped")
            return
        self.begin_landing(now_seconds, f"{self.fault}: every position goal dropped, landing where it is")

    def check_flight_time(self, now_seconds):
        if self.phase == "landing" or self.flight_time_limit_reached:
            return
        limit = self.settings.flight_time_limit_seconds
        if now_seconds - self.start_seconds >= limit:
            self.flight_time_limit_reached = True
            self.begin_landing(now_seconds, f"flight time limit of {limit:g} s reached, landing")

    def hold_while_position_reports_stopped(self, now_seconds):
        """None while position reports are fresh; otherwise a hold, or a landing after 5 s."""
        if self.position_is_fresh(now_seconds):
            self.position_wait_since_seconds = None
            return None
        if self.position_wait_since_seconds is None:
            self.position_wait_since_seconds = now_seconds
        waited = now_seconds - self.position_wait_since_seconds
        if waited >= self.settings.longest_wait_for_input_seconds:
            longest = self.settings.longest_wait_for_input_seconds
            self.begin_landing(now_seconds, f"position reports stopped for {longest:g} s, landing")
            return self.land(now_seconds)
        return self.position_command(self.current_setpoint, "position reports stopped, holding")

    # ------------------------------------------------------------------ the phases
    def take_off(self, now_seconds):
        settings = self.settings
        elapsed = now_seconds - self.start_seconds
        target_height = min(settings.cruise_height_metres,
                            settings.lowest_position_target_metres + settings.climb_speed_metres_per_second * elapsed)
        held = self.hold_while_position_reports_stopped(now_seconds)
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
            self.current_setpoint = cruise
            self.course.start(now_seconds, cruise)
            self.phase = "flying_course"
            return self.position_command(cruise, f"at cruise height {settings.cruise_height_metres:g} m")
        if elapsed > settings.takeoff_time_limit_seconds:
            self.begin_landing(now_seconds, f"did not reach cruise height within {settings.takeoff_time_limit_seconds:g} s, landing")
            return self.land(now_seconds)
        target = (self.home[0], self.home[1], self.ground_height_metres + target_height, self.start_heading_radians)
        return self.position_command(target, f"climbing to {settings.cruise_height_metres:g} m")

    def fly_course(self, now_seconds, seconds_since_last_step):
        held = self.hold_while_position_reports_stopped(now_seconds)
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
        return self.position_command(decision.setpoint, f"{decision.stage}: {decision.reason}")

    def begin_landing(self, now_seconds, why):
        self.history.append((now_seconds, self.phase, why))
        self.pending_log_lines.append(f"{self.phase}: {why}")
        setpoint = self.current_setpoint
        if setpoint is None:
            setpoint = (self.position[0], self.position[1], self.position[2], self.heading_radians)
        self.landing = Landing(self.settings, self.floor_reading_metres, self.ground_height_metres)
        self.landing.begin(now_seconds, self.position, setpoint, sideways_goal_allowed=self.fault is None)
        self.phase = "landing"

    def land(self, now_seconds):
        position = self.position if self.position_is_fresh(now_seconds) else None
        laser = self.laser_metres if self.laser_is_fresh(now_seconds) else None
        landing_command = self.landing.step(now_seconds, position, laser)
        if landing_command.hold_position is not None:
            return self.position_command(landing_command.hold_position, landing_command.reason)
        return Command("velocity", velocity=landing_command.velocity, reason=landing_command.reason)

    def position_command(self, setpoint, reason):
        self.current_setpoint = tuple(setpoint)
        return Command("position", position=tuple(setpoint[:3]), heading_radians=setpoint[3], reason=reason)
