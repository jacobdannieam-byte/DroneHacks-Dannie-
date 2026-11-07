"""A pretend drone for the tests: PX4 and the organizers' flight interface, reduced to what the
mission program depends on. Test code only. It is never installed or run on the drone.

What it imitates:
* The hand-over (organizers' auto_offboard.py): Offboard starts only if the drone is armed, on the
  ground, and has heard a target within the last second.
* Position targets never go below 0.3 m above the start (organizers' envelope.py); velocity
  commands expire after 1 s and are capped at 1.5 m/s, and when one expires the drone flies back to
  the last position target (organizers' flight_interface.py).
* The drone flies toward a position target at about 0.95 times the distance per second, at most
  1 m/s (team plan, section 6), and follows velocity commands directly. The floor stops it.
* PX4's land detector, simplified from the simulator results in the team plan (section 7): with a
  descent commanded on the floor, the motors reach low power after about 1 s when pushing at
  1 m/s, or about 5 s at a gentle speed; PX4 marks the drone landed 2.5 s later, and switches the
  motors off 2 s after that.
* The bottom laser never reads below 0.1 m (PX4 setting EKF2_MIN_RNG), and drifts upward by up to
  8 cm while the drone sits on the floor (section 7).
* Camera tracking's reported position wobbles by a few micrometres from report to report, as real
  tracking does even when the drone is still. tracking_frozen makes it repeat one position exactly,
  as drone 16's did after tracking died (2026-11-07). still_reports_identical makes it repeat one
  position exactly while it stands on the floor, as a healthy drone 16 can (2026-11-07), and
  stamps_frozen makes every report carry the same timestamp.
* Optionally, PX4 switches the motors off if the drone has not taken off within 10 s of arming
  (PX4 setting COM_DISARM_PRFLT).
"""
import math
from collections import namedtuple

LOWEST_POSITION_TARGET_METRES = 0.3
VELOCITY_COMMAND_LIFETIME_SECONDS = 1.0
FASTEST_VELOCITY_COMMAND_METRES_PER_SECOND = 1.5
POSITION_RESPONSE_PER_SECOND = 0.95
FASTEST_POSITION_FLIGHT_METRES_PER_SECOND = 1.0
TURN_RATE_RADIANS_PER_SECOND = math.radians(60)
LANDING_CHECK_DESCENT_METRES_PER_SECOND = 0.275
LOW_POWER_SECONDS_WHEN_PUSHING = 1.0
LOW_POWER_SECONDS_WHEN_GENTLE = 5.0
LANDED_SECONDS_AFTER_LOW_POWER = 2.5
MOTORS_OFF_SECONDS_AFTER_LANDED = 2.0
LASER_LOWEST_READING_METRES = 0.1
LASER_DRIFT_ON_FLOOR_METRES = 0.08
LASER_DRIFT_PER_SECOND = 0.02
# Our own choice for the tests: far too small to matter, but enough that no two reports are identical.
TRACKING_WOBBLE_METRES = 1e-6


class FakeDrone:
    def __init__(self, heading_radians=0.0, laser_height_on_floor_metres=0.08,
                 floor_height=lambda x, y: 0.0, motors_can_lift=True, disarms_if_not_flown_after_seconds=None):
        self.position = [0.0, 0.0, 0.0]
        self.heading_radians = heading_radians
        self.armed = False
        self.offboard = False
        self.landed = True
        self.laser_height_on_floor_metres = laser_height_on_floor_metres
        self.floor_height = floor_height
        self.motors_can_lift = motors_can_lift
        self.pose_target = None
        self.pose_heading = None
        self.pose_seconds = None
        self.velocity_command = None
        self.velocity_seconds = None
        self.low_power_progress = 0.0
        self.landed_timer_seconds = 0.0
        self.motors_off_timer_seconds = 0.0
        self.seconds_on_floor = 0.0
        self.has_flown = False
        self.sends_position_reports = True        # tests turn this off to imitate camera tracking stopping
        self.sends_px4_reports = True             # ... or PX4's position reports stopping
        self.sends_flight_state = True            # ... or the organizers' flight state stopping
        self.px4_link = True                      # what the flight state says about the link to PX4
        self.tracking_frozen = False              # tests turn this on to imitate camera tracking freezing
        self.frozen_report = None
        self.reports_made = 0
        self.last_reported = (0.0, 0.0, 0.0)
        self.disarms_if_not_flown_after_seconds = disarms_if_not_flown_after_seconds
        self.armed_at_seconds = None
        self.frozen_orientation = None
        self.xy_reset_counter = 0                 # PX4 raises these when it resets its estimates
        self.tracking_error_metres = [0.0, 0.0]   # tests set this to imitate camera tracking drift (x, y)
        self.still_reports_identical = False      # no wobble while standing on the floor
        self.stamps_frozen = False                # every report carries the same timestamp
        self.frozen_stamp = None
        self.heading_reset_counter = 0

    # ------------------------------------------------------------------ what the mission hears
    def reported_position(self):
        """What camera tracking reports: the position with a tiny wobble, or a frozen copy."""
        if self.tracking_frozen:
            if self.frozen_report is None:
                self.frozen_report = self.last_reported
            return self.frozen_report
        self.frozen_report = None
        self.reports_made += 1
        wobble = TRACKING_WOBBLE_METRES * (self.reports_made % 3 - 1)
        if self.still_reports_identical and self.on_floor():
            wobble = 0.0
        error_x, error_y = self.tracking_error_metres
        self.last_reported = (self.position[0] + error_x + wobble, self.position[1] + error_y, self.position[2])
        return self.last_reported

    def report_stamp(self, now_seconds):
        """The timestamp in a camera tracking report: the time now, or a stuck copy."""
        if not self.stamps_frozen:
            self.frozen_stamp = None
            return now_seconds
        if self.frozen_stamp is None:
            self.frozen_stamp = now_seconds
        return self.frozen_stamp

    def reported_orientation(self):
        """Camera tracking's orientation (x, y, z, w) for the heading, frozen along with the position."""
        if self.tracking_frozen:
            if self.frozen_orientation is None:
                self.frozen_orientation = self.orientation_now()
            return self.frozen_orientation
        self.frozen_orientation = None
        return self.orientation_now()

    def orientation_now(self):
        return (0.0, 0.0, math.sin(self.heading_radians / 2), math.cos(self.heading_radians / 2))

    def px4_nav_state(self):
        """PX4's status: 14 is Offboard, 2 is Position."""
        return 14 if self.offboard else 2

    def flight_state(self):
        return self.offboard, self.armed, self.landed, self.px4_link

    def laser_reading(self):
        floor = self.floor_height(self.position[0], self.position[1])
        reading = max(LASER_LOWEST_READING_METRES, self.position[2] - floor + self.laser_height_on_floor_metres)
        if self.on_floor() and self.has_flown:
            reading += min(LASER_DRIFT_ON_FLOOR_METRES, LASER_DRIFT_PER_SECOND * self.seconds_on_floor)
        return reading

    def on_floor(self):
        return self.position[2] <= self.floor_height(self.position[0], self.position[1]) + 1e-9

    # ------------------------------------------------------------------ the pilot and the hand-over
    def arm(self, now_seconds=None):
        self.armed = True
        self.armed_at_seconds = now_seconds

    def switch_to_position_mode(self, now_seconds):
        """The pilot's switch after arming. Returns True if Offboard started."""
        heard = [stamp for stamp in (self.pose_seconds, self.velocity_seconds) if stamp is not None]
        recent = heard and now_seconds - max(heard) <= VELOCITY_COMMAND_LIFETIME_SECONDS
        if self.armed and self.landed and recent:
            self.offboard = True
        return self.offboard

    # ------------------------------------------------------------------ what the mission sends
    def receive(self, now_seconds, command):
        before = getattr(command, "position_before_velocity", None)
        if command.kind == "velocity" and before is not None:
            self.receive_position_target(now_seconds, before[:3], before[3])
        if command.kind == "position":
            self.receive_position_target(now_seconds, command.position, command.heading_radians)
        elif command.kind == "velocity":
            east, north, up = command.velocity
            size = math.sqrt(east * east + north * north + up * up)
            scale = min(1.0, FASTEST_VELOCITY_COMMAND_METRES_PER_SECOND / size) if size > 0 else 1.0
            self.velocity_command = (east * scale, north * scale, up * scale)
            self.velocity_seconds = now_seconds

    def receive_position_target(self, now_seconds, position, heading_radians):
        x, y, z = position
        self.pose_target = (x, y, max(z, LOWEST_POSITION_TARGET_METRES))
        self.pose_heading = heading_radians
        self.pose_seconds = now_seconds
        self.velocity_command = None

    # ------------------------------------------------------------------ physics, much simplified
    def commanded_velocity(self, now_seconds):
        if not (self.armed and self.offboard):
            return 0.0, 0.0, 0.0
        if self.velocity_command is not None and now_seconds - self.velocity_seconds < VELOCITY_COMMAND_LIFETIME_SECONDS:
            return self.velocity_command
        if self.pose_target is None:
            return 0.0, 0.0, 0.0
        # PX4 flies to a target using camera tracking's position, drift and all.
        error_x, error_y = self.tracking_error_metres
        believed = (self.position[0] + error_x, self.position[1] + error_y, self.position[2])
        difference = [target - now for target, now in zip(self.pose_target, believed)]
        velocity = [POSITION_RESPONSE_PER_SECOND * value for value in difference]
        size = math.sqrt(sum(value * value for value in velocity))
        if size > FASTEST_POSITION_FLIGHT_METRES_PER_SECOND:
            velocity = [value * FASTEST_POSITION_FLIGHT_METRES_PER_SECOND / size for value in velocity]
        return tuple(velocity)

    def advance(self, now_seconds, seconds):
        east, north, up = self.commanded_velocity(now_seconds)
        if not self.motors_can_lift:
            up = min(up, 0.0)
        floor = self.floor_height(self.position[0], self.position[1])
        if not self.on_floor() or up > 0.0:
            self.position[0] += east * seconds
            self.position[1] += north * seconds
        self.position[2] = max(floor, self.position[2] + up * seconds)
        if self.position[2] > floor + 0.05:
            self.has_flown = True
            self.landed = False
        self.seconds_on_floor = self.seconds_on_floor + seconds if self.on_floor() else 0.0
        if self.offboard and self.pose_heading is not None and self.velocity_command is None:
            error = (self.pose_heading - self.heading_radians + math.pi) % (2 * math.pi) - math.pi
            largest = TURN_RATE_RADIANS_PER_SECOND * seconds
            self.heading_radians += max(-largest, min(largest, error))
        self.run_land_detector(up, seconds)
        self.disarm_if_never_took_off(now_seconds)

    def disarm_if_never_took_off(self, now_seconds):
        limit = self.disarms_if_not_flown_after_seconds
        if limit is None or not self.armed or self.has_flown or self.armed_at_seconds is None:
            return
        if now_seconds - self.armed_at_seconds >= limit:
            self.armed = False

    def run_land_detector(self, commanded_up_metres_per_second, seconds):
        descending_on_floor = (self.armed and self.has_flown and self.on_floor()
                               and commanded_up_metres_per_second <= -LANDING_CHECK_DESCENT_METRES_PER_SECOND)
        if not descending_on_floor:
            self.low_power_progress = self.landed_timer_seconds = self.motors_off_timer_seconds = 0.0
            return
        pushing = commanded_up_metres_per_second <= -0.9
        self.low_power_progress += seconds / (LOW_POWER_SECONDS_WHEN_PUSHING if pushing else LOW_POWER_SECONDS_WHEN_GENTLE)
        if self.low_power_progress < 1.0:
            return
        self.landed_timer_seconds += seconds
        if self.landed_timer_seconds < LANDED_SECONDS_AFTER_LOW_POWER:
            return
        self.landed = True
        self.motors_off_timer_seconds += seconds
        if self.motors_off_timer_seconds >= MOTORS_OFF_SECONDS_AFTER_LANDED:
            self.armed = False


# One step of a test flight: the time, what the mission published, its phase afterwards, and where
# the drone was when the command was sent.
FlightStep = namedtuple("FlightStep", ["seconds", "command", "phase", "drone_position"])


def fly(mission, drone, seconds, step_seconds=0.1, arm_at_seconds=0.5, switch_at_seconds=1.0, every_step=None,
        start_seconds=0.0, until=None):
    """Run the mission against the fake drone. every_step(now_seconds, mission, drone) runs before
    each step, for tests that change something mid-flight; until(flight_step) ends the run after
    the first step for which it is true. Returns a list of FlightStep."""
    record = []
    now = start_seconds
    switched = False
    while now < start_seconds + seconds - 1e-9:
        if every_step is not None:
            every_step(now, mission, drone)
        if now >= arm_at_seconds and not drone.armed and not drone.has_flown and not switched:
            drone.arm(now)
        if now >= switch_at_seconds and not switched and drone.armed:
            switched = True
            drone.switch_to_position_mode(now)
        if drone.sends_position_reports:
            mission.observe_position(now, drone.reported_position(), drone.heading_radians, drone.report_stamp(now),
                                     drone.reported_orientation())
        mission.observe_px4_status(now, drone.px4_nav_state())
        if drone.sends_flight_state:
            mission.observe_flight_state(now, *drone.flight_state())
        if drone.sends_px4_reports:
            mission.observe_px4_position_report(now, drone.laser_reading(), drone.xy_reset_counter,
                                                drone.heading_reset_counter)
        position_when_sent = tuple(drone.position)
        command = mission.step(now)
        drone.receive(now, command)
        drone.advance(now, step_seconds)
        record.append(FlightStep(now, command, mission.phase, position_when_sent))
        now = round(now + step_seconds, 9)
        if until is not None and until(record[-1]):
            break
    return record
