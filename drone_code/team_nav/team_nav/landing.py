"""Landing with the bottom laser sensor, following section 7 of docs/ARCHITECTURE.md.

How it fits: mission.py starts a Landing whenever the flight must end (the course is finished, a
time limit ran out, or a stage cannot go on), then calls step twenty times a second. Each step
returns either a position target (while settling over the spot) or a velocity command (while
descending), with a reason. mission.py sees when PX4 switches the motors off and ends the flight
there. This file never talks to ROS and never reads a clock.

Why velocity commands: the organizers' code never lets a position target go below 0.3 m above the
starting height, so the descent uses velocity commands, which have no such floor. A velocity
command expires after 1 s, so mission.py resends one every step until the motors are off.
Velocity commands are in world directions (x East, y North, z Up), whatever way the nose points:
the organizers' code ignores the frame named in the message. So the sideways corrections here are
worked out in world directions too.

Why the laser: PX4 switches the motors off by itself once it decides the drone has landed, and one
of its tests is that the motors are at low power. Commanding a fast descent lowers the power
quickly, but that is only safe once the drone is on the floor. The laser tells us when that is.

Steps, numbered as in section 7 (step 1, recording the floor reading on the launch mark, is done by
mission.py before takeoff):

  2. settle over the landing spot, for up to 3 s
  3. check that the laser reads about the height we expect; if not, hold and look again
  4. descend at 0.5 m/s until 0.3 m above the floor
  5. descend at 0.35 m/s; no sideways corrections below 0.2 m
  6. touchdown: all three of these, together, for 0.3 s: we are commanding a descent fast enough for
     PX4's landing check; position reports show the drone has stopped moving down; and the laser
     reads within 3 cm of the floor reading
  7. push down at 1 m/s
  8. wait for the motors to switch off; after 15 s, lift to 0.3 m and try once more

Two changes from section 7, from the organizer code audit and our own checks:
* The laser is never enough on its own, and neither is stopping. PX4 never reports less than 0.1 m
  (EKF2_MIN_RNG), so a laser at the floor reading can still mean a few centimetres up. Without a
  usable laser or without position reports, touchdown is never confirmed: the drone keeps
  descending gently, which PX4's own landing check accepts, and PX4 lands it a few seconds later.
* A drone that is hovering is also "not moving down". So stopping only counts after the drone has
  been seen moving down during this descent. Otherwise a broken laser stuck at the floor reading
  would make the start of a descent look like a touchdown, and the drone would push down at 1 m/s
  from 0.8 m.

What it does not do: choose where to land, or switch the motors off (only PX4 can).
"""
import math
from dataclasses import dataclass

# Our own choice: the drone counts as really descending once seen moving down this fast.
DESCENDING_FOR_REAL_METRES_PER_SECOND = 0.15
# From PX4 source code: its landing check needs a commanded descent of at least
# 1.1 x LNDMC_Z_VEL_MAX, which is 0.25 m/s on drone 16.
PX4_LANDING_CHECK_DESCENT_METRES_PER_SECOND = 0.275
# Our own choice: speeds are measured over about this much position history.
SPEED_WINDOW_SECONDS = 0.3


@dataclass
class LandingCommand:
    """Either hold_position (x, y, z, heading) or velocity (east, north, up) in metres per second."""
    reason: str
    hold_position: tuple | None = None
    velocity: tuple | None = None


class Landing:
    def __init__(self, settings, floor_reading_metres, ground_height_metres):
        self.settings = settings
        if floor_reading_metres is None:
            floor_reading_metres = settings.laser_floor_reading_if_unrecorded_metres
        self.floor_reading_metres = floor_reading_metres
        self.ground_height_metres = ground_height_metres
        self.step_name = "settling"
        self.step_start_seconds = None
        self.landing_spot = None              # (x, y): the drone stays over it while above 0.2 m
        self.hold_target = None               # (x, y, z, heading) while settling and checking
        self.position_history = []            # [(seconds, x, y, z)]
        self.laser_history = []               # [(seconds, distance)]
        self.touching_since_seconds = None
        self.seen_descending = False
        self.retries_used = 0
        self.touched_down = False             # touchdown confirmed and pushing down

    # ------------------------------------------------------------------ calls from mission.py
    def begin(self, now_seconds, position, setpoint):
        """position = where the drone is now; setpoint = (x, y, z, heading) it was flying to."""
        self.landing_spot = (float(position[0]), float(position[1]))
        self.hold_target = (self.landing_spot[0], self.landing_spot[1], float(setpoint[2]), float(setpoint[3]))
        self.go_to("settling", now_seconds)

    def step(self, now_seconds, position, laser_metres):
        """position = (x, y, z) or None when position reports have stopped; laser_metres = the
        bottom laser reading or None when it has stopped."""
        self.remember(now_seconds, position, laser_metres)
        handler = getattr(self, "step_" + self.step_name)
        return handler(now_seconds, position, laser_metres)

    # ------------------------------------------------------------------ history and measurements
    def go_to(self, step_name, now_seconds):
        self.step_name = step_name
        self.step_start_seconds = now_seconds
        self.touching_since_seconds = None
        self.touched_down = step_name == "pushing_down"

    def remember(self, now_seconds, position, laser_metres):
        oldest_kept = now_seconds - 2 * SPEED_WINDOW_SECONDS
        if position is None:
            self.position_history = []
        else:
            self.position_history.append((now_seconds, float(position[0]), float(position[1]), float(position[2])))
            self.position_history = [entry for entry in self.position_history if entry[0] >= oldest_kept]
        if laser_metres is None:
            self.laser_history = []
        else:
            self.laser_history.append((now_seconds, float(laser_metres)))
            self.laser_history = [entry for entry in self.laser_history if entry[0] >= oldest_kept]

    def history_span_seconds(self, history):
        return history[-1][0] - history[0][0] if len(history) >= 2 else 0.0

    def downward_speed(self):
        """Metres per second downward from the position history, or None if too little history."""
        history = self.position_history
        if self.history_span_seconds(history) < SPEED_WINDOW_SECONDS - 1e-9:
            return None
        return (history[0][3] - history[-1][3]) / self.history_span_seconds(history)

    def sideways_speed(self):
        history = self.position_history
        if self.history_span_seconds(history) < SPEED_WINDOW_SECONDS - 1e-9:
            return None
        moved = math.hypot(history[-1][1] - history[0][1], history[-1][2] - history[0][2])
        return moved / self.history_span_seconds(history)

    def laser_is_stuck(self):
        history = self.laser_history
        if self.history_span_seconds(history) < self.settings.touchdown_confirm_seconds - 1e-9:
            return False
        readings = [distance for _, distance in history]
        return max(readings) - min(readings) < self.settings.laser_stuck_change_metres

    def laser_height_above_floor(self, laser_metres):
        return None if laser_metres is None else laser_metres - self.floor_reading_metres

    def tracking_height_above_ground(self, position):
        return None if position is None else position[2] - self.ground_height_metres

    def sideways_correction(self, position, laser_metres):
        """Velocity (east, north), in world directions, back over the landing spot; zero when
        too low, or when the position is not known."""
        settings = self.settings
        heights = [height for height in (self.laser_height_above_floor(laser_metres),
                                         self.tracking_height_above_ground(position)) if height is not None]
        too_low = not heights or min(heights) < settings.no_sideways_corrections_below_metres
        if position is None or too_low:
            return 0.0, 0.0
        east = settings.landing_sideways_gain_per_second * (self.landing_spot[0] - position[0])
        north = settings.landing_sideways_gain_per_second * (self.landing_spot[1] - position[1])
        size = math.hypot(east, north)
        largest = settings.sideways_speed_limit_metres_per_second
        if size > largest:
            east, north = east * largest / size, north * largest / size
        return east, north

    # ------------------------------------------------------------------ the steps
    def step_settling(self, now_seconds, position, laser_metres):
        settings = self.settings
        speed = self.sideways_speed()
        settled = speed is not None and speed < settings.settled_speed_metres_per_second
        waited = now_seconds - self.step_start_seconds
        if settled or waited >= settings.landing_settle_time_limit_seconds:
            self.go_to("checking_laser", now_seconds)
            return self.step_checking_laser(now_seconds, position, laser_metres)
        return LandingCommand("settling over the landing spot", hold_position=self.hold_target)

    def step_checking_laser(self, now_seconds, position, laser_metres):
        settings = self.settings
        if laser_metres is None:
            self.go_to("descending_gentle", now_seconds)
            return LandingCommand("laser has no reading: descending gently, PX4 will detect the landing",
                                  velocity=(0.0, 0.0, -settings.gentle_descent_speed_metres_per_second))
        expected = (self.hold_target[2] - self.ground_height_metres) + self.floor_reading_metres
        if abs(laser_metres - expected) <= settings.laser_expected_tolerance_metres:
            self.go_to("descending_medium", now_seconds)
            return self.step_descending_medium(now_seconds, position, laser_metres)
        if now_seconds - self.step_start_seconds >= settings.laser_recheck_seconds:
            self.go_to("descending_gentle", now_seconds)
            return LandingCommand(f"laser reads {laser_metres:.2f} m, expected {expected:.2f} m: descending gently",
                                  velocity=(0.0, 0.0, -settings.gentle_descent_speed_metres_per_second))
        return LandingCommand("laser reading unexpected: something below, holding to look again",
                              hold_position=self.hold_target)

    def step_descending_medium(self, now_seconds, position, laser_metres):
        settings = self.settings
        self.note_if_descending()
        heights = [height for height in (self.laser_height_above_floor(laser_metres),
                                         self.tracking_height_above_ground(position)) if height is not None]
        near_floor = any(height <= settings.gentle_descent_height_metres for height in heights)
        laser_broken = laser_metres is None or (self.seen_descending and self.laser_is_stuck())
        if near_floor or laser_broken:
            self.go_to("descending_gentle", now_seconds)
            return self.step_descending_gentle(now_seconds, position, laser_metres)
        east, north = self.sideways_correction(position, laser_metres)
        return LandingCommand("descending at the medium speed",
                              velocity=(east, north, -settings.medium_descent_speed_metres_per_second))

    def step_descending_gentle(self, now_seconds, position, laser_metres):
        settings = self.settings
        descent = settings.gentle_descent_speed_metres_per_second
        self.note_if_descending()
        if self.touchdown_confirmed(now_seconds, descent, laser_metres):
            self.go_to("pushing_down", now_seconds)
            return LandingCommand("touchdown confirmed: pushing down", velocity=(0.0, 0.0, -settings.push_down_speed_metres_per_second))
        east, north = self.sideways_correction(position, laser_metres)
        reason = "descending gently"
        if laser_metres is None or self.laser_is_stuck() or position is None:
            reason = "descending gently; touchdown cannot be confirmed, PX4 will detect the landing"
        return LandingCommand(reason, velocity=(east, north, -descent))

    def note_if_descending(self):
        speed = self.downward_speed()
        if speed is not None and speed >= DESCENDING_FOR_REAL_METRES_PER_SECOND:
            self.seen_descending = True

    def touching_floor_now(self, commanded_descent_metres_per_second, laser_metres):
        """All three signs at once; none of them is enough alone."""
        settings = self.settings
        commanding_descent = commanded_descent_metres_per_second >= PX4_LANDING_CHECK_DESCENT_METRES_PER_SECOND
        speed = self.downward_speed()
        stopped = speed is not None and speed < settings.stopped_moving_down_metres_per_second
        laser_on_floor = (laser_metres is not None
                          and abs(laser_metres - self.floor_reading_metres) <= settings.touchdown_laser_tolerance_metres)
        return commanding_descent and stopped and self.seen_descending and laser_on_floor

    def touchdown_confirmed(self, now_seconds, commanded_descent_metres_per_second, laser_metres):
        if not self.touching_floor_now(commanded_descent_metres_per_second, laser_metres):
            self.touching_since_seconds = None
            return False
        if self.touching_since_seconds is None:
            self.touching_since_seconds = now_seconds
        return now_seconds - self.touching_since_seconds >= self.settings.touchdown_confirm_seconds - 1e-9

    def step_pushing_down(self, now_seconds, position, laser_metres):
        # Once on the floor the laser is never checked again: on the floor it drifts upward
        # (section 7), which would look like lifting off.
        settings = self.settings
        pushed = now_seconds - self.step_start_seconds
        push = (0.0, 0.0, -settings.push_down_speed_metres_per_second)
        if pushed >= settings.landing_retry_after_seconds and self.retries_used == 0:
            self.retries_used += 1
            self.go_to("lifting_for_retry", now_seconds)
            return self.step_lifting_for_retry(now_seconds, position, laser_metres)
        if pushed >= settings.motors_off_patience_seconds:
            return LandingCommand("motors still on: still pushing down", velocity=push)
        return LandingCommand("pushing down, waiting for PX4 to switch the motors off", velocity=push)

    def step_lifting_for_retry(self, now_seconds, position, laser_metres):
        settings = self.settings
        height = self.tracking_height_above_ground(position)
        lifted = height is not None and height >= settings.landing_retry_lift_metres
        if lifted or now_seconds - self.step_start_seconds >= settings.landing_retry_lift_time_limit_seconds:
            self.go_to("descending_gentle", now_seconds)
            self.seen_descending = False
            return LandingCommand("lifted: descending gently to try the landing once more",
                                  velocity=(0.0, 0.0, -settings.gentle_descent_speed_metres_per_second))
        return LandingCommand("motors did not stop: lifting to try the landing once more",
                              velocity=(0.0, 0.0, settings.landing_retry_lift_speed_metres_per_second))
