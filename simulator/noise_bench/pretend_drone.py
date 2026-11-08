"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

A small pretend drone 16 with PX4 flying it: how the real drone moves when it is given position
targets, speed targets and headings, with the wobbles we measured on 2026-11-08.

How it fits in
--------------
bench_link.py feeds it the targets our real flight program sends (in camera tracking's frame,
"odom": metres forward, left and up from where the drone sat on the launch mark) and calls
step() every few milliseconds of simulated time. It answers with the true position, heading,
tilt and turn rate, and PX4's own reports (armed, landed, height).

The model, in plain words
-------------------------
  - Sideways: PX4 asks for a speed proportional to the distance to the target (capped), and the
    drone reaches that speed after a short lag, never accelerating harder than a limit. Targets
    reach PX4 0.1 s late.
  - Up and down: the same, with its own numbers, plus a 1.65 s wait after arming before the drone
    leaves the floor (the motors spin up).
  - Heading: follows the asked-for heading 0.40 s late, no faster than a turn-rate limit.
  - Three wobbles PX4 does not remove at once: hover wander, the slide during a turn, and the
    drift during takeoff. Each is an offset added to the true position that dies away slowly.
  - PX4's position estimate is the truth plus the estimate error that bench_link.py hands in
    (camera tracking's slow drift, and dead reckoning while PX4 has dropped the camera). PX4
    flies its estimate to the target, so that error moves the real drone.
  - PX4's land detector and automatic disarm.

What it does not do: tilt dynamics, battery sag, wind from props near walls, contact forces.
A crash is decided by bench_referee.py, not here.
"""
import math

import numpy as np

# ---- Sideways ------------------------------------------------------------------------------------

# Estimated, fitted between two measurements: about 0.6 m/s of speed per metre of target lead in
# the 10:44 course run, and a 1 m step reaching 63 % in about 1.4 s (understand.json). With the
# lag below, 0.8 gives a 1 m step t63 of about 1.4 s and no overshoot (python pretend_drone.py).
SIDEWAYS_SPEED_PER_METRE_OF_ERROR = 0.8
# Measured upper end: the drone was never seen faster than about 0.7 m/s in our flights.
LARGEST_SIDEWAYS_SPEED_METRES_PER_SECOND = 0.7
# Estimated: PX4's MPC_ACC_HOR is about 2 m/s^2 on drone 16's settings.
LARGEST_SIDEWAYS_ACCELERATION_METRES_PER_SECOND_SQUARED = 2.0
# Estimated: how quickly the drone's speed follows PX4's wanted speed.
SPEED_LAG_SECONDS = 0.15
# Measured (understand.json, "Horizontal response to targets"): movement starts about 0.1 s after
# a new target.
TARGET_DELAY_SECONDS = 0.1

# ---- Up and down ----------------------------------------------------------------------------------

# Measured on 2026-11-08 (10:20, 10:44): climbing starts 1.6-1.7 s after the first climb target.
SPIN_UP_SECONDS = 1.65
# Measured: 10-90 % rise of about 3.9 s for 0.5 m, so about 0.56 per second (first order).
UPWARD_SPEED_PER_METRE_OF_ERROR = 0.56
# Measured: largest climb speed 0.38-0.57 m/s. Estimated: the same downward.
LARGEST_UPWARD_SPEED_METRES_PER_SECOND = 0.5
LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND = 1.0
# Measured: PX4's height settles 3-5 cm below the target.
HEIGHT_SETTLES_BELOW_TARGET_METRES = 0.03
# Estimated.
UPWARD_SPEED_LAG_SECONDS = 0.3

# ---- Heading --------------------------------------------------------------------------------------

# Measured (10:23, 10:44): PX4's heading follows our heading 0.40 s late.
HEADING_DELAY_SECONDS = 0.40
# Estimated: PX4's automatic yaw-rate limit; our program turns at 15 deg/s, well below it.
LARGEST_TURN_RATE_DEGREES_PER_SECOND = 60.0

# ---- Wobbles --------------------------------------------------------------------------------------

# Measured: hover spread 1-3 cm per axis; 2.5-7.5 cm during and just after turns. Estimated: a
# correlation time of about 2.5 s ("Ornstein-Uhlenbeck" noise: a random walk pulled back to zero).
HOVER_WANDER_METRES = 0.02
TURNING_WANDER_METRES = 0.06
WANDER_CORRELATION_SECONDS = 2.5
TURNING_IS_FASTER_THAN_DEGREES_PER_SECOND = 5.0
# Measured: an 80 degree turn from rest slides the drone 0.08-0.12 m; 0.28 m when the turn
# starts while moving at about 0.23 m/s.
SLIDE_PER_80_DEGREES_FROM_REST_METRES = (0.08, 0.12)
SLIDE_PER_80_DEGREES_WHILE_MOVING_METRES = 0.28
MOVING_IS_FASTER_THAN_METRES_PER_SECOND = 0.1
# Measured: PX4 pulls a 0.2 m error back at only 0.06-0.09 m/s, a time constant of 2-3 s.
OFFSET_RECOVERY_SECONDS = 2.5
# Measured over 14 takeoffs: drift 0.11-0.72 m, backward in 13 of 14, peaking in the first
# 0.5 m of climb. Our model: 0.15 m backward plus up to 0.30 m in a random direction.
TAKEOFF_DRIFT_BACKWARD_METRES = 0.15
TAKEOFF_DRIFT_RANDOM_LARGEST_METRES = 0.30
TAKEOFF_DRIFT_PEAKS_AT_HEIGHT_METRES = 0.5

# ---- Land detector --------------------------------------------------------------------------------

# Estimated: PX4 says "landed" after about a second on the floor while being pushed down.
SECONDS_ON_FLOOR_BEFORE_LANDED = 1.0
# From PX4's default COM_DISARM_LAND: motors off 2 s after landing is detected.
SECONDS_LANDED_BEFORE_DISARM = 2.0
ON_FLOOR_BELOW_METRES = 0.01
# From PX4: a speed target expires after 1 s in flight_interface, which then holds the last
# position target (organizers' code).
SPEED_TARGET_LIFETIME_SECONDS = 1.0

GRAVITY_METRES_PER_SECOND_SQUARED = 9.81


class PretendDrone:
    """The true state of the drone in camera tracking's frame at the launch: forward, left, up."""

    def __init__(self, random):
        self.random = random
        self.flown_position = np.zeros(3)      # Where PX4's control has put the drone.
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(2)
        self.heading_radians = 0.0
        self.turn_rate_radians_per_second = 0.0
        self.wander_offset = np.zeros(2)
        self.slide_offset = np.zeros(2)
        self.slide_direction = np.zeros(2)
        self.takeoff_drift_offset = np.zeros(2)
        self.takeoff_drift_full = self._random_takeoff_drift()
        self.has_reached_takeoff_peak = False
        self.armed = False
        self.armed_at_seconds = None
        self.landed = True
        self.on_floor_since_seconds = None
        self.landed_since_seconds = None
        self.position_targets = []   # (time sent, target position, heading or None), oldest first.
        self.speed_target = None     # (time sent, speed)
        self.target_position = None
        self.target_heading_history = []
        self.target_heading_radians = 0.0

    def _random_takeoff_drift(self):
        angle = self.random.uniform(0.0, 2.0 * math.pi)
        size = self.random.uniform(0.0, TAKEOFF_DRIFT_RANDOM_LARGEST_METRES)
        return np.array([-TAKEOFF_DRIFT_BACKWARD_METRES, 0.0]) + size * np.array([math.cos(angle), math.sin(angle)])

    # ---- What the outside sees ---------------------------------------------------------------------

    def true_position(self):
        offsets = self.wander_offset + self.slide_offset + self.takeoff_drift_offset
        return self.flown_position + np.array([offsets[0], offsets[1], 0.0])

    def tilt_degrees(self):
        return math.degrees(math.atan(np.linalg.norm(self.acceleration) / GRAVITY_METRES_PER_SECOND_SQUARED))

    # ---- Commands ----------------------------------------------------------------------------------

    def arm(self, now_seconds):
        self.armed = True
        self.armed_at_seconds = now_seconds

    def cut_motors(self):
        self.armed = False

    def receive_position_target(self, now_seconds, target_metres, heading_radians):
        self.position_targets.append((now_seconds, np.asarray(target_metres, dtype=float), heading_radians))
        self.speed_target = None

    def receive_speed_target(self, now_seconds, speed_metres_per_second):
        self.speed_target = (now_seconds, np.asarray(speed_metres_per_second, dtype=float))

    # ---- Moving ------------------------------------------------------------------------------------

    def step(self, now_seconds, step_seconds, estimate_error_metres):
        self._take_targets_that_have_arrived(now_seconds)
        self._update_land_detector(now_seconds)
        if not self.armed:
            self.velocity[:] = 0.0
            if self.flown_position[2] > 0.0:      # Motors off in the air: it falls.
                self.flown_position[2] = 0.0
            return
        spinning_up = now_seconds - self.armed_at_seconds < SPIN_UP_SECONDS
        if not spinning_up:
            self._move_up_or_down(now_seconds, step_seconds)
            if self.flown_position[2] > ON_FLOOR_BELOW_METRES:
                self._move_sideways(now_seconds, step_seconds, estimate_error_metres)
        self._turn(now_seconds, step_seconds)
        self._update_wobbles(step_seconds)

    def _take_targets_that_have_arrived(self, now_seconds):
        while self.position_targets and self.position_targets[0][0] <= now_seconds - TARGET_DELAY_SECONDS:
            _sent, self.target_position, heading = self.position_targets.pop(0)
            if heading is not None:
                self.target_heading_history.append((now_seconds, heading))

    def _speed_target_in_force(self, now_seconds):
        if self.speed_target is None:
            return None
        sent_seconds, speed = self.speed_target
        if now_seconds - sent_seconds > SPEED_TARGET_LIFETIME_SECONDS:
            return None
        return speed

    def _move_up_or_down(self, now_seconds, step_seconds):
        speed_target = self._speed_target_in_force(now_seconds)
        if speed_target is not None:
            wanted_upward_speed = speed_target[2]
        elif self.target_position is not None:
            error = self.target_position[2] - HEIGHT_SETTLES_BELOW_TARGET_METRES - self.flown_position[2]
            wanted_upward_speed = UPWARD_SPEED_PER_METRE_OF_ERROR * error
        else:
            wanted_upward_speed = 0.0
        wanted_upward_speed = min(max(wanted_upward_speed, -LARGEST_DOWNWARD_SPEED_METRES_PER_SECOND),
                                  LARGEST_UPWARD_SPEED_METRES_PER_SECOND)
        self.velocity[2] += (wanted_upward_speed - self.velocity[2]) * min(step_seconds / UPWARD_SPEED_LAG_SECONDS, 1.0)
        self.flown_position[2] += self.velocity[2] * step_seconds
        if self.flown_position[2] < 0.0:
            self.flown_position[2] = 0.0
            self.velocity[2] = max(self.velocity[2], 0.0)

    def _move_sideways(self, now_seconds, step_seconds, estimate_error_metres):
        speed_target = self._speed_target_in_force(now_seconds)
        if speed_target is not None:
            wanted_speed = speed_target[:2].copy()
        elif self.target_position is not None:
            px4_estimate = self.true_position()[:2] + estimate_error_metres[:2]
            wanted_speed = SIDEWAYS_SPEED_PER_METRE_OF_ERROR * (self.target_position[:2] - px4_estimate)
        else:
            wanted_speed = np.zeros(2)
        wanted_size = np.linalg.norm(wanted_speed)
        if wanted_size > LARGEST_SIDEWAYS_SPEED_METRES_PER_SECOND:
            wanted_speed *= LARGEST_SIDEWAYS_SPEED_METRES_PER_SECOND / wanted_size
        change = (wanted_speed - self.velocity[:2]) * min(step_seconds / SPEED_LAG_SECONDS, 1.0)
        largest_change = LARGEST_SIDEWAYS_ACCELERATION_METRES_PER_SECOND_SQUARED * step_seconds
        change_size = np.linalg.norm(change)
        if change_size > largest_change:
            change *= largest_change / change_size
        self.acceleration = change / step_seconds
        self.velocity[:2] += change
        self.flown_position[:2] += self.velocity[:2] * step_seconds

    def _turn(self, now_seconds, step_seconds):
        while self.target_heading_history and self.target_heading_history[0][0] <= now_seconds - HEADING_DELAY_SECONDS:
            self.target_heading_radians = self.target_heading_history.pop(0)[1]
        if self.flown_position[2] <= ON_FLOOR_BELOW_METRES:
            self.turn_rate_radians_per_second = 0.0
            return
        difference = math.atan2(math.sin(self.target_heading_radians - self.heading_radians),
                                math.cos(self.target_heading_radians - self.heading_radians))
        largest_step = math.radians(LARGEST_TURN_RATE_DEGREES_PER_SECOND) * step_seconds
        turned = max(-largest_step, min(largest_step, difference))
        self.heading_radians += turned
        self.turn_rate_radians_per_second = turned / step_seconds

    # ---- Wobbles -----------------------------------------------------------------------------------

    def _update_wobbles(self, step_seconds):
        turning = abs(math.degrees(self.turn_rate_radians_per_second)) > TURNING_IS_FASTER_THAN_DEGREES_PER_SECOND
        self._update_wander(step_seconds, turning)
        self._update_slide(step_seconds, turning)
        self._update_takeoff_drift(step_seconds)

    def _update_wander(self, step_seconds, turning):
        size = TURNING_WANDER_METRES if turning else HOVER_WANDER_METRES
        pull_back = step_seconds / WANDER_CORRELATION_SECONDS
        # The usual Ornstein-Uhlenbeck step: the random kick is sized so the spread stays "size".
        kick = size * math.sqrt(2.0 * pull_back)
        self.wander_offset += -self.wander_offset * pull_back + self.random.normal(0.0, kick, size=2)

    def _update_slide(self, step_seconds, turning):
        if not turning:
            self.slide_direction[:] = 0.0
            self.slide_offset -= self.slide_offset * step_seconds / OFFSET_RECOVERY_SECONDS
            return
        if not self.slide_direction.any():
            angle = self.random.uniform(0.0, 2.0 * math.pi)
            moving = np.linalg.norm(self.velocity[:2]) > MOVING_IS_FASTER_THAN_METRES_PER_SECOND
            slide_per_80_degrees = (SLIDE_PER_80_DEGREES_WHILE_MOVING_METRES if moving
                                    else self.random.uniform(*SLIDE_PER_80_DEGREES_FROM_REST_METRES))
            self.slide_direction = slide_per_80_degrees / math.radians(80.0) * np.array([math.cos(angle), math.sin(angle)])
        turned_radians = abs(self.turn_rate_radians_per_second) * step_seconds
        self.slide_offset += self.slide_direction * turned_radians

    def _update_takeoff_drift(self, step_seconds):
        height = self.flown_position[2]
        if not self.has_reached_takeoff_peak:
            self.takeoff_drift_offset = self.takeoff_drift_full * min(height / TAKEOFF_DRIFT_PEAKS_AT_HEIGHT_METRES, 1.0)
            self.has_reached_takeoff_peak = height >= TAKEOFF_DRIFT_PEAKS_AT_HEIGHT_METRES
            return
        self.takeoff_drift_offset -= self.takeoff_drift_offset * step_seconds / OFFSET_RECOVERY_SECONDS

    # ---- PX4's land detector -----------------------------------------------------------------------

    def _update_land_detector(self, now_seconds):
        on_floor = self.flown_position[2] <= ON_FLOOR_BELOW_METRES
        if not on_floor:
            self.on_floor_since_seconds = None
            self.landed_since_seconds = None
            self.landed = False
            return
        if not self.armed:
            self.landed = True
            return
        pushing_down = self._is_being_pushed_down(now_seconds)
        if self.on_floor_since_seconds is None or not pushing_down:
            self.on_floor_since_seconds = now_seconds if pushing_down else None
            return
        if now_seconds - self.on_floor_since_seconds >= SECONDS_ON_FLOOR_BEFORE_LANDED:
            self.landed = True
            if self.landed_since_seconds is None:
                self.landed_since_seconds = now_seconds
            if now_seconds - self.landed_since_seconds >= SECONDS_LANDED_BEFORE_DISARM:
                self.armed = False

    def _is_being_pushed_down(self, now_seconds):
        if self.armed_at_seconds is None or now_seconds - self.armed_at_seconds < SPIN_UP_SECONDS + 3.0:
            return False      # Just armed on the floor: not a landing.
        speed_target = self._speed_target_in_force(now_seconds)
        if speed_target is not None:
            return speed_target[2] < 0.0
        return self.target_position is not None and self.target_position[2] <= HEIGHT_SETTLES_BELOW_TARGET_METRES


def print_step_response():
    """Calibration: prints the pretend drone's 1 m sideways step and an 80 degree turn's slide,
    next to the measured numbers. Run: python pretend_drone.py"""
    step_seconds = 0.02
    drone = PretendDrone(np.random.default_rng(1))
    drone.flown_position[:] = (0.0, 0.0, 0.8)
    drone.has_reached_takeoff_peak = True
    drone.armed, drone.armed_at_seconds = True, -100.0
    drone.target_position = np.array([0.0, 0.0, 0.8 + HEIGHT_SETTLES_BELOW_TARGET_METRES])
    drone.receive_position_target(0.0, (1.0, 0.0, 0.8 + HEIGHT_SETTLES_BELOW_TARGET_METRES), None)
    reached_63_at, largest = None, 0.0
    for step_number in range(int(8.0 / step_seconds)):
        now = step_number * step_seconds
        drone.step(now, step_seconds, np.zeros(3))
        drone.wander_offset[:] = 0.0
        forward = drone.true_position()[0]
        largest = max(largest, forward)
        if reached_63_at is None and forward >= 0.63:
            reached_63_at = now
    print(f"1 m step: t63 {reached_63_at:.2f} s (measured about 1.4 s), overshoot {100 * (largest - 1.0):.1f} cm "
          f"(measured at most 4 cm)")
    for _ in range(3):
        drone.receive_position_target(10.0, (1.0, 0.0, 0.83), math.radians(80.0))
        largest_slide = 0.0
        for step_number in range(int(10.0 / step_seconds)):
            drone.step(10.0 + step_number * step_seconds, step_seconds, np.zeros(3))
            largest_slide = max(largest_slide, np.linalg.norm(drone.slide_offset))
        print(f"80 degree turn from rest: slide "
              f"{largest_slide:.3f} m (measured 0.08-0.12 m)")
        drone.heading_radians = 0.0
        drone.slide_offset[:] = 0.0


if __name__ == "__main__":
    print_step_response()
