"""Watches for the trouble a boulder under the drone can cause, for the referee. SIMULATOR ONLY.

A boulder under the bottom laser makes the laser read the boulder's top instead of the floor,
so PX4's height estimate can go wrong (docs/ARCHITECTURE.md, section 7). The sudden jump in the
reading upsets PX4's laser check for a few seconds. If the drone moves sideways slower than
about 0.32 m/s at any moment in that time, PX4 stops using the laser for height, usually for
the rest of the flight, and the height estimate drifts. The window's lower wall does the same.
So this watches two things during the flight:

  - Each time the drone's centre is above a boulder: for how long, and how slowly it moved
    sideways there.
  - How far PX4's height estimate is from the true height that the physics reports.

The referee (referee.py) calls check() at every sample, and puts summary() into its summary.
run_test_flight.sh fails the flight if the drone stayed above a boulder too long, or if the
height estimate went too far wrong.

"Above a boulder" uses the drone's centre. The pretend laser (pretend_bottom_laser.py) shoots a
single thin ray 2.7 cm to the left of the centre, which is close enough. The boulders are the
world's, which is what is really there, not the drone's map.
"""
import math

from px4_msgs.msg import VehicleLocalPosition

from course_simulator.true_drone_state import PX4_MESSAGE_SETTINGS

# Our own choice. Crossing a 46 cm boulder at 0.4 m/s takes about 1.2 seconds. Longer means
# the drone slowed down or stopped above it, which is when PX4 is most likely to stop using the
# laser (docs/ARCHITECTURE.md, section 7). The height check below catches the result.
LONGEST_ACCEPTABLE_TIME_ABOVE_A_BOULDER_SECONDS = 2.0

# Our own choice. In the window on our scan, the drone's body has about 17 cm to spare above
# and below, so the height estimate must be well within that.
LARGEST_ACCEPTABLE_HEIGHT_ERROR_METRES = 0.10


class BoulderWatch:

    def __init__(self, ros_node, world_map):
        self.boulders = list(world_map.low_obstacles)
        self.px4_estimated_height_metres = None
        ros_node.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self._on_px4_estimate, PX4_MESSAGE_SETTINGS,
        )
        # PX4 counts height from wherever its estimate started, which isn't exactly the floor.
        # So we note the difference while the drone stands on the launch mark, and take it off.
        self.estimate_offset_on_the_ground_metres = None
        self.largest_height_error_metres = 0.0
        self.largest_height_error_at_seconds = None

        self.previous_position = None
        self.previous_seconds = None
        # Each time above a boulder: [boulder name, seconds since takeoff when it began, seconds
        # above it, slowest sideways speed there in m/s].
        self.times_above_boulders = []
        self.boulder_below_now = None

    def _on_px4_estimate(self, message):
        if message.z_valid:
            self.px4_estimated_height_metres = -float(message.z)  # PX4's z points downward.

    def check(self, true_position_metres, now_seconds, seconds_since_takeoff):
        """seconds_since_takeoff is None before takeoff and after the motors have stopped."""
        sideways_speed = self._sideways_speed_metres_per_second(true_position_metres, now_seconds)
        self._check_height_estimate(true_position_metres, seconds_since_takeoff)
        self._check_boulder_below(true_position_metres, seconds_since_takeoff, sideways_speed)

    def summary(self):
        times_above = []
        for boulder_name, began_at_seconds, seconds_above, slowest_speed in self.times_above_boulders:
            if math.isinf(slowest_speed):
                slowest_speed = None   # Above it for a single sample only, so no speed was measured.
            else:
                slowest_speed = round(slowest_speed, 2)
            times_above.append({
                "boulder": boulder_name,
                "seconds_after_takeoff": round(began_at_seconds, 1),
                "seconds_above_it": round(seconds_above, 2),
                "slowest_sideways_speed_metres_per_second": slowest_speed,
            })
        longest_seconds = max([entry["seconds_above_it"] for entry in times_above], default=0.0)
        return {
            "times_above_boulders": times_above,
            "stayed_above_a_boulder_too_long": longest_seconds > LONGEST_ACCEPTABLE_TIME_ABOVE_A_BOULDER_SECONDS,
            "largest_height_estimate_error_metres": round(self.largest_height_error_metres, 3),
            "largest_height_estimate_error_at_seconds": self.largest_height_error_at_seconds,
            "height_estimate_went_wrong": self.largest_height_error_metres > LARGEST_ACCEPTABLE_HEIGHT_ERROR_METRES,
        }

    def _sideways_speed_metres_per_second(self, true_position_metres, now_seconds):
        speed = None
        if self.previous_position is not None and now_seconds > self.previous_seconds:
            moved_metres = math.hypot(
                true_position_metres[0] - self.previous_position[0], true_position_metres[1] - self.previous_position[1],
            )
            speed = moved_metres / (now_seconds - self.previous_seconds)
        self.previous_position = true_position_metres
        self.previous_seconds = now_seconds
        return speed

    def _check_height_estimate(self, true_position_metres, seconds_since_takeoff):
        if self.px4_estimated_height_metres is None:
            return
        difference_metres = self.px4_estimated_height_metres - float(true_position_metres[2])
        if seconds_since_takeoff is None:
            if self.largest_height_error_at_seconds is None:
                self.estimate_offset_on_the_ground_metres = difference_metres
            return
        if self.estimate_offset_on_the_ground_metres is None:
            return
        error_metres = abs(difference_metres - self.estimate_offset_on_the_ground_metres)
        if error_metres > self.largest_height_error_metres:
            self.largest_height_error_metres = error_metres
            self.largest_height_error_at_seconds = round(seconds_since_takeoff, 1)

    def _check_boulder_below(self, true_position_metres, seconds_since_takeoff, sideways_speed):
        boulder_name = None
        if seconds_since_takeoff is not None:
            boulder_name = self._boulder_below(true_position_metres)
        if boulder_name is None:
            self.boulder_below_now = None
            return
        if boulder_name != self.boulder_below_now:
            self.times_above_boulders.append([boulder_name, seconds_since_takeoff, 0.0, math.inf])
            self.boulder_below_now = boulder_name
            self.began_above_at_seconds = seconds_since_takeoff
        current = self.times_above_boulders[-1]
        current[2] = seconds_since_takeoff - self.began_above_at_seconds
        if sideways_speed is not None:
            current[3] = min(current[3], sideways_speed)

    def _boulder_below(self, true_position_metres):
        for boulder in self.boulders:
            distance_metres = math.hypot(
                true_position_metres[0] - boulder.centre_metres[0], true_position_metres[1] - boulder.centre_metres[1],
            )
            if distance_metres < boulder.radius_metres:
                return boulder.name
        return None
