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

The flight's three parts, for the height estimate
-------------------------------------------------
  1. The takeoff climb: from takeoff until the drone first moves sideways, more than
     TAKEOFF_CLIMB_ENDS_AFTER_SIDEWAYS_METRES from where it took off, or until
     TAKEOFF_CLIMB_LONGEST_SECONDS have passed, whichever comes first. PX4's estimate trails the
     true height here: in every simulator flight on 2026-11-07 it fell up to 12 cm behind
     while climbing, about 4 s after takeoff, so the drone overshot its cruise height by about
     7 cm, and the gap was gone about 3 s after the climb ended. This is real, not a
     measuring mistake of ours: PX4 really does think the drone is lower than it is. It is
     harmless, because the drone is climbing straight up above the launch mark, with a metre of
     open air above it, and our program only moves sideways after it has held still at its
     cruise height for 2 s. It gets its own, looser limit, so that a climb that really goes
     wrong (a 52 cm error in one flight with drifting camera height) still fails the test.
  2. The flight itself, which keeps the 10 cm limit.
  3. On the floor after touchdown: not counted. While the drone presses down on the floor to
     make PX4 switch the motors off, PX4's estimate drifts up to 45 cm away from the true
     height. That is harmless, because the drone is already down. The largest error there is
     still reported, for information. "On the floor" means back within
     ON_THE_FLOOR_HEIGHT_TOLERANCE_METRES of the launch mark's height, after having been well
     above it. A drone that sinks onto a boulder is not on the floor, so that still counts.

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

# Our own choice. Straight above the launch mark nothing is within a metre above the drone, so
# a larger error is safe there. It is about 1.7 times the largest seen in a normal climb (12 cm).
LARGEST_ACCEPTABLE_HEIGHT_ERROR_DURING_TAKEOFF_CLIMB_METRES = 0.20
# Our own choice. Camera tracking drifts a few centimetres while the drone climbs, so the drone
# may wander that far without having started the route. The route's first move is longer.
TAKEOFF_CLIMB_ENDS_AFTER_SIDEWAYS_METRES = 0.30
# Our own choice. In every flight the climb's error had gone by about 7 s after takeoff. The cap
# keeps flights that never move sideways, such as rise-and-land, under the normal limit after it.
TAKEOFF_CLIMB_LONGEST_SECONDS = 10.0

# Our own choice. The drone counts as standing on the floor when its true height is within this
# of what it was on the launch mark, once it has been more than WELL_ABOVE_THE_FLOOR_METRES
# above it. Standing still, the physics' height varies by under 1 cm.
ON_THE_FLOOR_HEIGHT_TOLERANCE_METRES = 0.02
WELL_ABOVE_THE_FLOOR_METRES = 0.20


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
        # Where the drone stood on the launch mark: (east, north, height) in metres.
        self.launch_spot_metres = None
        self.has_taken_off = False
        self.has_been_well_above_the_floor = False
        self.takeoff_climb_is_over = False
        # The largest error in each part of the flight, and when (seconds after takeoff).
        self.largest_height_error_metres = 0.0
        self.largest_height_error_at_seconds = None
        self.largest_takeoff_climb_error_metres = 0.0
        self.largest_takeoff_climb_error_at_seconds = None
        self.largest_error_on_the_floor_metres = 0.0

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
            "largest_height_estimate_error_during_takeoff_climb_metres": round(self.largest_takeoff_climb_error_metres, 3),
            "largest_height_estimate_error_during_takeoff_climb_at_seconds": self.largest_takeoff_climb_error_at_seconds,
            "largest_height_estimate_error_on_the_floor_after_touchdown_metres":
                round(self.largest_error_on_the_floor_metres, 3),
            "height_estimate_went_wrong": self._height_estimate_went_wrong(),
        }

    def _height_estimate_went_wrong(self):
        flight_went_wrong = self.largest_height_error_metres > LARGEST_ACCEPTABLE_HEIGHT_ERROR_METRES
        climb_went_wrong = self.largest_takeoff_climb_error_metres > LARGEST_ACCEPTABLE_HEIGHT_ERROR_DURING_TAKEOFF_CLIMB_METRES
        return flight_went_wrong or climb_went_wrong

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
            # Before takeoff, or after the motors have stopped.
            if not self.has_taken_off:
                self._note_the_launch_mark(true_position_metres, difference_metres)
            return
        self.has_taken_off = True
        if self.estimate_offset_on_the_ground_metres is None:
            return
        error_metres = abs(difference_metres - self.estimate_offset_on_the_ground_metres)
        when_seconds = round(seconds_since_takeoff, 1)
        if self._is_on_the_floor_after_flying(true_position_metres):
            self.largest_error_on_the_floor_metres = max(self.largest_error_on_the_floor_metres, error_metres)
            return
        self._end_the_takeoff_climb_when_its_time(true_position_metres, seconds_since_takeoff)
        if not self.takeoff_climb_is_over:
            if error_metres > self.largest_takeoff_climb_error_metres:
                self.largest_takeoff_climb_error_metres = error_metres
                self.largest_takeoff_climb_error_at_seconds = when_seconds
            return
        if error_metres > self.largest_height_error_metres:
            self.largest_height_error_metres = error_metres
            self.largest_height_error_at_seconds = when_seconds

    def _note_the_launch_mark(self, true_position_metres, difference_metres):
        self.estimate_offset_on_the_ground_metres = difference_metres
        self.launch_spot_metres = [float(value) for value in true_position_metres[:3]]

    def _is_on_the_floor_after_flying(self, true_position_metres):
        """True while the drone stands on the floor after having flown. Just after takeoff it
        has not yet been well above the floor, so leaving the launch mark counts as the climb."""
        height_above_launch_mark_metres = float(true_position_metres[2]) - self.launch_spot_metres[2]
        if height_above_launch_mark_metres > WELL_ABOVE_THE_FLOOR_METRES:
            self.has_been_well_above_the_floor = True
        return self.has_been_well_above_the_floor and height_above_launch_mark_metres < ON_THE_FLOOR_HEIGHT_TOLERANCE_METRES

    def _end_the_takeoff_climb_when_its_time(self, true_position_metres, seconds_since_takeoff):
        sideways_metres = math.hypot(true_position_metres[0] - self.launch_spot_metres[0],
                                     true_position_metres[1] - self.launch_spot_metres[1])
        moved_sideways = sideways_metres > TAKEOFF_CLIMB_ENDS_AFTER_SIDEWAYS_METRES
        if moved_sideways or seconds_since_takeoff > TAKEOFF_CLIMB_LONGEST_SECONDS:
            self.takeoff_climb_is_over = True

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
