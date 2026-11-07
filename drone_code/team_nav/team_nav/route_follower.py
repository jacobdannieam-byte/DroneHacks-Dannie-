"""Works out where the drone should aim next, as it flies along a planned route.

This runs on the drone and in the simulator. It is plain arithmetic, with no ROS or PX4. The
flight program gives it the drone's position about 10 times per second, and sends PX4 the
target position it returns. Positions are on the course map's directions (start_frame.py
converts to and from the camera tracking's).

How it works
------------
PX4 flies the drone towards a target position. The further away the target, the faster it
flies: in the simulator, about 0.77 m/s sideways for every metre between them (section 9 of
docs/ARCHITECTURE.md). So we never give PX4 the far end of the route. Instead, a target point
slides along the route ahead of the drone, like a carrot on a stick, at the cruising speed.
Two rules keep the drone on the planned line:

  - The target never gets more than TARGET_LEAD_METRES ahead of the drone, measured sideways
    (seen from above). If the drone falls behind, the target waits for it. This lead also sets
    the drone's speed: 0.77 x 0.65 m is about 0.5 m/s.
  - The target stops at each corner of the route until the drone is there: within
    CORNER_TOLERANCE_METRES sideways and CORNER_HEIGHT_TOLERANCE_METRES in height. Otherwise
    the drone, which trails its target, would cut the corner and could clip a wall the planner
    went round.

Height is left out of the lead because a climb should not slow the drone down sideways. It has
its own, looser tolerance at corners because the camera tracking's height and PX4's height
can differ by a few centimetres, and PX4 flies to its own. With a single 5 cm tolerance in all
three directions, a simulator flight hovered for seconds at corners waiting for them to agree.

A waypoint where the route carries on in nearly the same direction, seen from above, is not a
corner, and the target slides straight past it. This matters most at the window. The window
crossing is four waypoints in a straight line (opening_crossings.py), and the drone must fly
over the window's bottom edge without slowing down. When the bottom laser passes over that
edge, its reading jumps, and for a few seconds afterwards PX4 stops trusting the laser if the
drone moves sideways slower than about 0.32 m/s (docs/ARCHITECTURE.md, section 7). A simulator
flight that paused just before and just after the window's bottom edge lost its height
estimate by up to half a metre.

So the drone pauses briefly at every real corner. The planner keeps corners off boulders where
it can, but the drone's map may be wrong, so a corner can still be above a boulder nobody knew
about. That is one of the things the simulator's boulder flights test.

What it does not do
-------------------
It does not check for walls or boulders; the route planner already did. It does not take off
or land; the flight program does those before and after.
"""
import numpy as np

# Our own choice, using a measurement. In a simulator flight on 2026-11-07, PX4 flew the drone
# at about 0.77 m/s sideways for each metre between it and its target, not the 0.95 that its
# setting MPC_XY_P suggests. With a 0.5 m lead the drone only reached about 0.33 m/s, which is
# too close to PX4's 0.32 m/s limit for the laser (section 7). 0.65 m gives about 0.5 m/s.
TARGET_LEAD_METRES = 0.65
# Our own choice. The target slides at this speed while it is allowed to move.
CRUISE_SPEED_METRES_PER_SECOND = 0.5
# Our own choice. The planner leaves 5 cm to spare beyond the propeller tips, so the drone may
# cut a corner by at most about this much.
CORNER_TOLERANCE_METRES = 0.05
# Our own choice. The camera tracking's height and PX4's height can differ by this much.
CORNER_HEIGHT_TOLERANCE_METRES = 0.10
# Our own choice. A waypoint where the route turns less than this, seen from above, is passed
# without stopping. test_route_follower.py checks that the drone cuts such a turn by less than
# CORNER_TOLERANCE_METRES.
STRAIGHT_ON_LARGEST_TURN_DEGREES = 10.0


class RouteFollower:

    def __init__(self, waypoints_metres):
        """waypoints_metres: the planned route, one (east, north, height) row per waypoint."""
        self.waypoints_metres = np.asarray(waypoints_metres, dtype=float)
        self.target_metres = self.waypoints_metres[0].copy()
        # The waypoint the target is sliding towards.
        self.next_waypoint_number = 1
        self.is_corner = _find_corners(self.waypoints_metres)

    def has_finished(self, drone_position_metres):
        """True once the target has reached the last waypoint and the drone is there too."""
        at_last_waypoint = self.next_waypoint_number >= len(self.waypoints_metres)
        return at_last_waypoint and self._drone_is_at_target(drone_position_metres)

    def next_target(self, drone_position_metres, seconds_since_last_target):
        """Moves the target on, if the rules allow, and returns it as (east, north, height)."""
        self._move_past_waypoint_if_allowed(drone_position_metres)
        if self.next_waypoint_number >= len(self.waypoints_metres):
            return self.target_metres.copy()
        if self._sideways_distance_to_target_metres(drone_position_metres) >= TARGET_LEAD_METRES:
            return self.target_metres.copy()

        distance_to_slide_metres = CRUISE_SPEED_METRES_PER_SECOND * seconds_since_last_target
        next_waypoint = self.waypoints_metres[self.next_waypoint_number]
        distance_to_waypoint_metres = float(np.linalg.norm(next_waypoint - self.target_metres))
        if distance_to_slide_metres >= distance_to_waypoint_metres:
            # Stop exactly at the waypoint. The next call decides whether to move past it.
            self.target_metres = next_waypoint.copy()
        else:
            direction = (next_waypoint - self.target_metres) / distance_to_waypoint_metres
            self.target_metres = self.target_metres + direction * distance_to_slide_metres
        return self.target_metres.copy()

    def _move_past_waypoint_if_allowed(self, drone_position_metres):
        if self.next_waypoint_number >= len(self.waypoints_metres):
            return
        target_is_at_waypoint = np.allclose(self.target_metres, self.waypoints_metres[self.next_waypoint_number])
        if not target_is_at_waypoint:
            return
        straight_on = not self.is_corner[self.next_waypoint_number]
        if straight_on or self._drone_is_at_target(drone_position_metres):
            self.next_waypoint_number += 1

    def _drone_is_at_target(self, drone_position_metres):
        drone_position_metres = np.asarray(drone_position_metres, dtype=float)
        height_difference_metres = abs(drone_position_metres[2] - self.target_metres[2])
        return (self._sideways_distance_to_target_metres(drone_position_metres) < CORNER_TOLERANCE_METRES
                and height_difference_metres < CORNER_HEIGHT_TOLERANCE_METRES)

    def _sideways_distance_to_target_metres(self, drone_position_metres):
        """The distance from the drone to the target seen from above, leaving out height."""
        drone_position_metres = np.asarray(drone_position_metres, dtype=float)
        return float(np.linalg.norm(drone_position_metres[:2] - self.target_metres[:2]))


def _find_corners(waypoints_metres):
    """For each waypoint, True if the drone must stop there: the last waypoint, and every one
    where the route turns by more than STRAIGHT_ON_LARGEST_TURN_DEGREES seen from above. A leg
    that only climbs or descends, with no sideways part, also counts as a turn."""
    is_corner = [True] * len(waypoints_metres)
    for number in range(1, len(waypoints_metres) - 1):
        arriving = waypoints_metres[number][:2] - waypoints_metres[number - 1][:2]
        leaving = waypoints_metres[number + 1][:2] - waypoints_metres[number][:2]
        arriving_length, leaving_length = np.linalg.norm(arriving), np.linalg.norm(leaving)
        if arriving_length < CORNER_TOLERANCE_METRES or leaving_length < CORNER_TOLERANCE_METRES:
            continue
        cosine_of_turn = float(arriving @ leaving) / (arriving_length * leaving_length)
        turn_degrees = np.degrees(np.arccos(np.clip(cosine_of_turn, -1.0, 1.0)))
        is_corner[number] = turn_degrees > STRAIGHT_ON_LARGEST_TURN_DEGREES
    return is_corner
