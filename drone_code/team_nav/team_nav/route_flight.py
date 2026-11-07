"""The route mission: fly a list of waypoints, one leg at a time, each leg at its own height.

How it fits: mission.py uses this instead of course_stages.py when launched with mission:=route.
Once the drone is at cruise height, mission.py calls step twenty times a second; step answers with
a position target, a short reason, and whether the route is finished (then mission.py lands). The
position watchdog and every other rule in mission.py stay active the whole time. This file never
talks to ROS and never reads a clock.

The waypoints come from a data file (missions/route_waypoints.json), not from code: the judges take
points off for routes written into code. Each waypoint is (forward, left) in metres from the
takeoff spot, with "forward" being where the nose pointed at takeoff. A waypoint may carry a third
number, a height in metres above the launch floor: the leg into that waypoint is flown at that
height (for example through the window). Legs into waypoints without one are flown at cruise height.

Each leg: a target (the "carrot") moves along the straight line to the next waypoint at
route_speed_metres_per_second, never more than the carrot lead ahead of the drone.

At each waypoint, what happens depends on how much the next leg turns:
* By route_fly_through_degrees or more (and at the takeoff spot and the last waypoint): the carrot
  stops at the waypoint, the leg ends once the drone is within route_arrive_metres of it, and then
     1. stop: hold the corner until the drone is slower than settled_speed_metres_per_second (or for
        route_corner_settle_time_limit_seconds at most);
     2. turn in place to face the next waypoint, never faster than route_turn_rate_degrees_per_second;
     3. if the next leg is flown at a different height, change height in place, never faster than
        climb_speed_metres_per_second, until the drone is within route_height_tolerance_metres of it.
        Height changes only here, before a leg, never during one.
  The route also tells mission.py when the drone has stopped at a corner and has not turned yet
  (RouteDecision.stopped_at_corner): the moment for the drift correction (drift_correction.py).
* By less, and the next leg is at the same height: fly through. The carrot carries straight on
  along the next leg as soon as it reaches the waypoint, without stopping, and the heading turns
  toward the new leg at the same turn rate while flying.
* Whatever the turn: the waypoints in must_fly_through (laser_rule.py, the bottom laser rule) are
  always flown through, at the route speed. If the next leg is at another height, the height then
  changes while flying that leg, at climb speed, because stopping is what the rule forbids.
After the last waypoint the drone stops once more, and the route is complete.

What it deliberately does not do: look for obstacles. The route is assumed clear; only the
watchdog and the other flight rules guard it.
"""
import math
from dataclasses import dataclass

from .course_stages import body_to_world, limit, wrap_angle


@dataclass
class RouteDecision:
    reason: str
    setpoint: tuple                   # (x, y, z, heading)
    complete: bool = False
    stopped_at_corner: bool = False   # True on the one step the drone has stopped and not yet turned


def split_waypoints(waypoints):
    """[(forward, left) or (forward, left, height), ...] -> [(forward, left)], [height or None]."""
    places, heights = [], []
    for number, waypoint in enumerate(waypoints, start=1):
        if len(waypoint) not in (2, 3):
            raise ValueError(f"Waypoint {number} of the route has {len(waypoint)} numbers; it needs 2 (forward, left) "
                             "or 3 (forward, left, height). Fix the route file.")
        places.append((float(waypoint[0]), float(waypoint[1])))
        heights.append(float(waypoint[2]) if len(waypoint) == 3 else None)
    return places, heights


class RouteFlight:
    def __init__(self, settings, waypoints_forward_left_metres, must_fly_through=None):
        """must_fly_through: {waypoint index: why}, waypoints never to stop at (laser_rule.py)."""
        if not waypoints_forward_left_metres:
            raise ValueError("The route has no waypoints. Check the route file given by the route_file parameter.")
        self.settings = settings
        self.waypoints_forward_left_metres, self.heights_above_floor_metres = split_waypoints(waypoints_forward_left_metres)
        self.must_fly_through = dict(must_fly_through or {})
        self.waypoints = []           # the same waypoints in the camera tracking frame (x, y)
        self.leg_heights_metres = []  # the height (camera tracking z) of the leg into each waypoint
        self.home = None              # (x, y) of the takeoff spot
        self.height_metres = None     # the height (camera tracking z) targeted now
        self.ground_height_metres = None
        self.height_target_reached_seconds = None
        self.next_waypoint = 0        # index of the waypoint being flown to
        self.corner = None            # (x, y) the drone stops and turns at
        self.step_name = None         # "stopping", "turning", "changing_height" or "flying"
        self.step_start_seconds = None
        self.setpoint_heading_radians = None
        self.distance_along_leg_metres = 0.0
        self.reached = 0              # how many waypoints have been reached

    def start(self, now_seconds, cruise_setpoint, ground_height_metres):
        """cruise_setpoint = (x, y, z, heading) over the takeoff spot, nose as it was at takeoff;
        ground_height_metres is camera tracking's z on the launch mark."""
        home_x, home_y, cruise_z, heading = cruise_setpoint
        self.waypoints = [body_to_world((home_x, home_y), heading, forward, left)
                          for forward, left in self.waypoints_forward_left_metres]
        self.leg_heights_metres = []
        for height_above_floor in self.heights_above_floor_metres:
            if height_above_floor is None:
                self.leg_heights_metres.append(cruise_z)
            else:
                self.leg_heights_metres.append(ground_height_metres + height_above_floor)
        self.height_metres = cruise_z
        self.ground_height_metres = ground_height_metres
        self.home = (home_x, home_y)
        self.corner = (home_x, home_y)
        self.setpoint_heading_radians = heading
        self.go_to("stopping", now_seconds)

    def go_to(self, step_name, now_seconds):
        self.step_name = step_name
        self.step_start_seconds = now_seconds

    def waypoint_name(self, index):
        return f"waypoint {index + 1} of {len(self.waypoints)}"

    def step(self, now_seconds, seconds_since_last_step, position, heading_radians, horizontal_speed):
        if self.step_name == "stopping":
            return self.stop_at_corner(now_seconds, horizontal_speed)
        if self.step_name == "turning":
            return self.turn_in_place(now_seconds, seconds_since_last_step, heading_radians)
        if self.step_name == "changing_height":
            return self.change_height(now_seconds, seconds_since_last_step, position)
        return self.fly_leg(now_seconds, seconds_since_last_step, position)

    def corner_name(self):
        return "the takeoff spot" if self.reached == 0 else self.waypoint_name(self.reached - 1)

    def corner_setpoint(self):
        return (self.corner[0], self.corner[1], self.height_metres, self.setpoint_heading_radians)

    # ------------------------------------------------------------------ the steps at a corner
    def stop_at_corner(self, now_seconds, horizontal_speed):
        settings = self.settings
        where = self.corner_name()
        stopped = horizontal_speed is not None and horizontal_speed < settings.settled_speed_metres_per_second
        waited_long_enough = now_seconds - self.step_start_seconds >= settings.route_corner_settle_time_limit_seconds
        if not (stopped or waited_long_enough):
            return RouteDecision(f"stopping at {where}", self.corner_setpoint())
        if self.reached == len(self.waypoints):
            return RouteDecision(f"route complete: reached {self.waypoint_name(len(self.waypoints) - 1)}",
                                 self.corner_setpoint(), complete=True)
        self.go_to("turning", now_seconds)
        return RouteDecision(f"turning to face {self.waypoint_name(self.next_waypoint)}", self.corner_setpoint(),
                             stopped_at_corner=True)

    def leg_heading(self):
        target = self.waypoints[self.next_waypoint]
        return math.atan2(target[1] - self.corner[1], target[0] - self.corner[0])

    def turn_setpoint_heading_toward(self, wanted_radians, seconds_since_last_step):
        largest_turn = math.radians(self.settings.route_turn_rate_degrees_per_second) * seconds_since_last_step
        error = wrap_angle(wanted_radians - self.setpoint_heading_radians)
        self.setpoint_heading_radians = wrap_angle(self.setpoint_heading_radians + limit(error, largest_turn))

    def flies_through(self, index):
        """Whether waypoint index is passed without stopping: never the last one; always one the laser
        rule names; otherwise when the next leg is at the same height and turns by less than
        route_fly_through_degrees."""
        if index >= len(self.waypoints) - 1:
            return False
        if index in self.must_fly_through:
            return True
        return self.flies_through_by_the_turn(index)

    def flies_through_by_the_turn(self, index):
        """The usual rule alone, without the laser rule."""
        if index >= len(self.waypoints) - 1:
            return False
        if self.leg_heights_metres[index + 1] != self.leg_heights_metres[index]:
            return False
        before = self.home if index == 0 else self.waypoints[index - 1]
        here, after = self.waypoints[index], self.waypoints[index + 1]
        heading_in = math.atan2(here[1] - before[1], here[0] - before[0])
        heading_out = math.atan2(after[1] - here[1], after[0] - here[0])
        return abs(wrap_angle(heading_out - heading_in)) < math.radians(self.settings.route_fly_through_degrees)

    def turn_in_place(self, now_seconds, seconds_since_last_step, heading_radians):
        settings = self.settings
        wanted = self.leg_heading()
        self.turn_setpoint_heading_toward(wanted, seconds_since_last_step)
        target_reached = abs(wrap_angle(wanted - self.setpoint_heading_radians)) < 1e-6
        nose_there = abs(wrap_angle(wanted - heading_radians)) < math.radians(settings.route_heading_tolerance_degrees)
        if target_reached and nose_there:
            return self.after_turning(now_seconds)
        return RouteDecision(f"turning to face {self.waypoint_name(self.next_waypoint)}", self.corner_setpoint())

    def after_turning(self, now_seconds):
        """Start the leg, or first change height if the leg is flown at a different height."""
        wanted = self.leg_heights_metres[self.next_waypoint]
        if abs(wanted - self.height_metres) < 1e-9:
            return self.start_leg(now_seconds)
        self.height_target_reached_seconds = None
        self.go_to("changing_height", now_seconds)
        return self.changing_height_decision(wanted)

    def start_leg(self, now_seconds):
        self.distance_along_leg_metres = 0.0
        self.go_to("flying", now_seconds)
        return RouteDecision(f"flying to {self.waypoint_name(self.next_waypoint)}", self.corner_setpoint())

    def change_height(self, now_seconds, seconds_since_last_step, position):
        """Move the target up or down to the next leg's height at climb speed; start the leg once
        the drone is within route_height_tolerance_metres of it, or once it has had
        route_height_settle_time_limit_seconds to get there."""
        settings = self.settings
        wanted = self.leg_heights_metres[self.next_waypoint]
        largest_change = settings.climb_speed_metres_per_second * seconds_since_last_step
        self.height_metres += limit(wanted - self.height_metres, largest_change)
        if abs(wanted - self.height_metres) > 1e-9:
            self.height_target_reached_seconds = None
            return self.changing_height_decision(wanted)
        if self.height_target_reached_seconds is None:
            self.height_target_reached_seconds = now_seconds
        drone_there = abs(position[2] - wanted) < settings.route_height_tolerance_metres
        waited = now_seconds - self.height_target_reached_seconds
        if not (drone_there or waited >= settings.route_height_settle_time_limit_seconds):
            return self.changing_height_decision(wanted)
        return self.start_leg(now_seconds)

    def changing_height_decision(self, wanted_metres):
        above_floor = wanted_metres - self.ground_height_metres
        return RouteDecision(f"changing height to {above_floor:.2f} m before {self.waypoint_name(self.next_waypoint)}",
                             self.corner_setpoint())

    def fly_leg(self, now_seconds, seconds_since_last_step, position):
        settings = self.settings
        target = self.waypoints[self.next_waypoint]
        leg_east, leg_north = target[0] - self.corner[0], target[1] - self.corner[1]
        leg_length = math.hypot(leg_east, leg_north)
        along_east, along_north = (leg_east / leg_length, leg_north / leg_length) if leg_length > 1e-9 else (0.0, 0.0)
        # After flying through a waypoint the nose still points along the previous leg: blend it round.
        self.turn_setpoint_heading_toward(math.atan2(leg_north, leg_east), seconds_since_last_step)
        # Normally the height already is the leg's (changed at the corner); after a waypoint the
        # laser rule made us fly through, it changes here, at climb speed.
        wanted_height = self.leg_heights_metres[self.next_waypoint]
        self.height_metres += limit(wanted_height - self.height_metres,
                                    settings.climb_speed_metres_per_second * seconds_since_last_step)
        # How far along the leg the drone itself is: the carrot never leads it by more than the lead.
        drone_along = (position[0] - self.corner[0]) * along_east + (position[1] - self.corner[1]) * along_north
        self.distance_along_leg_metres = min(
            self.distance_along_leg_metres + settings.route_speed_metres_per_second * seconds_since_last_step,
            leg_length, max(drone_along, 0.0) + settings.carrot_lead_metres)
        carrot = (self.corner[0] + along_east * self.distance_along_leg_metres,
                  self.corner[1] + along_north * self.distance_along_leg_metres)
        setpoint = (carrot[0], carrot[1], self.height_metres, self.setpoint_heading_radians)
        at_the_end = self.distance_along_leg_metres >= leg_length - 1e-9
        if at_the_end and self.flies_through(self.next_waypoint):
            self.reached += 1
            self.corner = target
            self.next_waypoint += 1
            self.distance_along_leg_metres = 0.0
            return RouteDecision(f"reached {self.waypoint_name(self.reached - 1)} (flying through)", setpoint)
        close = math.hypot(position[0] - target[0], position[1] - target[1]) < settings.route_arrive_metres
        if not (at_the_end and close):
            return RouteDecision(f"flying to {self.waypoint_name(self.next_waypoint)}", setpoint)
        self.reached += 1
        self.corner = target
        self.next_waypoint += 1
        self.go_to("stopping", now_seconds)
        return RouteDecision(f"reached {self.waypoint_name(self.reached - 1)}", self.corner_setpoint())
