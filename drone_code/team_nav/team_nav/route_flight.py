"""The route mission: fly a list of waypoints at cruise height, one leg at a time.

How it fits: mission.py uses this instead of course_stages.py when launched with mission:=route.
Once the drone is at cruise height, mission.py calls step twenty times a second; step answers with
a position target, a short reason, and whether the route is finished (then mission.py lands). The
position watchdog and every other rule in mission.py stay active the whole time. This file never
talks to ROS and never reads a clock.

The waypoints come from a data file (missions/route_waypoints.json), not from code: the judges take
points off for routes written into code. Each waypoint is (forward, left) in metres from the
takeoff spot, with "forward" being where the nose pointed at takeoff.

Each leg: a target (the "carrot") moves along the straight line to the next waypoint at
route_speed_metres_per_second, never more than the carrot lead ahead of the drone.

At each waypoint, what happens depends on how much the next leg turns:
* By route_fly_through_degrees or more (and at the takeoff spot and the last waypoint): the carrot
  stops at the waypoint, the leg ends once the drone is within route_arrive_metres of it, and then
     1. stop: hold the corner until the drone is slower than settled_speed_metres_per_second (or for
        route_corner_settle_time_limit_seconds at most);
     2. turn in place to face the next waypoint, never faster than route_turn_rate_degrees_per_second.
* By less: fly through. The carrot carries straight on along the next leg as soon as it reaches the
  waypoint, without stopping, and the heading turns toward the new leg at the same turn rate while
  flying.
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


class RouteFlight:
    def __init__(self, settings, waypoints_forward_left_metres):
        if not waypoints_forward_left_metres:
            raise ValueError("The route has no waypoints. Check the route file given by the route_file parameter.")
        self.settings = settings
        self.waypoints_forward_left_metres = [tuple(waypoint) for waypoint in waypoints_forward_left_metres]
        self.waypoints = []           # the same waypoints in the camera tracking frame (x, y)
        self.home = None              # (x, y) of the takeoff spot
        self.cruise_height_metres = None
        self.next_waypoint = 0        # index of the waypoint being flown to
        self.corner = None            # (x, y) the drone stops and turns at
        self.step_name = None         # "stopping", "turning" or "flying"
        self.step_start_seconds = None
        self.setpoint_heading_radians = None
        self.distance_along_leg_metres = 0.0
        self.reached = 0              # how many waypoints have been reached

    def start(self, now_seconds, cruise_setpoint):
        """cruise_setpoint = (x, y, z, heading) over the takeoff spot, nose as it was at takeoff."""
        home_x, home_y, cruise_z, heading = cruise_setpoint
        self.waypoints = [body_to_world((home_x, home_y), heading, forward, left)
                          for forward, left in self.waypoints_forward_left_metres]
        self.cruise_height_metres = cruise_z
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
        return self.fly_leg(now_seconds, seconds_since_last_step, position)

    def corner_setpoint(self):
        return (self.corner[0], self.corner[1], self.cruise_height_metres, self.setpoint_heading_radians)

    # ------------------------------------------------------------------ the three steps at a corner
    def stop_at_corner(self, now_seconds, horizontal_speed):
        settings = self.settings
        where = "the takeoff spot" if self.reached == 0 else self.waypoint_name(self.reached - 1)
        stopped = horizontal_speed is not None and horizontal_speed < settings.settled_speed_metres_per_second
        waited_long_enough = now_seconds - self.step_start_seconds >= settings.route_corner_settle_time_limit_seconds
        if not (stopped or waited_long_enough):
            return RouteDecision(f"stopping at {where}", self.corner_setpoint())
        if self.reached == len(self.waypoints):
            return RouteDecision(f"route complete: reached {self.waypoint_name(len(self.waypoints) - 1)}",
                                 self.corner_setpoint(), complete=True)
        self.go_to("turning", now_seconds)
        return RouteDecision(f"turning to face {self.waypoint_name(self.next_waypoint)}", self.corner_setpoint())

    def leg_heading(self):
        target = self.waypoints[self.next_waypoint]
        return math.atan2(target[1] - self.corner[1], target[0] - self.corner[0])

    def turn_setpoint_heading_toward(self, wanted_radians, seconds_since_last_step):
        largest_turn = math.radians(self.settings.route_turn_rate_degrees_per_second) * seconds_since_last_step
        error = wrap_angle(wanted_radians - self.setpoint_heading_radians)
        self.setpoint_heading_radians = wrap_angle(self.setpoint_heading_radians + limit(error, largest_turn))

    def flies_through(self, index):
        """Whether waypoint index is passed without stopping: not the last one, and the next leg
        turns by less than route_fly_through_degrees."""
        if index >= len(self.waypoints) - 1:
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
            self.distance_along_leg_metres = 0.0
            self.go_to("flying", now_seconds)
            return RouteDecision(f"flying to {self.waypoint_name(self.next_waypoint)}", self.corner_setpoint())
        return RouteDecision(f"turning to face {self.waypoint_name(self.next_waypoint)}", self.corner_setpoint())

    def fly_leg(self, now_seconds, seconds_since_last_step, position):
        settings = self.settings
        target = self.waypoints[self.next_waypoint]
        leg_east, leg_north = target[0] - self.corner[0], target[1] - self.corner[1]
        leg_length = math.hypot(leg_east, leg_north)
        along_east, along_north = (leg_east / leg_length, leg_north / leg_length) if leg_length > 1e-9 else (0.0, 0.0)
        # After flying through a waypoint the nose still points along the previous leg: blend it round.
        self.turn_setpoint_heading_toward(math.atan2(leg_north, leg_east), seconds_since_last_step)
        # How far along the leg the drone itself is: the carrot never leads it by more than the lead.
        drone_along = (position[0] - self.corner[0]) * along_east + (position[1] - self.corner[1]) * along_north
        self.distance_along_leg_metres = min(
            self.distance_along_leg_metres + settings.route_speed_metres_per_second * seconds_since_last_step,
            leg_length, max(drone_along, 0.0) + settings.carrot_lead_metres)
        carrot = (self.corner[0] + along_east * self.distance_along_leg_metres,
                  self.corner[1] + along_north * self.distance_along_leg_metres)
        setpoint = (carrot[0], carrot[1], self.cruise_height_metres, self.setpoint_heading_radians)
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
