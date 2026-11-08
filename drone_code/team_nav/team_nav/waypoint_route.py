"""Plan and fly a map-generated, fixed-height waypoint route with live clearance checks.

Planning uses the current course map and mission. The flight reads only fresh onboard
observations, and refuses to detour through a mapped net.
"""

import argparse
import math
from dataclasses import replace

import numpy as np

from team_nav.course_map import load_course_map
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES, REQUIRED_ROOM_METRES
from team_nav.live_map import LiveMap
from team_nav.live_navigation import (
    CORRIDOR_STEERING_LIMIT_DEGREES, HOLDING, STOPPING_MARGIN_METRES, TURN_FIRST_DEGREES, TURN_ROOM_METRES,
    LiveNavigator, NavigationStep,
)
from team_nav.live_planner import ROOM_NEEDED_METRES, Goal, corridor_exit_reached
from team_nav.mission import load_mission
from team_nav.route_planner import RoutePlanningError, plan_route

# Our own choice. A waypoint counts once the observed centre is within this distance.
# The route planner leaves the same extra room when laying out the checkpoint crossings.
WAYPOINT_ARRIVAL_METRES = 0.08


def plan_waypoint_route(course_map, mission, other_possible_boulders=(), start_heading_radians=math.pi / 2):
    """Find a below-wall route and reject a crossing or landing the drone cannot use."""
    if mission.cruise_height_metres + DRONE_HALF_HEIGHT_METRES >= course_map.wall_top_height_metres:
        raise RoutePlanningError("The cruise height leaves no clearance below the course wall tops.")
    if course_map.landing_pad_outer_radius_metres < REQUIRED_ROOM_METRES + WAYPOINT_ARRIVAL_METRES:
        raise RoutePlanningError("The landing pad is too small for the whole drone and its arrival tolerance.")
    checkpoints = []
    for checkpoint in mission.checkpoints:
        line_names = [name for name in checkpoint.line_names_in_order_of_preference
                      if course_map.checkpoint_line_named(name).opening_name is None]
        if not line_names:
            raise RoutePlanningError(f"'{checkpoint.name}' needs a ground-level option for fixed-height flight.")
        checkpoints.append(replace(checkpoint, line_names_in_order_of_preference=line_names))
    route = plan_route(course_map, replace(mission, checkpoints=checkpoints), other_possible_boulders=other_possible_boulders,
                       checkpoint_margin_metres=WAYPOINT_ARRIVAL_METRES, keep_clear_of_boulders=True)
    if route.legs_near_boulders or route.stops_near_boulders:
        raise RoutePlanningError("The proposed route passes or stops too close to a mapped boulder. " + route.describe())
    if route.least_room_metres < REQUIRED_ROOM_METRES:
        raise RoutePlanningError("The route's straightened legs do not leave room for the propellers.")
    check_waypoint_turns(course_map, route, start_heading_radians)
    return route


def check_waypoint_turns(course_map, route, start_heading_radians):
    """Reject a planned pivot with insufficient space for the observed slide."""
    live_map = LiveMap(course_map, route.waypoints_metres[0, 2], DRONE_HALF_HEIGHT_METRES)
    previous_heading = start_heading_radians
    first_checkpoint = route.checkpoints_cleared[0][2]
    for index, (start, end) in enumerate(zip(route.waypoints_metres, route.waypoints_metres[1:])):
        east_metres, north_metres = end[:2] - start[:2]
        heading = math.atan2(north_metres, east_metres)
        if index < first_checkpoint:
            if abs(math.remainder(heading - start_heading_radians, math.tau)) > math.radians(CORRIDOR_STEERING_LIMIT_DEGREES):
                raise RoutePlanningError("The first corridor route would require sideways flight from the launch heading.")
            continue
        turn_degrees = abs(math.degrees(math.remainder(heading - previous_heading, math.tau)))
        if turn_degrees > TURN_FIRST_DEGREES:
            square = live_map.square_of(start)
            room_metres = live_map.mapped_room_metres[square]
            if room_metres < TURN_ROOM_METRES:
                raise RoutePlanningError(
                    f"Waypoint {index + 1} at ({start[0]:.2f}, {start[1]:.2f}) needs a {turn_degrees:.0f}° turn "
                    f"with only {room_metres:.2f} m of mapped room. A sharp turn needs {TURN_ROOM_METRES:.2f} m."
                )
        previous_heading = heading


class WaypointNavigator(LiveNavigator):
    """Use the live navigator's motion checks with one fixed map waypoint at a time."""

    def __init__(self, live_map, course_map, route, landing_stage_name, stop_after_stage, start_heading_radians):
        checkpoints = {index: (name.lower(), course_map.checkpoint_line_named(line_name))
                       for name, line_name, index in route.checkpoints_cleared}
        self.checkpoints = checkpoints
        self.waypoints_metres = route.waypoints_metres
        goals = []
        stage_name = route.checkpoints_cleared[0][0].lower() if checkpoints else landing_stage_name
        for index, waypoint_metres in enumerate(self.waypoints_metres[1:], 1):
            checkpoint = checkpoints.get(index)
            line = checkpoint[1] if checkpoint else None
            goals.append(Goal(stage_name, waypoint_metres[:2],
                              line.start_metres if line else None, line.end_metres if line else None,
                              start_heading_radians if stage_name == "first corridor" else None))
            if checkpoint:
                next_checkpoint = next((item for number, item in checkpoints.items() if number > index), None)
                stage_name = next_checkpoint[0] if next_checkpoint else landing_stage_name
        super().__init__(live_map, goals, self.waypoints_metres[0, 2], stop_after_stage)

    def _replan_if_due(self, drone_metres, now_seconds):
        self.path = [self.current_goal.point_metres]

    def _check_goal_reached(self, drone_metres):
        self._pending_events = ([], [])
        goal = self.current_goal
        if np.linalg.norm(goal.point_metres - drone_metres[:2]) > WAYPOINT_ARRIVAL_METRES:
            return None
        checkpoint = self.checkpoints.get(self.goal_number + 1)
        if checkpoint:
            # Use the declared far side of the line, independent of the waypoint's location.
            stage_name, line = checkpoint
            crossing = Goal(stage_name, line.far_side_point_metres, line.start_metres, line.end_metres)
            if not corridor_exit_reached(crossing, drone_metres):
                return None
        if self.goal_number == len(self.goals) - 1 or (checkpoint and goal.stage_name == self.stop_after_stage):
            heading = goal.straight_heading_radians
            if heading is None:
                inbound = goal.point_metres - self.waypoints_metres[self.goal_number, :2]
                heading = math.atan2(inbound[1], inbound[0])
            self.hold_point = None
            return NavigationStep(self._hold_target(drone_metres), heading, HOLDING, finished=True,
                                  stages_finished=[goal.stage_name])
        self.goal_number += 1
        self.seconds_without_path_since = None
        self.hold_point = None
        if checkpoint:
            self._pending_events = ([goal.stage_name], [self.current_goal.stage_name])
        return None

    def _turn_room_metres(self):
        return self.live_map.room_metres(keep_mapped_walls=True)

    def _safe_lead_metres(self, drone_metres, direction, distance_to_aim_metres, room_metres=None,
                          stopping_margin_metres=STOPPING_MARGIN_METRES, room_needed_metres=ROOM_NEEDED_METRES):
        if room_metres is None:
            room_metres = self._turn_room_metres()
            room_metres[self.live_map.near_a_boulder()] = 0.0
        return super()._safe_lead_metres(drone_metres, direction, distance_to_aim_metres, room_metres,
                                         stopping_margin_metres, room_needed_metres)


def main():
    """Preview the same route the drone will plan, from a laptop without ROS."""
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("map_file")
    parser.add_argument("mission_file")
    parser.add_argument("--other-boulder-map", action="append", default=[])
    parser.add_argument("--start-heading-degrees", type=float, default=90.0)
    args = parser.parse_args()
    try:
        other_boulders = [boulder for path in args.other_boulder_map for boulder in load_course_map(path).low_obstacles]
        route = plan_waypoint_route(load_course_map(args.map_file), load_mission(args.mission_file), other_boulders,
                                    math.radians(args.start_heading_degrees))
    except (OSError, ValueError, RoutePlanningError) as problem:
        parser.exit(2, f"No usable waypoint route: {problem}\n")
    print(route.describe())


if __name__ == "__main__":
    main()
