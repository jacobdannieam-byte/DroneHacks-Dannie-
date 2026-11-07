"""A test flight through the whole course: plan a route from the drone's map, fly it, and land
on the pad.

SIMULATOR TEST. Do not run this on the real drone: it asks for the simulator's start service.
It is not the competition program either. That program will also watch the cameras, correct
drift and replan; this one only plans once and follows the route.

Two maps, on purpose
--------------------
The simulator's world, which is what is really there, is chosen with run_test_flight.sh's
--world. The drone's map, which is what the drone believes is there, is chosen here with
--drone-map. They can be different files, to test what happens when the drone's map is wrong,
for example when a boulder has been moved. This program never reads the world's map file.

    simulator/run_test_flight.sh --world <what is really there> route_test.py --drone-map <what the drone believes>

Both are map files named from the top of the repository. --drone-map defaults to our best scan
map.

What it does
------------
  1. Plans the route from the drone's map and the rulebook's mission, before anything moves.
     The route and its numbers are printed.
  2. Takes off and climbs to 0.8 m above the launch mark.
  3. Follows the route with drone_code/team_nav/team_nav/route_follower.py, which slides a target
     along the route ahead of the drone, and pauses at each corner.
  4. Lands above the pad, the same way landing_test.py does.

The referee (simulator/ros_packages/course_simulator/course_simulator/referee.py) watches the
flight and writes the summary: touches, checkpoints, time spent above boulders, and how far
PX4's height estimate strayed from the truth.
"""
import argparse
import os
import sys
import time

import rclpy

from team_nav.course_map import load_course_map
from team_nav.mission import load_mission
from team_nav.route_follower import RouteFollower
from team_nav.route_planner import RoutePlanningError, plan_route
from team_nav.start_frame import StartFrame

from simulator_test_flight import SECONDS_TO_SETTLE_AT_EACH_POINT, SimulatorTestFlight

REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_DRONE_MAP = "drone_code/team_nav/course_maps/phone_scan_2026-11-06.json"
RULEBOOK_MISSION_FILE = os.path.join(REPOSITORY, "drone_code", "team_nav", "missions", "dronehacks_2026.json")

# The simulator's assumption, from true_drone_state.py: the drone starts facing along the course's
# north direction, straight down the first corridor. The rules don't say which way the real drone
# will face.
START_HEADING_DEGREES = 90.0

# Our own choice. An 18 m route at 0.5 m/s takes under a minute, plus about 2.5 seconds at each
# of up to 30 corners.
LONGEST_ROUTE_SECONDS = 240.0


class RouteTest(SimulatorTestFlight):

    def __init__(self, drone_map_file):
        super().__init__("route_test")
        self.drone_map_file = drone_map_file

    def run(self):
        self.refuse_to_run_outside_the_simulator()
        route = self.plan_the_route()
        self.wait_for_simulator()
        self.remember_the_floor()
        self.press_start()
        self.climb()
        self.follow(route)
        time_of_touchdown = self.descend_until_touchdown()
        self.wait_for_motors_off(time_of_touchdown)

    def plan_the_route(self):
        drone_map = load_course_map(os.path.join(REPOSITORY, self.drone_map_file))
        self.start_frame = StartFrame(drone_map.launch_mark_centre_metres, START_HEADING_DEGREES)
        self.say(f"Planning with the drone's map, {self.drone_map_file}.")
        route = plan_route(drone_map, load_mission(RULEBOOK_MISSION_FILE))
        self.say(route.describe())
        return route

    def follow(self, route):
        follower = RouteFollower(route.waypoints_metres)
        time_of_last_target = time.monotonic()

        def drone_on_course():
            return self.start_frame.course_from_start_frame(self.tracked_position())

        def send_next_target():
            nonlocal time_of_last_target
            now = time.monotonic()
            target_on_course = follower.next_target(drone_on_course(), now - time_of_last_target)
            time_of_last_target = now
            self.send_target_position(*self.start_frame.start_frame_from_course(target_on_course))

        self.say("Following the route.")
        self.run_until(lambda: follower.has_finished(drone_on_course()), send_next_target,
                       LONGEST_ROUTE_SECONDS, "the drone to reach the end of the route")
        self.say("Reached the end of the route, above the landing pad.")
        last_target_in_start_frame = self.start_frame.start_frame_from_course(route.waypoints_metres[-1])
        self.run_for_seconds(SECONDS_TO_SETTLE_AT_EACH_POINT, lambda: self.send_target_position(*last_target_in_start_frame))


def main():
    parser = argparse.ArgumentParser(description="Plan a route from the drone's map, fly it and land, in the simulator.")
    parser.add_argument("--drone-map", default=DEFAULT_DRONE_MAP,
                        help="The map the drone plans with, named from the top of the repository.")
    arguments = parser.parse_args()
    if not os.path.isfile(os.path.join(REPOSITORY, arguments.drone_map)):
        raise SystemExit(f"There is no map file at {arguments.drone_map}. Name it from the top of the repository.")

    rclpy.init()
    test = RouteTest(arguments.drone_map)
    try:
        test.run()
    except (TimeoutError, RoutePlanningError) as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
