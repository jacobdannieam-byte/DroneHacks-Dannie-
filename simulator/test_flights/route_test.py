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
map. Boulder positions are uncertain, so the planner also keeps the drone's stops away from the
boulders in the maps given with --also-avoid-boulders-from (the organizers' drawing, unless
told otherwise).

--mission chooses which checkpoint lines to prefer. It defaults to the side-gap-first mission
that the team chose for the first competition run. Use missions/dronehacks_2026.json to try the
window first.

What it does
------------
  1. Plans the route from the drone's map and the mission, before anything moves. The route
     and its numbers are printed.
  2. Takes off and climbs to 0.8 m above the launch mark.
  3. Follows the route with drone_code/team_nav/team_nav/route_follower.py, which slides a target
     along the route ahead of the drone, and pauses at each corner.
  4. Lands above the pad, the same way landing_test.py does.

Stopping the run early
----------------------
While it follows the route, three things stop the run at once. The drone then lands where it
is, and the test fails:
  - The referee reports a touch. SIMULATOR ONLY: the real drone has no referee.
  - touch_detector.py sees a sudden tilt or spin, which is how the real drone would notice a
    touch.
  - height_check.py sees PX4's height and the bottom laser's disagree over clear floor, which
    means PX4's height can no longer be trusted. The landing then uses the laser's raw reading
    instead of PX4's height.

The referee (simulator/ros_packages/course_simulator/course_simulator/referee.py) watches the
flight and writes the summary: touches, checkpoints, time spent above boulders, and how far
PX4's height estimate strayed from the truth.
"""
import argparse
import os
import sys
import time

import rclpy
from std_msgs.msg import String

from team_nav.course_map import load_course_map
from team_nav.height_check import HeightCheck
from team_nav.mission import load_mission
from team_nav.route_follower import RouteFollower
from team_nav.route_planner import RoutePlanningError, plan_route
from team_nav.start_frame import StartFrame
from team_nav.touch_detector import TouchDetector

from simulator_test_flight import SECONDS_TO_SETTLE_AT_EACH_POINT, SimulatorTestFlight

REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DEFAULT_DRONE_MAP = "drone_code/team_nav/course_maps/phone_scan_2026-11-06.json"
DEFAULT_MAPS_WITH_OTHER_POSSIBLE_BOULDERS = ["drone_code/team_nav/course_maps/organizers_drawing.json"]
DEFAULT_MISSION = "drone_code/team_nav/missions/dronehacks_2026_side_gap_first.json"

# The simulator's assumption, from true_drone_state.py: the drone starts facing along the course's
# north direction, straight down the first corridor. The rules don't say which way the real drone
# will face.
START_HEADING_DEGREES = 90.0

# Our own choice. An 18 m route at 0.5 m/s takes under a minute, plus about 2.5 seconds at each
# of up to 30 corners.
LONGEST_ROUTE_SECONDS = 240.0


class RunMustStop(Exception):
    """Something happened that ends the run. The message says what, in full sentences."""


class RouteTest(SimulatorTestFlight):

    def __init__(self, drone_map_file, maps_with_other_possible_boulders, mission_file):
        super().__init__("route_test")
        self.drone_map_file = drone_map_file
        self.maps_with_other_possible_boulders = maps_with_other_possible_boulders
        self.mission_file = mission_file
        self.touch_reported_by_referee = None
        # SIMULATOR ONLY. The referee announces every touch here.
        self.create_subscription(String, "/simulator_only/touches", self.on_touch_reported_by_referee, 10)

    def on_touch_reported_by_referee(self, message):
        if self.touch_reported_by_referee is None:
            self.touch_reported_by_referee = message.data

    def run(self):
        self.refuse_to_run_outside_the_simulator()
        route = self.plan_the_route()
        self.wait_for_simulator()
        self.check_the_laser_reading_arrives()
        self.remember_the_floor()
        self.press_start()
        self.climb()
        try:
            self.follow(route)
        except RunMustStop as reason:
            self.say(f"STOPPING THE RUN. {reason}")
            self.say("Landing where the drone is, using the laser's raw reading for height.")
            time_of_touchdown = self.descend_until_touchdown(self.laser_height_above_floor_metres)
            self.wait_for_motors_off(time_of_touchdown)
            raise
        time_of_touchdown = self.descend_until_touchdown()
        self.wait_for_motors_off(time_of_touchdown)

    def plan_the_route(self):
        drone_map = load_course_map(os.path.join(REPOSITORY, self.drone_map_file))
        self.start_frame = StartFrame(drone_map.launch_mark_centre_metres, START_HEADING_DEGREES)
        self.other_possible_boulders = []
        for map_file in self.maps_with_other_possible_boulders:
            self.other_possible_boulders += load_course_map(os.path.join(REPOSITORY, map_file)).low_obstacles
        self.height_check = HeightCheck(drone_map, self.other_possible_boulders)
        self.touch_detector = TouchDetector()

        self.say(f"Planning with the drone's map, {self.drone_map_file}, and the mission {self.mission_file}.")
        if self.maps_with_other_possible_boulders:
            self.say(f"Also keeping stops away from the boulders in {', '.join(self.maps_with_other_possible_boulders)}.")
        mission = load_mission(os.path.join(REPOSITORY, self.mission_file))
        route = plan_route(drone_map, mission, other_possible_boulders=self.other_possible_boulders)
        self.say(route.describe())
        return route

    def check_the_laser_reading_arrives(self):
        self.run_for_seconds(1.0, None)
        if self.raw_laser_reading_metres() is None or self.px4_odometry is None:
            raise TimeoutError(
                "No laser reading has arrived on /fmu/out/distance_sensor, or no attitude report on "
                "/fmu/out/vehicle_odometry, so the height and touch checks can't run. In the simulator, "
                "pretend_bottom_laser.py publishes the laser reading. Check that it is running."
            )

    def follow(self, route):
        follower = RouteFollower(route.waypoints_metres)
        time_of_last_target = time.monotonic()

        def drone_on_course():
            return self.start_frame.course_from_start_frame(self.tracked_position())

        def send_next_target():
            nonlocal time_of_last_target
            self.stop_the_run_if_needed(drone_on_course())
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

    def stop_the_run_if_needed(self, drone_position_on_course):
        """Raises RunMustStop if the referee reported a touch, the drone moved as if it hit
        something, or PX4's height stopped matching the laser."""
        if self.touch_reported_by_referee is not None:
            raise RunMustStop(f"The referee reports a touch. {self.touch_reported_by_referee}")

        now_seconds = time.monotonic()
        suspected_touch = self.touch_detector.check(now_seconds, self.tilt_degrees_now(), self.rotation_degrees_per_second_now())
        if suspected_touch is not None:
            raise RunMustStop(suspected_touch)

        height_problem = self.height_check.check(
            now_seconds, drone_position_on_course,
            self.px4_height_risen_since_launch_metres(), self.laser_height_above_floor_metres(),
        )
        if height_problem is not None:
            raise RunMustStop(height_problem)


def main():
    parser = argparse.ArgumentParser(description="Plan a route from the drone's map, fly it and land, in the simulator.")
    parser.add_argument("--drone-map", default=DEFAULT_DRONE_MAP,
                        help="The map the drone plans with, named from the top of the repository.")
    parser.add_argument("--also-avoid-boulders-from", nargs="*", default=DEFAULT_MAPS_WITH_OTHER_POSSIBLE_BOULDERS,
                        help="Other maps of the course whose boulders the drone's stops should also keep away from. "
                             "Give the option with no files to use only the drone's map.")
    parser.add_argument("--mission", default=DEFAULT_MISSION,
                        help="The mission file, named from the top of the repository.")
    arguments = parser.parse_args()
    for file_name in [arguments.drone_map, arguments.mission] + arguments.also_avoid_boulders_from:
        if not os.path.isfile(os.path.join(REPOSITORY, file_name)):
            raise SystemExit(f"There is no file at {file_name}. Name it from the top of the repository.")

    rclpy.init()
    test = RouteTest(arguments.drone_map, arguments.also_avoid_boulders_from, arguments.mission)
    try:
        test.run()
    except (TimeoutError, RoutePlanningError, RunMustStop) as problem:
        print(f"TEST FAILED: {problem}", flush=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
