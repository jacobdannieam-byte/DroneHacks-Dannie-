"""The competition program: flies the whole obstacle course on the real drone and lands on the pad.

REAL DRONE CODE. Start it with "ros2 run team_nav course_run" or, on the drone, with
launch/course_run.launch.py. In the simulator, simulator/test_flights/real_program_test.py runs
this same program unchanged and plays the teammate's part.

What it does
------------
  1. Reads the run mode once (run_mode.py): competition, or practice. Says it on screen and in
     the records (run_log.py), and checks any stage to stop after before anything else happens.
  2. Plans the route from the map file and the mission file (route_planner.py). Nothing about
     the route is written in this code. The route, the files used and the start heading are
     saved in the run's records folder.
  3. Waits until the drone is ready on the ground (flight_steps.py), sending nothing.
  4. Sends a target 0.8 m (the mission's cruise height) above the launch mark, and keeps sending
     it until a teammate arms the drone and flips the switch to Position. The organizers'
     flight_interface then hands control to us ("Offboard"; docs/ARCHITECTURE.md, section 8).
     This program never arms the drone and never asks PX4 to change mode. If control never
     comes, it simply keeps waiting and reminds the people every few seconds.
  5. Climbs, then flies to the pad stage by stage, then holds above the pad and lands with the
     push-down step, and waits for PX4 to switch the motors off. There are two ways to fly to
     the pad, chosen with --navigation:
       live       (the default) live_flight.py: the drone looks around with its depth camera,
                  keeps a live map, re-plans its path all the time and faces where it flies.
                  The map file is only a first guess, so moved walls are handled.
       map_route  route_follower.py: follows the route planned once from the map, never turning.
                  This failed on 2026-11-08 when walls had moved; kept to compare.

The stages, in order
--------------------
  "climb", then one stage per checkpoint in the mission, named after the checkpoint in lower
  case, then "to the landing pad" and the landing. With the current missions the stages one can
  stop after are:
      climb, first corridor, maze, tunnel, chicane
  With map_route, a stage ends when the route's target moves past the point where the drone's
  whole body has crossed that checkpoint's line. With live, it ends when the drone crosses the
  line or comes within 25 cm of the goal point beyond it (live_planner.has_reached). In practice mode, TEAM_NAV_STOP_AFTER_STAGE=<name> lands the
  drone where that stage ends instead of flying on. Otherwise the flying is identical in both
  modes. Competition mode refuses TEAM_NAV_STOP_AFTER_STAGE (run_mode.py).

What stops the run (run_safety.py)
----------------------------------
  The checks run in every stage where the drone is airborne: the climb, the route, the settle
  above the pad, and the landing's descent until touchdown. During the descent the drone is
  already landing, so a height problem there only switches the rest of the descent to the
  laser's raw reading (flight_steps.DescentHeight); anything else is recorded.
  - A touch the drone can notice (a sudden tilt or spin): land where it is.
  - PX4's height has gone wrong: PX4's height and the bottom laser's raw reading disagree over
    clear floor, or PX4's height and camera tracking's height differ by more than 30 cm (15 cm
    once PX4 has stopped using the laser), or PX4 reports the laser is faulty. PX4 only
    stopping to use the laser is a warning, and the flight goes on. Land where it is, finding the floor with the laser's raw reading if
    we have it, else with PX4's distance to the ground if PX4 still reports one, else by
    descending gently until PX4 says it has landed (flight_steps.choose_landing_height_source).
  - Camera tracking silent for 0.5 s or reporting it is lost: hold still; after 5 s, land.
  - A target outside the course: refused, and the drone lands where it is.
  - Control no longer ours (disarmed, or PX4 left Offboard): stop sending targets and finish.
  - Any error in this program, or Ctrl+C: if control is ours and the motors are on, land where
    the drone is (choosing how to find the floor the same way), then finish.
    A second Ctrl+C abandons that landing.

What happens if this program dies mid-flight
--------------------------------------------
From the organizers' code (flight_interface.py): a position target stays in force until replaced,
and a speed target expires after 1 second, after which flight_interface holds the last position
target. So if this program is killed (for example by "kill -9" or a power problem on the Jetson
that spares flight_interface):
  - while following the route, the drone flies to the last target, at most 0.65 m ahead, and
    hovers there until the battery runs low or a teammate takes over;
  - while descending (speed targets), the drone climbs back to the last position target, the
    hover point above where the landing began, and hovers there.
PX4 stays in Offboard mode in both cases, because flight_interface keeps its own 10 Hz stream
going. The teammate must take over with the mode switch or stop the motors with the kill switch.
If flight_interface dies too (for example when the whole launch file is stopped), PX4 loses its
Offboard stream and does whatever its failsafe settings say (COM_OBL_RC_ACT). We have not tested
any of this on the real drone.

The fallback missions
--------------------
If TEAM_NAV_MISSION is fallback_pad or fallback_tunnel, this program flies that high route over
the walls instead of the course route (fallback_run.py explains them). Without it, nothing changes.
The competition launch file never sets it.

Exit codes: 0 the run finished with the motors off (or a practice run finished as asked);
1 the run stopped early or failed; 2 it refused to start (bad settings or no route); 130 Ctrl+C.

What this program does not do yet: correct camera tracking's drift against the map, or fly
through openings that need a height change (such as the window) with live navigation.
"""
import argparse
import json
import os
import signal
import sys
import traceback

import numpy as np

from team_nav.course_map import CourseMapError, load_course_map
from team_nav.live_flight import LiveFlight
from team_nav.mission import MissionError, load_mission
from team_nav.route_follower import RouteFollower
from team_nav.route_planner import RoutePlanningError, plan_route
from team_nav.run_mode import STOP_AFTER_STAGE_SETTING, read_run_mode
from team_nav.run_safety import RunSafety
from team_nav.start_frame import StartFrame
from team_nav import flight_steps
from team_nav.flight_steps import ControlIsNoLongerOurs, RunMustStop, SafetyWatch

PROGRAM_NAME = "course_run"

# Our own choices: the files used when none are named. The team chose the side-gap mission for
# the first competition run on 2026-11-07. The walls moved on 2026-11-08; the sketch map is the
# phone scan with the team's sketch of the changed north half, checked by the team that day.
DEFAULT_MAP_FILE = "sketch_2026-11-08.json"
DEFAULT_MISSION_FILE = "dronehacks_2026_side_gap_first.json"
DEFAULT_OTHER_BOULDER_MAP_FILES = ["organizers_drawing.json"]

# Estimated: the simulator's assumption (true_drone_state.py), that the drone's nose points along
# the course's north, straight down the first corridor. The rules don't say which way the real
# drone will face (docs/ARCHITECTURE.md, section 12), so it must be checked on the day.
DEFAULT_START_HEADING_DEGREES = 90.0

CLIMB_STAGE = "climb"
# The leg from the last checkpoint to above the landing pad. It is not a stage to stop after,
# because the run then lands at the pad anyway.
TO_THE_PAD_STAGE = "to the landing pad"

LIVE_NAVIGATION = "live"
MAP_ROUTE_NAVIGATION = "map_route"

# Our own choice. An 18 m route at 0.5 m/s takes under a minute, plus about 2.5 seconds at each of
# up to 30 corners. The scan route took about 80 s in the simulator.
LONGEST_ROUTE_SECONDS = 240.0

# Our own choice. The target never slides further at once than two cycles' worth, so it doesn't
# jump forward after the drone has held still.
LONGEST_SLIDE_STEP_SECONDS = 2.0 * flight_steps.CYCLE_SECONDS

EXIT_FINISHED = 0
EXIT_STOPPED_OR_FAILED = 1
EXIT_REFUSED_TO_START = 2
EXIT_INTERRUPTED = 130


# =================================================================================================
# Planning, before anything moves
# =================================================================================================

class CoursePlan:
    """The map, the mission, the route and the stages, worked out at startup."""

    def __init__(self, course_map, mission, other_possible_boulders, route, start_heading_degrees):
        self.course_map = course_map
        self.mission = mission
        self.other_possible_boulders = other_possible_boulders
        self.route = route                   # None if live navigation flies without one.
        self.start_frame = StartFrame(course_map.launch_mark_centre_metres, start_heading_degrees)
        # (stage name, number of the waypoint where it ends), for the stages along the route.
        self.route_stages = []
        if route is not None:
            for checkpoint_name, _line_name, waypoint_number in route.checkpoints_cleared:
                self.route_stages.append((checkpoint_name.lower(), waypoint_number))

    def stage_names(self):
        return [CLIMB_STAGE] + [checkpoint.name.lower() for checkpoint in self.mission.checkpoints]

    def waypoints_to_fly(self, stop_after_stage):
        """The whole route, or the part up to where stop_after_stage ends."""
        for name, last_waypoint_number in self.route_stages:
            if name == stop_after_stage:
                return self.route.waypoints_metres[:last_waypoint_number + 1]
        return self.route.waypoints_metres


def plan_the_course(arguments, run_mode, log):
    """Loads the files, plans the route, records it all, and checks the stage to stop after."""
    map_path = find_package_file(arguments.map, "course_maps")
    mission_path = find_package_file(arguments.mission, "missions")
    other_map_paths = [find_package_file(name, "course_maps") for name in arguments.other_boulder_maps]
    course_map = load_course_map(map_path)
    mission = load_mission(mission_path)
    other_possible_boulders = []
    for path in other_map_paths:
        other_possible_boulders += load_course_map(path).low_obstacles

    announce_the_inputs(log, map_path, mission_path, other_map_paths, arguments.start_heading_degrees)
    route = plan_a_route_if_needed(arguments.navigation, course_map, mission, other_possible_boulders, log)
    plan = CoursePlan(course_map, mission, other_possible_boulders, route, arguments.start_heading_degrees)
    if route is not None:
        record_the_route(log, plan)
    check_stop_after_stage(run_mode, plan.stage_names())
    return plan


def plan_a_route_if_needed(navigation, course_map, mission, other_possible_boulders, log):
    """map_route needs a route from the map, and refuses to start without one. Live navigation
    plans in flight, so a map route that fails (for example because the map is out of date)
    is only written down."""
    if navigation == MAP_ROUTE_NAVIGATION:
        return plan_route(course_map, mission, other_possible_boulders=other_possible_boulders)
    try:
        route = plan_route(course_map, mission, other_possible_boulders=other_possible_boulders)
    except RoutePlanningError as problem:
        log.event("warning", f"No route through the map file ({problem}). Live navigation doesn't need one: it "
                             f"plans in flight from what the depth camera sees.")
        return None
    log.event("route", "A route through the map file exists. Live navigation only records it, for comparison.")
    return route


def find_package_file(name_or_path, folder_name):
    """A file named on the command line: used as it is if it exists, otherwise looked for in this
    package's installed folder (course_maps or missions)."""
    if os.path.isfile(name_or_path):
        return os.path.abspath(name_or_path)
    from ament_index_python.packages import get_package_share_directory
    installed_path = os.path.join(get_package_share_directory("team_nav"), folder_name, name_or_path)
    if os.path.isfile(installed_path):
        return installed_path
    raise FileNotFoundError(
        f"There is no file '{name_or_path}', and none called that in {os.path.dirname(installed_path)}. "
        f"Give a full path, or the name of a file in team_nav's {folder_name} folder, and rebuild the package "
        f"if you added a file."
    )


def announce_the_inputs(log, map_path, mission_path, other_map_paths, start_heading_degrees):
    log.event("inputs", f"Map: {map_path}. Mission: {mission_path}. Other possible boulders from: "
                        f"{', '.join(other_map_paths) or 'none'}.",
              map_file=map_path, mission_file=mission_path, other_boulder_map_files=other_map_paths)
    log.event("start_heading",
              f"START HEADING: assuming the drone's nose points {start_heading_degrees:.1f} degrees from the course's "
              f"east towards north (90 = straight down the first corridor). If the drone faces another way, the "
              f"whole route is turned and will hit walls. Change it with --start-heading-degrees.",
              start_heading_degrees=start_heading_degrees)
    for path in [map_path, mission_path] + other_map_paths:
        with open(path, encoding="utf-8") as copied_file:
            log.save_text_file(f"input_{os.path.basename(path)}", copied_file.read())


def record_the_route(log, plan):
    route = plan.route
    log.event("route", route.describe(), length_metres=route.length_metres,
              waypoint_count=len(route.waypoints_metres), checkpoints_cleared=route.checkpoints_cleared)
    log.save_text_file("route.txt", route.describe() + "\n")
    route_record = {
        "map_name": route.map_name,
        "waypoints_east_north_height_metres": np.round(route.waypoints_metres, 4).tolist(),
        "checkpoints_cleared": route.checkpoints_cleared,
        "stages": plan.route_stages,
        "length_metres": route.length_metres,
        "least_room_metres": route.least_room_metres,
        "least_room_near": route.least_room_near,
        "legs_near_boulders": route.legs_near_boulders,
        "stops_near_boulders": route.stops_near_boulders,
    }
    log.save_text_file("route.json", json.dumps(route_record, indent=2, default=str) + "\n")


def check_stop_after_stage(run_mode, stage_names):
    if run_mode.stop_after_stage and run_mode.stop_after_stage.lower() not in stage_names:
        raise ValueError(
            f"{STOP_AFTER_STAGE_SETTING} is '{run_mode.stop_after_stage}', which is not a stage of this run. "
            f"The stages are: {', '.join(stage_names)}. Use one of those names, or remove the setting."
        )


# =================================================================================================
# Flying
# =================================================================================================

class CourseRun:

    def __init__(self, plan, run_mode, link, navigation=LIVE_NAVIGATION):
        self.plan = plan
        self.navigation = navigation
        self.link = link
        self.log = link.log
        self.stop_after_stage = run_mode.stop_after_stage.lower()
        self.watch = self.make_safety_watch()
        self.stage_name = None
        self.stage_start_seconds = None
        self.flight_start_seconds = None

    def make_safety_watch(self):
        """The safety checks and what acts on them. fallback_run.py replaces this to also watch
        whether the position stays healthy."""
        return SafetyWatch(self.link, self.make_run_safety(), self.plan.start_frame)

    def make_run_safety(self):
        """The safety checks (run_safety.py). fallback_run.py replaces this to allow its height."""
        return RunSafety(self.plan.course_map, self.plan.other_possible_boulders)

    def make_route_follower(self):
        """The route follower for the route, cut short after the stage to stop after, if any.
        fallback_run.py replaces this to give the follower its own speeds."""
        return RouteFollower(self.plan.waypoints_to_fly(self.stop_after_stage))

    def run(self):
        """Flies, and handles every way the flight can end. Returns the exit code."""
        try:
            self.fly()
            return EXIT_FINISHED
        except RunMustStop as stop:
            self.log.event("stop", f"STOPPING THE RUN during '{self.link.stage}'. {stop}", stage=self.link.stage)
            self.land_after_a_problem(stop.land_using_laser)
        except ControlIsNoLongerOurs as reason:
            self.log.event("control_lost", f"CONTROL IS NO LONGER OURS during '{self.link.stage}'. {reason} "
                                           f"Not sending any more targets.", stage=self.link.stage)
        except KeyboardInterrupt:
            self.log.event("interrupted", f"Interrupted (Ctrl+C) during '{self.link.stage}'.", stage=self.link.stage)
            self.land_after_a_problem(land_using_laser=False)
            return EXIT_INTERRUPTED
        except Exception:
            self.log.event("error", f"ERROR during '{self.link.stage}':\n{traceback.format_exc()}", stage=self.link.stage)
            self.land_after_a_problem(land_using_laser=False)
        return EXIT_STOPPED_OR_FAILED

    def fly(self):
        cruise_height_metres = self.plan.mission.cruise_height_metres
        if not self.wait_for_the_start(cruise_height_metres):
            return
        self.flight_start_seconds = self.link.now_seconds()

        self.begin_stage(CLIMB_STAGE)
        flight_steps.climb_above_launch(self.link, self.watch, cruise_height_metres)
        self.finish_stage()
        if self.stop_after_stage == CLIMB_STAGE:
            pass
        elif self.navigation == LIVE_NAVIGATION:
            LiveFlight(self.link, self.watch, self.plan.course_map, self.plan.mission, TO_THE_PAD_STAGE,
                       self.stop_after_stage, self).fly()
        else:
            self.follow_the_route()
        self.land_where_the_route_ends()

    def wait_for_the_start(self, cruise_height_metres):
        """Waits until the drone is ready and control is ours. Returns False if the run must not
        start; course_run always starts, but fallback_run.py adds a check on the launch pose."""
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, cruise_height_metres)
        return True

    def follow_the_route(self):
        """Slides the target along the route, stage by stage, with the safety checks running."""
        follower = self.make_route_follower()
        stages_still_to_finish = list(self.plan.route_stages)
        self.begin_stage(stages_still_to_finish[0][0])
        time_of_last_target = [self.link.now_seconds()]

        def send_next_target():
            now = self.link.now_seconds()
            seconds_to_slide = min(now - time_of_last_target[0], LONGEST_SLIDE_STEP_SECONDS)
            time_of_last_target[0] = now
            target_on_course = follower.next_target(self.watch.drone_on_course(), seconds_to_slide)
            self.finish_stages_passed(stages_still_to_finish, follower.next_waypoint_number)
            self.watch.send_target_on_course(target_on_course)

        flight_steps.fly_until(self.link, lambda: follower.has_finished(self.watch.drone_on_course()),
                               send_next_target, LONGEST_ROUTE_SECONDS, "the drone to reach the end of the route",
                               self.watch)
        # The stage still in progress: the one to stop after, or the last leg to the pad.
        self.finish_stage()

    def finish_stages_passed(self, stages_still_to_finish, next_waypoint_number):
        """Finishes every stage whose last waypoint the target has moved past, and begins the next."""
        while stages_still_to_finish and next_waypoint_number > stages_still_to_finish[0][1]:
            finished_stage_name = stages_still_to_finish.pop(0)[0]
            if finished_stage_name == self.stop_after_stage:
                # The route was cut short here, so this stage ends when the drone reaches its end.
                return
            self.finish_stage()
            if stages_still_to_finish:
                self.begin_stage(stages_still_to_finish[0][0])
            else:
                self.begin_stage(TO_THE_PAD_STAGE)

    def land_where_the_route_ends(self):
        self.link.stage = "settling before the landing"
        flight_steps.hold_still_for(self.link, self.watch, flight_steps.SECONDS_TO_SETTLE)
        self.begin_stage("landing")
        height_source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, height_source, self.watch)
        self.finish_stage()
        total_seconds = self.link.now_seconds() - self.flight_start_seconds
        if self.stop_after_stage:
            self.log.event("result", f"PRACTICE RUN FINISHED: landed after the stage '{self.stop_after_stage}', as asked. "
                                     f"{total_seconds:.1f} s from the hand-over to motors off.", total_seconds=total_seconds)
            return
        self.log.event("result", f"RUN FINISHED: landed above the pad and the motors are off, {total_seconds:.1f} s "
                                 f"after the hand-over.", total_seconds=total_seconds)

    # ---- Stage records ------------------------------------------------------------------------------

    def begin_stage(self, name):
        self.stage_name = name
        self.stage_start_seconds = self.link.now_seconds()
        self.link.stage = name
        self.log.event("stage", f"Stage '{name}' begins.", stage=name, event="begins")

    def finish_stage(self):
        seconds = self.link.now_seconds() - self.stage_start_seconds
        self.log.event("stage", f"Stage '{self.stage_name}' finished after {seconds:.1f} s.",
                       stage=self.stage_name, event="finished", duration_seconds=round(seconds, 2))

    # ---- After a problem --------------------------------------------------------------------------

    def land_after_a_problem(self, land_using_laser):
        """Lands where the drone is, if it is ours and its motors are on. Never raises."""
        if not (self.link.control_is_ours() and self.link.is_armed()):
            self.log.event("landing", "No landing by this program: control is not ours or the motors are off.")
            return
        px4_height_is_in_doubt = land_using_laser or self.watch.px4_height_is_in_doubt()
        try:
            height_source = flight_steps.choose_landing_height_source(self.link, px4_height_is_in_doubt)
            flight_steps.land(self.link, height_source, self.watch)
            self.log.event("landing", "Landed where the drone was after the problem. The motors are off.")
        except ControlIsNoLongerOurs as reason:
            self.log.event("control_lost", f"Control was taken away during the landing. {reason}")
        except KeyboardInterrupt:
            self.log.event("interrupted", "Interrupted again: the landing is abandoned. The drone holds its last "
                                          "position target until a teammate takes over or stops the motors.")
        except Exception:
            self.log.event("error", f"ERROR during the landing after the problem:\n{traceback.format_exc()}"
                                    f"The drone holds its last position target. Take over with the remote.")


# =================================================================================================
# Starting the program
# =================================================================================================

def parse_arguments(command_line_words):
    parser = argparse.ArgumentParser(
        prog=PROGRAM_NAME,
        description="Flies the whole obstacle course. The mode comes from TEAM_NAV_RUN_MODE (missing = competition).")
    parser.add_argument("--map", default=DEFAULT_MAP_FILE,
                        help="The drone's course map: a path, or a file name in team_nav's course_maps folder.")
    parser.add_argument("--mission", default=DEFAULT_MISSION_FILE,
                        help="The mission: a path, or a file name in team_nav's missions folder.")
    parser.add_argument("--other-boulder-maps", nargs="*", default=DEFAULT_OTHER_BOULDER_MAP_FILES,
                        help="Other maps of the course whose boulders the stops should also avoid. Names may also "
                             "be given in one word, separated by commas or spaces, as the launch file does. "
                             "Give the option with no files, or an empty word, to use only the drone's map.")
    parser.add_argument("--start-heading-degrees", type=float, default=DEFAULT_START_HEADING_DEGREES,
                        help="Which way the drone's nose points on the launch mark, in degrees from the course's "
                             "east towards north. 90 means straight down the first corridor.")
    parser.add_argument("--navigation", choices=[LIVE_NAVIGATION, MAP_ROUTE_NAVIGATION], default=LIVE_NAVIGATION,
                        help="live (default): look with the depth camera, re-plan in flight and face where the drone "
                             "flies. map_route: follow the route planned once from the map file.")
    parser.add_argument("--ignore-raw-laser", action="store_true",
                        help="Behave as if the bottom laser's raw reading did not exist, as on the real drone. "
                             "For testing that form of the safety checks in the simulator.")
    arguments = parser.parse_args(command_line_words)
    arguments.other_boulder_maps = split_file_names(arguments.other_boulder_maps)
    return arguments


def split_file_names(words):
    """["a.json, b.json", "c.json"] -> ["a.json", "b.json", "c.json"]. Empty words are dropped."""
    names = []
    for word in words:
        for name in word.replace(",", " ").split():
            names.append(name)
    return names


def plan_then_fly(arguments, run_mode, log):
    """Returns the exit code."""
    # Imported here because fallback_run.py builds on this file.
    from team_nav import fallback_run
    try:
        fallback_route_name = fallback_run.fallback_route_from_environment()
        if fallback_route_name is None:
            plan, run_class = plan_the_course(arguments, run_mode, log), CourseRun
        else:
            plan = fallback_run.plan_the_fallback(arguments, run_mode, log, fallback_route_name)
            run_class = fallback_run.FallbackRun
    except (OSError, CourseMapError, MissionError, RoutePlanningError, ValueError) as problem:
        log.event("refused", f"REFUSING TO START, before anything was sent to the drone. {problem}")
        return EXIT_REFUSED_TO_START
    log.event("stages", "The stages of this run: " + ", ".join(plan.stage_names()) + ", then the landing.",
              stages=plan.stage_names())

    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    # By default rclpy shuts ROS down on Ctrl+C, which would stop us sending the landing targets.
    # Without its handler, Ctrl+C raises KeyboardInterrupt, which CourseRun.run handles by landing.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(PROGRAM_NAME)
    try:
        link = flight_steps.DroneLink(node, log, ignore_raw_laser=arguments.ignore_raw_laser)
        return run_class(plan, run_mode, link, arguments.navigation).run()
    except Exception:
        log.event("error", f"ERROR before the flight could begin:\n{traceback.format_exc()}")
        return EXIT_STOPPED_OR_FAILED
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def treat_terminate_as_ctrl_c(_signal_number, _frame):
    # "ros2 launch" stops a program with SIGINT, then SIGTERM. Python ignores neither, but
    # SIGTERM would end the program at once, without landing or closing the records.
    raise KeyboardInterrupt


def main():
    from rclpy.utilities import remove_ros_args
    try:
        run_mode = read_run_mode()
    except ValueError as problem:
        print(f"REFUSING TO START: {problem}", flush=True)
        sys.exit(EXIT_REFUSED_TO_START)
    arguments = parse_arguments(remove_ros_args(sys.argv)[1:])
    signal.signal(signal.SIGTERM, treat_terminate_as_ctrl_c)

    from team_nav.run_log import RunLog
    log = RunLog(PROGRAM_NAME, run_mode)
    try:
        exit_code = plan_then_fly(arguments, run_mode, log)
        log.event("exit", f"course_run is ending with exit code {exit_code}.", exit_code=exit_code)
    finally:
        log.close()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
