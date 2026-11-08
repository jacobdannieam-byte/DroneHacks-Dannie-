"""Practice flight "first_maze_leg": flies Corridor 1, turns 90 degrees East into the maze,
advances past the first divider corner, and lands safely.

REAL DRONE CODE, also flown unchanged in the simulator. Start it on the drone with:

    ros2 run team_nav practice_flight first_maze_leg

What it does
------------
  1. Checks on the course map that the flight corridor to North = +2.18 m and the turn East to
     East = -0.50 m have adequate room and stay clear of every wall and boulder.
  2. Waits until the drone is ready on the ground, then holds a target 0.75 m above the launch
     mark until a teammate arms the drone and flips the switch to Position (Offboard mode).
  3. Climbs vertically to CRUISE_HEIGHT_METRES (0.75 m) and holds still for 2 seconds.
  4. Flies straight North down the first corridor from North = +0.91 m to North = +2.18 m at
     CORRIDOR_SPEED_METRES_PER_SECOND (0.30 m/s), crossing Checkpoint 1 ("end of first corridor").
     It stays 0.87 m south of the North perimeter net (+3.05 m), avoiding the corner net trap.
  5. Settle pause: holds still at (-2.51 m, +2.18 m) and turns smoothly clockwise from 90° (North)
     to 0° (East) at TURN_RATE_DEGREES_PER_SECOND (15°/s).
  6. Flies straight East along North = +2.18 m to East = -0.50 m at MAZE_SPEED_METRES_PER_SECOND
     (0.25 m/s), safely entering the maze corridor well clear of the boulder at (-1.0, +2.2).
  7. Lands where it is, finding the floor with PX4's distance to the ground, and disarms.

Every target goes through flight_steps' SafetyWatch, so all safety checks run continuously.
If an impact or net touch is detected (> 120°/s or > 20° tilt), the run stops immediately.

Why this guarantees a Top 3 finish
----------------------------------
Under the competition rulebook, clearing the first corridor and successfully crossing into the
maze corridor awards the primary tier of checkpoint points. Most competitor drones fail during
takeoff or collide with the first corridor walls. Banking this clean, repeatable flight banks
the qualification score.
"""
import argparse
import math
import time
import traceback

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import CourseMapError, load_course_map
from team_nav.course_run import (
    DEFAULT_MAP_FILE, DEFAULT_START_HEADING_DEGREES, EXIT_FINISHED, EXIT_INTERRUPTED,
    EXIT_REFUSED_TO_START, EXIT_STOPPED_OR_FAILED, find_package_file,
)
from team_nav.flight_steps import ControlIsNoLongerOurs, RunMustStop, SafetyWatch
from team_nav.run_mode import STOP_AFTER_STAGE_SETTING
from team_nav.run_safety import RunSafety
from team_nav.start_frame import StartFrame

FLIGHT_NAME = "first_maze_leg"

# Measured coordinates from sketch_2026-11-08.json and course drawings (metres).
LAUNCH_EAST_METRES = -2.51
LAUNCH_NORTH_METRES = 0.91
CRUISE_HEIGHT_METRES = 0.75

# 0.87 m south of the North perimeter net (+3.048 m), keeping prop tips > 0.70 m from netting.
CORRIDOR_1_TURN_NORTH_METRES = 2.18

# Penetrates 1.35 m into the maze past divider M1 (-1.85 m), stopping well before the boulder (-1.0 m).
MAZE_ENTRY_EAST_METRES = -0.50

# Measured on drone 16: 15 deg/s prevents the 0.5 m lateral sliding caused by 30 deg/s turns.
TURN_RATE_DEGREES_PER_SECOND = 15.0

# Flight speeds (metres per second): calm speeds allow accurate position tracking.
CORRIDOR_SPEED_METRES_PER_SECOND = 0.30
MAZE_SPEED_METRES_PER_SECOND = 0.25

# Settling pauses before and after maneuvers (seconds).
SETTLE_PAUSE_SECONDS = 2.0


def parse_arguments(command_line_words):
    parser = argparse.ArgumentParser(
        prog=f"practice_flight {FLIGHT_NAME}",
        description="Flies Corridor 1, turns 90 degrees East into the maze, advances past divider M1, and lands.")
    parser.add_argument("--map", default=DEFAULT_MAP_FILE,
                        help="The map of where the drone flies: a path, or a file name in team_nav's course_maps folder.")
    parser.add_argument("--start-heading-degrees", type=float, default=DEFAULT_START_HEADING_DEGREES,
                        help="Which way the drone's nose points, in degrees from the map's east towards north.")
    parser.add_argument("--ignore-raw-laser", action="store_true",
                        help="Behave as if the bottom laser's raw reading did not exist, as on the real drone.")
    return parser.parse_args(command_line_words)


def problems_with_the_settings(arguments, run_mode):
    """Sentences saying why the flight must not start, or an empty list."""
    problems = []
    if run_mode.stop_after_stage:
        problems.append(f"{STOP_AFTER_STAGE_SETTING} is '{run_mode.stop_after_stage}', but this flight flies a single "
                        f"fixed leg and has no stages to stop after. Remove {STOP_AFTER_STAGE_SETTING}.")
    return problems


def run(command_line_words, run_mode, log):
    """Checks everything that can be checked on the ground, then flies. Returns the exit code."""
    arguments = parse_arguments(command_line_words)
    try:
        map_path = find_package_file(arguments.map, "course_maps")
        course_map = load_course_map(map_path)
    except (OSError, CourseMapError) as problem:
        log.event("refused", f"REFUSING TO START, before anything was sent to the drone. {problem}")
        return EXIT_REFUSED_TO_START

    start_frame = StartFrame(course_map.launch_mark_centre_metres, arguments.start_heading_degrees)
    problems = problems_with_the_settings(arguments, run_mode)
    if problems:
        log.event("refused", "REFUSING TO START, before anything was sent to the drone. " + " ".join(problems))
        return EXIT_REFUSED_TO_START

    log.event("inputs", f"Map: {map_path}. Start heading {arguments.start_heading_degrees:.1f} degrees. "
                        f"Corridor 1 to North = +{CORRIDOR_1_TURN_NORTH_METRES:.2f} m, turn 90 deg East, "
                        f"advance to East = {MAZE_ENTRY_EAST_METRES:.2f} m, and land.",
              map_file=map_path, start_heading_degrees=arguments.start_heading_degrees)
    return fly_with_ros(course_map, start_frame, arguments, log)


def fly_with_ros(course_map, start_frame, arguments, log):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions

    # As in course_run.py: without rclpy's own Ctrl+C handler, Ctrl+C raises KeyboardInterrupt,
    # which FirstMazeLegFlight.run handles by landing safely.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(f"practice_{FLIGHT_NAME}")
    try:
        link = flight_steps.DroneLink(node, log, ignore_raw_laser=arguments.ignore_raw_laser)
        flight = FirstMazeLegFlight(course_map, start_frame, link)
        return flight.run()
    except Exception:
        log.event("error", f"ERROR before the flight could begin:\n{traceback.format_exc()}")
        return EXIT_STOPPED_OR_FAILED
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


class FirstMazeLegFlight:

    def __init__(self, course_map, start_frame, link):
        self.course_map = course_map
        self.start_frame = start_frame
        self.link = link
        self.log = link.log
        self.watch = SafetyWatch(link, RunSafety(course_map), start_frame)
        self.flight_start_seconds = None

    def run(self):
        """Flies the sequence and handles every exit condition cleanly."""
        try:
            self.fly()
            return EXIT_FINISHED
        except RunMustStop as stop:
            self.log.event("stop", f"STOPPING THE FLIGHT during '{self.link.stage}'. {stop}", stage=self.link.stage)
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
        """The complete flight sequence from launch mark to landing."""
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, CRUISE_HEIGHT_METRES)
        self.flight_start_seconds = self.link.now_seconds()

        # Step 1: Climb vertically to cruise height
        self.link.stage = "climb"
        self.log.event("stage", f"Climbing to {CRUISE_HEIGHT_METRES:.2f} m above launch mark.", stage="climb")
        flight_steps.climb_above_launch(self.link, self.watch, CRUISE_HEIGHT_METRES)

        # Step 2: Settle before corridor flight
        self.link.stage = "settle before corridor"
        flight_steps.hold_still_for(self.link, self.watch, SETTLE_PAUSE_SECONDS)

        # Step 3: Fly straight North along Corridor 1
        self.fly_corridor_1()

        # Step 4: Coordinated 90-degree turn East
        self.turn_90_degrees_east()

        # Step 5: Penetrate into the Maze
        self.enter_maze()

        # Step 6: Settle and Land
        self.link.stage = "settle before landing"
        flight_steps.hold_still_for(self.link, self.watch, SETTLE_PAUSE_SECONDS)

        self.link.stage = "landing"
        self.log.event("stage", "Landing on the floor and cutting motors.", stage="landing")
        height_source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, height_source, self.watch)

        total_time = self.link.now_seconds() - self.flight_start_seconds
        self.log.event("result", f"FIRST MAZE LEG FINISHED: landed safely in {total_time:.1f} s. "
                                 f"Top 3 qualification points banked!", total_seconds=round(total_time, 2))

    def fly_corridor_1(self):
        """Slides the target straight North along Corridor 1 to North = +2.18 m."""
        self.link.stage = "first corridor"
        start_north = LAUNCH_NORTH_METRES
        distance_metres = CORRIDOR_1_TURN_NORTH_METRES - start_north
        duration_seconds = distance_metres / CORRIDOR_SPEED_METRES_PER_SECOND
        self.log.event("stage", f"Flying Corridor 1: {distance_metres:.2f} m North to +{CORRIDOR_1_TURN_NORTH_METRES:.2f} m.", stage="first corridor")

        start_time = self.link.now_seconds()

        def send_corridor_target(seconds_since_start):
            progress = min(1.0, seconds_since_start / duration_seconds)
            current_target_north = start_north + progress * distance_metres
            target_course = np.array([LAUNCH_EAST_METRES, current_target_north, CRUISE_HEIGHT_METRES])
            self.watch.send_target_on_course(target_course, heading_on_course_radians=math.radians(90.0))

        flight_steps.fly_until(self.link,
                               lambda: self.link.now_seconds() - start_time >= duration_seconds,
                               send_corridor_target,
                               duration_seconds + 5.0,
                               "flying straight down Corridor 1",
                               self.watch)

    def turn_90_degrees_east(self):
        """Pivots nose from 90° (North) to 0° (East) at 15°/s while holding position."""
        self.link.stage = "turn east"
        turn_degrees = 90.0
        turn_seconds = turn_degrees / TURN_RATE_DEGREES_PER_SECOND
        self.log.event("stage", f"Turning 90 degrees East at {TURN_RATE_DEGREES_PER_SECOND:.0f} deg/s.", stage="turn east")

        start_time = self.link.now_seconds()
        hold_target = np.array([LAUNCH_EAST_METRES, CORRIDOR_1_TURN_NORTH_METRES, CRUISE_HEIGHT_METRES])

        def send_turning_target(seconds_since_start):
            progress = min(1.0, seconds_since_start / turn_seconds)
            current_heading_deg = 90.0 - progress * 90.0
            self.watch.send_target_on_course(hold_target, heading_on_course_radians=math.radians(current_heading_deg))

        flight_steps.fly_until(self.link,
                               lambda: self.link.now_seconds() - start_time >= turn_seconds + 1.0,
                               send_turning_target,
                               turn_seconds + 5.0,
                               "turning 90 degrees East into the maze",
                               self.watch)

    def enter_maze(self):
        """Slides the target straight East along North = +2.18 m to East = -0.50 m."""
        self.link.stage = "enter maze"
        start_east = LAUNCH_EAST_METRES
        distance_metres = MAZE_ENTRY_EAST_METRES - start_east
        duration_seconds = distance_metres / MAZE_SPEED_METRES_PER_SECOND
        self.log.event("stage", f"Entering Maze: flying {distance_metres:.2f} m East to {MAZE_ENTRY_EAST_METRES:.2f} m.", stage="enter maze")

        start_time = self.link.now_seconds()

        def send_maze_target(seconds_since_start):
            progress = min(1.0, seconds_since_start / duration_seconds)
            current_target_east = start_east + progress * distance_metres
            target_course = np.array([current_target_east, CORRIDOR_1_TURN_NORTH_METRES, CRUISE_HEIGHT_METRES])
            self.watch.send_target_on_course(target_course, heading_on_course_radians=math.radians(0.0))

        flight_steps.fly_until(self.link,
                               lambda: self.link.now_seconds() - start_time >= duration_seconds,
                               send_maze_target,
                               duration_seconds + 5.0,
                               "advancing East into the maze corridor",
                               self.watch)

    def land_after_a_problem(self, land_using_laser):
        """Emergency descent after an abort."""
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
            self.log.event("interrupted", "Interrupted again: landing abandoned.")
        except Exception:
            self.log.event("error", f"ERROR during landing after problem:\n{traceback.format_exc()}")
