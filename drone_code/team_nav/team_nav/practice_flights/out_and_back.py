"""Practice flight "out_and_back": from a low hover, fly 1 m forward and back, then land, and
measure how quickly the drone follows its target. This is our first tuning measurement.

REAL DRONE CODE, also flown unchanged in the simulator (simulator/test_flights/real_program_test.py
plays the teammate's part). Start it with "ros2 run team_nav practice_flight out_and_back"
(run_practice_flight.py, which also checks practice mode and opens the records).

What it does
------------
  1. Before anything moves: reads the course map and checks, on the map, that the 1 m line ahead
     of the launch mark has room for the drone and keeps the bottom laser off every boulder.
     If not, it refuses to start.
  2. Waits until the drone is ready on the ground, then sends a target hover_height above the
     launch mark until a teammate arms the drone and flips the switch to Position, which hands
     control to us ("Offboard"). It never arms the drone or changes PX4's mode itself.
  3. Climbs to the hover height and holds still for a few seconds.
  4. OUT, a step: the target jumps 1 m forward at once and stays there for STEP_WATCH_SECONDS.
     From this we measure the response delay (the time to cover 63% of the step), the time to
     settle within 5 cm, and any overshoot (response_measurements.py).
  5. BACK, a slide: the target slides back to above the launch mark at SLIDE_SPEED, then stays
     there. From this we measure how far the drone runs behind a moving target.
  6. Lands above the launch mark with flight_steps' landing, and writes the results.

Every target goes through flight_steps' SafetyWatch, so the same safety checks run as in the
competition program (run_safety.py): a touch or PX4's height going wrong lands the drone where it
is; camera tracking going quiet makes it hold still; a target outside the map's course edges or
more than 1.2 m from the drone is refused and the drone lands.

The results
-----------
Printed on the screen and saved in the run's records folder (run_log.py):
  - events.jsonl, kind "measurement" and "result": the numbers, in sentences;
  - out_and_back_result.json: the same numbers, for a spreadsheet;
  - out_and_back_samples.csv: the target and camera tracking's forward position every cycle
    during the step and the slide, to plot;
  - drone_condition.csv and targets.csv, as in every flight (flight_steps.py).

What it does not do: plan a route (it flies one straight line, checked against the map), or
change any setting. It never stops after a chosen stage: TEAM_NAV_STOP_AFTER_STAGE is refused.
"""
import argparse
import json
import traceback

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import CourseMapError, distance_to_shapes_metres, load_course_map, shapes_blocking_between_heights
from team_nav.course_run import (
    DEFAULT_MAP_FILE, DEFAULT_START_HEADING_DEGREES, EXIT_FINISHED, EXIT_INTERRUPTED, EXIT_REFUSED_TO_START,
    EXIT_STOPPED_OR_FAILED, find_package_file,
)
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES, heights_spanned_metres
from team_nav.flight_steps import ControlIsNoLongerOurs, RunMustStop, SafetyWatch
from team_nav.practice_flights.response_measurements import (
    PositionSample, measure_step_response, measure_trailing_distance,
)
from team_nav.route_planner import LASER_KEEP_OFF_BOULDER_METRES
from team_nav.run_mode import STOP_AFTER_STAGE_SETTING
from team_nav.run_safety import RunSafety
from team_nav.start_frame import StartFrame

FLIGHT_NAME = "out_and_back"

# From docs/ARCHITECTURE.md, section 11, the test order: "fly 1 m out and back".
OUT_DISTANCE_METRES = 1.0

# Our own choice: a low hover, as the test order asks, the same height as the rise-and-land
# smoke flight. Give --hover-height-metres 0.8 to measure at the course's cruise height.
DEFAULT_HOVER_HEIGHT_METRES = 0.5
# Our own choices. Below 0.4 m the drone is close to the floor, where the air it pushes down
# changes how it flies, and run_safety.py's laser check only works above 0.3 m. Above 1.0 m it
# would be close to the mock walls' 1.2 m tops.
LOWEST_HOVER_HEIGHT_METRES = 0.4
HIGHEST_HOVER_HEIGHT_METRES = 1.0

# Our own choice: a short still hover before the step, so the step starts from rest.
HOVER_BEFORE_STEP_SECONDS = 3.0
# Our own choice. Section 9 estimates 2.5 s to settle within 5 cm, but in the first simulator
# flight (2026-11-07) the drone overshot by 16 cm and was still 7 cm past the target after 6 s.
# 10 s shows the whole step response, so the time to settle can be measured.
STEP_WATCH_SECONDS = 10.0
# Our own choice. Slow enough that the 1 m slide lasts 4 s, about three time constants, so the
# drone's distance behind the target has time to become steady before the slide ends.
SLIDE_SPEED_METRES_PER_SECOND = 0.25
# Our own choice: the same as the step, so the drone is settled above the launch mark before landing.
HOLD_AFTER_SLIDE_SECONDS = 6.0
# Our own choice: the room check looks at the line every 2 cm.
ROOM_CHECK_SPACING_METRES = 0.02

STEP_STAGE = "out: step 1 m forward"
SLIDE_STAGE = "back: slide to above the launch mark"
HOLD_STAGE = "back: hold above the launch mark"


# =================================================================================================
# Before anything moves
# =================================================================================================

def parse_arguments(command_line_words, flight_name=FLIGHT_NAME):
    """The settings of out_and_back and of turn_in_place, which hover in the same way."""
    parser = argparse.ArgumentParser(
        prog=f"practice_flight {flight_name}",
        description="Hover above the launch mark, do this flight's moves (its file says which), and land.")
    parser.add_argument("--map", default=DEFAULT_MAP_FILE,
                        help="The map of where the drone flies: a path, or a file name in team_nav's course_maps folder. "
                             "Its launch mark must be where the drone sits.")
    parser.add_argument("--start-heading-degrees", type=float, default=DEFAULT_START_HEADING_DEGREES,
                        help="Which way the drone's nose points, in degrees from the map's east towards north.")
    parser.add_argument("--hover-height-metres", type=float, default=DEFAULT_HOVER_HEIGHT_METRES,
                        help=f"Height of the hover, from {LOWEST_HOVER_HEIGHT_METRES} to {HIGHEST_HOVER_HEIGHT_METRES} m.")
    parser.add_argument("--ignore-raw-laser", action="store_true",
                        help="Behave as if the bottom laser's raw reading did not exist, as on the real drone.")
    return parser.parse_args(command_line_words)


def problems_with_the_settings(arguments, run_mode):
    """Sentences saying why the flight must not start, or an empty list."""
    problems = []
    if run_mode.stop_after_stage:
        problems.append(f"{STOP_AFTER_STAGE_SETTING} is '{run_mode.stop_after_stage}', but this flight has no stages to "
                        f"stop after. Remove {STOP_AFTER_STAGE_SETTING}.")
    if not LOWEST_HOVER_HEIGHT_METRES <= arguments.hover_height_metres <= HIGHEST_HOVER_HEIGHT_METRES:
        problems.append(f"The hover height {arguments.hover_height_metres} m is outside {LOWEST_HOVER_HEIGHT_METRES} to "
                        f"{HIGHEST_HOVER_HEIGHT_METRES} m. Choose a height in that range.")
    return problems


def problems_with_the_line_flown(course_map, start_frame, hover_height_metres):
    """Checks, on the map, the straight line from above the launch mark to 1 m forward. Returns
    sentences saying what is wrong, or an empty list."""
    points_on_course = []
    for forward_metres in np.arange(0.0, OUT_DISTANCE_METRES + ROOM_CHECK_SPACING_METRES / 2, ROOM_CHECK_SPACING_METRES):
        points_on_course.append(start_frame.course_from_start_frame((forward_metres, 0.0, hover_height_metres))[:2])
    points_on_course = np.array(points_on_course)

    problems = []
    shapes = shapes_blocking_between_heights(course_map, *heights_spanned_metres(hover_height_metres))
    room_metres, nearest_names = distance_to_shapes_metres(points_on_course, shapes)
    tightest = int(np.argmin(room_metres))
    if room_metres[tightest] < REQUIRED_ROOM_METRES:
        problems.append(f"On the map '{course_map.name}', the line 1 m forward from the launch mark passes "
                        f"{room_metres[tightest]:.2f} m from {nearest_names[tightest]}, but the drone needs "
                        f"{REQUIRED_ROOM_METRES:.2f} m. Fly it from a launch mark with 1 m of clear floor ahead, and give "
                        f"that place's map with --map.")
    for boulder in course_map.low_obstacles:
        distance_to_edge_metres = float(np.min(np.linalg.norm(points_on_course - boulder.centre_metres, axis=1))) - boulder.radius_metres
        if distance_to_edge_metres < LASER_KEEP_OFF_BOULDER_METRES:
            problems.append(f"The line 1 m forward passes {distance_to_edge_metres:.2f} m from the edge of {boulder.name}, "
                            f"closer than {LASER_KEEP_OFF_BOULDER_METRES:.2f} m, so the bottom laser could see it and PX4's "
                            f"height could jump. Move the boulder or fly elsewhere.")
    return problems


# =================================================================================================
# Flying
# =================================================================================================

class OutAndBack:

    def __init__(self, course_map, start_frame, hover_height_metres, link):
        self.start_frame = start_frame
        self.hover_height_metres = hover_height_metres
        self.link = link
        self.log = link.log
        self.watch = SafetyWatch(link, RunSafety(course_map), start_frame)
        self.results = {}

    def run(self):
        """Flies, and handles every way the flight can end, as course_run.py does. Returns the exit code."""
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
        finally:
            self.save_results()
        return EXIT_STOPPED_OR_FAILED

    def fly(self):
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, self.hover_height_metres)
        self.link.stage = "climb"
        flight_steps.climb_above_launch(self.link, self.watch, self.hover_height_metres)
        self.link.stage = "hover before the step"
        flight_steps.hold_still_for(self.link, self.watch, HOVER_BEFORE_STEP_SECONDS)
        self.step_out()
        self.slide_back()
        height_source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, height_source, self.watch)
        self.log.event("result", "OUT AND BACK FINISHED: landed above the launch mark and the motors are off. "
                                 + " ".join(self.results.get("sentences", [])), **self.numbers_only())

    def step_out(self):
        """The target jumps OUT_DISTANCE_METRES forward and stays there for STEP_WATCH_SECONDS."""
        self.link.stage = STEP_STAGE
        start_forward_metres = self.drone_forward_metres()
        samples = []
        step_start_seconds = self.link.now_seconds()
        self.log.event("stage", f"The target jumps {OUT_DISTANCE_METRES:.1f} m forward, from a drone at "
                                f"{start_forward_metres:+.3f} m.", stage=STEP_STAGE)
        self.fly_with_target_for(lambda _seconds_since_start: OUT_DISTANCE_METRES, STEP_WATCH_SECONDS, samples)
        step = measure_step_response(samples, step_start_seconds, start_forward_metres, OUT_DISTANCE_METRES)
        self.remember("step", step.describe(), step.__dict__)

    def slide_back(self):
        """The target slides back to above the launch mark at SLIDE_SPEED_METRES_PER_SECOND, then
        stays there for HOLD_AFTER_SLIDE_SECONDS."""
        self.link.stage = SLIDE_STAGE
        slide_seconds = OUT_DISTANCE_METRES / SLIDE_SPEED_METRES_PER_SECOND
        samples = []
        slide_start_seconds = self.link.now_seconds()
        self.log.event("stage", f"The target slides back at {SLIDE_SPEED_METRES_PER_SECOND:.2f} m/s.", stage=SLIDE_STAGE)

        def target_during_slide(seconds_since_start):
            return max(0.0, OUT_DISTANCE_METRES - SLIDE_SPEED_METRES_PER_SECOND * seconds_since_start)

        self.fly_with_target_for(target_during_slide, slide_seconds, samples)
        slide_end_seconds = self.link.now_seconds()
        trailing = measure_trailing_distance(samples, slide_start_seconds, slide_end_seconds, -SLIDE_SPEED_METRES_PER_SECOND)
        self.remember("slide", trailing.describe(), trailing.__dict__)
        self.link.stage = HOLD_STAGE
        self.fly_with_target_for(lambda _seconds_since_start: 0.0, HOLD_AFTER_SLIDE_SECONDS, [])

    def fly_with_target_for(self, forward_target_after, seconds, samples):
        """For this many seconds, every cycle: send the target forward_target_after(seconds since
        this began) metres forward of the launch mark, at the hover height, with the safety checks
        running, and add a PositionSample to samples."""
        start_seconds = self.link.now_seconds()

        def send_target():
            now = self.link.now_seconds()
            target_forward_metres = forward_target_after(now - start_seconds)
            self.record_sample(samples, now, target_forward_metres)
            target_on_course = self.start_frame.course_from_start_frame((target_forward_metres, 0.0, self.hover_height_metres))
            self.watch.send_target_on_course(target_on_course)

        flight_steps.fly_until(self.link, lambda: self.link.now_seconds() - start_seconds >= seconds, send_target,
                               seconds + 1.0, self.link.stage, self.watch)

    def record_sample(self, samples, now_seconds, target_forward_metres):
        from_launch = self.link.position_from_launch()
        if from_launch is None:
            return  # The safety checks hold the drone still when camera tracking is silent.
        samples.append(PositionSample(now_seconds, target_forward_metres, float(from_launch[0])))
        self.log.sample("out_and_back_samples", stage=self.link.stage, target_forward_metres=target_forward_metres,
                        drone_forward_metres=from_launch[0], drone_left_metres=from_launch[1],
                        drone_up_metres=from_launch[2])

    def drone_forward_metres(self):
        from_launch = self.link.position_from_launch()
        if from_launch is None:
            raise RunMustStop("Camera tracking has no position, so the step can't start. Check that cuVSLAM is running.")
        return float(from_launch[0])

    # ---- Results ---------------------------------------------------------------------------------

    def remember(self, name, sentence, numbers):
        self.log.event("measurement", sentence, measurement=name, **numbers)
        self.results.setdefault("sentences", []).append(sentence)
        self.results[name] = numbers

    def numbers_only(self):
        return {name: value for name, value in self.results.items() if name != "sentences"}

    def save_results(self):
        record = {"hover_height_metres": self.hover_height_metres, "out_distance_metres": OUT_DISTANCE_METRES,
                  "slide_speed_metres_per_second": SLIDE_SPEED_METRES_PER_SECOND}
        record.update(self.results)
        self.log.save_text_file("out_and_back_result.json", json.dumps(record, indent=2, default=str) + "\n")

    # ---- After a problem: the same as course_run.py ---------------------------------------------

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
# Starting the flight
# =================================================================================================

def run(command_line_words, run_mode, log):
    """Checks everything that can be checked on the ground, then flies. Returns the exit code."""
    return run_flight(command_line_words, run_mode, log, FLIGHT_NAME, problems_with_the_line_flown, OutAndBack)


def run_flight(command_line_words, run_mode, log, flight_name, problems_with_the_place, flight_class):
    """Shared with turn_in_place.py, which hovers in the same way: reads the settings and the map,
    refuses to start if problems_with_the_place(course_map, start_frame, hover height) finds a
    problem, then flies flight_class. Returns the exit code."""
    arguments = parse_arguments(command_line_words, flight_name)
    try:
        map_path = find_package_file(arguments.map, "course_maps")
        course_map = load_course_map(map_path)
    except (OSError, CourseMapError) as problem:
        log.event("refused", f"REFUSING TO START, before anything was sent to the drone. {problem}")
        return EXIT_REFUSED_TO_START
    start_frame = StartFrame(course_map.launch_mark_centre_metres, arguments.start_heading_degrees)
    problems = problems_with_the_settings(arguments, run_mode)
    problems += problems_with_the_place(course_map, start_frame, arguments.hover_height_metres)
    if problems:
        log.event("refused", "REFUSING TO START, before anything was sent to the drone. " + " ".join(problems))
        return EXIT_REFUSED_TO_START
    log.event("inputs", f"Flight {flight_name}. Map: {map_path}. Start heading {arguments.start_heading_degrees:.1f} "
                        f"degrees. Hover at {arguments.hover_height_metres:.2f} m.", flight=flight_name,
              map_file=map_path, start_heading_degrees=arguments.start_heading_degrees,
              hover_height_metres=arguments.hover_height_metres)
    return fly_with_ros(course_map, start_frame, arguments, log, flight_name, flight_class)


def fly_with_ros(course_map, start_frame, arguments, log, flight_name, flight_class):
    import rclpy
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    # As in course_run.py: without rclpy's own Ctrl+C handler, Ctrl+C raises KeyboardInterrupt,
    # which OutAndBack.run handles by landing.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node(f"practice_{flight_name}")
    try:
        link = flight_steps.DroneLink(node, log, ignore_raw_laser=arguments.ignore_raw_laser)
        return flight_class(course_map, start_frame, arguments.hover_height_metres, link).run()
    except Exception:
        log.event("error", f"ERROR before the flight could begin:\n{traceback.format_exc()}")
        return EXIT_STOPPED_OR_FAILED
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
