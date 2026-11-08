"""Practice flight "turn_in_place": hover above the launch mark, turn on the spot, and measure how
far the drone slides away while it turns. Then land.

REAL DRONE CODE. Start it with "ros2 run team_nav practice_flight turn_in_place"
(run_practice_flight.py, which also checks practice mode and opens the records).

Why this flight exists
----------------------
In the course runs of 2026-11-08 the drone slid, and sometimes moved the wrong way, while it
turned, so it never got far along the course. This flight does only the turns, with nothing else
going on, so we can see if that problem is gone. The turns are the same as on the course: 15
degrees a second (live_flight.TURN_RATE_DEGREES_PER_SECOND), and one full circle, as live
navigation does when it finds no path.

What it does
------------
  1. Before anything moves: checks on the map that the point above the launch mark has room for
     the drone. If not, it refuses to start.
  2. Waits until the drone is ready on the ground (this includes "PX4 uses camera tracking for
     position and heading"), then holds a target above the launch mark until a teammate arms the
     drone and flips the switch to Position.
  3. Climbs, and holds still for a few seconds.
  4. Turns 90 degrees left, holds; turns back 90 degrees right, holds; turns one full circle
     left, holds. The position target stays above the launch mark the whole time.
  5. Lands above the launch mark.

The position target never moves, so any movement of the drone is a slide. For each turn and hold
it reports the largest distance from the point above the launch mark, and it saves every
cycle in turn_in_place_samples.csv.

Every target goes through the same safety checks as in the competition program (run_safety.py),
including the check that lands the drone if PX4 stops following camera tracking's heading.
The ways the flight can end, and the landing after a problem, are those of out_and_back.py.

What it does not do: fly anywhere, or change any setting.
"""
import json
import math

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES, heights_spanned_metres
from team_nav.live_flight import TURN_RATE_DEGREES_PER_SECOND
from team_nav.practice_flights import out_and_back
from team_nav.practice_flights.out_and_back import OutAndBack

FLIGHT_NAME = "turn_in_place"

# Our own choices: still hovers before the turns and between them, long enough to see whether a
# slide stops once the turning stops.
HOVER_BEFORE_TURNS_SECONDS = 3.0
HOLD_AFTER_EACH_TURN_SECONDS = 5.0

# The turns, in order: (name, degrees, positive is to the left). The full circle is the same as
# live navigation's look-around (live_navigation.start_look_around).
TURNS = [
    ("turn 90 degrees left", 90.0),
    ("turn 90 degrees right, back to the start heading", -90.0),
    ("turn one full circle left", 360.0),
]


class TurnInPlace(OutAndBack):
    """Uses OutAndBack's run(), which handles every way a flight can end, and its landing after a
    problem. Only the flying and the results are different."""

    def fly(self):
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, self.hover_height_metres)
        self.link.stage = "climb"
        flight_steps.climb_above_launch(self.link, self.watch, self.hover_height_metres)
        self.link.stage = "hover before the turns"
        flight_steps.hold_still_for(self.link, self.watch, HOVER_BEFORE_TURNS_SECONDS)
        heading_on_course_radians = self.start_frame.course_heading_from_start_frame(0.0)
        for name, degrees in TURNS:
            heading_on_course_radians = self.turn(name, heading_on_course_radians, math.radians(degrees))
            self.hold(f"hold after: {name}", heading_on_course_radians)
        height_source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, height_source, self.watch)
        self.log.event("result", "TURN IN PLACE FINISHED: landed above the launch mark and the motors are off. "
                                 + " ".join(self.results.get("sentences", [])), **self.numbers_only())

    def turn(self, name, start_heading_radians, turn_radians):
        """Turns the nose by turn_radians at the course's turn rate, holding position. Returns the
        heading at the end."""
        turn_seconds = abs(math.degrees(turn_radians)) / TURN_RATE_DEGREES_PER_SECOND
        turn_rate_radians_per_second = math.copysign(math.radians(TURN_RATE_DEGREES_PER_SECOND), turn_radians)

        def heading_after(seconds_since_start):
            if seconds_since_start >= turn_seconds:
                return start_heading_radians + turn_radians
            return start_heading_radians + turn_rate_radians_per_second * seconds_since_start

        self.fly_holding_position(name, heading_after, turn_seconds)
        return start_heading_radians + turn_radians

    def hold(self, name, heading_radians):
        self.fly_holding_position(name, lambda _seconds_since_start: heading_radians, HOLD_AFTER_EACH_TURN_SECONDS)

    def fly_holding_position(self, name, heading_after, seconds):
        """For this many seconds, every cycle: send the target above the launch mark with the heading
        heading_after(seconds since this began), and note how far the drone is from that point."""
        self.link.stage = name
        self.log.event("stage", f"{name}, holding position above the launch mark.", stage=name)
        target_on_course = self.start_frame.course_from_start_frame((0.0, 0.0, self.hover_height_metres))
        start_seconds = self.link.now_seconds()
        largest_slide = {"metres": 0.0}

        def send_target():
            heading = heading_after(self.link.now_seconds() - start_seconds)
            self.note_slide(name, heading, largest_slide)
            self.watch.send_target_on_course(target_on_course, heading_on_course_radians=heading)

        flight_steps.fly_until(self.link, lambda: self.link.now_seconds() - start_seconds >= seconds, send_target,
                               seconds + 1.0, name, self.watch)
        sentence = f"{name}: the drone slid at most {largest_slide['metres']:.2f} m from the point above the launch mark."
        self.remember(name, sentence, {"largest_slide_metres": largest_slide["metres"]})

    def note_slide(self, name, target_heading_on_course_radians, largest_slide):
        from_launch = self.link.position_from_launch()
        if from_launch is None:
            return  # The safety checks hold the drone still when camera tracking is silent.
        slide_metres = float(math.hypot(from_launch[0], from_launch[1]))
        largest_slide["metres"] = max(largest_slide["metres"], slide_metres)
        self.log.sample("turn_in_place_samples", stage=name, slide_metres=slide_metres,
                        drone_forward_metres=from_launch[0], drone_left_metres=from_launch[1],
                        drone_up_metres=from_launch[2],
                        target_heading_on_course_degrees=math.degrees(target_heading_on_course_radians),
                        drone_heading_on_course_degrees=_degrees_or_none(self.watch.drone_heading_on_course()),
                        px4_heading_gap_degrees=self.link.px4_heading_gap_degrees())

    def save_results(self):
        record = {"hover_height_metres": self.hover_height_metres, "turns": TURNS}
        record.update(self.results)
        self.log.save_text_file("turn_in_place_result.json", json.dumps(record, indent=2, default=str) + "\n")


def _degrees_or_none(radians):
    return None if radians is None else math.degrees(radians)


def problems_with_the_place(course_map, start_frame, hover_height_metres):
    """Checks, on the map, that the point above the launch mark has room for the drone. Returns
    sentences saying what is wrong, or an empty list."""
    point_on_course = np.array([start_frame.course_from_start_frame((0.0, 0.0, hover_height_metres))[:2]])
    shapes = shapes_blocking_between_heights(course_map, *heights_spanned_metres(hover_height_metres))
    room_metres, nearest_names = distance_to_shapes_metres(point_on_course, shapes)
    if room_metres[0] < REQUIRED_ROOM_METRES:
        return [f"On the map '{course_map.name}', the point above the launch mark is {room_metres[0]:.2f} m from "
                f"{nearest_names[0]}, but the drone needs {REQUIRED_ROOM_METRES:.2f} m. Fly it from a launch mark "
                f"with clear floor all round, and give that place's map with --map."]
    return []


def run(command_line_words, run_mode, log):
    """The same start as out_and_back.run, with this flight's own check of the place and its own
    flight. Returns the exit code."""
    return out_and_back.run_flight(command_line_words, run_mode, log, FLIGHT_NAME,
                                   problems_with_the_place, TurnInPlace)
