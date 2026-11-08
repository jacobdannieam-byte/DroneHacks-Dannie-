"""Fly the checked sketch-map route through four checkpoints and land on the pad.

REAL DRONE CODE, tested in Gazebo. Uses measured arrival before advancing, a short target
lead, and a fixed north heading. It does not change PX4 parameters or use simulator truth.
"""
import math
import numpy as np

from team_nav import flight_steps
from team_nav.course_map import load_course_map
from team_nav.course_run import EXIT_REFUSED_TO_START, find_package_file
from team_nav.practice_flights import first_maze_leg
from team_nav.practice_flights.full_course_route import CRUISE_HEIGHT_METRES, route_waypoints
from team_nav.start_frame import StartFrame

FLIGHT_NAME = 'full_course'
# Our own choices, conservative relative to the original first-leg speeds.
SPEED_METRES_PER_SECOND = 0.25
TARGET_LEAD_METRES = 0.25
ARRIVAL_TOLERANCE_METRES = 0.025
ARRIVAL_HEIGHT_TOLERANCE_METRES = 0.08
SETTLE_SECONDS = 1.0
# Our own choice: slower descent after the first full-course trial showed 12.7 cm of
# estimator lag during the original 0.5/0.35 m/s landing, with 8.7 cm or less on the route.
LANDING_SPEED_METRES_PER_SECOND = 0.15


def run(words, run_mode, log):
    arguments = first_maze_leg.parse_arguments(words)
    try:
        course_map = load_course_map(find_package_file(arguments.map, 'course_maps'))
        route_waypoints(course_map)
        if first_maze_leg.problems_with_the_settings(arguments, run_mode):
            raise ValueError('The full-course practice flight does not accept stop-after-stage.')
    except (OSError, ValueError) as problem:
        log.event('refused', f'Full-course route refused before sending targets: {problem}')
        return EXIT_REFUSED_TO_START
    frame = StartFrame(course_map.launch_mark_centre_metres, arguments.start_heading_degrees)
    return first_maze_leg.fly_with_ros(course_map, frame, arguments, log, flight_class=FullCourseFlight)


class FullCourseFlight(first_maze_leg.FirstMazeLegFlight):
    def fly(self):
        points = route_waypoints(self.course_map)
        self.log.save_text_file('planned_waypoints.csv', 'east,north,height\n' +
                               '\n'.join(','.join(map(str, p)) for p in points))
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, CRUISE_HEIGHT_METRES)
        self.flight_start_seconds = self.link.now_seconds()
        self.link.stage = 'climb'
        flight_steps.climb_above_launch(self.link, self.watch, CRUISE_HEIGHT_METRES)
        for number, (start, end) in enumerate(zip(points, points[1:]), 1):
            self.link.stage = f'full course leg {number}'
            self.log.event('stage', f'{self.link.stage}: target {end.tolist()}', stage=self.link.stage)
            self.fly_leg(start, end)
            flight_steps.hold_still_for(self.link, self.watch, SETTLE_SECONDS)
        self.link.stage = 'landing on pad'
        source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, source, self.watch, descent_speed_limit=LANDING_SPEED_METRES_PER_SECOND)
        elapsed = self.link.now_seconds() - self.flight_start_seconds
        self.log.event('result', f'FULL COURSE FINISHED: motors off after {elapsed:.1f} s. '
                       'Check the independent referee for checkpoints, contacts and height error.',
                       total_seconds=round(elapsed, 2))

    def fly_leg(self, start, end):
        direction = end - start
        length = float(np.linalg.norm(direction))
        direction = direction / length
        target = start.copy()
        previous_seconds = self.link.now_seconds()

        def arrived():
            position = self.watch.drone_on_course()
            return (np.allclose(target, end) and position is not None
                    and np.linalg.norm(position[:2] - end[:2]) < ARRIVAL_TOLERANCE_METRES
                    and abs(position[2] - end[2]) < ARRIVAL_HEIGHT_TOLERANCE_METRES)

        def send_next_target():
            nonlocal target, previous_seconds
            now = self.link.now_seconds()
            # A safety hold must not cause a target jump when tracking resumes.
            seconds = min(now - previous_seconds, flight_steps.CYCLE_SECONDS * 2)
            previous_seconds = now
            position = self.watch.drone_on_course()
            if position is not None and np.linalg.norm(target[:2] - position[:2]) < TARGET_LEAD_METRES:
                remaining = float(np.linalg.norm(end - target))
                target = end.copy() if remaining <= SPEED_METRES_PER_SECOND * seconds else (
                    target + direction * SPEED_METRES_PER_SECOND * seconds)
            self.watch.send_target_on_course(target, heading_on_course_radians=math.pi / 2)

        flight_steps.fly_until(self.link, arrived, send_next_target, length / SPEED_METRES_PER_SECOND + 30,
                               f'arrival at {end.tolist()}', self.watch)
