"""Fly five course targets with measured stop/turn/advance at intermediate corners.

Real ROS flight code: consumes the checked mission and ForwardDepth guard. It neither
replans the surveyed route nor reads simulator truth. Yaw means the nose's compass direction.
"""
import json
import math
import numpy as np

from team_nav import flight_steps
from team_nav.course_map import load_course_map
from team_nav.course_run import EXIT_REFUSED_TO_START, find_package_file
from team_nav.practice_flights import first_maze_leg
from team_nav.practice_flights.five_waypoint_plan import angle_difference, segments, validate_plan
from team_nav.practice_flights.forward_depth import ForwardDepth
from team_nav.practice_flights.full_course_route import CRUISE_HEIGHT_METRES
from team_nav.practice_flights.full_course import (
    SPEED_METRES_PER_SECOND, TARGET_LEAD_METRES, ARRIVAL_TOLERANCE_METRES,
    ARRIVAL_HEIGHT_TOLERANCE_METRES, SETTLE_SECONDS, LANDING_SPEED_METRES_PER_SECOND,
)
from team_nav.start_frame import StartFrame
from team_nav.height_keeper import HeightKeeper

FLIGHT_NAME = 'five_waypoints'
# Our choices: gentle yaw, measured alignment before motion, and bounded waits for depth.
TURN_RATE_RADIANS_PER_SECOND = math.radians(15)
TURN_TOLERANCE_RADIANS = math.radians(5)
TRAVEL_TOLERANCE_RADIANS = math.radians(8)
MAXIMUM_HOLD_SECONDS = 20
MAXIMUM_CYCLE_SECONDS = flight_steps.CYCLE_SECONDS * 2
TURN_EXTRA_WAIT_SECONDS = 30
TRAVEL_EXTRA_WAIT_SECONDS = 60
COMMAND_ALIGNMENT_RADIANS = 1e-6
# Our choice: correct small offsets, with half the existing live navigator's 0.30 m limit.
MAXIMUM_HEIGHT_CORRECTION_METRES = .15


def turn_towards(current, wanted, seconds, rate=TURN_RATE_RADIANS_PER_SECOND):
    step = np.clip(angle_difference(wanted, current),
                   -rate * seconds,
                   rate * seconds)
    return angle_difference(current + step, 0)


def run(words, run_mode, log):
    arguments = first_maze_leg.parse_arguments(words)
    try:
        course_map = load_course_map(find_package_file(arguments.map, 'course_maps'))
        with open(find_package_file('five_waypoints.json', 'missions'), encoding='utf-8') as source:
            plan = json.load(source)
        validate_plan(plan, course_map)
        if first_maze_leg.problems_with_the_settings(arguments, run_mode):
            raise ValueError('Five-waypoint flight does not accept stop-after-stage.')
        if not math.isclose(arguments.start_heading_degrees, 90):
            raise ValueError('This checked mission starts facing north (90 degrees).')
    except (OSError, ValueError, KeyError) as problem:
        log.event('refused', f'Five-waypoint mission refused before sending targets: {problem}')
        return EXIT_REFUSED_TO_START
    frame = StartFrame(course_map.launch_mark_centre_metres, arguments.start_heading_degrees)
    def make_flight(*args, **kwargs):
        return FiveWaypointFlight(*args, plan=plan, **kwargs)
    return first_maze_leg.fly_with_ros(course_map, frame, arguments, log, flight_class=make_flight)


class FiveWaypointFlight(first_maze_leg.FirstMazeLegFlight):
    def __init__(self, *args, plan, **kwargs):
        super().__init__(*args, **kwargs)
        self.plan = plan
        self.depth = ForwardDepth(self.link, self.start_frame)
        self.height_keeper = HeightKeeper()

    def correct_height(self, wanted, actual, seconds):
        if actual is None:
            return
        correction = self.height_keeper.update(wanted, actual[2], seconds)
        self.height_keeper.correction_metres = float(np.clip(correction,
            -MAXIMUM_HEIGHT_CORRECTION_METRES, MAXIMUM_HEIGHT_CORRECTION_METRES))
        self.link.height_correction_metres = self.height_keeper.correction_metres

    def fly(self):
        route = list(segments(self.plan))
        points = [route[0][1]] + [leg[2] for leg in route]
        self.log.save_text_file('planned_waypoints.csv', 'east,north,height\n' +
                               '\n'.join(','.join(map(str, p)) for p in points))
        self.log.save_text_file('mission.json', json.dumps(self.plan, indent=2))
        flight_steps.wait_until_ready_on_the_ground(self.link)
        flight_steps.hold_above_launch_until_control_is_ours(self.link, CRUISE_HEIGHT_METRES)
        self.flight_start_seconds = self.link.now_seconds()
        self.link.stage = 'climb'
        flight_steps.climb_above_launch(self.link, self.watch, CRUISE_HEIGHT_METRES)
        for stage, start, end, previous, heading, reverse in route:
            self.link.stage = stage['name'] + ': turn'
            self.turn_and_settle(start, heading)
            self.link.stage = stage['name'] + (': retreat' if reverse else ': advance')
            self.log.event('stage', f'{self.link.stage}, target {end.tolist()}, heading {math.degrees(heading):.1f}',
                           stage=self.link.stage)
            self.advance(start, end, heading, reverse)
        self.link.stage = 'landing on pad'
        source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, source, self.watch, descent_speed_limit=LANDING_SPEED_METRES_PER_SECOND)
        self.log.event('result', 'PLANNED ROUTE FINISHED. Check the independent referee for results.',
                       total_seconds=round(self.link.now_seconds()-self.flight_start_seconds, 2),
                       depth_pictures=self.depth.messages_used)

    def turn_and_settle(self, position, heading, turn_rate=TURN_RATE_RADIANS_PER_SECOND):
        commanded = self.watch.drone_heading_on_course()
        if commanded is None:
            raise flight_steps.RunMustStop('No tracked heading available before turn.')
        longest = abs(angle_difference(heading, commanded))/turn_rate + TURN_EXTRA_WAIT_SECONDS
        previous = self.link.now_seconds()
        settled = None
        complete = False
        def send():
            nonlocal commanded, previous, settled, complete
            now = self.link.now_seconds()
            if now-previous > MAXIMUM_CYCLE_SECONDS:
                settled = None  # A safety hold cannot count toward continuous settling.
            seconds = min(now-previous, MAXIMUM_CYCLE_SECONDS)
            commanded = turn_towards(commanded, heading, seconds, turn_rate)
            previous = now
            measured = self.watch.drone_heading_on_course()
            actual = self.watch.drone_on_course()
            self.correct_height(position[2], actual, seconds)
            aligned = (measured is not None and actual is not None
                       and abs(angle_difference(heading, measured)) < TURN_TOLERANCE_RADIANS
                       and np.linalg.norm(actual[:2]-position[:2]) < ARRIVAL_TOLERANCE_METRES
                       and abs(actual[2]-position[2]) < ARRIVAL_HEIGHT_TOLERANCE_METRES
                       and abs(angle_difference(heading, commanded)) < COMMAND_ALIGNMENT_RADIANS)
            settled = (now if settled is None else settled) if aligned else None
            complete = settled is not None and now-settled >= SETTLE_SECONDS
            self.watch.send_target_on_course(position, heading_on_course_radians=commanded)
        flight_steps.fly_until(self.link, lambda: complete, send, longest, 'heading and position to settle', self.watch)

    def advance(self, start, end, heading, reverse):
        direction = end-start
        length = float(np.linalg.norm(direction))
        direction /= length
        target = start.copy()
        previous = self.link.now_seconds()
        hold_since = settled = None
        complete = False
        last_reason = None
        def send():
            nonlocal target, previous, hold_since, settled, complete, last_reason
            now = self.link.now_seconds()
            if now-previous > MAXIMUM_CYCLE_SECONDS:
                settled = None
            seconds = min(now-previous, MAXIMUM_CYCLE_SECONDS)
            previous = now
            actual = self.watch.drone_on_course()
            self.correct_height(end[2], actual, seconds)
            measured = self.watch.drone_heading_on_course()
            reason = 'tracking unavailable' if actual is None or measured is None else None
            if reason is None and abs(angle_difference(heading, measured)) > TRAVEL_TOLERANCE_RADIANS:
                reason = 'heading is not aligned with the segment'
            if reason is None:
                reason = self.depth.reason_to_hold(actual, end, heading, reverse)
            if reason:
                if hold_since is None:
                    hold_since = now
                    if actual is not None:
                        target = actual.copy()
                        target[2] = end[2]
                settled = None
                if reason != last_reason:
                    self.log.event('hold', reason, stage=self.link.stage)
                if now-hold_since > MAXIMUM_HOLD_SECONDS:
                    raise flight_steps.RunMustStop(reason + ' for more than 20 seconds.')
            else:
                if hold_since is not None:
                    # Resume along the checked line without jumping to an old moving target.
                    target = start + direction*np.clip(np.dot(actual-start, direction), 0, length)
                    self.log.event('resume', 'Depth and heading allow the next step.', stage=self.link.stage)
                hold_since = None
                if np.linalg.norm(target[:2]-actual[:2]) < TARGET_LEAD_METRES:
                    remaining = float(np.linalg.norm(end-target))
                    target = end.copy() if remaining <= SPEED_METRES_PER_SECOND*seconds else target+direction*SPEED_METRES_PER_SECOND*seconds
                arrived = (np.allclose(target, end)
                           and np.linalg.norm(actual[:2]-end[:2]) < ARRIVAL_TOLERANCE_METRES
                           and abs(actual[2]-end[2]) < ARRIVAL_HEIGHT_TOLERANCE_METRES)
                settled = (now if settled is None else settled) if arrived else None
                complete = settled is not None and now-settled >= SETTLE_SECONDS
            last_reason = reason
            self.watch.send_target_on_course(target, heading_on_course_radians=heading)
        flight_steps.fly_until(self.link, lambda: complete, send, length/SPEED_METRES_PER_SECOND+TRAVEL_EXTRA_WAIT_SECONDS,
                               f'arrival at {end.tolist()}', self.watch)
