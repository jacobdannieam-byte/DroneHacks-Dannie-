"""Fly the checked sketch-map route through four checkpoints and land on the pad.

REAL DRONE CODE, tested in Gazebo. Start it on the drone with:

    TEAM_NAV_RUN_MODE=practice ./drone.sh practice_flight full_course

It flies the fixed route in full_course_route.py with the nose pointing north throughout. Each
leg slides a target along the straight line between two waypoints, never more than
TARGET_LEAD_METRES ahead of the drone, and the next leg starts only once the drone has arrived.
It does not change PX4 parameters or use simulator truth.

Four additions, each on unless switched off
-------------------------------------------
Added on 2026-11-08 on top of Aditya's version (Gazebo run 20261108_122536). Each one has its
own option to switch it off. --baseline switches off all four, which gives back his version.

  1. Start check (--no-start-check). Before anything is sent, refuse to start if camera
     tracking is far from where it started (tracking_start_check.py).
  2. Takeoff settle (--no-takeoff-settle). Climb with the launch mark as the target, then wait
     until the drone holds steady over it (TAKEOFF_SETTLE_RULE) before the first leg. If it is
     not steady within LONGEST_TAKEOFF_SETTLE_SECONDS, land. Records the largest sideways
     distance from the launch mark during the climb. When switched off: Aditya's climb and fixed
     2 s hold.
  3. Stop options (--no-stop-options). In practice mode, TEAM_NAV_STOP_AFTER_STAGE set to
     corridor, gate_a, tunnel or chicane lands after that checkpoint (full_course_stops.py).
     When switched off, any TEAM_NAV_STOP_AFTER_STAGE is refused, as before.
  4. Arrival rule (--no-new-arrival-rule). A waypoint counts as reached once ARRIVAL_RULE has
     held: close, slow, and staying so. The first checkpoint is the exception: the turn-back
     point is the middle of the room past the line, and it counts as reached as soon as the
     whole drone is past the line, with no pause. When switched off: within 2.5 cm, then a 1 s
     hold, at every waypoint.

Words used here: "camera tracking" works out the drone's position from its cameras. The
"launch frame" measures positions from where the drone sat on the launch mark when it was
armed. "The footprint" is the rectangle, seen from above, that holds every propeller.
"""
import math
from dataclasses import asdict, dataclass

import numpy as np

from team_nav import flight_steps
from team_nav.course_map import load_course_map
from team_nav.course_run import EXIT_INTERRUPTED, EXIT_REFUSED_TO_START, find_package_file
from team_nav.practice_flights import first_maze_leg
from team_nav.practice_flights.full_course_route import (
    BASELINE_FIRST_CHECKPOINT_NORTH_METRES, CHECKPOINT_LINE_NAMES, CRUISE_HEIGHT_METRES,
    MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES, footprint_clears_line, footprint_distance_past_line,
    footprint_points, route_waypoints,
)
from team_nav.practice_flights.full_course_stops import describe_every_option, plan_stop
from team_nav.practice_flights.steadiness import SteadinessRule, SteadinessWatch
from team_nav.practice_flights.tracking_start_check import problem_with_tracking_on_the_launch_mark
from team_nav.run_mode import STOP_AFTER_STAGE_SETTING
from team_nav.start_frame import StartFrame

FLIGHT_NAME = 'full_course'

# Our own choices, conservative relative to the original first-leg speeds.
SPEED_METRES_PER_SECOND = 0.25
TARGET_LEAD_METRES = 0.25

# Aditya's arrival rule, used with --no-new-arrival-rule. Our own choices. five_waypoints.py
# uses these three too, so they keep their original names.
ARRIVAL_TOLERANCE_METRES = 0.025
ARRIVAL_HEIGHT_TOLERANCE_METRES = 0.08
SETTLE_SECONDS = 1.0

# Our own choice (the team's request of 2026-11-08). In run 20261108_122536 the drone reached
# Aditya's 2.5 cm while still moving at about 0.1 m/s, then slowed to 0.01 m/s during his 1 s
# hold. Asking for "slow" directly replaces both.
ARRIVAL_RULE = SteadinessRule(largest_horizontal_error_metres=0.08, largest_height_error_metres=0.08,
                              largest_distance_metres=math.inf, largest_speed_metres_per_second=0.05,
                              seconds_held=0.5)

# Our own choices (the team's request of 2026-11-08). In run 20261108_122536 the climb ended at
# 0.015 to 0.024 m/s, so a steady drone meets this within about a second.
TAKEOFF_SETTLE_RULE = SteadinessRule(largest_horizontal_error_metres=math.inf, largest_height_error_metres=math.inf,
                                     largest_distance_metres=0.10, largest_speed_metres_per_second=0.05,
                                     seconds_held=1.0)
LONGEST_TAKEOFF_SETTLE_SECONDS = 8.0

# Our own choice: straight above where the drone sat when armed, in the launch frame.
HOVER_ABOVE_LAUNCH_MARK_METRES = np.array([0.0, 0.0, CRUISE_HEIGHT_METRES])

# Our own choice: slower descent after the first full-course trial showed 12.7 cm of
# estimator lag during the original 0.5/0.35 m/s landing, with 8.7 cm or less on the route.
LANDING_SPEED_METRES_PER_SECOND = 0.15

# The nose points north throughout, because the route's footprint checks assume it.
NORTH_HEADING_RADIANS = math.pi / 2


class StartRefused(Exception):
    """The start check failed. The message says what is wrong and how to fix it."""


@dataclass(frozen=True)
class Additions:
    """Which of the four additions are switched on."""
    start_check: bool
    takeoff_settle: bool
    stop_options: bool
    new_arrival_rule: bool

    def describe(self):
        words = []
        for name, is_on in asdict(self).items():
            words.append(f"{name.replace('_', ' ')} {'on' if is_on else 'OFF'}")
        return ', '.join(words) + '.'


@dataclass(frozen=True)
class FlightPlan:
    whole_route_metres: np.ndarray      # The checked route from the launch mark to the pad.
    route_to_fly_metres: np.ndarray     # The same, or shorter for a stop option.
    stop: object = None                 # The StopPlan (full_course_stops.py), or None for the whole course.


def parse_arguments(words):
    parser = first_maze_leg.make_argument_parser(
        FLIGHT_NAME, 'Flies the checked sketch-map route through the four checkpoints and lands on the pad.')
    parser.add_argument('--no-start-check', action='store_true',
                        help='Do not refuse to start when camera tracking is far from where it started.')
    parser.add_argument('--no-takeoff-settle', action='store_true',
                        help="Use Aditya's climb and fixed 2 s hold instead of waiting until the drone is steady.")
    parser.add_argument('--no-stop-options', action='store_true',
                        help=f'Refuse any {STOP_AFTER_STAGE_SETTING}, as before.')
    parser.add_argument('--no-new-arrival-rule', action='store_true',
                        help="Use Aditya's arrival rule: within 2.5 cm, then a 1 s hold at every waypoint.")
    parser.add_argument('--baseline', action='store_true',
                        help="Switch off all four additions, which gives Aditya's version (run 20261108_122536).")
    return parser.parse_args(words)


def additions_from_arguments(arguments):
    return Additions(
        start_check=not (arguments.baseline or arguments.no_start_check),
        takeoff_settle=not (arguments.baseline or arguments.no_takeoff_settle),
        stop_options=not (arguments.baseline or arguments.no_stop_options),
        new_arrival_rule=not (arguments.baseline or arguments.no_new_arrival_rule),
    )


def plan_the_flight(course_map, additions, stop_after_stage):
    """The FlightPlan. Raises ValueError, with a sentence saying why, if it can't be flown."""
    first_checkpoint_north_metres = BASELINE_FIRST_CHECKPOINT_NORTH_METRES
    if additions.new_arrival_rule:
        first_checkpoint_north_metres = MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES
    whole_route = route_waypoints(course_map, first_checkpoint_north_metres)
    if not stop_after_stage:
        return FlightPlan(whole_route, whole_route)
    if not additions.stop_options:
        raise ValueError(f"{STOP_AFTER_STAGE_SETTING} is '{stop_after_stage}', but stop options are switched off "
                         f"(--no-stop-options or --baseline). Remove {STOP_AFTER_STAGE_SETTING}.")
    stop = plan_stop(course_map, whole_route, stop_after_stage)
    return FlightPlan(whole_route, stop.route_to_fly_metres, stop)


def run(words, run_mode, log):
    arguments = parse_arguments(words)
    additions = additions_from_arguments(arguments)
    try:
        course_map = load_course_map(find_package_file(arguments.map, 'course_maps'))
        plan = plan_the_flight(course_map, additions, run_mode.stop_after_stage)
    except (OSError, ValueError) as problem:
        log.event('refused', f'Full-course route refused before sending targets: {problem}')
        return EXIT_REFUSED_TO_START
    describe_the_plan(log, course_map, additions, plan)
    frame = StartFrame(course_map.launch_mark_centre_metres, arguments.start_heading_degrees)

    # fly_with_ros makes the flight with these four arguments; ours also needs the additions and plan.
    def make_full_course_flight(course_map, start_frame, link, bare_bones_safety=False):
        return FullCourseFlight(course_map, start_frame, link, additions, plan, bare_bones_safety)

    return first_maze_leg.fly_with_ros(course_map, frame, arguments, log, flight_class=make_full_course_flight)


def describe_the_plan(log, course_map, additions, plan):
    """Says on the screen and in the records what will fly, and where every stop option lands."""
    log.event('inputs', 'Additions: ' + additions.describe(), **asdict(additions))
    if additions.stop_options:
        for sentence in describe_every_option(course_map, plan.whole_route_metres):
            log.event('stop_options', 'Stop option ' + sentence)
    if plan.stop is None:
        log.event('plan', f'Flying the whole course: {len(plan.route_to_fly_metres) - 1} legs, then landing on the pad.')
    else:
        log.event('plan', f'Stopping early as asked. {plan.stop.describe()}', stop_option=plan.stop.option,
                  landing_spot_metres=plan.stop.landing_spot_metres.tolist())


class FullCourseFlight(first_maze_leg.FirstMazeLegFlight):

    def __init__(self, course_map, start_frame, link, additions, plan, bare_bones_safety=False):
        super().__init__(course_map, start_frame, link, bare_bones_safety=bare_bones_safety)
        self.additions = additions
        self.plan = plan
        self.body = footprint_points()
        self.largest_sideways_distance_from_launch_mark_metres = 0.0

    def run(self):
        """As FirstMazeLegFlight.run, but waits until ready and runs the start check first, so that
        a failed check refuses to start (exit code 2) before any target is sent."""
        try:
            flight_steps.wait_until_ready_on_the_ground(self.link)
            self.check_the_start()
        except StartRefused as refusal:
            self.log.event('refused', f'REFUSING TO START, before anything was sent to the drone. {refusal}')
            return EXIT_REFUSED_TO_START
        except KeyboardInterrupt:
            self.log.event('interrupted', 'Interrupted (Ctrl+C) before anything was sent to the drone.')
            return EXIT_INTERRUPTED
        return super().run()

    def check_the_start(self):
        if not self.additions.start_check:
            self.log.event('start_check', 'The start check is switched off (--no-start-check or --baseline).')
            return
        tracked = self.link.tracked_position()
        problem = problem_with_tracking_on_the_launch_mark(tracked)
        if problem:
            raise StartRefused(problem)
        self.log.event('start_check', f'Start check passed: camera tracking reads {np.round(tracked, 3).tolist()} '
                                      f'(forward, left, up) on the launch mark.')

    def fly(self):
        """From the start switch to the motors off. FirstMazeLegFlight.run lands after any problem."""
        self.save_planned_waypoints()
        flight_steps.hold_above_launch_until_control_is_ours(self.link, CRUISE_HEIGHT_METRES)
        self.flight_start_seconds = self.link.now_seconds()
        self.link.stage = 'climb'
        if self.additions.takeoff_settle:
            self.climb_and_settle_over_the_launch_mark()
        else:
            flight_steps.climb_above_launch(self.link, self.watch, CRUISE_HEIGHT_METRES)
        self.fly_the_route()
        self.land_at_the_end()
        self.report_the_result()

    def save_planned_waypoints(self):
        lines = ['east,north,height']
        for point in self.plan.route_to_fly_metres:
            lines.append(','.join(str(value) for value in point))
        self.log.save_text_file('planned_waypoints.csv', '\n'.join(lines))

    # ---- Takeoff (addition 2) ---------------------------------------------------------------------

    def climb_and_settle_over_the_launch_mark(self):
        climb_drift_metres = self.climb_over_the_launch_mark()
        settle_seconds, settle_drift_metres = self.settle_over_the_launch_mark()
        self.log.event('climb', f'Largest sideways distance from the launch mark: {climb_drift_metres:.3f} m during '
                                f'the climb and {settle_drift_metres:.3f} m while settling. Steady over the mark '
                                f'after {settle_seconds:.1f} s of settling.',
                       largest_distance_from_launch_mark_during_climb_metres=round(climb_drift_metres, 3),
                       largest_distance_from_launch_mark_while_settling_metres=round(settle_drift_metres, 3),
                       settle_seconds=round(settle_seconds, 2))

    def climb_over_the_launch_mark(self):
        """Climbs straight up to cruise height with the launch mark as the target, as
        flight_steps.climb_above_launch does but without its fixed hold. Returns the largest
        sideways distance from the mark during the climb."""
        self.largest_sideways_distance_from_launch_mark_metres = 0.0

        def has_reached_height():
            from_launch = self.link.position_from_launch()
            return (from_launch is not None
                    and abs(from_launch[2] - CRUISE_HEIGHT_METRES) < flight_steps.HEIGHT_TOLERANCE_METRES)

        flight_steps.fly_until(self.link, has_reached_height, self.hover_over_the_launch_mark,
                               flight_steps.LONGEST_CLIMB_SECONDS, 'the climb', self.watch)
        self.log.event('climb', f'Reached {self.link.position_from_launch()[2]:.2f} m above the launch mark.')
        return self.largest_sideways_distance_from_launch_mark_metres

    def settle_over_the_launch_mark(self):
        """Keeps the launch mark as the target until TAKEOFF_SETTLE_RULE has held. Returns (seconds
        it took, largest sideways distance from the mark meanwhile). If the drone isn't steady within
        LONGEST_TAKEOFF_SETTLE_SECONDS, fly_until raises RunMustStop and the drone lands."""
        self.link.stage = 'settle over launch mark'
        self.largest_sideways_distance_from_launch_mark_metres = 0.0
        steadiness = SteadinessWatch(TAKEOFF_SETTLE_RULE)
        start_seconds = self.link.now_seconds()

        def is_steady():
            return steadiness.update(self.link.now_seconds(), self.link.position_from_launch(),
                                     HOVER_ABOVE_LAUNCH_MARK_METRES, self.link.px4_speed_metres_per_second())

        flight_steps.fly_until(self.link, is_steady, self.hover_over_the_launch_mark, LONGEST_TAKEOFF_SETTLE_SECONDS,
                               'the drone to hold steady over the launch mark after the climb (within 0.10 m and '
                               'slower than 0.05 m/s for 1 s)', self.watch)
        return self.link.now_seconds() - start_seconds, self.largest_sideways_distance_from_launch_mark_metres

    def hover_over_the_launch_mark(self):
        """Sends the target straight above the launch mark, and remembers the drone's largest
        sideways distance from the mark."""
        from_launch = self.link.position_from_launch()
        if from_launch is not None:
            sideways_metres = math.hypot(from_launch[0], from_launch[1])
            self.largest_sideways_distance_from_launch_mark_metres = max(
                self.largest_sideways_distance_from_launch_mark_metres, sideways_metres)
        self.link.send_position_from_launch(HOVER_ABOVE_LAUNCH_MARK_METRES)

    # ---- The route (addition 4) -------------------------------------------------------------------

    def fly_the_route(self):
        route = self.plan.route_to_fly_metres
        last_leg_number = len(route) - 1
        for number, (start, end) in enumerate(zip(route, route[1:]), 1):
            self.link.stage = f'full course leg {number}'
            self.log.event('stage', f'{self.link.stage}: target {end.tolist()}', stage=self.link.stage)
            self.fly_leg(start, end, self.arrival_test_for_leg(number, end, number == last_leg_number))
            if not self.additions.new_arrival_rule:
                flight_steps.hold_still_for(self.link, self.watch, SETTLE_SECONDS)

    def arrival_test_for_leg(self, number, end, is_last_leg):
        """The function that says when this leg is done. Leg 1 ends past the first checkpoint line;
        if the flight lands there, the drone must be steady first, like at any landing spot."""
        if not self.additions.new_arrival_rule:
            return self.baseline_arrival_at(end)
        if number == 1 and not is_last_leg:
            return self.whole_drone_past_the_first_line()
        return self.steady_arrival_at(end)

    def fly_leg(self, start, end, has_arrived):
        """Slides the target from start to end, keeping it close ahead of the drone, until
        has_arrived(the current target) says the leg is done."""
        direction = end - start
        length = float(np.linalg.norm(direction))
        direction = direction / length
        target = start.copy()
        previous_seconds = self.link.now_seconds()

        def is_finished():
            return has_arrived(target)

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
            self.watch.send_target_on_course(target, heading_on_course_radians=NORTH_HEADING_RADIANS)

        flight_steps.fly_until(self.link, is_finished, send_next_target, length / SPEED_METRES_PER_SECOND + 30,
                               f'arrival at {end.tolist()}', self.watch)

    def baseline_arrival_at(self, end):
        """Aditya's rule: the target has reached the end, and the drone is within 2.5 cm sideways
        and 8 cm in height of it. fly_the_route then holds for 1 s."""
        def has_arrived(target):
            position = self.watch.drone_on_course()
            return (np.allclose(target, end) and position is not None
                    and np.linalg.norm(position[:2] - end[:2]) < ARRIVAL_TOLERANCE_METRES
                    and abs(position[2] - end[2]) < ARRIVAL_HEIGHT_TOLERANCE_METRES)
        return has_arrived

    def steady_arrival_at(self, end):
        """The new rule: the target has reached the end, and ARRIVAL_RULE has held since."""
        steadiness = SteadinessWatch(ARRIVAL_RULE)

        def has_arrived(target):
            if not np.allclose(target, end):
                steadiness.reset()
                return False
            return steadiness.update(self.link.now_seconds(), self.watch.drone_on_course(), end,
                                     self.link.px4_speed_metres_per_second())
        return has_arrived

    def whole_drone_past_the_first_line(self):
        """Leg 1's rule: done as soon as camera tracking puts the whole footprint past the first
        checkpoint line. No pause: the next leg heads back south at once."""
        line = self.course_map.checkpoint_line_named(CHECKPOINT_LINE_NAMES[0])

        def has_arrived(target):
            position = self.watch.drone_on_course()
            return position is not None and footprint_clears_line(position, self.body, line)
        return has_arrived

    # ---- Landing (addition 3 for a stop) ------------------------------------------------------------

    def land_at_the_end(self):
        stop = self.plan.stop
        self.link.stage = 'landing on pad'
        if stop is not None:
            self.link.stage = f'landing after {stop.option}'
            self.report_where_the_drone_is_against_the_line(stop)
        source = flight_steps.choose_landing_height_source(self.link, self.watch.px4_height_is_in_doubt())
        flight_steps.land(self.link, source, self.watch, descent_speed_limit=LANDING_SPEED_METRES_PER_SECOND)

    def report_where_the_drone_is_against_the_line(self, stop):
        position = self.watch.drone_on_course()
        if position is None:
            self.log.event('stop_option', f"Landing after '{stop.option}' without a position from camera tracking.")
            return
        line = self.course_map.checkpoint_line_named(stop.line_name)
        distance_past_metres, _ = footprint_distance_past_line(position, self.body, line)
        verdict = 'past' if distance_past_metres > 0 else 'NOT wholly past'
        self.log.event('stop_option', f"Landing straight down after '{stop.option}'. Camera tracking puts the drone at "
                                      f"east {position[0]:.3f} m, north {position[1]:.3f} m: the whole drone is {verdict} "
                                      f"'{stop.line_name}', by {distance_past_metres:.3f} m.",
                       distance_past_line_metres=round(distance_past_metres, 3))

    def report_the_result(self):
        elapsed = self.link.now_seconds() - self.flight_start_seconds
        text = f'FULL COURSE FINISHED: motors off after {elapsed:.1f} s.'
        if self.plan.stop is not None:
            text = (f"STOPPED AFTER '{self.plan.stop.option}' AS ASKED: landed past '{self.plan.stop.line_name}' "
                    f"with the motors off after {elapsed:.1f} s.")
        self.log.event('result', text + ' Check the independent referee for checkpoints, contacts and height error.',
                       total_seconds=round(elapsed, 2))
