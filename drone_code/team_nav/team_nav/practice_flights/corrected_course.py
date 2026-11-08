"""Fly the corrected course with a sensor-aligned window crossing.

Reads the participant map, never simulator truth. The through/over window alternatives
are proposals: their clearance and height checks run before any ROS control starts.
The full through route requires fresh measured alignment at the window. The over route
still refuses the existing course-height violation. A bounded first-two-goal mode remains.
"""
import argparse
import json
import math
import numpy as np

from team_nav.course_map import load_course_map
from team_nav.course_run import EXIT_REFUSED_TO_START, find_package_file
from team_nav.drone_dimensions import DRONE_HALF_HEIGHT_METRES, HALF_HEIGHT_NEEDED_METRES
from team_nav.practice_flights import first_maze_leg
from team_nav.practice_flights.astar_five_waypoints import AstarFlight
from team_nav.practice_flights.astar_geometry import RouteGeometry, MOTORS_METRES, PROPELLER_RADIUS_METRES, BODY_MARGIN_METRES
from team_nav.start_frame import StartFrame
from team_nav.practice_flights.window_route import full_window_plan, STANDOFF_METRES
from team_nav.practice_flights.window_flight import WindowFlight

FLIGHT_NAME = 'corrected_course'
MAP_NAME = 'real_course_2026-11-08.json'
# Our choices: preserve the existing five goals and 0.75 m cruise height.
FIRST_GOAL = [-2.5146, 2.451, .75]
SECOND_GOAL = [1.65, 2.12, .75]


def window_proposal(course_map, alternative):
    wall = next(wall for wall in course_map.walls if wall.openings)
    opening = wall.openings[0]
    along = wall.direction()
    normal = np.array([-along[1], along[0]])
    middle = wall.point_along_wall_metres((opening.start_along_wall_metres + opening.end_along_wall_metres) / 2)
    before = middle - STANDOFF_METRES * normal
    after = middle + STANDOFF_METRES * normal
    height = (opening.bottom_height_metres + opening.top_height_metres) / 2
    if alternative == 'over':
        height = wall.top_height_metres + HALF_HEIGHT_NEEDED_METRES
    width = opening.end_along_wall_metres - opening.start_along_wall_metres
    body_width = float(np.ptp(MOTORS_METRES[:, 1]) + 2 * PROPELLER_RADIUS_METRES)
    side_room = (width - body_width) / 2
    ceiling = course_map.wall_top_height_metres - DRONE_HALF_HEIGHT_METRES
    reasons = []
    if height > ceiling:
        reasons.append(f'The {height:.3f} m crossing exceeds the existing {ceiling:.3f} m target-height limit.')
    if alternative == 'through' and side_room < BODY_MARGIN_METRES:
        reasons.append(f'The map leaves {side_room:.3f} m per side, below the existing {BODY_MARGIN_METRES:.3f} m margin. '
                       'The nominal plan cannot fit. Verify the aperture before flying through it.')
    return {'alternative': alternative, 'flight_ready': False, 'refusal_reasons': reasons,
            'requires_sensor_alignment': alternative == 'through',
            'window_width_metres': width, 'nominal_side_room_metres': side_room,
            'crossing_height_metres': height, 'target_height_limit_metres': ceiling,
            'points_metres': [np.append(before, .75).tolist(), np.append(before, height).tolist(),
                              np.append(after, height).tolist(), np.append(after, .75).tolist()]}


def first_two_plan(course_map):
    geometry = RouteGeometry(course_map)
    start = np.append(course_map.launch_mark_centre_metres, .75)
    plan = {'start_metres': start.tolist(), 'stages': []}
    heading = math.pi / 2
    for name, checkpoint, point in [('First corridor', 'end of first corridor', FIRST_GOAL),
                                    ('Maze exit', 'Gate A', SECOND_GOAL)]:
        end = np.array(point)
        wanted = math.atan2(end[1] - start[1], end[0] - start[0])
        if not geometry.turn_clear(start[:2], heading, wanted) or not geometry.segment_clear(start[:2], end[:2], wanted):
            raise ValueError(f'The direct segment to {name} is obstructed. Recheck the updated map.')
        if not geometry.crossing_clear(end[:2], wanted, course_map.checkpoint_line_named(checkpoint)):
            raise ValueError(f'The full airframe does not clear {checkpoint}. Recheck the goal.')
        plan['stages'].append({'name': name, 'checkpoint': checkpoint, 'points': [point]})
        start, heading = end, wanted
    return plan


def run(words, run_mode, log):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', default=MAP_NAME)
    parser.add_argument('--route', choices=['through', 'over', 'verify-first-two'], default='through')
    arguments = parser.parse_args(words)
    arguments.start_heading_degrees = 90
    arguments.ignore_raw_laser = False
    arguments.bare_bones_safety = False
    try:
        if run_mode.stop_after_stage:
            raise ValueError('Use --route verify-first-two for the bounded verification flight.')
        course_map = load_course_map(find_package_file(arguments.map, 'course_maps'))
        for alternative in ['through', 'over']:
            proposal = window_proposal(course_map, alternative)
            log.save_text_file(f'{alternative}_window_proposal.json', json.dumps(proposal, indent=2))
        plan = first_two_plan(course_map)
        if arguments.route != 'verify-first-two':
            proposal = window_proposal(course_map, arguments.route)
            if proposal['refusal_reasons']:
                raise ValueError(' '.join(proposal['refusal_reasons']))
            plan = full_window_plan(course_map,plan)
        log.save_text_file('astar_plan.json', json.dumps(plan, indent=2))
        rows = ['stage,east_metres,north_metres,height_metres']
        for stage in plan['stages']:
            for point in stage['points']:
                rows.append(stage['name']+','+','.join(map(str,point)))
        log.save_text_file('waypoints_with_heights.csv','\n'.join(rows)+'\n')
        log.event('scope', f"Corrected-course mode: {arguments.route}; {len(plan['stages'])} main goals.")
    except (OSError, ValueError, KeyError, StopIteration) as problem:
        log.event('refused', f'Corrected-course flight refused before arming: {problem}')
        return EXIT_REFUSED_TO_START
    frame = StartFrame(course_map.launch_mark_centre_metres, 90)
    def make_flight(*args, **kwargs):
        flight_type = WindowFlight if arguments.route == 'through' else AstarFlight
        return flight_type(*args, plan=plan, **kwargs)
    result = first_maze_leg.fly_with_ros(course_map, frame, arguments, log, flight_class=make_flight)
    log.event('scope_result', f'Corrected-course {arguments.route} ended with code {result}. Check the referee for completed checkpoints.')
    return result
