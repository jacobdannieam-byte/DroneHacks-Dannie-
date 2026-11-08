"""Check a five-target mission, including the airframe's swept area during turns.

Uses the participant map, never simulator truth. A single short reverse segment follows
the just-flown first corridor to reach a wider turning pocket.
"""
import math
import numpy as np
from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.practice_flights.full_course_route import (
    footprint_points, footprint_clears_line, MINIMUM_BODY_CLEARANCE_METRES,
    MINIMUM_LASER_CLEARANCE_METRES, SAMPLE_SPACING_METRES, CHECKPOINT_LINE_NAMES,
)

# Our own choices: 3-degree swept-turn sampling and a bounded 0.4 m retreat plus rounding room.
TURN_SAMPLE_RADIANS = math.radians(3)
MAXIMUM_REVERSE_METRES = 0.41


def angle_difference(target, current):
    return math.atan2(math.sin(target-current), math.cos(target-current))


def rotated_footprint(heading):
    angle = heading - math.pi / 2
    rotation = np.array([[math.cos(angle), -math.sin(angle)], [math.sin(angle), math.cos(angle)]])
    return footprint_points() @ rotation.T


def segments(plan):
    start = np.array(plan['start_metres'], dtype=float)
    heading = math.pi / 2
    for stage in plan['stages']:
        for number, point in enumerate(stage['points']):
            end = np.array(point, dtype=float)
            reverse = number == 0 and stage.get('reverse_first_segment', False)
            wanted = heading if reverse else math.atan2(end[1]-start[1], end[0]-start[0])
            yield stage, start, end, heading, wanted, reverse
            start, heading = end, wanted


def validate_plan(plan, course_map):
    if len(plan['stages']) != 5:
        raise ValueError('The mission must contain five main targets.')
    if tuple(s['checkpoint'] for s in plan['stages'][:-1]) != CHECKPOINT_LINE_NAMES:
        raise ValueError('The first four targets must clear the course checkpoints in order.')
    if np.linalg.norm(np.array(plan['start_metres'][:2])-course_map.launch_mark_centre_metres) > .02:
        raise ValueError('The plan does not start at this map launch mark.')
    cleared = 0
    previous_start = None
    for stage, start, end, previous, heading, reverse in segments(plan):
        if not np.all(np.isfinite(end)) or np.linalg.norm(end-start) <= 0:
            raise ValueError('Every route segment must have a finite, distinct endpoint.')
        if not np.isclose(start[2], .75) or not np.isclose(end[2], .75):
            raise ValueError('This route is checked at 0.75 m cruise height.')
        if reverse and (np.linalg.norm(end-start) > MAXIMUM_REVERSE_METRES or stage['name'] != 'Maze exit'):
            raise ValueError('Only the short first-corridor retreat may use reverse flight.')
        if reverse:
            if previous_start is None:
                raise ValueError('A retreat must follow an already flown segment.')
            retrace = previous_start-start
            fraction = np.dot(end-start, retrace)/np.dot(retrace, retrace)
            if not 0 < fraction < 1 or np.linalg.norm(end-(start+fraction*retrace)) > .005:
                raise ValueError('The retreat must retrace the preceding segment. Check its endpoint.')
        shapes = shapes_blocking_between_heights(course_map, .55, .95)
        turn = angle_difference(heading, previous)
        turn_samples = max(2, int(math.ceil(abs(turn)/TURN_SAMPLE_RADIANS))+1)
        swept = np.concatenate([start[:2]+rotated_footprint(a) for a in np.linspace(previous, previous+turn, turn_samples)])
        _require_clear(swept, shapes, 'turn')
        body = rotated_footprint(heading)
        count = max(2, int(math.ceil(np.linalg.norm(end-start)/SAMPLE_SPACING_METRES))+1)
        centres = np.linspace(start[:2], end[:2], count)
        _require_clear((centres[:,None,:]+body).reshape(-1,2), shapes, 'translation')
        for obstacle in course_map.low_obstacles:
            distance = np.linalg.norm(centres-obstacle.centre_metres, axis=1).min()-obstacle.radius_metres
            if distance < MINIMUM_LASER_CLEARANCE_METRES:
                raise ValueError(f'The laser path is too close to {obstacle.name}.')
        for centre in centres:
            if cleared < len(CHECKPOINT_LINE_NAMES):
                line = course_map.checkpoint_line_named(CHECKPOINT_LINE_NAMES[cleared])
                if footprint_clears_line(centre, body, line):
                    cleared += 1
        previous_start = start
    if cleared != 4:
        raise ValueError('The complete airframe does not clear all four checkpoint lines.')
    if np.linalg.norm(end[:2]+body-course_map.landing_pad_centre_metres, axis=1).max()+MINIMUM_BODY_CLEARANCE_METRES >= course_map.landing_pad_outer_radius_metres:
        raise ValueError('The landing footprint does not fit inside the pad.')


def _require_clear(points, shapes, phase):
    distances, names = distance_to_shapes_metres(points, shapes)
    index = int(np.argmin(distances))
    if distances[index] < MINIMUM_BODY_CLEARANCE_METRES:
        raise ValueError(f'Unsafe {phase}: {names[index]} has only {distances[index]:.3f} m clearance.')
