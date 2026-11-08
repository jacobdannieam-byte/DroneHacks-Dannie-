"""A surveyed full-course route for the sketch map, checked before flight.

Uses the participant's map and measured airframe dimensions, never simulator truth. The
nose stays north throughout, allowing a footprint check at the narrow first checkpoint.
The route avoids boulder overflight so the bottom range sensor continues seeing the floor.

A "checkpoint line" is a line on the floor that the referee checks the whole drone has
crossed. "The footprint" is the rectangle, seen from above, that holds every motor and
propeller. full_course_stops.py uses the crossing helpers here to choose where a practice run
that stops early lands.
"""
import numpy as np

from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights

# Our own choices: the same cruise height as first_maze_leg and 2.5 cm geometry sampling.
CRUISE_HEIGHT_METRES = 0.75
SAMPLE_SPACING_METRES = 0.025
MINIMUM_BODY_CLEARANCE_METRES = 0.05
MINIMUM_LASER_CLEARANCE_METRES = 0.15

# From the measured motor offsets and estimated 7-inch propellers in drone_dimensions.py:
# north-facing footprint: east/west +/-0.249 m, south 0.239 m, north 0.169 m.
FOOTPRINT_EAST_HALF_WIDTH_METRES = 0.249
FOOTPRINT_SOUTH_METRES = 0.239
FOOTPRINT_NORTH_METRES = 0.169

# Our own choices from sketch_2026-11-08.json. The first checkpoint is cleared before
# backing south and passing below the boulder. Gate A is crossed at east=1.65, followed
# by the tunnel centre, the bottom of the chicane, Gate B, and the pad.
ROUTE_EAST_NORTH_METRES = (
    (-2.5146, 0.9144), (-2.51, 2.50), (-2.51, 1.65), (-1.15, 1.60),
    (-0.55, 2.12), (1.65, 2.12), (2.06, 1.60), (2.06, -0.65),
    (2.05, -1.80), (1.55, -1.90), (0.35, -1.85), (0.25, -0.85),
    (-0.50, -0.85), (-1.20, -1.25), (-2.0422, -0.7925),
)
CHECKPOINT_LINE_NAMES = ('end of first corridor', 'Gate A', 'tunnel exit', 'Gate B')

# The second route point is the turn-back point past the first checkpoint line. Two choices:
# Our own choice (Aditya's original route): 2.50 m north.
BASELINE_FIRST_CHECKPOINT_NORTH_METRES = 2.50
# Our own choice, from sketch_2026-11-08.json: the middle of the room between "the whole drone is
# past the line" and "the propellers touch the top wall". The line is at north 2.1912 m and the
# footprint reaches 0.239 m south, so the whole drone is past from 2.4302 m. The top wall's
# centre is at 2.82 m and it is 0.10 m thick, so its face is at 2.77 m; the footprint reaches
# 0.169 m north, so it touches from 2.601 m. Halfway is 2.5156 m. A test recomputes this.
MIDDLE_OF_FIRST_CHECKPOINT_ROOM_NORTH_METRES = 2.515

# Our own choice, copied from footprint_clears_line's original rule: the whole footprint must be
# at least 2 cm past a line before we count it as crossed.
LINE_CROSSING_MARGIN_METRES = 0.02


def footprint_points():
    """A filled rectangular envelope containing the motors and propellers, facing north."""
    east = np.arange(-FOOTPRINT_EAST_HALF_WIDTH_METRES, FOOTPRINT_EAST_HALF_WIDTH_METRES,
                     SAMPLE_SPACING_METRES)
    north = np.arange(-FOOTPRINT_SOUTH_METRES, FOOTPRINT_NORTH_METRES, SAMPLE_SPACING_METRES)
    east = np.append(east, FOOTPRINT_EAST_HALF_WIDTH_METRES)
    north = np.append(north, FOOTPRINT_NORTH_METRES)
    return np.array([(x, y) for x in east for y in north])


def route_waypoints(course_map, first_checkpoint_north_metres=BASELINE_FIRST_CHECKPOINT_NORTH_METRES):
    """Return the fixed route only after body, laser, checkpoint and pad checks pass.
    first_checkpoint_north_metres moves the turn-back point past the first checkpoint line."""
    points = np.column_stack((ROUTE_EAST_NORTH_METRES,
                              np.full(len(ROUTE_EAST_NORTH_METRES), CRUISE_HEIGHT_METRES)))
    points[1, 1] = first_checkpoint_north_metres
    if np.linalg.norm(points[0, :2] - course_map.launch_mark_centre_metres) > 0.02:
        raise ValueError('This route requires the sketch map launch mark.')
    return check_route(course_map, points)


def check_route(course_map, points):
    """Return the route (N x 3, launch mark first) only after body, laser, checkpoint and pad
    checks pass. Raises ValueError, with a sentence saying why, otherwise."""
    if np.linalg.norm(points[0, :2] - course_map.launch_mark_centre_metres) > 0.02:
        raise ValueError("The route does not start on this map's launch mark.")
    body = footprint_points()
    shapes = shapes_blocking_between_heights(course_map, CRUISE_HEIGHT_METRES - 0.20,
                                             CRUISE_HEIGHT_METRES + 0.20)
    for start, end in zip(points, points[1:]):
        samples = samples_along_leg(start, end)
        distances, names = distance_to_shapes_metres((samples[:, None, :] + body).reshape(-1, 2), shapes)
        tightest = int(np.argmin(distances))
        if distances[tightest] < MINIMUM_BODY_CLEARANCE_METRES:
            raise ValueError(f'Route is too close to {names[tightest]}: {distances[tightest]:.3f} m.')
        for obstacle in course_map.low_obstacles:
            clearance = np.linalg.norm(samples - obstacle.centre_metres, axis=1).min() - obstacle.radius_metres
            if clearance < MINIMUM_LASER_CLEARANCE_METRES:
                raise ValueError(f'Route puts the bottom laser too close to {obstacle.name}.')
    crossings = first_full_crossings(course_map, points)
    if len(crossings) != len(CHECKPOINT_LINE_NAMES):
        raise ValueError(f'Route does not clear checkpoint {CHECKPOINT_LINE_NAMES[len(crossings)]}.')
    pad_distances = np.linalg.norm(points[-1, :2] + body - course_map.landing_pad_centre_metres, axis=1)
    if pad_distances.max() + MINIMUM_BODY_CLEARANCE_METRES >= course_map.landing_pad_outer_radius_metres:
        raise ValueError('The whole airframe does not fit inside the landing pad.')
    return points


def samples_along_leg(start, end):
    """Points every SAMPLE_SPACING_METRES or less from start to end, seen from above (N x 2)."""
    count = max(2, int(np.ceil(np.linalg.norm(end - start) / SAMPLE_SPACING_METRES)) + 1)
    return np.linspace(start[:2], end[:2], count)


def first_full_crossings(course_map, points):
    """Where the route first has the whole footprint past each checkpoint line, in course order.

    Returns a list of (line name, leg number, centre east/north) with one entry per line that the
    route crosses. Leg number n is the leg from points[n] to points[n + 1]. A line is only looked
    for after the line before it is crossed, as the referee counts them."""
    body = footprint_points()
    crossings = []
    for leg_number, (start, end) in enumerate(zip(points, points[1:])):
        for centre in samples_along_leg(start, end):
            if len(crossings) == len(CHECKPOINT_LINE_NAMES):
                return crossings
            name = CHECKPOINT_LINE_NAMES[len(crossings)]
            if footprint_clears_line(centre, body, course_map.checkpoint_line_named(name)):
                crossings.append((name, leg_number, centre.copy()))
    return crossings


def footprint_distance_past_line(centre, body, line):
    """How far the footprint's rearmost point is past the line, towards its far side, in metres.
    Negative while part of the drone has not crossed. Also returns whether the centre is between
    the line's two ends."""
    along = line.end_metres - line.start_metres
    length = np.linalg.norm(along)
    along = along / length
    normal = np.array([-along[1], along[0]])
    if np.dot(line.far_side_point_metres - line.start_metres, normal) < 0:
        normal = -normal
    past = (np.asarray(centre)[:2] + body - line.start_metres) @ normal
    within_ends = np.dot(np.asarray(centre)[:2] - line.start_metres, along)
    return float(past.min()), bool(0 <= within_ends <= length)


def footprint_clears_line(centre, body, line):
    """True once the whole footprint is LINE_CROSSING_MARGIN_METRES past the line, between its ends."""
    distance_past_metres, is_within_ends = footprint_distance_past_line(centre, body, line)
    return distance_past_metres > LINE_CROSSING_MARGIN_METRES and is_within_ends
