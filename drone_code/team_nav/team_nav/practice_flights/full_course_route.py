"""A surveyed full-course route for the sketch map, checked before flight.

Uses the participant's map and measured airframe dimensions, never simulator truth. The
nose stays north throughout, allowing a footprint check at the narrow first checkpoint.
The route avoids boulder overflight so the bottom range sensor continues seeing the floor.
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


def footprint_points():
    """A filled rectangular envelope containing the motors and propellers, facing north."""
    east = np.arange(-FOOTPRINT_EAST_HALF_WIDTH_METRES, FOOTPRINT_EAST_HALF_WIDTH_METRES,
                     SAMPLE_SPACING_METRES)
    north = np.arange(-FOOTPRINT_SOUTH_METRES, FOOTPRINT_NORTH_METRES, SAMPLE_SPACING_METRES)
    east = np.append(east, FOOTPRINT_EAST_HALF_WIDTH_METRES)
    north = np.append(north, FOOTPRINT_NORTH_METRES)
    return np.array([(x, y) for x in east for y in north])


def route_waypoints(course_map):
    """Return the fixed route only after body, laser, checkpoint and pad checks pass."""
    points = np.column_stack((ROUTE_EAST_NORTH_METRES,
                              np.full(len(ROUTE_EAST_NORTH_METRES), CRUISE_HEIGHT_METRES)))
    if np.linalg.norm(points[0, :2] - course_map.launch_mark_centre_metres) > 0.02:
        raise ValueError('This route requires the sketch map launch mark.')
    body = footprint_points()
    shapes = shapes_blocking_between_heights(course_map, CRUISE_HEIGHT_METRES - 0.20,
                                             CRUISE_HEIGHT_METRES + 0.20)
    next_checkpoint = 0
    for start, end in zip(points, points[1:]):
        count = max(2, int(np.ceil(np.linalg.norm(end - start) / SAMPLE_SPACING_METRES)) + 1)
        samples = np.linspace(start[:2], end[:2], count)
        distances, names = distance_to_shapes_metres((samples[:, None, :] + body).reshape(-1, 2), shapes)
        tightest = int(np.argmin(distances))
        if distances[tightest] < MINIMUM_BODY_CLEARANCE_METRES:
            raise ValueError(f'Route is too close to {names[tightest]}: {distances[tightest]:.3f} m.')
        for obstacle in course_map.low_obstacles:
            clearance = np.linalg.norm(samples - obstacle.centre_metres, axis=1).min() - obstacle.radius_metres
            if clearance < MINIMUM_LASER_CLEARANCE_METRES:
                raise ValueError(f'Route puts the bottom laser too close to {obstacle.name}.')
        for centre in samples:
            if next_checkpoint < len(CHECKPOINT_LINE_NAMES):
                line = course_map.checkpoint_line_named(CHECKPOINT_LINE_NAMES[next_checkpoint])
                if footprint_clears_line(centre, body, line):
                    next_checkpoint += 1
    if next_checkpoint != len(CHECKPOINT_LINE_NAMES):
        raise ValueError(f'Route does not clear checkpoint {CHECKPOINT_LINE_NAMES[next_checkpoint]}.')
    pad_distances = np.linalg.norm(points[-1, :2] + body - course_map.landing_pad_centre_metres, axis=1)
    if pad_distances.max() + MINIMUM_BODY_CLEARANCE_METRES >= course_map.landing_pad_outer_radius_metres:
        raise ValueError('The whole airframe does not fit inside the landing pad.')
    return points


def footprint_clears_line(centre, body, line):
    along = line.end_metres - line.start_metres
    length = np.linalg.norm(along)
    along = along / length
    normal = np.array([-along[1], along[0]])
    if np.dot(line.far_side_point_metres - line.start_metres, normal) < 0:
        normal = -normal
    past = (centre + body - line.start_metres) @ normal
    within_ends = np.dot(centre - line.start_metres, along)
    return bool(past.min() > 0.02 and 0 <= within_ends <= length)
