"""Where a practice run of full_course lands when it is told to stop after a checkpoint.

REAL DRONE CODE. Plain arithmetic on the course map, with no ROS, so it is checked on the
ground before anything is sent, and the tests can check every option.

How a stop is chosen
--------------------
The teammate sets TEAM_NAV_STOP_AFTER_STAGE to one of the names in STOP_OPTIONS. Each names a
checkpoint line: a line on the floor that the whole drone must cross. Seen from above, "the
footprint" is the rectangle that holds every motor and propeller (full_course_route.py).

  1. Find where, along the route, the whole footprint is first past that line.
  2. Go a little further along the route, until the whole footprint is
     LANDING_DISTANCE_PAST_LINE_METRES past the line (more for the options in
     LANDING_DISTANCE_PAST_LINE_METRES_FOR_OPTION). This may be on a later leg.
  3. If that spot is open floor on the map, the drone lands straight down there.
  4. If not, it carries on to the next waypoint of the route and lands there, if that is open
     floor. If neither is, the flight refuses to start.

"Open floor" means that, seen from above, the footprint keeps MINIMUM_BODY_CLEARANCE_METRES
from every wall, the tunnel, the safety net and every boulder, at every height from the floor
up to just above the flying height, so nothing is under or beside the drone as it comes down.

This file does not fly anything. full_course.py flies the shortened route it returns.
"""
from dataclasses import dataclass

import numpy as np

from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights
from team_nav.practice_flights.full_course_route import (
    CRUISE_HEIGHT_METRES, LINE_CROSSING_MARGIN_METRES, MINIMUM_BODY_CLEARANCE_METRES, first_full_crossings,
    footprint_distance_past_line, footprint_points, samples_along_leg,
)

# Our own choices: the names a teammate types, and the map's checkpoint line each one means.
STOP_OPTIONS = (
    ('corridor', 'end of first corridor'),
    ('gate_a', 'Gate A'),
    ('tunnel', 'tunnel exit'),
    ('chicane', 'Gate B'),
)

# Our own choice: the 2 cm that counts as crossed, plus the arrival rule's 8 cm sideways
# tolerance (full_course.py). Then the drone is still wholly past the line if it stops anywhere
# within the arrival tolerance of the landing spot.
LANDING_DISTANCE_PAST_LINE_METRES = 0.10

# The team's requirement of 2026-11-08, for options where camera tracking may have drifted further
# by the time the drone gets there. In Gazebo run 20261108_125416 the pretend tracking had drifted
# 0.12 to 0.15 m by the tunnel exit, and a spot 0.107 m past the line left the propellers 2 cm short.
LANDING_DISTANCE_PAST_LINE_METRES_FOR_OPTION = {'tunnel': 0.25}

# Our own choice: the column the drone comes down through must be clear from the floor to the
# route's own checked band above the flying height (full_course_route.py checks +/- 0.20 m).
TOP_OF_LANDING_COLUMN_METRES = CRUISE_HEIGHT_METRES + 0.20

STRAIGHT_DOWN = 'straight down, just past the line'
AT_NEXT_WAYPOINT = 'at the next waypoint, because the spot just past the line is not open floor'


@dataclass(frozen=True)
class StopPlan:
    option: str                         # The name the teammate typed, such as 'tunnel'.
    line_name: str                      # The checkpoint line on the map.
    how: str                            # STRAIGHT_DOWN or AT_NEXT_WAYPOINT.
    landing_spot_metres: np.ndarray     # (east, north, flying height) above the landing spot.
    route_to_fly_metres: np.ndarray     # The route from the launch mark, ending above the landing spot.
    distance_past_line_metres: float    # How far the whole footprint is past the line there.
    clearance_metres: float             # The least gap from the footprint to anything on the map.
    nearest_thing: str                  # What that nearest thing is.

    def describe(self):
        """One sentence for the screen and the records."""
        east, north = self.landing_spot_metres[:2]
        return (f"{self.option}: after the whole drone crosses '{self.line_name}', it lands {self.how}, at east "
                f"{east:.3f} m, north {north:.3f} m. There the whole drone is {self.distance_past_line_metres:.3f} m "
                f"past the line, and the nearest thing on the map is {self.nearest_thing}, "
                f"{self.clearance_metres:.3f} m from the propellers.")


def stop_option_names():
    names = []
    for name, _ in STOP_OPTIONS:
        names.append(name)
    return names


def plan_stop(course_map, route_metres, option):
    """The StopPlan for this option. Raises ValueError, with a sentence saying why, if the option
    is unknown or neither landing spot is open floor."""
    line_name = _line_for_option(option)
    leg_number, crossing_centre = _first_full_crossing(course_map, route_metres, line_name)
    line = course_map.checkpoint_line_named(line_name)
    wanted_distance_metres = LANDING_DISTANCE_PAST_LINE_METRES_FOR_OPTION.get(option, LANDING_DISTANCE_PAST_LINE_METRES)
    spot_leg_number, spot = _spot_past_the_line(route_metres, leg_number, crossing_centre, line, wanted_distance_metres)
    if _is_open_floor(course_map, spot):
        route = _route_ending_at(route_metres, spot_leg_number, spot)
        return _make_plan(course_map, option, line, STRAIGHT_DOWN, spot, route)
    next_waypoint_number = spot_leg_number + 1
    if np.allclose(spot, route_metres[spot_leg_number + 1][:2]):
        next_waypoint_number = spot_leg_number + 2
    if next_waypoint_number >= len(route_metres):
        raise ValueError(f"The stop option '{option}' has no waypoint after its line to land at.")
    next_waypoint = route_metres[next_waypoint_number]
    if not _is_open_floor(course_map, next_waypoint[:2]):
        raise ValueError(f"Neither the spot just past '{line_name}' nor the next waypoint is open floor on the map, so "
                         f"the stop option '{option}' has nowhere safe to land. Choose another option.")
    route = route_metres[:next_waypoint_number + 1].copy()
    return _make_plan(course_map, option, line, AT_NEXT_WAYPOINT, next_waypoint[:2], route)


def describe_every_option(course_map, route_metres):
    """One sentence per stop option, saying where it lands, or why it can't be used."""
    sentences = []
    for option in stop_option_names():
        try:
            sentences.append(plan_stop(course_map, route_metres, option).describe())
        except ValueError as problem:
            sentences.append(f"{option}: cannot be used. {problem}")
    return sentences


def _line_for_option(option):
    for name, line_name in STOP_OPTIONS:
        if name == option:
            return line_name
    raise ValueError(f"TEAM_NAV_STOP_AFTER_STAGE is '{option}', which full_course does not know. Use one of: "
                     f"{', '.join(stop_option_names())}, or remove it to fly the whole course.")


def _first_full_crossing(course_map, route_metres, line_name):
    """(leg number, centre east/north) where the route first has the whole footprint past the line."""
    for name, leg_number, centre in first_full_crossings(course_map, route_metres):
        if name == line_name:
            return leg_number, centre
    raise ValueError(f"The route never has the whole drone past '{line_name}', so it can't stop after it.")


def _spot_past_the_line(route_metres, leg_number, crossing_centre, line, wanted_distance_metres):
    """(leg number, spot): the first point along the route, from the crossing on, with the whole
    footprint wanted_distance_metres past the line. The search carries on into later legs when the
    crossing's leg ends too soon. The route's end if there is no such point."""
    body = footprint_points()
    for number in range(leg_number, len(route_metres) - 1):
        leg_start = route_metres[number]
        samples = samples_along_leg(leg_start, route_metres[number + 1])
        distance_from_start = np.linalg.norm(samples - leg_start[:2], axis=1)
        skip_before_metres = 0.0
        if number == leg_number:
            skip_before_metres = np.linalg.norm(crossing_centre - leg_start[:2])
        for sample, distance in zip(samples, distance_from_start):
            if distance < skip_before_metres:
                continue
            distance_past_metres, is_within_ends = footprint_distance_past_line(sample, body, line)
            if is_within_ends and distance_past_metres >= wanted_distance_metres:
                return number, sample.copy()
    last_leg_number = len(route_metres) - 2
    return last_leg_number, np.asarray(route_metres[-1][:2], dtype=float).copy()


def _clearance_at(course_map, spot_east_north):
    """(least gap from the footprint to anything from the floor to above the flying height, its name).

    The tunnel is measured separately, as the rectangle it covers on the floor. The shared
    distance function (course_map.distance_to_shapes_metres) gives every strip rounded ends as
    wide as the strip, which is right for thin walls. But near the floor the whole 1.55 m width
    of the tunnel blocks, so its strip would bulge 0.78 m past the tunnel's open exit, where
    the floor is in fact clear."""
    footprint = np.asarray(spot_east_north) + footprint_points()
    tunnel_names = set()
    for tube in course_map.tubes:
        tunnel_names.add(tube.name)
    shapes = []
    for shape in shapes_blocking_between_heights(course_map, 0.0, TOP_OF_LANDING_COLUMN_METRES):
        if shape.name not in tunnel_names:
            shapes.append(shape)
    distances, names = distance_to_shapes_metres(footprint, shapes)
    nearest = int(np.argmin(distances))
    clearance_metres, nearest_thing = float(distances[nearest]), str(names[nearest])
    for tube in course_map.tubes:
        tube_clearance_metres = float(_distance_to_floor_rectangle_metres(footprint, tube).min())
        if tube_clearance_metres < clearance_metres:
            clearance_metres, nearest_thing = tube_clearance_metres, tube.name
    return clearance_metres, nearest_thing


def _distance_to_floor_rectangle_metres(points_metres, tube):
    """Each point's distance to the rectangle the tube covers on the floor (zero inside it)."""
    length_metres = tube.length_metres()
    along = (tube.exit_centre_metres - tube.entrance_centre_metres) / length_metres
    to_the_left = np.array([-along[1], along[0]])
    from_entrance = points_metres - tube.entrance_centre_metres
    distance_along_metres = from_entrance @ along
    distance_sideways_metres = np.abs(from_entrance @ to_the_left)
    beyond_the_ends_metres = np.maximum(0.0, np.maximum(-distance_along_metres, distance_along_metres - length_metres))
    beyond_the_sides_metres = np.maximum(0.0, distance_sideways_metres - tube.outside_width_metres / 2.0)
    return np.hypot(beyond_the_ends_metres, beyond_the_sides_metres)


def _is_open_floor(course_map, spot_east_north):
    clearance_metres, _ = _clearance_at(course_map, spot_east_north)
    return clearance_metres >= MINIMUM_BODY_CLEARANCE_METRES


def _route_ending_at(route_metres, leg_number, spot_east_north):
    """The route up to the leg's start, then the spot at flying height."""
    spot = np.array([spot_east_north[0], spot_east_north[1], CRUISE_HEIGHT_METRES])
    if np.allclose(spot, route_metres[leg_number + 1]):
        return route_metres[:leg_number + 2].copy()
    return np.vstack((route_metres[:leg_number + 1], spot))


def _make_plan(course_map, option, line, how, spot_east_north, route):
    distance_past_metres, _ = footprint_distance_past_line(spot_east_north, footprint_points(), line)
    if distance_past_metres <= LINE_CROSSING_MARGIN_METRES:
        raise ValueError(f"The landing spot for '{option}' does not have the whole drone past '{line.name}'.")
    required_metres = LANDING_DISTANCE_PAST_LINE_METRES_FOR_OPTION.get(option)
    if required_metres is not None and distance_past_metres < required_metres - 0.001:
        raise ValueError(f"The route ends too soon after '{line.name}' for the '{option}' option to land "
                         f"{required_metres:.2f} m past it. Move that goal further past the line.")
    clearance_metres, nearest_thing = _clearance_at(course_map, spot_east_north)
    spot = np.array([spot_east_north[0], spot_east_north[1], CRUISE_HEIGHT_METRES])
    return StopPlan(option, line.name, how, spot, route, distance_past_metres, clearance_metres, nearest_thing)
