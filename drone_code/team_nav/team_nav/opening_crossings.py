"""Works out how the drone flies through an opening in a wall, such as the window.

This runs on the drone and in the simulator. route_planner.py calls it for every checkpoint
line that runs across an opening.

The drone always crosses an opening the same way, straight through its middle and at right
angles to the wall, in four points:

    line-up point --straight up or down--> start of level part --level--> end of level part --climb or descend--> exit point
    (usual height)                         (opening's middle height)      (opening's middle height)              (usual height)

The line-up point is directly below (or above) the start of the level part. The drone stops
there, changes height on the spot, stops again, and then flies level through the opening. The
line-up point is where the flight program can stop, look at the opening with the camera, and
line up before crossing.

Why the height change happens on the spot. Until 2026-11-07 the drone climbed while flying
towards the wall, over HEIGHT_CHANGE_DISTANCE_METRES, so the line-up point was that much
further from the wall. On our scan that put it 13 cm from the boulder in front of the window,
pinned there by the safety net behind it, and the drone pauses at the line-up point: the worst
place for a boulder under the bottom laser (docs/ARCHITECTURE.md, section 7). Climbing on the
spot moves the line-up point 0.6 m closer to the wall, 47 cm from that boulder's edge. It also
means the drone reaches the opening's height while holding still, so it starts the level part
at the right height instead of trailing its target upwards.

The level part starts and ends far enough from the wall that no part of the drone is above or
below the wall's solid edge while it changes height. It is TRAILING_ALLOWANCE_METRES longer at
each end than that, because the drone trails its target (route_follower.py): after the opening
it starts the descent a little before the plan says. On the way in this allowance is no longer
needed, since the drone climbs while holding still, but it is kept so that the line-up point
stays a little further from the wall.

On the far side, the drone descends while flying on, over HEIGHT_CHANGE_DISTANCE_METRES, as
before, so it doesn't stop just past the wall.

This file does not search for a route to the line-up point; route_planner.py does.
"""
from dataclasses import dataclass

import numpy as np

from team_nav.clearance_grid import room_along_line_metres
from team_nav.course_map import shapes_blocking_between_heights
from team_nav.drone_dimensions import (
    DRONE_RADIUS_METRES, HALF_HEIGHT_NEEDED_METRES, REQUIRED_ROOM_METRES, SIDEWAYS_SAFETY_MARGIN_METRES,
    heights_spanned_metres,
)

# Our own choice. The drone changes height over this distance after the level part. 0.6 m to
# descend about 0.4 m is a gentle slope.
HEIGHT_CHANGE_DISTANCE_METRES = 0.6

# Our own choice, from a pretend-drone test (test_route_follower.py). Without this extra level
# length, the pretend drone was up to 12 cm below the window's middle height on our scan while
# part of it was still above the wall's lower edge. With 0.2 m, about 6 cm. A longer allowance
# would help more, but at 0.3 m our scan has no room left to line up for the window.
TRAILING_ALLOWANCE_METRES = 0.2


class OpeningCrossingNotPossible(Exception):
    """The drone can't fly through this opening. The message says why, in a full sentence."""


@dataclass
class OpeningCrossing:
    line_name: str
    opening_name: str
    line_up_point_metres: np.ndarray        # x, y, height
    level_start_metres: np.ndarray
    level_end_metres: np.ndarray
    exit_point_metres: np.ndarray
    least_room_metres: float
    least_room_near: str

    def points_in_order(self):
        return [self.line_up_point_metres, self.level_start_metres, self.level_end_metres, self.exit_point_metres]


def plan_opening_crossing(course_map, line, cruise_height_metres):
    """Returns an OpeningCrossing for the checkpoint line, or raises OpeningCrossingNotPossible."""
    wall, opening = course_map.opening_named(line.opening_name)
    height_needed_metres = 2.0 * HALF_HEIGHT_NEEDED_METRES
    if opening.height_metres() < height_needed_metres:
        raise OpeningCrossingNotPossible(
            f"The {opening.name} is {opening.height_metres():.2f} m tall, but the drone needs "
            f"{height_needed_metres:.2f} m to pass with its safety margin."
        )

    crossing_height_metres = opening.centre_height_metres()
    opening_middle = wall.point_along_wall_metres(
        (opening.start_along_wall_metres + opening.end_along_wall_metres) / 2.0
    )
    through_the_wall = _direction_through_wall_towards(wall, line.far_side_point_metres)
    level_half_length_metres = (
        DRONE_RADIUS_METRES + wall.thickness_metres / 2.0 + SIDEWAYS_SAFETY_MARGIN_METRES + TRAILING_ALLOWANCE_METRES
    )
    exit_distance_metres = level_half_length_metres + HEIGHT_CHANGE_DISTANCE_METRES

    # Directly below (or above) the start of the level part: the height changes on the spot.
    line_up_point = opening_middle - through_the_wall * level_half_length_metres
    exit_point = opening_middle + through_the_wall * exit_distance_metres
    least_room_metres, least_room_near = _room_through_opening(
        course_map, opening.name, line_up_point, exit_point, cruise_height_metres, crossing_height_metres
    )
    if least_room_metres < REQUIRED_ROOM_METRES:
        raise OpeningCrossingNotPossible(
            f"Flying straight through the middle of the {opening.name} leaves only {least_room_metres:.2f} m "
            f"between the drone's centre and the {least_room_near}, but the drone needs {REQUIRED_ROOM_METRES:.2f} m."
        )

    return OpeningCrossing(
        line_name=line.name,
        opening_name=opening.name,
        line_up_point_metres=np.append(line_up_point, cruise_height_metres),
        level_start_metres=np.append(opening_middle - through_the_wall * level_half_length_metres, crossing_height_metres),
        level_end_metres=np.append(opening_middle + through_the_wall * level_half_length_metres, crossing_height_metres),
        exit_point_metres=np.append(exit_point, cruise_height_metres),
        least_room_metres=least_room_metres,
        least_room_near=least_room_near,
    )


def _direction_through_wall_towards(wall, far_side_point_metres):
    """A unit vector at right angles to the wall, pointing to the side the drone must end up on."""
    across = np.array([-wall.direction()[1], wall.direction()[0]])
    if np.dot(far_side_point_metres - wall.start_metres, across) < 0:
        return -across
    return across


def _room_through_opening(course_map, opening_name, line_up_point, exit_point, cruise_height_metres, crossing_height_metres):
    """The least sideways room on the straight crossing. While crossing, the drone's body is
    somewhere between its usual height and the opening's height, so everything that blocks
    any of those heights counts, except the opening itself."""
    cruise_lowest, cruise_highest = heights_spanned_metres(cruise_height_metres)
    crossing_lowest, crossing_highest = heights_spanned_metres(crossing_height_metres)
    shapes = shapes_blocking_between_heights(
        course_map, min(cruise_lowest, crossing_lowest), max(cruise_highest, crossing_highest),
        openings_to_ignore=(opening_name,),
    )
    return room_along_line_metres(shapes, line_up_point, exit_point)
