"""A grid over the course that says how much room the drone has in each square.

This runs on the drone and in the simulator. route_planner.py builds one of these from the
course map and searches it for a route.

"Room" in this file means: the distance, seen from above, from the drone's centre to the
nearest wall, tunnel side, boulder or safety net. Only things the drone could hit at its
flying height count. For example, a boulder lower than the drone's body is left out, and the
tunnel's curved sides count as narrower or wider depending on the height.

The grid also holds, for each square, the distance from the drone's centre to the edge of the
nearest boulder, whatever the boulder's height (negative above a boulder; see
boulder_distances.py). The bottom laser is at the drone's centre, and a boulder under it makes
the drone's height reading jump, so the planner prefers to keep away. These boulders can include
other places a boulder might be, from other maps of the course. Those only count for the laser,
not for room.

The grid is made of squares GRID_SQUARE_METRES across. Each square's room is worked out
exactly at its centre, from the shapes in the map. The grid only answers "how much room is
there?". It does not decide where the drone may go; route_planner.py does.
"""
from dataclasses import dataclass

import numpy as np

from team_nav.boulder_distances import distance_to_nearest_boulder_metres
from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights

# Our own choice. Small enough that the 0.9 m window has several free squares across its
# middle, large enough that a route can be searched in about a second.
GRID_SQUARE_METRES = 0.05

# Our own choice. When checking a straight line, room is measured at points this far apart,
# which is half a grid square.
LINE_CHECK_SPACING_METRES = 0.025


@dataclass
class ClearanceGrid:
    lowest_corner_metres: np.ndarray    # The south-west corner of the grid.
    squares_east: int
    squares_north: int
    room_metres: np.ndarray             # squares_east x squares_north
    nearest_shape_names: np.ndarray     # The name of the nearest shape to each square's centre.
    shapes: list                        # The shapes the room was measured to.
    lowest_height_metres: float         # The heights the drone's body spans.
    highest_height_metres: float
    boulder_distance_metres: np.ndarray # squares_east x squares_north. Negative above a boulder, infinite if there are none.
    boulders: list                      # Every low obstacle in the map, whatever its height, and every other possible boulder.

    def centre_of_square_metres(self, east_index, north_index):
        return self.lowest_corner_metres + (np.array([east_index, north_index]) + 0.5) * GRID_SQUARE_METRES

    def square_containing(self, point_metres):
        """Returns (east index, north index) of the square a point is in, or None if the point
        is outside the grid."""
        east_index, north_index = np.floor((np.asarray(point_metres) - self.lowest_corner_metres) / GRID_SQUARE_METRES).astype(int)
        if not (0 <= east_index < self.squares_east and 0 <= north_index < self.squares_north):
            return None
        return int(east_index), int(north_index)

    def room_along_line_metres(self, start_metres, end_metres):
        """The least room anywhere on the straight line from start to end, and the name of the
        shape that is closest there."""
        return room_along_line_metres(self.shapes, start_metres, end_metres)


def make_clearance_grid(course_map, lowest_height_metres, highest_height_metres, other_possible_boulders=()):
    """Builds the grid for a drone whose body spans the two heights. Openings in walls are
    only open if the whole height range fits inside them. other_possible_boulders are boulders
    from other maps of the course: they count for the boulder distances, not for room."""
    squares_east, squares_north = np.ceil(
        (course_map.highest_corner_metres - course_map.lowest_corner_metres) / GRID_SQUARE_METRES
    ).astype(int)
    east_indices, north_indices = np.meshgrid(np.arange(squares_east), np.arange(squares_north), indexing="ij")
    centres_metres = course_map.lowest_corner_metres + (np.stack([east_indices.ravel(), north_indices.ravel()], axis=1) + 0.5) * GRID_SQUARE_METRES

    shapes = shapes_blocking_between_heights(course_map, lowest_height_metres, highest_height_metres)
    room_metres, nearest_names = distance_to_shapes_metres(centres_metres, shapes)
    boulders = list(course_map.low_obstacles) + list(other_possible_boulders)
    boulder_distance_metres, _ = distance_to_nearest_boulder_metres(centres_metres, boulders)
    return ClearanceGrid(
        lowest_corner_metres=course_map.lowest_corner_metres.copy(),
        squares_east=int(squares_east),
        squares_north=int(squares_north),
        room_metres=room_metres.reshape(squares_east, squares_north),
        nearest_shape_names=nearest_names.reshape(squares_east, squares_north),
        shapes=shapes,
        lowest_height_metres=lowest_height_metres,
        highest_height_metres=highest_height_metres,
        boulder_distance_metres=boulder_distance_metres.reshape(squares_east, squares_north),
        boulders=boulders,
    )


def room_along_line_metres(shapes, start_metres, end_metres):
    """The least room anywhere on the straight line from start to end, measured to the given
    shapes, and the name of the closest shape there."""
    length_metres = float(np.linalg.norm(np.asarray(end_metres) - np.asarray(start_metres)))
    point_count = max(2, int(np.ceil(length_metres / LINE_CHECK_SPACING_METRES)) + 1)
    fractions = np.linspace(0.0, 1.0, point_count)[:, None]
    points_metres = np.asarray(start_metres) + fractions * (np.asarray(end_metres) - np.asarray(start_metres))
    room_metres, nearest_names = distance_to_shapes_metres(points_metres, shapes)
    tightest = int(np.argmin(room_metres))
    return float(room_metres[tightest]), str(nearest_names[tightest])
