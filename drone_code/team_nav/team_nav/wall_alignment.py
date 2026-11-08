"""Corrects camera tracking's slow drift by lining up what the depth camera sees with the walls we
are sure of. REAL DRONE CODE, also used unchanged in the simulator.

The problem
-----------
Camera tracking (cuVSLAM) works out the drone's position by following points in its pictures.
Small errors add up, so the position it reports slowly "drifts" away from the truth. In the
Gazebo run of 2026-11-08 the drift reached about 15 cm over the course, and the live map showed
the long barrier twice, 15 cm apart.

The idea, as simply as possible
-------------------------------
Each depth picture gives points on the walls in front of the drone. If the drone's position were
exactly right, the points on a wall we are sure of would lie exactly on that wall in the map. If
they all lie, say, 10 cm north of it, the drone is really 10 cm further south than it believes.
So we slide the drone's believed position by a small part of that error, picture after picture.
This is the usual indoor-robot method, often called "scan matching".

How the sum works
-----------------
A grid holds, for every 5 cm square, the distance to the nearest trusted wall surface (negative
inside a wall), and which way that distance grows fastest (its "slope"). For each point within MATCHING_DISTANCE_METRES of a wall,
the distance says how far off the point is, and the slope says in which direction. The best
slide for all points together is a small least-squares sum (a 2 x 2 system of equations). In a
straight corridor the points only say where the drone is across the corridor, not along it; the
sum then leaves the "along" direction alone (DAMPING makes that safe).

Which walls are trusted: every wall in the map file except those read from the team's sketch
(their names start with SKETCH_WALL_NAME_START), because a moved or misdrawn wall would pull the
drone to a wrong place, and except walls with openings (the window), whose holes the camera sees
through. The net is not used: the camera sees through it. Only points within
FARTHEST_POINT_METRES of the camera are used, because far stereo distances are rough.

Safety limits: each picture moves the correction by at most STEP_FRACTION of what it measured,
and the whole correction never exceeds LARGEST_CORRECTION_METRES. So one bad picture can't throw
the drone off, and a wrong wall can pull it at most that far.

What it does not do: correct the heading or the height, or read ROS messages (depth_to_map.py
calls it).
"""
import numpy as np
from scipy.ndimage import distance_transform_edt

from team_nav.clearance_grid import GRID_SQUARE_METRES

# Our own choice: walls whose name starts like this were read from the team's sketch (±15 cm).
SKETCH_WALL_NAME_START = "Sketch"

# Our own choice. Points further than this from every trusted wall are on something else (a new
# wall, a boulder, a person) and are not used.
MATCHING_DISTANCE_METRES = 0.25

# Our own choice. Only points this near the camera are used. Stereo distance errors grow with the
# square of the distance: at 512 pixels wide, one pixel of shift error is about 4 cm at 1.5 m but
# about 30 cm at 3.5 m. In the Gazebo run of 2026-11-08, using all points made the correction
# swing by 20 cm without making the position better.
FARTHEST_POINT_METRES = 1.5

# Our own choice. A picture is used only if this many of its points lie near trusted walls.
FEWEST_MATCHED_POINTS = 40

# Our own choice. Each picture moves the correction by this fraction of what it measured. At 10
# pictures a second, a steady error is mostly corrected within about 2 seconds.
STEP_FRACTION = 0.1

# Our own choice. The correction never grows beyond this. The drift seen in Gazebo was 15 cm.
LARGEST_CORRECTION_METRES = 0.4

# Our own choice. Stops the sum from moving the drone along a corridor, where the walls say
# nothing about that direction. In squared metres per point, compared with 1 for a clear view.
DAMPING = 0.05


class WallAlignment:

    def __init__(self, course_map):
        trusted = _trusted_wall_squares(course_map)
        self.lowest_corner_metres = np.asarray(course_map.lowest_corner_metres, dtype=float)
        # Signed distance to the nearest wall surface: positive outside walls, negative inside, so
        # points lying right on a wall face count with about zero.
        self.distance_metres = (distance_transform_edt(~trusted) - distance_transform_edt(trusted)) * GRID_SQUARE_METRES
        # The slope of the distance, east and north, in metres per metre.
        self.slope_east, self.slope_north = np.gradient(self.distance_metres, GRID_SQUARE_METRES)
        self.correction_metres = np.zeros(2)
        self.pictures_used = 0

    def add_points(self, camera_on_course_metres, points_on_course_metres):
        """points: N x 2 (or N x 3) wall points at flying height, placed on the course with the
        current correction already included, and the camera's place the same way. Updates
        correction_metres."""
        points = np.asarray(points_on_course_metres, dtype=float)[:, :2]
        camera = np.asarray(camera_on_course_metres, dtype=float)[:2]
        points = points[np.linalg.norm(points - camera, axis=1) <= FARTHEST_POINT_METRES]
        squares = np.floor((points - self.lowest_corner_metres) / GRID_SQUARE_METRES).astype(int)
        inside = ((squares[:, 0] >= 0) & (squares[:, 0] < self.distance_metres.shape[0])
                  & (squares[:, 1] >= 0) & (squares[:, 1] < self.distance_metres.shape[1]))
        squares = squares[inside]
        distances = self.distance_metres[squares[:, 0], squares[:, 1]]
        near = np.abs(distances) < MATCHING_DISTANCE_METRES
        if near.sum() < FEWEST_MATCHED_POINTS:
            return
        slopes = np.column_stack([self.slope_east[squares[near, 0], squares[near, 1]],
                                  self.slope_north[squares[near, 0], squares[near, 1]]])
        slide = _best_slide(slopes, distances[near])
        self.correction_metres = self.correction_metres + STEP_FRACTION * slide
        size = np.linalg.norm(self.correction_metres)
        if size > LARGEST_CORRECTION_METRES:
            self.correction_metres *= LARGEST_CORRECTION_METRES / size
        self.pictures_used += 1


def _best_slide(slopes, distances):
    """The slide s that makes distance + slope . s closest to zero for all points (least squares)."""
    count = len(distances)
    left_side = slopes.T @ slopes + DAMPING * count * np.eye(2)
    right_side = -(slopes.T @ distances)
    return np.linalg.solve(left_side, right_side)


def _trusted_wall_squares(course_map):
    """True for every square under a trusted wall."""
    shape = tuple(np.ceil((np.asarray(course_map.highest_corner_metres) - np.asarray(course_map.lowest_corner_metres))
                          / GRID_SQUARE_METRES).astype(int))
    centres_east = course_map.lowest_corner_metres[0] + (np.arange(shape[0]) + 0.5) * GRID_SQUARE_METRES
    centres_north = course_map.lowest_corner_metres[1] + (np.arange(shape[1]) + 0.5) * GRID_SQUARE_METRES
    east, north = np.meshgrid(centres_east, centres_north, indexing="ij")
    trusted = np.zeros(shape, dtype=bool)
    for wall in course_map.walls:
        if wall.name.startswith(SKETCH_WALL_NAME_START) or wall.openings:
            continue
        trusted |= _distance_to_segment(east, north, wall.start_metres, wall.end_metres) <= wall.thickness_metres / 2.0
    return trusted


def _distance_to_segment(east, north, start, end):
    start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
    along = end - start
    fraction = ((east - start[0]) * along[0] + (north - start[1]) * along[1]) / max(along @ along, 1e-9)
    fraction = np.clip(fraction, 0.0, 1.0)
    return np.hypot(east - (start[0] + fraction * along[0]), north - (start[1] + fraction * along[1]))
