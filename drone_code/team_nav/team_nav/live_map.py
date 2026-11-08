"""The drone's live map: a grid over the course that says, for each 5 cm square, "blocked",
"free" or "not seen yet", kept up to date from the depth camera during the flight.

REAL DRONE CODE, also used unchanged in the simulator.

The idea, as simply as possible
-------------------------------
Think of a sheet of squared paper over the course. Each square holds a score:
  - above BLOCKED_SCORE: something is there at the drone's flying height;
  - below FREE_SCORE: we have seen through it, so it is empty;
  - in between: we don't know.
At the start, the squares under the walls of our course map get PRIOR_WALL_SCORE: "probably
blocked". Walls may have moved since the map was made, so this is only a first guess.

Each depth picture then changes the scores. For every point the camera sees at flying height:
  - the square where the point is gets HIT_SCORE added: something is there;
  - every square on the straight line from the camera to that point gets FREE_SCORE_STEP taken
    off: the camera saw through it, so it must be empty.
A wall that has moved away is "seen through" a few times and becomes free. A new wall gets hit
a few times and becomes blocked. One stray wrong point is not enough to block a square, because
HIT_SCORE is smaller than BLOCKED_SCORE.

The safety net around the course and everything outside the grid are always blocked.

If the map is so out of date that no path exists, the planner may ask for the map "without the
map's walls": then only squares the camera itself has hit count as blocked, and the map's walls
count as unknown. The drone then flies toward them, facing them, and sees whether they are real.

Boulders are kept apart. They are lower than the drone, so they can't be hit, but flying over
one makes the bottom laser's reading jump, and then PX4 may stop trusting the laser and lose its
height (it happened in the Gazebo run of 2026-11-08 over the sketch's north-west boulder). So the
map also says which squares are near a boulder: those from the map file, and those where the
camera sees something between LOWEST_LOW_OBSTACLE_METRES and the drone's height band. The planner
avoids them when it can (live_planner.BOULDER_COST_FACTOR); they never block a path.

What it does not do: plan (live_planner.py) or read ROS messages (flight_steps.py does).
"""
import numpy as np
from scipy.ndimage import distance_transform_edt

from team_nav.clearance_grid import GRID_SQUARE_METRES, make_clearance_grid

# All scores are our own choice. A square counts as blocked above BLOCKED_SCORE.
BLOCKED_SCORE = 1.0
# A wall from the course map starts here: blocked, but two clear views through it make it free.
PRIOR_WALL_SCORE = 1.5
# One point seen in a square adds this. Two points in a row (two pictures) block it.
HIT_SCORE = 0.6
# One view through a square takes off this.
FREE_SCORE_STEP = 0.5
# Scores are kept between these, so old news can always be overturned by a few new pictures.
LOWEST_SCORE = -2.0
HIGHEST_SCORE = 3.0
# Below this a square counts as "seen free". Between this and BLOCKED_SCORE it is unknown.
FREE_SCORE = -0.4

# Our own choice. A point closer to the camera than this along the ray is not counted as a hit
# near its end: the last part of each ray, next to the point, is not cleared, because depth
# errors put the point a little in front of or behind the real surface.
KEEP_NEAR_POINT_METRES = 0.1

# Our own choice. Points within this height of the drone's flying band count. The drone's body
# spans about 20 cm; walls are taller than the drone flies, boulders are lower.
HEIGHT_BAND_MARGIN_METRES = 0.15

# Our own choice. Points between this height and the drone's height band count as low obstacles
# (boulders). Lower points are the floor: 15 cm is well above the stereo error on the floor at 2 m.
LOWEST_LOW_OBSTACLE_METRES = 0.15

# Our own choice. The drone's centre (where the laser points down) keeps at least this far from
# the edge of a boulder when it can.
BOULDER_KEEP_AWAY_METRES = 0.2

# Our own choice. At most this many points are used from each picture, picked evenly, so that
# one update stays well under the 0.1 s control cycle.
LARGEST_POINTS_PER_UPDATE = 1500


class LiveMap:

    def __init__(self, course_map, flying_height_metres, body_half_height_metres):
        lowest = flying_height_metres - body_half_height_metres
        highest = flying_height_metres + body_half_height_metres
        grid = make_clearance_grid(course_map, lowest, highest)
        self.lowest_corner_metres = grid.lowest_corner_metres
        self.shape = (grid.squares_east, grid.squares_north)
        self.lowest_counted_height_metres = lowest - HEIGHT_BAND_MARGIN_METRES
        self.highest_counted_height_metres = highest + HEIGHT_BAND_MARGIN_METRES
        self.mapped_room_metres = grid.room_metres
        # A square whose centre is within half a square of a map shape is under that shape.
        under_a_map_shape = grid.room_metres < GRID_SQUARE_METRES * 0.75
        self.scores = np.where(under_a_map_shape, PRIOR_WALL_SCORE, 0.0).astype(np.float32)
        self.always_blocked = _edge_squares(self.shape)
        self.hit_by_camera = np.zeros(self.shape, dtype=bool)
        # Distance from each square's centre to the nearest map boulder's edge (negative above one).
        self.map_boulder_distance_metres = grid.boulder_distance_metres
        self.low_obstacle_scores = np.zeros(self.shape, dtype=np.float32)
        self.scores[self.always_blocked] = HIGHEST_SCORE
        self.updates = 0

    # ---- Reading the map -----------------------------------------------------------------------

    def blocked(self, trust_map_walls=True):
        """True for blocked squares. With trust_map_walls False, a square only counts as blocked
        if the camera itself has seen something there."""
        blocked = self.scores > BLOCKED_SCORE
        if not trust_map_walls:
            blocked &= self.hit_by_camera
        return blocked | self.always_blocked

    def near_a_boulder(self):
        """True for squares where the drone's centre would be within BOULDER_KEEP_AWAY_METRES of
        a boulder, from the map file or seen by the camera."""
        seen_low = self.low_obstacle_scores > BLOCKED_SCORE
        distance_to_seen = distance_transform_edt(~seen_low) * GRID_SQUARE_METRES
        return (self.map_boulder_distance_metres < BOULDER_KEEP_AWAY_METRES) | (distance_to_seen < BOULDER_KEEP_AWAY_METRES)

    def seen_free(self):
        return self.scores < FREE_SCORE

    def room_metres(self, trust_map_walls=True, keep_mapped_walls=False):
        """For every square: the distance from its centre to the nearest blocked square's centre."""
        room_metres = distance_transform_edt(~self.blocked(trust_map_walls)) * GRID_SQUARE_METRES
        if keep_mapped_walls:
            # Rays can hit a real surface beyond a web. Those returns do not clear the web itself.
            np.minimum(room_metres, self.mapped_room_metres, out=room_metres)
        return room_metres

    def square_of(self, point_metres):
        """(east index, north index) of the square holding an (east, north) point, or None if outside."""
        indices = np.floor((np.asarray(point_metres[:2]) - self.lowest_corner_metres) / GRID_SQUARE_METRES).astype(int)
        if not (0 <= indices[0] < self.shape[0] and 0 <= indices[1] < self.shape[1]):
            return None
        return int(indices[0]), int(indices[1])

    def centre_of(self, square):
        return self.lowest_corner_metres + (np.asarray(square, dtype=float) + 0.5) * GRID_SQUARE_METRES

    def free_distance_metres(self, start_metres, direction, room_needed_metres, longest_metres, room=None):
        """How far the drone can go from start in this direction (east, north) before its centre
        comes closer than room_needed_metres to something blocked. Measured in half squares."""
        if room is None:
            room = self.room_metres()
        direction = np.asarray(direction[:2], dtype=float)
        direction = direction / max(np.linalg.norm(direction), 1e-9)
        step = GRID_SQUARE_METRES / 2.0
        distance = 0.0
        while distance < longest_metres:
            square = self.square_of(np.asarray(start_metres[:2]) + direction * (distance + step))
            if square is None or room[square] < room_needed_metres:
                return distance
            distance += step
        return longest_metres

    def seen_distance_metres(self, start_metres, direction, longest_metres, skip_metres):
        """How far the camera has seen empty space from start in this direction (east, north):
        the distance to the first square that is not "seen free". The first skip_metres are not
        checked, because the camera can't see the space right under and around the drone."""
        seen_free = self.seen_free()
        direction = np.asarray(direction[:2], dtype=float)
        direction = direction / max(np.linalg.norm(direction), 1e-9)
        step = GRID_SQUARE_METRES / 2.0
        distance = skip_metres
        while distance < longest_metres:
            square = self.square_of(np.asarray(start_metres[:2]) + direction * (distance + step))
            if square is None or not seen_free[square]:
                return distance
            distance += step
        return longest_metres

    # ---- Adding a depth picture ------------------------------------------------------------------

    def add_points(self, camera_on_course_metres, points_on_course_metres):
        """camera_on_course_metres: where the camera was, (east, north, height).
        points_on_course_metres: N x 3 points the camera saw, on the course."""
        points = _evenly_picked(np.asarray(points_on_course_metres, dtype=float), LARGEST_POINTS_PER_UPDATE)
        heights = points[:, 2]
        low = (heights >= LOWEST_LOW_OBSTACLE_METRES) & (heights < self.lowest_counted_height_metres)
        if low.any():
            self.low_obstacle_scores.ravel()[np.unique(self._flat_squares(points[low, :2]))] += HIT_SCORE
            np.clip(self.low_obstacle_scores, 0.0, HIGHEST_SCORE, out=self.low_obstacle_scores)
        in_band = (heights >= self.lowest_counted_height_metres) & (heights <= self.highest_counted_height_metres)
        points = points[in_band]
        if len(points) == 0:
            return
        self._clear_along_rays(np.asarray(camera_on_course_metres[:2], dtype=float), points[:, :2])
        self._add_hits(points[:, :2])
        np.clip(self.scores, LOWEST_SCORE, HIGHEST_SCORE, out=self.scores)
        self.scores[self.always_blocked] = HIGHEST_SCORE
        self.updates += 1

    def _clear_along_rays(self, camera_metres, ends_metres):
        """Takes FREE_SCORE_STEP off each square that some ray passes through (once per picture)."""
        offsets = ends_metres - camera_metres
        lengths = np.linalg.norm(offsets, axis=1)
        cleared_lengths = np.maximum(lengths - KEEP_NEAR_POINT_METRES, 0.0)
        step = GRID_SQUARE_METRES / 2.0
        steps_per_ray = int(np.ceil(cleared_lengths.max() / step)) if len(lengths) else 0
        if steps_per_ray == 0:
            return
        fractions = np.arange(steps_per_ray) * step
        directions = offsets / np.maximum(lengths, 1e-9)[:, None]
        samples = camera_metres + directions[:, None, :] * fractions[None, :, None]
        inside_ray = fractions[None, :] < cleared_lengths[:, None]
        samples = samples[inside_ray]
        squares = self._flat_squares(samples)
        self.scores.ravel()[np.unique(squares)] -= FREE_SCORE_STEP

    def _add_hits(self, ends_metres):
        squares = np.unique(self._flat_squares(ends_metres))
        self.scores.ravel()[squares] += HIT_SCORE
        self.hit_by_camera.ravel()[squares] = True

    def _flat_squares(self, points_metres):
        """Flat indices of the squares holding the points; points outside the grid are dropped."""
        indices = np.floor((points_metres - self.lowest_corner_metres) / GRID_SQUARE_METRES).astype(int)
        inside = (indices[:, 0] >= 0) & (indices[:, 0] < self.shape[0]) & (indices[:, 1] >= 0) & (indices[:, 1] < self.shape[1])
        indices = indices[inside]
        return indices[:, 0] * self.shape[1] + indices[:, 1]


def _edge_squares(shape):
    """The outermost ring of squares: the safety net."""
    edges = np.zeros(shape, dtype=bool)
    edges[0, :] = edges[-1, :] = edges[:, 0] = edges[:, -1] = True
    return edges


def _evenly_picked(points, largest_count):
    if len(points) <= largest_count:
        return points
    return points[np.linspace(0, len(points) - 1, largest_count).astype(int)]
