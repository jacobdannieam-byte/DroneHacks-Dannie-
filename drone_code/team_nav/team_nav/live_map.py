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
The same is taken off along the line to every point on the floor or on a boulder top within
LOW_OBSTACLE_LONGEST_RANGE_METRES: walls reach down to the floor, so seeing the floor there means
no wall is in between.
A wall that has moved away is "seen through" a few times and becomes free. A new wall gets hit
a few times and becomes blocked. A square needs hits in three pictures to become blocked, so one or
two stray wrong points are not enough. Points nearer than NEAREST_TRUSTED_POINT_METRES are thrown
away: on 2026-11-08 the depth camera's wrong matches piled up there (see that constant).

Scores fade. Every cycle, each square's score moves back toward where it started (PRIOR_WALL_SCORE
under a map wall, 0 elsewhere) by FADE_SCORE_PER_SECOND. A small cluster of wrong points that
just blocked a square unblocks about 2 s after the camera stops seeing it, while a real wall in
view is hit several times a second and stays blocked. Old "seen free" squares slowly become
"unknown" again, so the drone only moves into space it has looked at recently.

The safety net around the course and everything outside the grid are always blocked.

If the map is so out of date that no path exists, the planner may ask for the map "without the
map's walls": then only squares the camera itself has hit count as blocked, and the map's walls
count as unknown. The drone then flies toward them, facing them, and sees whether they are real.

Boulders are kept apart. They are lower than the drone, so they can't be hit, but flying over
one makes the bottom laser's reading jump, and then PX4 may stop trusting the laser and lose its
height (it happened in the Gazebo run of 2026-11-08 over the sketch's north-west boulder). So the
map also says which squares are near a boulder: those from the map file, and those where the
camera sees something between LOWEST_LOW_OBSTACLE_METRES and the drone's height band, within
LOW_OBSTACLE_LONGEST_RANGE_METRES of the camera. Their scores fade too. The planner
avoids them when it can (live_planner.BOULDER_COST_FACTOR); they never block a path.

What it does not do: plan (live_planner.py) or read ROS messages (flight_steps.py does).
"""
import numpy as np
from scipy.ndimage import distance_transform_edt

from team_nav.clearance_grid import GRID_SQUARE_METRES, make_clearance_grid

# All scores are our own choice. A square counts as blocked above BLOCKED_SCORE.
BLOCKED_SCORE = 1.0
# A wall from the course map starts here: blocked, but two clear views through it make it free.
# Scores fade back toward this (see FADE_SCORE_PER_SECOND), so the map's guess slowly returns
# unless the camera keeps seeing through the wall.
PRIOR_WALL_SCORE = 1.5
# One point seen in a square adds this. Three pictures with a hit there block it (3 x 0.4 = 1.2).
# It was 0.6 (two pictures) until 2026-11-08, when two pictures of wrong near points were enough
# to close the first corridor in the 10:23 and 10:44 runs.
HIT_SCORE = 0.4
# One view through a square takes off this.
FREE_SCORE_STEP = 0.5
# Scores are kept between these, so old news can always be overturned by a few new pictures.
LOWEST_SCORE = -2.0
HIGHEST_SCORE = 3.0
# Below this a square counts as "seen free". Between this and BLOCKED_SCORE it is unknown.
FREE_SCORE = -0.4

# Our own choice. Each square's score moves back toward its starting score by this much each
# second. A square just blocked by three pictures of wrong points (score 1.2) is free to fly
# through again 2 s later. A real wall in view gets HIT_SCORE about 8 times a second (the depth
# program's measured rate on 2026-11-08 was 8.6 pictures a second), far more than it loses.
# A square seen free (LOWEST_SCORE) becomes "unknown" again after about 16 s without a new look.
FADE_SCORE_PER_SECOND = 0.1

# Our own choice. A point closer to the camera than this along the ray is not counted as a hit
# near its end: the last part of each ray, next to the point, is not cleared, because depth
# errors put the point a little in front of or behind the real surface.
KEEP_NEAR_POINT_METRES = 0.1

# Measured on drone 16 on 2026-11-08: the stereo matcher can't measure nearer than about 0.37 m
# (stereo_depth.LARGEST_SHIFT_PIXELS), and its wrong matches piled up just there: the nearest
# point ahead read 0.376 to 0.45 m in almost every picture of the 10:23 and 10:44 runs, in the
# air and on the ground. Points nearer than this to the camera are therefore not trusted. The
# drone keeps further than this from walls almost everywhere (drone_dimensions.REQUIRED_ROOM_METRES
# plus the camera sitting 0.1 m in front of the centre). The one exception is the last creep past
# the first checkpoint's line (live_navigation.CREEP_GAP_TO_WALL_METRES): there the map keeps the
# wall it saw on the way in.
NEAREST_TRUSTED_POINT_METRES = 0.45

# Our own choice. Points within this height of the drone's flying band count. The drone's body
# spans about 20 cm; walls are taller than the drone flies, boulders are lower. It was 0.15 m,
# which put the band's bottom only 2 cm above the sketch's north-west boulder top (0.53 m), so in
# the 3D rendered sim depth noise made that boulder a wall and closed the passage beside it
# (2026-11-08). The boulders are small rocks that cannot hurt the drone (the team, 2026-11-08), so
# with 0.08 m their tops stay out of the band and go to the low-obstacle layer instead.
HEIGHT_BAND_MARGIN_METRES = 0.08

# Our own choice. Points between this height and the drone's height band count as low obstacles
# (boulders). Lower points are the floor: 15 cm is well above the stereo error on the floor at 2 m.
LOWEST_LOW_OBSTACLE_METRES = 0.15

# Our own choice. Low points further than this from the camera are not counted as boulders. Far
# away, one pixel of depth error moves a floor point up or down by about 0.1 m or more (at 3.5 m,
# 1 pixel is about 0.5 m of range along a ray 13 degrees down), so far floor points can look like
# boulders. Boulders that matter are seen again from nearer anyway.
LOW_OBSTACLE_LONGEST_RANGE_METRES = 2.0

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
        self.scores[self.always_blocked] = HIGHEST_SCORE
        # What each score fades back to (see FADE_SCORE_PER_SECOND).
        self.starting_scores = self.scores.copy()
        self.hit_by_camera = np.zeros(self.shape, dtype=bool)
        # Distance from each square's centre to the nearest map boulder's edge (negative above one).
        self.map_boulder_distance_metres = grid.boulder_distance_metres
        self.low_obstacle_scores = np.zeros(self.shape, dtype=np.float32)
        self.updates = 0

    def fade(self, seconds):
        """Moves every score toward its starting score by FADE_SCORE_PER_SECOND x seconds, and every
        low-obstacle score toward 0 the same way. Called once per control cycle."""
        largest_change = FADE_SCORE_PER_SECOND * max(seconds, 0.0)
        self.scores += np.clip(self.starting_scores - self.scores, -largest_change, largest_change)
        self.low_obstacle_scores -= np.minimum(self.low_obstacle_scores, largest_change)

    # ---- Reading the map -----------------------------------------------------------------------

    def blocked(self, trust_map_walls=True):
        """True for blocked squares. With trust_map_walls False, a square only counts as blocked
        if the camera itself has seen something there."""
        blocked = self.scores > BLOCKED_SCORE
        if not trust_map_walls:
            blocked &= self.hit_by_camera
        return blocked | self.always_blocked

    def near_a_boulder(self, keep_away_metres=BOULDER_KEEP_AWAY_METRES):
        """True for squares where the drone's centre would be within keep_away_metres of a
        boulder, from the map file or seen by the camera."""
        seen_low = self.low_obstacle_scores > BLOCKED_SCORE
        distance_to_seen = distance_transform_edt(~seen_low) * GRID_SQUARE_METRES
        return (self.map_boulder_distance_metres < keep_away_metres) | (distance_to_seen < keep_away_metres)

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
        """How far the camera has seen this direction (east, north) from start: the distance to
        the first square it knows nothing about, that is, neither "seen free" nor a wall the camera
        itself has seen. The first skip_metres are not checked, because the camera can't see the
        space right under and around the drone.
        A seen wall counts as known: free_distance_metres already keeps the drone away from it.
        Stopping at the wall's near edge here was up to 5 cm stricter than that room rule, and it
        held the first-corridor creep that much further from the wall (3D rendered sim, run
        20261108_225641)."""
        seen_free = self.seen_free() | self.blocked(trust_map_walls=False)
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
        distances_from_camera = np.linalg.norm(points - np.asarray(camera_on_course_metres, dtype=float), axis=1)
        trusted = distances_from_camera >= NEAREST_TRUSTED_POINT_METRES
        points = points[trusted]
        distances_from_camera = distances_from_camera[trusted]
        heights = points[:, 2]
        low = ((heights >= LOWEST_LOW_OBSTACLE_METRES) & (heights < self.lowest_counted_height_metres)
               & (distances_from_camera <= LOW_OBSTACLE_LONGEST_RANGE_METRES))
        if low.any():
            self.low_obstacle_scores.ravel()[np.unique(self._flat_squares(points[low, :2]))] += HIT_SCORE
            np.clip(self.low_obstacle_scores, 0.0, HIGHEST_SCORE, out=self.low_obstacle_scores)
        in_band = (heights >= self.lowest_counted_height_metres) & (heights <= self.highest_counted_height_metres)
        # A ray that reaches the floor or a boulder top nearby also proves that no wall stands
        # between the camera and that point (walls reach down to the floor). Without this, open
        # space with no wall behind it was never "seen free", and in the 3D rendered sim the drone
        # hovered beside the north-west boulder until a safety rule ended the run (2026-11-08).
        below_band_nearby = ((heights < self.lowest_counted_height_metres)
                             & (distances_from_camera <= LOW_OBSTACLE_LONGEST_RANGE_METRES))
        ray_ends = in_band | below_band_nearby
        if not ray_ends.any():
            return
        self._clear_along_rays(np.asarray(camera_on_course_metres[:2], dtype=float), points[ray_ends, :2])
        self._add_hits(points[in_band, :2])
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
