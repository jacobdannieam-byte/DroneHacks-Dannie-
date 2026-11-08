"""PRETEND / SIMULATOR ONLY. Never runs on the drone.

The TRUE course for one bench trial, and what the depth camera sees in it.

How it fits in
--------------
The flight program is given the course map file as it is. The pretend world is built from the
same file, but with every solid thing moved a little (MAP_ERROR_*), so the map the program
trusts is wrong by about as much as our real maps are. bench_link.py asks this file for depth
pictures (depth_points_on_body) and bench_referee.py asks it how close the drone is to a solid
thing (nearest_solid_distance_metres).

Everything solid is a box standing on the floor or floating in the air: walls (cut into pieces
around the window), the tunnel (two side walls and a roof, an approximation of its oval), the
boulders (square boxes) and the safety net (four thin, mostly see-through boxes). Depth is a
true 3D ray-cast against those boxes and the floor (no CourseModel tables, because those are
built from the unmoved map and cannot be moved per trial).

What it does not do: anything about the drone's motion (pretend_drone.py) or scoring
(bench_referee.py). It does not model the people, poles or lights outside the net.
"""
import json
import math

import numpy as np

# ---- How wrong the map is (per trial) ------------------------------------------------------------

# Estimated from the map's own notes: the sketch walls are "about 15 cm at best", the scanned
# walls closer. Each solid thing is moved by this much east and north, independently.
MAP_ERROR_SHIFT_STANDARD_DEVIATION_METRES = 0.10

# Estimated (understand.json, "Checkpoint 1 cannot be cleared on the sketch map"): on the sketch
# the north-west top wall's face is 0.58 m past the "end of first corridor" line; the
# organizers' drawing puts the line where it would be 0.65 m; we allow up to 0.75 m.
TOP_WALL_FACE_PAST_LINE_ONE_LOWEST_METRES = 0.58
TOP_WALL_FACE_PAST_LINE_ONE_HIGHEST_METRES = 0.75
TOP_WALL_NAME = "Sketch top wall (north-west)"
LINE_ONE_NAME = "end of first corridor"

# ---- The safety net -------------------------------------------------------------------------------

# From the rulebook: the net is 10 ft tall.
NET_HEIGHT_METRES = 3.05
# Our own choice: thin enough not to matter.
NET_THICKNESS_METRES = 0.02
# Estimated. The stereo camera sees the net only where it has texture (knots, sagging folds);
# most rays go through it to the hall beyond, which is further than the camera's range.
NET_RETURN_FRACTION = 0.25

# ---- The tunnel, as boxes -------------------------------------------------------------------------

# Estimated from understand.json "Heights": inside the scanned tunnel, free width at 0.8 m is
# 1.42 m and the roof is at about 1.62 m. The oval is approximated by two vertical side walls up
# to the roof and a flat roof.
TUNNEL_INSIDE_ROOF_HEIGHT_METRES = 1.62

# ---- The depth camera -----------------------------------------------------------------------------

# From depth_camera.py (organizers' launch line on drone 16): where the camera sits on the drone.
CAMERA_FORWARD_OF_CENTRE_METRES = 0.1053
CAMERA_ABOVE_CENTRE_METRES = 0.0304
# From stereo_depth.py's numbers (317 px focal length, 512 x 320 pictures): about 78 x 54 degrees.
CAMERA_WIDE_DEGREES = 77.9
CAMERA_TALL_DEGREES = 53.6
# From depth_camera.EVERY_NTH_PIXEL (every 4th pixel of 512 x 320): rays cast per picture, before
# most are thrown away as "no depth here". This gives about 1500 points a picture, as measured
# on drone 16 (1200-2500); fewer points starve wall_alignment.py, which needs 40 near a wall.
RAYS_ACROSS = 128
RAYS_DOWN = 80
# Measured on drone 16 (understand.json, "Depth camera in flight"): 12 to 25 % of pixels are
# trusted. The task asks for 10-20 %; we keep 15 %, picked at random (the real ones are clumped
# on texture, so this is kinder than reality).
KEPT_PIXEL_FRACTION = 0.15
# Measured: the matcher's nearest possible distance (0.37 m), and our own choice in
# depth_camera.py: nothing beyond 3.5 m is published.
NEAREST_DEPTH_METRES = 0.37
FARTHEST_DEPTH_METRES = 3.5
# Estimated: stereo range noise, about 2 % of the distance (one standard deviation).
RANGE_NOISE_FRACTION = 0.02

# ---- False specks (wrong stereo matches) ----------------------------------------------------------

# Measured before the 2026-11-08 filter (understand.json): false points at 0.37-1.2 m in almost
# every picture, thousands of them. Estimated after the filter: about 10 % remain. These are
# our own stand-ins for those two settings: clusters per picture (Poisson mean), points per
# cluster, and cluster size.
SPECK_CLUSTERS_PER_PICTURE = {"filtered": 1.5, "old": 15.0}
SPECK_POINTS_PER_CLUSTER = {"filtered": (3, 15), "old": (20, 120)}
SPECK_NEAREST_METRES = 0.45
SPECK_FARTHEST_METRES = 1.2
SPECK_CLUSTER_SPREAD_DEGREES = 3.0


class Box:
    """A solid box: a footprint rectangle (centre, direction along its length, half length,
    half width) between two heights."""

    def __init__(self, name, centre_east_north, along_unit, half_length_metres, half_width_metres,
                 bottom_metres, top_metres, see_through=False):
        self.name = name
        self.centre = np.asarray(centre_east_north, dtype=float)
        self.along = np.asarray(along_unit, dtype=float)
        self.across = np.array([-self.along[1], self.along[0]])
        self.half_length_metres = half_length_metres
        self.half_width_metres = half_width_metres
        self.bottom_metres = bottom_metres
        self.top_metres = top_metres
        self.see_through = see_through


class PretendWorld:
    """The true course of one trial."""

    def __init__(self, map_path, random):
        with open(map_path, encoding="utf-8") as map_file:
            course = json.load(map_file)
        self.random = random
        self.lowest_corner = np.asarray(course["course_edges"]["lowest_corner_metres"], dtype=float)
        self.highest_corner = np.asarray(course["course_edges"]["highest_corner_metres"], dtype=float)
        self.checkpoint_lines = {line["name"]: line for line in course["checkpoint_lines"]}
        self.landing_pad_centre = np.asarray(course["landing_pad"]["centre_metres"], dtype=float)
        self.landing_pad_radius_metres = course["landing_pad"]["outer_radius_metres"]
        self.launch_mark = np.asarray(course["launch_mark"]["centre_metres"], dtype=float)
        self.boxes = []
        for wall in course["walls"]:
            self.boxes += self._wall_boxes(wall)
        for tube in course["tubes"]:
            self.boxes += self._tunnel_boxes(tube)
        for low_obstacle in course["low_obstacles"]:
            self.boxes.append(self._boulder_box(low_obstacle))
        self.boxes += self._net_boxes()
        self._stack_boxes_for_ray_casting()

    # ---- Building the true course ---------------------------------------------------------------

    def _random_shift(self):
        return self.random.normal(0.0, MAP_ERROR_SHIFT_STANDARD_DEVIATION_METRES, size=2)

    def _wall_boxes(self, wall):
        start = np.asarray(wall["start_metres"], dtype=float)
        end = np.asarray(wall["end_metres"], dtype=float)
        shift = self._random_shift()
        if wall["name"] == TOP_WALL_NAME:
            shift[1] = self._top_wall_north_shift(wall)
        start, end = self._shifted_keeping_net_ends(start, end, shift)
        length = float(np.linalg.norm(end - start))
        along = (end - start) / length
        pieces = []    # (from metres along, to metres along, bottom, top)
        cursor = 0.0
        for opening in sorted(wall["openings"], key=lambda opening: opening["start_along_wall_metres"]):
            pieces.append((cursor, opening["start_along_wall_metres"], wall["bottom_height_metres"], wall["top_height_metres"]))
            pieces.append((opening["start_along_wall_metres"], opening["end_along_wall_metres"],
                           wall["bottom_height_metres"], opening["bottom_height_metres"]))
            pieces.append((opening["start_along_wall_metres"], opening["end_along_wall_metres"],
                           opening["top_height_metres"], wall["top_height_metres"]))
            cursor = opening["end_along_wall_metres"]
        pieces.append((cursor, length, wall["bottom_height_metres"], wall["top_height_metres"]))
        boxes = []
        for from_metres, to_metres, bottom, top in pieces:
            if to_metres - from_metres <= 0.0 or top - bottom <= 0.0:
                continue
            centre = start + along * (from_metres + to_metres) / 2.0
            boxes.append(Box(wall["name"], centre, along, (to_metres - from_metres) / 2.0,
                             wall["thickness_metres"] / 2.0, bottom, top))
        return boxes

    def _top_wall_north_shift(self, wall):
        """The north shift that puts the top wall's south face the chosen distance past line 1."""
        line_north = self.checkpoint_lines[LINE_ONE_NAME]["start_metres"][1]
        face_past_line = self.random.uniform(TOP_WALL_FACE_PAST_LINE_ONE_LOWEST_METRES,
                                             TOP_WALL_FACE_PAST_LINE_ONE_HIGHEST_METRES)
        mapped_face_north = wall["start_metres"][1] - wall["thickness_metres"] / 2.0
        return line_north + face_past_line - mapped_face_north

    def _shifted_keeping_net_ends(self, start, end, shift):
        """Moves both ends; an end that touched the net in the map still touches it, so no new
        gap opens between a wall and the net."""
        moved = []
        for point in (start, end):
            new_point = point + shift
            for axis in range(2):
                for edge in (self.lowest_corner[axis], self.highest_corner[axis]):
                    if abs(point[axis] - edge) < 0.01:
                        new_point[axis] = edge
            moved.append(new_point)
        return moved[0], moved[1]

    def _tunnel_boxes(self, tube):
        entrance = np.asarray(tube["entrance_centre_metres"], dtype=float)
        exit_point = np.asarray(tube["exit_centre_metres"], dtype=float)
        shift = self._random_shift()
        entrance, exit_point = entrance + shift, exit_point + shift
        length = float(np.linalg.norm(exit_point - entrance))
        along = (exit_point - entrance) / length
        across = np.array([-along[1], along[0]])
        centre = (entrance + exit_point) / 2.0
        thickness = tube["wall_thickness_metres"]
        side_offset = tube["outside_width_metres"] / 2.0 - thickness / 2.0
        boxes = []
        for side in (-1.0, 1.0):
            boxes.append(Box(tube["name"], centre + side * side_offset * across, along, length / 2.0, thickness / 2.0,
                             0.0, TUNNEL_INSIDE_ROOF_HEIGHT_METRES))
        boxes.append(Box(tube["name"], centre, along, length / 2.0, side_offset,
                         TUNNEL_INSIDE_ROOF_HEIGHT_METRES, TUNNEL_INSIDE_ROOF_HEIGHT_METRES + thickness))
        return boxes

    def _boulder_box(self, low_obstacle):
        centre = np.asarray(low_obstacle["centre_metres"], dtype=float) + self._random_shift()
        radius = low_obstacle["radius_metres"]
        return Box(low_obstacle["name"], centre, (1.0, 0.0), radius, radius, 0.0, low_obstacle["top_height_metres"])

    def _net_boxes(self):
        low, high = self.lowest_corner, self.highest_corner
        middle = (low + high) / 2.0
        half_size = (high - low) / 2.0
        half_thickness = NET_THICKNESS_METRES / 2.0
        return [
            Box("net", (middle[0], low[1]), (1.0, 0.0), half_size[0], half_thickness, 0.0, NET_HEIGHT_METRES, True),
            Box("net", (middle[0], high[1]), (1.0, 0.0), half_size[0], half_thickness, 0.0, NET_HEIGHT_METRES, True),
            Box("net", (low[0], middle[1]), (0.0, 1.0), half_size[1], half_thickness, 0.0, NET_HEIGHT_METRES, True),
            Box("net", (high[0], middle[1]), (0.0, 1.0), half_size[1], half_thickness, 0.0, NET_HEIGHT_METRES, True),
        ]

    def _stack_boxes_for_ray_casting(self):
        """The boxes as arrays, so all rays meet all boxes in one NumPy sum."""
        self.box_centres = np.array([box.centre for box in self.boxes])
        self.box_alongs = np.array([box.along for box in self.boxes])
        self.box_acrosses = np.array([box.across for box in self.boxes])
        self.box_half_lengths = np.array([box.half_length_metres for box in self.boxes])
        self.box_half_widths = np.array([box.half_width_metres for box in self.boxes])
        self.box_bottoms = np.array([box.bottom_metres for box in self.boxes])
        self.box_tops = np.array([box.top_metres for box in self.boxes])
        self.box_see_through = np.array([box.see_through for box in self.boxes])

    # ---- Distances, for the referee -------------------------------------------------------------

    def nearest_solid_distance_metres(self, centre_east_north_up, half_height_metres):
        """(distance, name): the shortest sideways distance from the drone's centre to a solid
        box that reaches into the drone's height band."""
        height = centre_east_north_up[2]
        overlaps = (self.box_bottoms < height + half_height_metres) & (self.box_tops > height - half_height_metres)
        offsets = np.asarray(centre_east_north_up[:2]) - self.box_centres
        along = np.abs(np.sum(offsets * self.box_alongs, axis=1)) - self.box_half_lengths
        across = np.abs(np.sum(offsets * self.box_acrosses, axis=1)) - self.box_half_widths
        distances = np.hypot(np.maximum(along, 0.0), np.maximum(across, 0.0))
        distances[~overlaps] = np.inf
        nearest = int(np.argmin(distances))
        return float(distances[nearest]), self.boxes[nearest].name

    # ---- The depth camera -----------------------------------------------------------------------

    def depth_points_on_body(self, centre_east_north_up, heading_radians, speck_setting):
        """One depth picture: N x 3 float32 points (forward, left, up) from the drone's centre,
        as depth_camera.py publishes them. The drone is taken as level."""
        directions_on_body = self._kept_ray_directions_on_body()
        camera_on_body = np.array([CAMERA_FORWARD_OF_CENTRE_METRES, 0.0, CAMERA_ABOVE_CENTRE_METRES])
        turn = _turn_about_up(heading_radians)
        camera_on_course = np.asarray(centre_east_north_up, dtype=float) + turn @ camera_on_body
        directions_on_course = directions_on_body @ turn.T
        distances = self._ray_distances(camera_on_course, directions_on_course)
        distances = distances * (1.0 + self.random.normal(0.0, RANGE_NOISE_FRACTION, size=distances.shape))
        seen = (distances >= NEAREST_DEPTH_METRES) & (distances <= FARTHEST_DEPTH_METRES)
        points = camera_on_body + directions_on_body[seen] * distances[seen, None]
        specks = self._false_specks_on_body(camera_on_body, speck_setting)
        return np.vstack([points, specks]).astype(np.float32)

    def _kept_ray_directions_on_body(self):
        across_angles = np.radians(np.linspace(-CAMERA_WIDE_DEGREES / 2, CAMERA_WIDE_DEGREES / 2, RAYS_ACROSS))
        down_angles = np.radians(np.linspace(-CAMERA_TALL_DEGREES / 2, CAMERA_TALL_DEGREES / 2, RAYS_DOWN))
        left_angles, up_angles = np.meshgrid(across_angles, down_angles)
        keep = self.random.random(left_angles.shape) < KEPT_PIXEL_FRACTION
        left_slopes = np.tan(left_angles[keep])
        up_slopes = np.tan(up_angles[keep])
        directions = np.column_stack([np.ones_like(left_slopes), left_slopes, up_slopes])
        return directions / np.linalg.norm(directions, axis=1, keepdims=True)

    def _ray_distances(self, start, directions):
        """Distance along each unit ray to the first box or the floor, or inf.

        This is the "slab" method: in each box's own directions, a ray is inside the box between
        where it enters and leaves each pair of parallel faces; it hits the box if those three
        intervals overlap. Written with NumPy over all rays and boxes at once, because it runs for
        every picture."""
        offsets = start[:2] - self.box_centres                                   # boxes x 2
        start_along = np.sum(offsets * self.box_alongs, axis=1)                  # boxes
        start_across = np.sum(offsets * self.box_acrosses, axis=1)
        direction_along = directions[:, :2] @ self.box_alongs.T                  # rays x boxes
        direction_across = directions[:, :2] @ self.box_acrosses.T
        direction_up = np.repeat(directions[:, 2:3], len(self.boxes), axis=1)
        enter = np.zeros_like(direction_along)
        leave = np.full_like(direction_along, np.inf)
        for start_value, direction, low, high in (
                (start_along, direction_along, -self.box_half_lengths, self.box_half_lengths),
                (start_across, direction_across, -self.box_half_widths, self.box_half_widths),
                (np.full(len(self.boxes), start[2]), direction_up, self.box_bottoms, self.box_tops)):
            with np.errstate(divide="ignore", invalid="ignore"):
                first = (low - start_value) / direction
                second = (high - start_value) / direction
            parallel = np.abs(direction) < 1e-12
            outside_when_parallel = parallel & ((start_value < low) | (start_value > high))
            first[parallel] = -np.inf
            second[parallel] = np.inf
            enter = np.maximum(enter, np.minimum(first, second))
            leave = np.minimum(leave, np.maximum(first, second))
            leave[outside_when_parallel] = -np.inf
        hit = (enter <= leave) & (enter > 0.0)
        distances = np.where(hit, enter, np.inf)
        net_misses = self.box_see_through[None, :] & (self.random.random(distances.shape) > NET_RETURN_FRACTION)
        distances[net_misses] = np.inf
        nearest = distances.min(axis=1)
        going_down = directions[:, 2] < 0.0
        floor_distances = np.where(going_down, -start[2] / np.where(going_down, directions[:, 2], -1.0), np.inf)
        return np.minimum(nearest, floor_distances)

    def _false_specks_on_body(self, camera_on_body, speck_setting):
        cluster_count = self.random.poisson(SPECK_CLUSTERS_PER_PICTURE[speck_setting])
        fewest, most = SPECK_POINTS_PER_CLUSTER[speck_setting]
        clusters = []
        for _ in range(cluster_count):
            point_count = self.random.integers(fewest, most + 1)
            left_angle = math.radians(self.random.uniform(-CAMERA_WIDE_DEGREES / 2, CAMERA_WIDE_DEGREES / 2))
            up_angle = math.radians(self.random.uniform(-CAMERA_TALL_DEGREES / 2, CAMERA_TALL_DEGREES / 2))
            distance = self.random.uniform(SPECK_NEAREST_METRES, SPECK_FARTHEST_METRES)
            spread = math.radians(SPECK_CLUSTER_SPREAD_DEGREES)
            left_angles = left_angle + self.random.normal(0.0, spread, point_count)
            up_angles = up_angle + self.random.normal(0.0, spread, point_count)
            directions = np.column_stack([np.ones(point_count), np.tan(left_angles), np.tan(up_angles)])
            directions /= np.linalg.norm(directions, axis=1, keepdims=True)
            distances = distance * (1.0 + self.random.normal(0.0, RANGE_NOISE_FRACTION, point_count))
            clusters.append(camera_on_body + directions * distances[:, None])
        if not clusters:
            return np.zeros((0, 3))
        return np.vstack(clusters)


def _turn_about_up(heading_radians):
    """3 x 3 matrix turning (forward, left, up) on the drone into (east, north, up) on the course,
    for a nose pointing heading_radians from east toward north."""
    cosine, sine = math.cos(heading_radians), math.sin(heading_radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])
