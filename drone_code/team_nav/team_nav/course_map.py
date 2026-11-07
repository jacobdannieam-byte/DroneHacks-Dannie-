"""Reads a course map file, and answers simple questions about the shape of the course.

What a course map is
--------------------
A course map is a small text file (JSON) that describes the obstacle course with a few simple
shapes:

  - straight walls, which may have openings in them, such as the window;
  - tubes, such as the tunnel;
  - low obstacles, such as boulders, which the drone flies over;
  - the launch mark, the landing pad, and the checkpoint lines from the rulebook.

The drone's route is never written into our code. The route planner (route_planner.py) reads a
map file and works the route out, so a different course needs only a different map file. The
map files are in the course_maps folder next to this package's code.

Every shape in a map file has a "source" field that says where its numbers came from, for
example "measured from the phone scan" or "from the organizers' 3D model". Loading fails if a
shape has no source, so a guess can never pass for a measurement.

Positions
---------
All positions are in metres, on the same axes as the organizers' 3D model of the course. x and
y lie on the floor, with (0, 0) at the centre of the course. Heights are measured up from the
floor.

What this file does not do
--------------------------
It does not plan routes, and it does not draw anything. It only reads, checks, saves, and
describes the map.
"""
import json
import math
from dataclasses import dataclass, field

import numpy as np

# The version of the file layout that this code understands. If we ever change the layout, we
# raise this number, so that an old file can't be misread.
MAP_FORMAT_VERSION = 1


class CourseMapError(ValueError):
    """Raised when a map file is missing something or contains something impossible."""


@dataclass
class Opening:
    """A hole through a wall, such as the window. It is measured along the wall from the
    wall's start point."""
    name: str
    start_along_wall_metres: float
    end_along_wall_metres: float
    bottom_height_metres: float
    top_height_metres: float
    source: str

    def height_metres(self):
        return self.top_height_metres - self.bottom_height_metres

    def centre_height_metres(self):
        return (self.bottom_height_metres + self.top_height_metres) / 2.0


@dataclass
class Wall:
    """A straight wall: a flat slab standing on its long edge, seen from above as a thick line
    from start_metres to end_metres. It fills the space between its bottom and top heights.
    Its bottom height is above zero when the wall stands on legs."""
    name: str
    start_metres: np.ndarray
    end_metres: np.ndarray
    thickness_metres: float
    bottom_height_metres: float
    top_height_metres: float
    openings: list
    source: str

    def length_metres(self):
        return float(np.linalg.norm(self.end_metres - self.start_metres))

    def direction(self):
        """A vector of length 1 pointing along the wall, from its start to its end."""
        return (self.end_metres - self.start_metres) / self.length_metres()

    def point_along_wall_metres(self, distance_along_metres):
        return self.start_metres + self.direction() * distance_along_metres

    def solid_pieces_between_heights(self, lowest_metres, highest_metres, openings_to_ignore=()):
        """Returns the stretches of this wall that block something flying between these two
        heights, as a list of (start, end) distances along the wall.

        An opening only lets the flyer through if the whole height range fits inside it.
        Openings named in openings_to_ignore are treated as passable whatever their height,
        because the route planner changes the flying height there.
        """
        if highest_metres <= self.bottom_height_metres or lowest_metres >= self.top_height_metres:
            return []

        solid_pieces = [(0.0, self.length_metres())]
        for opening in self.openings:
            fits_inside_opening = (
                opening.bottom_height_metres <= lowest_metres
                and opening.top_height_metres >= highest_metres
            )
            if fits_inside_opening or opening.name in openings_to_ignore:
                solid_pieces = _cut_out(solid_pieces, opening.start_along_wall_metres, opening.end_along_wall_metres)
        return solid_pieces


@dataclass
class Tube:
    """A tunnel with an oval cross-section, lying on the floor. Seen from the end, it is an
    oval ring of the given outside width and height, with walls of the given thickness."""
    name: str
    entrance_centre_metres: np.ndarray
    exit_centre_metres: np.ndarray
    outside_width_metres: float
    outside_height_metres: float
    wall_thickness_metres: float
    source: str

    def length_metres(self):
        return float(np.linalg.norm(self.exit_centre_metres - self.entrance_centre_metres))

    def blocked_strips_between_heights(self, lowest_metres, highest_metres):
        """Returns the parts of the tube's width that block something flying between these
        heights, as a list of (nearest, farthest) sideways distances from the tube's centre
        line. Positive distances are on the left when looking from the entrance to the exit.
        """
        middle_height_metres = self.outside_height_metres / 2.0
        outside_half_width_metres = self.outside_width_metres / 2.0
        outside_half_height_metres = self.outside_height_metres / 2.0
        inside_half_width_metres = outside_half_width_metres - self.wall_thickness_metres
        inside_half_height_metres = outside_half_height_metres - self.wall_thickness_metres

        if lowest_metres >= self.outside_height_metres or highest_metres <= 0.0:
            return []

        # If the height range sticks out of the inside of the oval, nothing fits through.
        fits_inside_vertically = (
            lowest_metres >= middle_height_metres - inside_half_height_metres
            and highest_metres <= middle_height_metres + inside_half_height_metres
        )
        if not fits_inside_vertically:
            return [(-outside_half_width_metres, outside_half_width_metres)]

        # The inside is narrowest at whichever end of the height range is farther from the
        # middle. The outside is widest at the height closest to the middle.
        farthest_from_middle_metres = max(abs(lowest_metres - middle_height_metres), abs(highest_metres - middle_height_metres))
        free_half_width_metres = _oval_half_width(inside_half_width_metres, inside_half_height_metres, farthest_from_middle_metres)
        closest_height_metres = min(max(middle_height_metres, lowest_metres), highest_metres)
        blocked_half_width_metres = _oval_half_width(
            outside_half_width_metres, outside_half_height_metres, abs(closest_height_metres - middle_height_metres)
        )
        return [
            (-blocked_half_width_metres, -free_half_width_metres),
            (free_half_width_metres, blocked_half_width_metres),
        ]


@dataclass
class LowObstacle:
    """Something short standing on the floor, such as a boulder. Seen from above it is a
    circle. The drone normally flies over it."""
    name: str
    centre_metres: np.ndarray
    radius_metres: float
    top_height_metres: float
    source: str


@dataclass
class CheckpointLine:
    """A line on the floor that the whole drone must cross, in the direction that ends up on
    the same side as far_side_point_metres.

    If the line runs across an opening in a wall, such as the window, opening_name names it.
    The drone must then cross at the opening's height, not its usual flying height."""
    name: str
    start_metres: np.ndarray
    end_metres: np.ndarray
    far_side_point_metres: np.ndarray
    source: str
    opening_name: str = None


@dataclass
class CourseMap:
    name: str
    description: str
    lowest_corner_metres: np.ndarray    # The course edge (safety net), smallest x and y.
    highest_corner_metres: np.ndarray   # The course edge, largest x and y.
    wall_top_height_metres: float       # The rulebook says the drone must stay below this.
    edges_source: str
    launch_mark_centre_metres: np.ndarray
    launch_mark_source: str
    landing_pad_centre_metres: np.ndarray
    landing_pad_outer_radius_metres: float
    approach_ring_radius_metres: float
    landing_pad_source: str
    walls: list = field(default_factory=list)
    tubes: list = field(default_factory=list)
    low_obstacles: list = field(default_factory=list)
    checkpoint_lines: list = field(default_factory=list)

    def wall_named(self, name):
        for wall in self.walls:
            if wall.name == name:
                return wall
        raise CourseMapError(f"The map '{self.name}' has no wall called '{name}'.")

    def checkpoint_line_named(self, name):
        for line in self.checkpoint_lines:
            if line.name == name:
                return line
        raise CourseMapError(f"The map '{self.name}' has no checkpoint line called '{name}'.")

    def opening_named(self, name):
        """Returns (the wall, the opening) for the opening with this name."""
        for wall in self.walls:
            for opening in wall.openings:
                if opening.name == name:
                    return wall, opening
        raise CourseMapError(f"The map '{self.name}' has no opening called '{name}'.")


# ---- Reading and checking a map file ---------------------------------------------------------

def load_course_map(path_to_map_file):
    """Reads a map file and returns a CourseMap. Raises CourseMapError, with a sentence saying
    what is wrong, if anything is missing or impossible."""
    with open(path_to_map_file, encoding="utf-8") as map_file:
        try:
            contents = json.load(map_file)
        except json.JSONDecodeError as error:
            raise CourseMapError(
                f"{path_to_map_file} is not valid JSON ({error}). Check for a missing comma or bracket."
            ) from error
    return course_map_from_dictionary(contents, path_to_map_file)


def course_map_from_dictionary(contents, where_it_came_from="the map"):
    reader = _Reader(contents, where_it_came_from)
    version = reader.number("map_format_version")
    if version != MAP_FORMAT_VERSION:
        raise CourseMapError(
            f"{where_it_came_from} uses map format version {version}, but this code reads version "
            f"{MAP_FORMAT_VERSION}. Update the file or the code so they match."
        )

    edges = reader.section("course_edges")
    launch_mark = reader.section("launch_mark")
    landing_pad = reader.section("landing_pad")
    course_map = CourseMap(
        name=reader.text("name"),
        description=reader.text("description"),
        lowest_corner_metres=edges.point("lowest_corner_metres"),
        highest_corner_metres=edges.point("highest_corner_metres"),
        wall_top_height_metres=edges.positive_number("wall_top_height_metres"),
        edges_source=edges.source(),
        launch_mark_centre_metres=launch_mark.point("centre_metres"),
        launch_mark_source=launch_mark.source(),
        landing_pad_centre_metres=landing_pad.point("centre_metres"),
        landing_pad_outer_radius_metres=landing_pad.positive_number("outer_radius_metres"),
        approach_ring_radius_metres=landing_pad.positive_number("approach_ring_radius_metres"),
        landing_pad_source=landing_pad.source(),
        walls=[_read_wall(wall) for wall in reader.list_of_sections("walls")],
        tubes=[_read_tube(tube) for tube in reader.list_of_sections("tubes")],
        low_obstacles=[_read_low_obstacle(item) for item in reader.list_of_sections("low_obstacles")],
        checkpoint_lines=[_read_checkpoint_line(line) for line in reader.list_of_sections("checkpoint_lines")],
    )
    reader.check_nothing_unexpected()
    _check_names_are_unique(course_map, where_it_came_from)
    _check_openings_named_by_lines_exist(course_map, where_it_came_from)
    return course_map


def _read_wall(reader):
    wall = Wall(
        name=reader.text("name"),
        start_metres=reader.point("start_metres"),
        end_metres=reader.point("end_metres"),
        thickness_metres=reader.positive_number("thickness_metres"),
        bottom_height_metres=reader.number("bottom_height_metres"),
        top_height_metres=reader.positive_number("top_height_metres"),
        openings=[_read_opening(opening) for opening in reader.list_of_sections("openings", required=False)],
        source=reader.source(),
    )
    reader.check_nothing_unexpected()
    if wall.length_metres() <= 0.0:
        raise CourseMapError(f"Wall '{wall.name}' in {reader.where} starts and ends at the same point.")
    if wall.top_height_metres <= wall.bottom_height_metres:
        raise CourseMapError(f"Wall '{wall.name}' in {reader.where} has its top below its bottom.")
    for opening in wall.openings:
        inside_wall = 0.0 <= opening.start_along_wall_metres < opening.end_along_wall_metres <= wall.length_metres()
        if not inside_wall:
            raise CourseMapError(
                f"Opening '{opening.name}' in wall '{wall.name}' ({reader.where}) does not fit along the "
                f"wall, which is {wall.length_metres():.2f} m long."
            )
    return wall


def _read_opening(reader):
    opening = Opening(
        name=reader.text("name"),
        start_along_wall_metres=reader.number("start_along_wall_metres"),
        end_along_wall_metres=reader.number("end_along_wall_metres"),
        bottom_height_metres=reader.number("bottom_height_metres"),
        top_height_metres=reader.positive_number("top_height_metres"),
        source=reader.source(),
    )
    reader.check_nothing_unexpected()
    if opening.top_height_metres <= opening.bottom_height_metres:
        raise CourseMapError(f"Opening '{opening.name}' in {reader.where} has its top below its bottom.")
    return opening


def _read_tube(reader):
    tube = Tube(
        name=reader.text("name"),
        entrance_centre_metres=reader.point("entrance_centre_metres"),
        exit_centre_metres=reader.point("exit_centre_metres"),
        outside_width_metres=reader.positive_number("outside_width_metres"),
        outside_height_metres=reader.positive_number("outside_height_metres"),
        wall_thickness_metres=reader.positive_number("wall_thickness_metres"),
        source=reader.source(),
    )
    reader.check_nothing_unexpected()
    return tube


def _read_low_obstacle(reader):
    obstacle = LowObstacle(
        name=reader.text("name"),
        centre_metres=reader.point("centre_metres"),
        radius_metres=reader.positive_number("radius_metres"),
        top_height_metres=reader.positive_number("top_height_metres"),
        source=reader.source(),
    )
    reader.check_nothing_unexpected()
    return obstacle


def _read_checkpoint_line(reader):
    line = CheckpointLine(
        name=reader.text("name"),
        start_metres=reader.point("start_metres"),
        end_metres=reader.point("end_metres"),
        far_side_point_metres=reader.point("far_side_point_metres"),
        source=reader.source(),
        opening_name=reader.optional_text("opening_name"),
    )
    reader.check_nothing_unexpected()
    return line


def _check_openings_named_by_lines_exist(course_map, where_it_came_from):
    opening_names = [opening.name for wall in course_map.walls for opening in wall.openings]
    for line in course_map.checkpoint_lines:
        if line.opening_name is not None and line.opening_name not in opening_names:
            raise CourseMapError(
                f"The checkpoint line '{line.name}' in {where_it_came_from} runs through an opening called "
                f"'{line.opening_name}', but no wall has an opening with that name."
            )


def _check_names_are_unique(course_map, where_it_came_from):
    names = [wall.name for wall in course_map.walls]
    names += [opening.name for wall in course_map.walls for opening in wall.openings]
    names += [tube.name for tube in course_map.tubes]
    names += [obstacle.name for obstacle in course_map.low_obstacles]
    names += [line.name for line in course_map.checkpoint_lines]
    for name in names:
        if names.count(name) > 1:
            raise CourseMapError(f"The name '{name}' is used more than once in {where_it_came_from}. Every shape needs its own name.")


class _Reader:
    """Reads one section of a map file, and remembers which entries were used, so that a
    misspelt entry name is reported instead of silently ignored."""

    def __init__(self, contents, where):
        if not isinstance(contents, dict):
            raise CourseMapError(f"Expected a section with named entries in {where}.")
        self.contents = contents
        self.where = where
        self.used_names = set()

    def _get(self, name, required=True):
        self.used_names.add(name)
        if name not in self.contents:
            if required:
                raise CourseMapError(f"{self.where} is missing the entry '{name}'.")
            return None
        return self.contents[name]

    def text(self, name):
        value = self._get(name)
        if not isinstance(value, str) or not value.strip():
            raise CourseMapError(f"The entry '{name}' in {self.where} must be some text.")
        return value

    def optional_text(self, name):
        """Some text that may be left out. Returns None if it is."""
        if name not in self.contents:
            self.used_names.add(name)
            return None
        return self.text(name)

    def source(self):
        """Every shape must say where its numbers came from."""
        return self.text("source")

    def number(self, name):
        value = self._get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise CourseMapError(f"The entry '{name}' in {self.where} must be a number.")
        return float(value)

    def positive_number(self, name):
        value = self.number(name)
        if value <= 0.0:
            raise CourseMapError(f"The entry '{name}' in {self.where} must be more than zero.")
        return value

    def point(self, name):
        value = self._get(name)
        if not isinstance(value, list) or len(value) != 2 or not all(isinstance(number, (int, float)) for number in value):
            raise CourseMapError(f"The entry '{name}' in {self.where} must be a pair of numbers, [x, y].")
        return np.array(value, dtype=float)

    def section(self, name):
        return _Reader(self._get(name), f"'{name}' in {self.where}")

    def list_of_sections(self, name, required=True):
        value = self._get(name, required)
        if value is None:
            return []
        if not isinstance(value, list):
            raise CourseMapError(f"The entry '{name}' in {self.where} must be a list.")
        readers = []
        for number, item in enumerate(value):
            label = item.get("name", f"number {number + 1}") if isinstance(item, dict) else f"number {number + 1}"
            readers.append(_Reader(item, f"'{label}' in '{name}' in {self.where}"))
        return readers

    def check_nothing_unexpected(self):
        unexpected = sorted(set(self.contents) - self.used_names)
        if unexpected:
            raise CourseMapError(
                f"{self.where} has entries this code does not know: {', '.join(unexpected)}. "
                "Check the spelling, or update course_map.py if the map format has changed."
            )


# ---- Saving a map file -----------------------------------------------------------------------

def save_course_map(course_map, path_to_map_file):
    """Writes the map in the same layout that load_course_map reads."""
    with open(path_to_map_file, "w", encoding="utf-8") as map_file:
        json.dump(course_map_to_dictionary(course_map), map_file, indent=2)
        map_file.write("\n")


def course_map_to_dictionary(course_map):
    def point(array):
        return [round(float(array[0]), 4), round(float(array[1]), 4)]

    def number(value):
        return round(float(value), 4)

    return {
        "map_format_version": MAP_FORMAT_VERSION,
        "name": course_map.name,
        "description": course_map.description,
        "course_edges": {
            "lowest_corner_metres": point(course_map.lowest_corner_metres),
            "highest_corner_metres": point(course_map.highest_corner_metres),
            "wall_top_height_metres": number(course_map.wall_top_height_metres),
            "source": course_map.edges_source,
        },
        "launch_mark": {"centre_metres": point(course_map.launch_mark_centre_metres), "source": course_map.launch_mark_source},
        "landing_pad": {
            "centre_metres": point(course_map.landing_pad_centre_metres),
            "outer_radius_metres": number(course_map.landing_pad_outer_radius_metres),
            "approach_ring_radius_metres": number(course_map.approach_ring_radius_metres),
            "source": course_map.landing_pad_source,
        },
        "walls": [
            {
                "name": wall.name,
                "start_metres": point(wall.start_metres),
                "end_metres": point(wall.end_metres),
                "thickness_metres": number(wall.thickness_metres),
                "bottom_height_metres": number(wall.bottom_height_metres),
                "top_height_metres": number(wall.top_height_metres),
                "openings": [
                    {
                        "name": opening.name,
                        "start_along_wall_metres": number(opening.start_along_wall_metres),
                        "end_along_wall_metres": number(opening.end_along_wall_metres),
                        "bottom_height_metres": number(opening.bottom_height_metres),
                        "top_height_metres": number(opening.top_height_metres),
                        "source": opening.source,
                    }
                    for opening in wall.openings
                ],
                "source": wall.source,
            }
            for wall in course_map.walls
        ],
        "tubes": [
            {
                "name": tube.name,
                "entrance_centre_metres": point(tube.entrance_centre_metres),
                "exit_centre_metres": point(tube.exit_centre_metres),
                "outside_width_metres": number(tube.outside_width_metres),
                "outside_height_metres": number(tube.outside_height_metres),
                "wall_thickness_metres": number(tube.wall_thickness_metres),
                "source": tube.source,
            }
            for tube in course_map.tubes
        ],
        "low_obstacles": [
            {
                "name": obstacle.name,
                "centre_metres": point(obstacle.centre_metres),
                "radius_metres": number(obstacle.radius_metres),
                "top_height_metres": number(obstacle.top_height_metres),
                "source": obstacle.source,
            }
            for obstacle in course_map.low_obstacles
        ],
        "checkpoint_lines": [_checkpoint_line_to_dictionary(line, point) for line in course_map.checkpoint_lines],
    }


def _checkpoint_line_to_dictionary(line, point):
    contents = {
        "name": line.name,
        "start_metres": point(line.start_metres),
        "end_metres": point(line.end_metres),
        "far_side_point_metres": point(line.far_side_point_metres),
    }
    if line.opening_name is not None:
        contents["opening_name"] = line.opening_name
    contents["source"] = line.source
    return contents


# ---- What blocks the drone at a given height --------------------------------------------------

@dataclass
class BlockingStrip:
    """Seen from above, a thick straight line that the drone must not touch."""
    name: str
    start_metres: np.ndarray
    end_metres: np.ndarray
    half_thickness_metres: float


@dataclass
class BlockingCircle:
    """Seen from above, a filled circle that the drone must not touch."""
    name: str
    centre_metres: np.ndarray
    radius_metres: float


def shapes_blocking_between_heights(course_map, lowest_metres, highest_metres, openings_to_ignore=()):
    """Returns everything that blocks a flyer whose body spans these two heights, seen from
    above: a list of BlockingStrips and BlockingCircles. The safety net around the course is
    included as four strips of zero thickness."""
    shapes = _course_edge_strips(course_map)

    for wall in course_map.walls:
        for start_along_metres, end_along_metres in wall.solid_pieces_between_heights(lowest_metres, highest_metres, openings_to_ignore):
            shapes.append(BlockingStrip(
                name=wall.name,
                start_metres=wall.point_along_wall_metres(start_along_metres),
                end_metres=wall.point_along_wall_metres(end_along_metres),
                half_thickness_metres=wall.thickness_metres / 2.0,
            ))

    for tube in course_map.tubes:
        along = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
        to_the_left = np.array([-along[1], along[0]])
        for nearest_metres, farthest_metres in tube.blocked_strips_between_heights(lowest_metres, highest_metres):
            middle_offset = to_the_left * (nearest_metres + farthest_metres) / 2.0
            shapes.append(BlockingStrip(
                name=tube.name,
                start_metres=tube.entrance_centre_metres + middle_offset,
                end_metres=tube.exit_centre_metres + middle_offset,
                half_thickness_metres=(farthest_metres - nearest_metres) / 2.0,
            ))

    for obstacle in course_map.low_obstacles:
        if lowest_metres < obstacle.top_height_metres:
            shapes.append(BlockingCircle(obstacle.name, obstacle.centre_metres, obstacle.radius_metres))
    return shapes


def distance_to_shapes_metres(points_metres, shapes):
    """For an array of points seen from above (N x 2), returns each point's distance to the
    nearest shape, and the name of that shape. Inside a shape counts as distance zero."""
    points_metres = np.asarray(points_metres, dtype=float)
    nearest_distance_metres = np.full(len(points_metres), np.inf)
    nearest_name = np.full(len(points_metres), "", dtype=object)
    for shape in shapes:
        if isinstance(shape, BlockingStrip):
            distance_metres = distance_to_line_piece_metres(points_metres, shape.start_metres, shape.end_metres) - shape.half_thickness_metres
        else:
            distance_metres = np.linalg.norm(points_metres - shape.centre_metres, axis=1) - shape.radius_metres
        distance_metres = np.maximum(distance_metres, 0.0)
        closer = distance_metres < nearest_distance_metres
        nearest_distance_metres[closer] = distance_metres[closer]
        nearest_name[closer] = shape.name
    return nearest_distance_metres, nearest_name


def _course_edge_strips(course_map):
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    corners = [np.array(corner) for corner in ((low_x, low_y), (high_x, low_y), (high_x, high_y), (low_x, high_y))]
    strips = []
    for number in range(4):
        strips.append(BlockingStrip("the safety net", corners[number], corners[(number + 1) % 4], 0.0))
    return strips


def distance_to_line_piece_metres(points_metres, start_metres, end_metres):
    """The distance from each point to the nearest point on the straight piece from start to end."""
    piece = end_metres - start_metres
    length_squared = float(piece @ piece)
    if length_squared == 0.0:
        return np.linalg.norm(points_metres - start_metres, axis=1)
    # How far along the piece the nearest point is, from 0 (start) to 1 (end).
    fraction_along = np.clip((points_metres - start_metres) @ piece / length_squared, 0.0, 1.0)
    nearest_points = start_metres + fraction_along[:, np.newaxis] * piece
    return np.linalg.norm(points_metres - nearest_points, axis=1)


def _cut_out(pieces, cut_start, cut_end):
    """Removes the stretch from cut_start to cut_end from a list of (start, end) pieces."""
    remaining = []
    for piece_start, piece_end in pieces:
        if cut_end <= piece_start or cut_start >= piece_end:
            remaining.append((piece_start, piece_end))
            continue
        if piece_start < cut_start:
            remaining.append((piece_start, cut_start))
        if cut_end < piece_end:
            remaining.append((cut_end, piece_end))
    return remaining


def _oval_half_width(half_width_metres, half_height_metres, height_from_middle_metres):
    """How wide an oval is, from its middle to one side, at a given height from its middle."""
    if height_from_middle_metres >= half_height_metres:
        return 0.0
    return half_width_metres * math.sqrt(1.0 - (height_from_middle_metres / half_height_metres) ** 2)
