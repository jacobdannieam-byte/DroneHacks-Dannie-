"""The obstacle course, in a form that programs can quickly ask questions about.

SIMULATOR ONLY.

What this file does
-------------------
It reads the organizers' 3D model of the course and answers two questions:

  1. "How far is this point from the nearest surface?"
  2. "If I shoot a ray from here in this direction, how far does it travel before it
     hits something?" The pretend laser and the pretend depth camera both use this.

How it works
------------
Searching the whole 3D model on every question would be far too slow. So we prepare
an answer in advance. We divide the space into a grid of 2 cm cubes, and for every
corner of the grid we store the distance to the nearest surface. This takes about a
minute, so the result is saved to disk and reused. After that, each question is just
a lookup in a table.

A ray is then easy to follow. If the nearest surface is 40 cm away, the ray can
safely jump forward 40 cm, because nothing can be closer than that. We repeat this
until the distance becomes nearly zero, which means the ray has hit a surface.

We keep two tables:

  - "everything" includes the floor. The laser and camera can see the floor.
  - "obstacles" leaves out the floor and the markings painted on it. The referee uses
    this to detect touches, so that landing on the floor does not count as hitting
    something.

The net around the course is in neither table. Rays leave the course without hitting
it, and the referee checks for leaving the course separately.

Accuracy: answers are good to about 1 cm, because of the 2 cm grid.
"""
import os

import numpy as np
from scipy import ndimage

GRID_CELL_SIZE_METRES = 0.02

# The grid must surround the whole course. These limits come from the course model,
# which spans about -3.05 to +3.05 m sideways, and 0 to 3.05 m upward. We start the
# height a little below the floor so that a ray aimed at the floor can reach it.
GRID_LOWEST_CORNER_METRES = np.array([-3.10, -3.10, -0.10])
GRID_HIGHEST_CORNER_METRES = np.array([3.10, 3.10, 3.10])

# Surfaces that are flat on the floor, so they can never be flown into. These are the
# starting words of the part names in the organizers' model.
FLOOR_LEVEL_PART_NAMES = ("Floor", "Guideline", "Landing", "Launch", "Approach ring")
NET_PART_NAME = "Net"

# Each triangle in the model is covered with points this far apart. It is half the
# grid size, so that no grid cell along a surface can be missed.
SURFACE_POINT_SPACING_METRES = 0.01

# A ray has reached a surface when it is this close.
RAY_HIT_DISTANCE_METRES = 0.003
# Every jump is at least this long, so a ray skimming along a wall still moves on.
RAY_SMALLEST_STEP_METRES = 0.002
RAY_MOST_JUMPS = 48


def load_triangles_by_part(path_to_model_file):
    """Reads the model file. Returns {part name: array of triangles}.

    The file format is plain text. A line starting with "v" is a corner point, "g"
    starts a new named part, and "f" is a face that joins corner points. A face with
    more than three corners is split into triangles.
    """
    corner_points = []
    triangle_corner_numbers_by_part = {}
    current_part_name = None

    with open(path_to_model_file, encoding="utf-8") as model_file:
        for line in model_file:
            if line.startswith("v "):
                corner_points.append([float(number) for number in line.split()[1:4]])
            elif line.startswith("g "):
                current_part_name = line[2:].strip()
                triangle_corner_numbers_by_part.setdefault(current_part_name, [])
            elif line.startswith("f ") and current_part_name is not None:
                # A corner can be written "12/5/3". Only the first number matters,
                # and the file counts from 1, while Python counts from 0.
                corners = [int(word.split("/")[0]) - 1 for word in line.split()[1:]]
                for index in range(1, len(corners) - 1):
                    triangle_corner_numbers_by_part[current_part_name].append(
                        (corners[0], corners[index], corners[index + 1])
                    )

    corner_points = np.array(corner_points)
    triangles_by_part = {}
    for part_name, corner_numbers in triangle_corner_numbers_by_part.items():
        if corner_numbers:
            triangles_by_part[part_name] = corner_points[np.array(corner_numbers)]
    return triangles_by_part


def cover_triangle_with_points(triangle):
    """Returns points spread evenly over one triangle, close enough to fill the grid."""
    corner_a, corner_b, corner_c = triangle
    longest_side_metres = max(
        np.linalg.norm(corner_b - corner_a),
        np.linalg.norm(corner_c - corner_a),
        np.linalg.norm(corner_c - corner_b),
    )
    steps = max(1, int(np.ceil(longest_side_metres / SURFACE_POINT_SPACING_METRES)))

    # Every point on a triangle is a blend of its three corners. These two fractions
    # describe how much of corners B and C to use, and corner A gets the remainder.
    fraction_b, fraction_c = np.meshgrid(
        np.arange(steps + 1) / steps, np.arange(steps + 1) / steps
    )
    inside_triangle = fraction_b + fraction_c <= 1.0
    fraction_b = fraction_b[inside_triangle][:, np.newaxis]
    fraction_c = fraction_c[inside_triangle][:, np.newaxis]
    fraction_a = 1.0 - fraction_b - fraction_c
    return fraction_a * corner_a + fraction_b * corner_b + fraction_c * corner_c


def build_distance_table(triangles_by_part, part_names_to_include):
    """Returns a 3D table holding each grid corner's distance to the nearest surface."""
    grid_shape = _grid_shape()
    grid_corner_is_on_a_surface = np.zeros(grid_shape, dtype=bool)

    for part_name in part_names_to_include:
        for triangle in triangles_by_part[part_name]:
            points = cover_triangle_with_points(triangle)
            cell_numbers = np.round(
                (points - GRID_LOWEST_CORNER_METRES) / GRID_CELL_SIZE_METRES
            ).astype(int)
            inside_grid = np.all(
                (cell_numbers >= 0) & (cell_numbers < np.array(grid_shape)), axis=1
            )
            cell_numbers = cell_numbers[inside_grid]
            grid_corner_is_on_a_surface[
                cell_numbers[:, 0], cell_numbers[:, 1], cell_numbers[:, 2]
            ] = True

    # This standard routine finds, for every grid corner, how far it is from the
    # nearest True corner. The answer is counted in grid cells.
    distance_in_cells = ndimage.distance_transform_edt(~grid_corner_is_on_a_surface)
    return (distance_in_cells * GRID_CELL_SIZE_METRES).astype(np.float32)


def _grid_shape():
    grid_size_metres = GRID_HIGHEST_CORNER_METRES - GRID_LOWEST_CORNER_METRES
    return tuple(np.round(grid_size_metres / GRID_CELL_SIZE_METRES).astype(int) + 1)


class CourseModel:
    """Answers distance and ray questions about the course."""

    def __init__(self, path_to_model_file, folder_for_saved_tables):
        path_to_everything = os.path.join(folder_for_saved_tables, "distance_to_everything.npy")
        path_to_obstacles = os.path.join(folder_for_saved_tables, "distance_to_obstacles.npy")

        if not (os.path.exists(path_to_everything) and os.path.exists(path_to_obstacles)):
            print("Preparing the course distance tables. This takes about a minute, once.")
            os.makedirs(folder_for_saved_tables, exist_ok=True)
            self._build_and_save_tables(path_to_model_file, path_to_everything, path_to_obstacles)

        # mmap_mode means the file is read from disk as needed, so all the simulator
        # programs share one copy in memory.
        self.distance_to_everything = np.load(path_to_everything, mmap_mode="r")
        self.distance_to_obstacles = np.load(path_to_obstacles, mmap_mode="r")

    @staticmethod
    def _build_and_save_tables(path_to_model_file, path_to_everything, path_to_obstacles):
        triangles_by_part = load_triangles_by_part(path_to_model_file)
        visible_parts = [
            name for name in triangles_by_part if not name.startswith(NET_PART_NAME)
        ]
        obstacle_parts = [
            name for name in visible_parts if not name.startswith(FLOOR_LEVEL_PART_NAMES)
        ]
        # We write to a temporary name first, so two programs starting together can
        # never read a half-written table.
        for path, parts in ((path_to_everything, visible_parts), (path_to_obstacles, obstacle_parts)):
            temporary_path = f"{path}.{os.getpid()}.tmp.npy"
            np.save(temporary_path, build_distance_table(triangles_by_part, parts))
            os.replace(temporary_path, path)

    def distance_to_nearest_obstacle_metres(self, points):
        """For an array of points, returns each one's distance to the nearest obstacle."""
        return _look_up_smoothly(self.distance_to_obstacles, points)

    def cast_rays(self, ray_starts, ray_directions, longest_range_metres):
        """Follows many rays at once, and returns the distance each one travelled.

        A ray that hits nothing within range is given the value infinity. The ray
        directions must have a length of 1.
        """
        ray_starts = np.asarray(ray_starts, dtype=float)
        ray_directions = np.asarray(ray_directions, dtype=float)

        distance_travelled = np.zeros(len(ray_starts))
        ray_has_hit = np.zeros(len(ray_starts), dtype=bool)

        for _ in range(RAY_MOST_JUMPS):
            current_points = ray_starts + ray_directions * distance_travelled[:, np.newaxis]
            distance_to_surface = _look_up_smoothly(self.distance_to_everything, current_points)

            ray_has_hit |= distance_to_surface < RAY_HIT_DISTANCE_METRES
            still_flying = ~ray_has_hit & (distance_travelled < longest_range_metres)
            if not np.any(still_flying):
                break
            distance_travelled[still_flying] += np.maximum(
                distance_to_surface[still_flying], RAY_SMALLEST_STEP_METRES
            )

        distance_travelled[~ray_has_hit] = np.inf
        distance_travelled[distance_travelled > longest_range_metres] = np.inf
        return distance_travelled


def _look_up_smoothly(distance_table, points):
    """Reads the table at any point, not just at the grid corners.

    Each point sits inside one grid cube. We blend the values at that cube's 8 corners
    according to how close the point is to each one. This is known as trilinear
    interpolation. It makes the answers accurate to better than the size of a cell.
    """
    points = np.asarray(points, dtype=float)
    position_in_cells = (points - GRID_LOWEST_CORNER_METRES) / GRID_CELL_SIZE_METRES
    largest_lower_corner = np.array(distance_table.shape) - 2
    lower_corner = np.clip(np.floor(position_in_cells).astype(int), 0, largest_lower_corner)
    # How far the point has moved across its cube: 0 to 1, in each direction.
    x, y, z = np.clip(position_in_cells - lower_corner, 0.0, 1.0).T
    i, j, k = lower_corner.T

    return (
        distance_table[i, j, k] * (1 - x) * (1 - y) * (1 - z)
        + distance_table[i + 1, j, k] * x * (1 - y) * (1 - z)
        + distance_table[i, j + 1, k] * (1 - x) * y * (1 - z)
        + distance_table[i, j, k + 1] * (1 - x) * (1 - y) * z
        + distance_table[i + 1, j + 1, k] * x * y * (1 - z)
        + distance_table[i + 1, j, k + 1] * x * (1 - y) * z
        + distance_table[i, j + 1, k + 1] * (1 - x) * y * z
        + distance_table[i + 1, j + 1, k + 1] * x * y * z
    )
