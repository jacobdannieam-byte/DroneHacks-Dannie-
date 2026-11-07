"""Measures walls, the window, the tunnel and low objects from our phone scan's points.

This runs on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.
phone_scan_points.py first puts the scan's points on the course's axes. This file measures
shapes from them. make_map_from_phone_scan.py decides which measurements to trust and writes
the map files.

How a wall is measured
----------------------
We chose a search box around each wall by looking at a picture of the scan. Inside the box, at
heights between 0.5 m and 1.6 m (clear of the floor, boulders and the wall's top edge), the
points form a long thin band. The band's middle line is the wall's centre line, and its spread
across is the wall's thickness. The scan sees both faces of most walls, about 10 cm apart.

A scan is noisy and can draw part of the course twice when the phone loses track of where it
is, so every measurement here should be checked against a picture before it is trusted.
"""
import numpy as np
from scipy import ndimage

from team_nav.course_map import LowObstacle, Opening, Tube, Wall

SOURCE_START = "Measured from our phone scan of 2026-11-06 by course_map_tools/measure_phone_scan.py"

# ---- Walls ------------------------------------------------------------------------------------
# Our own choice. Wall points are taken from this height range, which is clear of the floor,
# the boulders and the wall's top edge.
WALL_LOWEST_METRES = 0.5
WALL_HIGHEST_METRES = 1.6
# Our own choice. The ends of a wall are where all but 1% of its points lie, so that a few
# stray points don't make it longer.
STRAY_POINTS_PERCENT = 1
# Our own choice. The thickness is the spread of the middle 80% of the points across the wall.
THICKNESS_PERCENTILES = (10, 90)
# From the organizers' 3D drawing: the thinnest wall panel is about 10 cm (4 inches) thick.
# A thinner measurement means the scan only saw one face of the wall.
THINNEST_WALL_METRES = 0.10
# Our own choice. The top of a wall is where all but 2% of the points below 2.5 m lie.
WALL_TOP_PERCENTILE = 98
HIGHEST_POINT_COUNTED_METRES = 2.5

# ---- The window -------------------------------------------------------------------------------
# Our own choice. Points closer than this to the wall's centre line belong to the wall.
POINTS_ON_WALL_WITHIN_METRES = 0.08
# Our own choice. The wall is looked at face-on, on a grid of squares this big.
FACE_ON_GRID_SQUARE_METRES = 0.03
# Our own choice. A square with fewer points than this is treated as empty.
POINTS_TO_COVER_A_SQUARE = 2

# ---- The tunnel -------------------------------------------------------------------------------
# Our own choice. Points higher than this, near the drawing's tunnel centre line, are the
# tunnel's roof. The walls nearby are farther from the centre line, and the boulders are lower.
TUNNEL_ROOF_LOWEST_METRES = 1.3
TUNNEL_ROOF_SEARCH_HALF_WIDTH_METRES = 0.4
TUNNEL_SEARCH_BEYOND_DRAWN_ENDS_METRES = 0.5
# Our own choice. The tunnel's sides are measured at about flying height.
TUNNEL_SIDES_LOWEST_METRES = 0.6
TUNNEL_SIDES_HIGHEST_METRES = 1.0
# Our own choice. Only points this far inside the tunnel's ends count as its sides, so that
# walls near its openings don't.
TUNNEL_SIDES_INSIDE_ENDS_METRES = 0.3
# Our own choice. The inside face is where the points closest to the middle are, ignoring the
# closest 2%, which are stray points.
TUNNEL_INSIDE_FACE_PERCENT = 2
# Our own choice. A tunnel narrower or wider than this means the measurement went wrong.
NARROWEST_BELIEVABLE_TUNNEL_METRES = 1.0
WIDEST_BELIEVABLE_TUNNEL_METRES = 2.0

# ---- Low objects ------------------------------------------------------------------------------
# Our own choice. Things standing on the floor between these heights, with nothing above them
# at wall height, are low objects such as boulders.
LOW_OBJECT_LOWEST_METRES = 0.15
LOW_OBJECT_HIGHEST_METRES = 0.75
WALL_HEIGHT_LOWEST_METRES = 0.9
WALL_HEIGHT_HIGHEST_METRES = 1.6
LOW_OBJECT_GRID_SQUARE_METRES = 0.05
# Our own choice. Objects covering fewer squares than this (0.03 square metres) are noise.
SMALLEST_LOW_OBJECT_SQUARES = 12
# Our own choice. Anything shorter than this is left out. The drone flies well above it.
SHORTEST_LOW_OBJECT_METRES = 0.2
# Our own choice. Only look inside the net, which is 3.048 m from the centre on each side.
LOW_OBJECT_SEARCH_HALF_SIZE_METRES = 3.0


def points_in_box(points, box, lowest_metres, highest_metres):
    """The points inside a box given as (smallest x, largest x, smallest y, largest y), between
    two heights."""
    smallest_x, largest_x, smallest_y, largest_y = box
    inside = (
        (points[:, 0] > smallest_x) & (points[:, 0] < largest_x)
        & (points[:, 1] > smallest_y) & (points[:, 1] < largest_y)
        & (points[:, 2] > lowest_metres) & (points[:, 2] < highest_metres)
    )
    return points[inside]


# ---- Walls ------------------------------------------------------------------------------------

def measure_wall(points, name, search_box):
    """Measures the wall whose points are inside search_box. Its ends are where its points end;
    the caller may extend them."""
    wall_points = points_in_box(points, search_box, WALL_LOWEST_METRES, WALL_HIGHEST_METRES)[:, :2]
    if len(wall_points) < 100:
        raise SystemExit(f"Found almost no scan points for '{name}' in the box {search_box}. Check the box against a picture of the scan.")
    middle = wall_points.mean(axis=0)
    along = pointing_north_or_east(_main_direction(wall_points - middle))
    across = np.array([-along[1], along[0]])
    along_values = (wall_points - middle) @ along
    across_values = (wall_points - middle) @ across

    centre_line_offset = np.median(across_values)
    thickness_low, thickness_high = np.percentile(across_values, THICKNESS_PERCENTILES)
    centre_line_point = middle + across * centre_line_offset
    tall_points = points_in_box(points, search_box, 0.0, HIGHEST_POINT_COUNTED_METRES)
    return Wall(
        name=name,
        start_metres=centre_line_point + along * np.percentile(along_values, STRAY_POINTS_PERCENT),
        end_metres=centre_line_point + along * np.percentile(along_values, 100 - STRAY_POINTS_PERCENT),
        thickness_metres=float(max(thickness_high - thickness_low, THINNEST_WALL_METRES)),
        bottom_height_metres=0.0,
        top_height_metres=float(np.percentile(tall_points[:, 2], WALL_TOP_PERCENTILE)),
        openings=[],
        source=f"{SOURCE_START}, from the scan points in the box x {search_box[0]} to {search_box[1]} m, y {search_box[2]} to {search_box[3]} m.",
    )


def _main_direction(points_around_zero):
    """The direction in which a cloud of points is longest (its first principal direction)."""
    _, _, directions = np.linalg.svd(points_around_zero, full_matrices=False)
    return directions[0]


def pointing_north_or_east(direction):
    """Every wall is stored running north if it runs mostly north-south, or east if it runs
    mostly east-west, so that the same wall always has the same start and end. The organizers'
    drawing is stored the same way."""
    runs_mostly_north_south = abs(direction[1]) > abs(direction[0])
    if runs_mostly_north_south and direction[1] < 0:
        return -direction
    if not runs_mostly_north_south and direction[0] < 0:
        return -direction
    return direction


def where_lines_meet(point, direction, other_point, other_direction):
    """Where the line through point along direction crosses the line through other_point
    along other_direction."""
    def cross(first, second):
        return first[0] * second[1] - first[1] * second[0]
    how_parallel = cross(direction, other_direction)
    if abs(how_parallel) < 1e-6:
        raise SystemExit("Tried to extend a wall to meet a line that runs alongside it, so they never meet. Check the wall ends in make_map_from_phone_scan.py.")
    distance_along = cross(other_point - point, other_direction) / how_parallel
    return point + direction * distance_along


# ---- The window -------------------------------------------------------------------------------

def measure_window(points, wall):
    """Looks at the wall face-on and finds the biggest hole through it that does not touch the
    wall's edges."""
    covered = _face_on_coverage(points, wall)
    columns, rows = covered.shape
    patches, patch_count = ndimage.label(~covered)
    biggest_hole = None
    for patch_number in range(1, patch_count + 1):
        squares = np.argwhere(patches == patch_number)
        touches_an_edge = (
            squares[:, 0].min() == 0 or squares[:, 1].min() == 0
            or squares[:, 0].max() == columns - 1 or squares[:, 1].max() == rows - 1
        )
        if touches_an_edge:
            continue
        if biggest_hole is None or len(squares) > len(biggest_hole):
            biggest_hole = squares
    if biggest_hole is None:
        raise SystemExit(f"Found no window in '{wall.name}'. Check its search box against a picture of the scan.")

    opening = Opening(
        name="window",
        start_along_wall_metres=float(biggest_hole[:, 0].min() * FACE_ON_GRID_SQUARE_METRES),
        end_along_wall_metres=float((biggest_hole[:, 0].max() + 1) * FACE_ON_GRID_SQUARE_METRES),
        bottom_height_metres=float(biggest_hole[:, 1].min() * FACE_ON_GRID_SQUARE_METRES),
        top_height_metres=float((biggest_hole[:, 1].max() + 1) * FACE_ON_GRID_SQUARE_METRES),
        source=(
            f"{SOURCE_START}: the biggest hole in the scanned wall, to the nearest "
            f"{FACE_ON_GRID_SQUARE_METRES * 100:.0f} cm. The organizers' drawing puts it 0.32 to 1.20 m "
            "high instead. Check with a photo."
        ),
    )
    print(f"Window: {opening.start_along_wall_metres:.2f} to {opening.end_along_wall_metres:.2f} m along the wall, "
          f"{opening.bottom_height_metres:.2f} to {opening.top_height_metres:.2f} m high.")
    return opening


def _face_on_coverage(points, wall):
    """A grid over the wall's face (distance along the wall by height). A square is True where
    the scan has points on the wall."""
    along = wall.direction()
    across = np.array([-along[1], along[0]])
    distance_along = (points[:, :2] - wall.start_metres) @ along
    distance_across = (points[:, :2] - wall.start_metres) @ across
    on_wall = (
        (np.abs(distance_across) < POINTS_ON_WALL_WITHIN_METRES)
        & (distance_along > 0) & (distance_along < wall.length_metres())
        & (points[:, 2] > 0) & (points[:, 2] < wall.top_height_metres)
    )
    columns = int(wall.length_metres() / FACE_ON_GRID_SQUARE_METRES)
    rows = int(wall.top_height_metres / FACE_ON_GRID_SQUARE_METRES)
    column = np.clip((distance_along[on_wall] / FACE_ON_GRID_SQUARE_METRES).astype(int), 0, columns - 1)
    row = np.clip((points[on_wall, 2] / FACE_ON_GRID_SQUARE_METRES).astype(int), 0, rows - 1)
    counts = np.zeros((columns, rows))
    np.add.at(counts, (column, row), 1)
    return counts >= POINTS_TO_COVER_A_SQUARE


# ---- The tunnel -------------------------------------------------------------------------------

def measure_tunnel(points, drawn_tube):
    """Measures the tunnel's ends and height from its roof, and its width from the inside faces
    of its sides. It assumes the tunnel runs in the same direction as in the drawing, and that
    the drawing's centre line runs somewhere inside the scanned tunnel."""
    along = (drawn_tube.exit_centre_metres - drawn_tube.entrance_centre_metres) / drawn_tube.length_metres()
    to_the_left = np.array([-along[1], along[0]])
    distance_along = (points[:, :2] - drawn_tube.entrance_centre_metres) @ along
    distance_left = (points[:, :2] - drawn_tube.entrance_centre_metres) @ to_the_left

    roof = (
        (points[:, 2] > TUNNEL_ROOF_LOWEST_METRES)
        & (np.abs(distance_left) < TUNNEL_ROOF_SEARCH_HALF_WIDTH_METRES)
        & (distance_along > -TUNNEL_SEARCH_BEYOND_DRAWN_ENDS_METRES)
        & (distance_along < drawn_tube.length_metres() + TUNNEL_SEARCH_BEYOND_DRAWN_ENDS_METRES)
    )
    entrance_along_metres = np.percentile(distance_along[roof], STRAY_POINTS_PERCENT)
    exit_along_metres = np.percentile(distance_along[roof], 100 - STRAY_POINTS_PERCENT)
    outside_height_metres = float(np.percentile(points[roof, 2], WALL_TOP_PERCENTILE))

    sides = (
        (points[:, 2] > TUNNEL_SIDES_LOWEST_METRES) & (points[:, 2] < TUNNEL_SIDES_HIGHEST_METRES)
        & (distance_along > entrance_along_metres + TUNNEL_SIDES_INSIDE_ENDS_METRES)
        & (distance_along < exit_along_metres - TUNNEL_SIDES_INSIDE_ENDS_METRES)
        & (np.abs(distance_left) < drawn_tube.outside_width_metres)
    )
    left_inside_face = np.percentile(distance_left[sides & (distance_left > 0)], TUNNEL_INSIDE_FACE_PERCENT)
    right_inside_face = np.percentile(distance_left[sides & (distance_left < 0)], 100 - TUNNEL_INSIDE_FACE_PERCENT)
    inside_width_metres = float(left_inside_face - right_inside_face)
    if not NARROWEST_BELIEVABLE_TUNNEL_METRES < inside_width_metres < WIDEST_BELIEVABLE_TUNNEL_METRES:
        raise SystemExit(f"The tunnel measured {inside_width_metres:.2f} m wide inside, which can't be right. Look at a picture of the scan around the tunnel.")

    centre_line_start = drawn_tube.entrance_centre_metres + to_the_left * (left_inside_face + right_inside_face) / 2.0
    tube = Tube(
        name=drawn_tube.name,
        entrance_centre_metres=centre_line_start + along * entrance_along_metres,
        exit_centre_metres=centre_line_start + along * exit_along_metres,
        outside_width_metres=inside_width_metres + 2.0 * drawn_tube.wall_thickness_metres,
        outside_height_metres=outside_height_metres,
        wall_thickness_metres=drawn_tube.wall_thickness_metres,
        source=(
            f"{SOURCE_START}: the ends and height from the scanned roof, and the width from the inside "
            "faces of the scanned sides at 0.6 to 1.0 m high. The direction and the wall thickness are "
            "copied from the organizers' drawing."
        ),
    )
    print(f"Tunnel: {inside_width_metres:.2f} m wide inside, {outside_height_metres:.2f} m tall, "
          f"{tube.length_metres():.2f} m long, centre line at x = {tube.entrance_centre_metres[0]:.2f} m.")
    return tube


# ---- Low objects ------------------------------------------------------------------------------

def find_low_objects(points):
    """Finds things standing on the floor with nothing above them at wall height, such as
    boulders. Seen from above, they are patches of squares that have points low down but are
    away from anything at wall height."""
    low = _squares_with_points(points, LOW_OBJECT_LOWEST_METRES, LOW_OBJECT_HIGHEST_METRES)
    near_walls = ndimage.binary_dilation(_squares_with_points(points, WALL_HEIGHT_LOWEST_METRES, WALL_HEIGHT_HIGHEST_METRES), iterations=3)
    # Closing fills small gaps, so one object with a few missing points stays one patch.
    patches, patch_count = ndimage.label(ndimage.binary_closing(low & ~near_walls, iterations=2))

    low_objects = []
    for patch_number in range(1, patch_count + 1):
        squares = np.argwhere(patches == patch_number)
        if len(squares) < SMALLEST_LOW_OBJECT_SQUARES:
            continue
        lowest_corner = squares.min(axis=0) * LOW_OBJECT_GRID_SQUARE_METRES - LOW_OBJECT_SEARCH_HALF_SIZE_METRES
        highest_corner = (squares.max(axis=0) + 1) * LOW_OBJECT_GRID_SQUARE_METRES - LOW_OBJECT_SEARCH_HALF_SIZE_METRES
        box = (lowest_corner[0], highest_corner[0], lowest_corner[1], highest_corner[1])
        object_points = points_in_box(points, box, LOW_OBJECT_LOWEST_METRES, LOW_OBJECT_HIGHEST_METRES)
        top_metres = float(np.percentile(object_points[:, 2], WALL_TOP_PERCENTILE))
        if top_metres < SHORTEST_LOW_OBJECT_METRES:
            continue
        low_objects.append(LowObstacle(
            name=f"Low object {len(low_objects) + 1}",
            centre_metres=(lowest_corner + highest_corner) / 2.0,
            radius_metres=float(max(highest_corner - lowest_corner) / 2.0),
            top_height_metres=top_metres,
            source=f"{SOURCE_START}. Something standing on the floor; the scan can't tell what. It may be a boulder.",
        ))
    for low_object in low_objects:
        print(f"{low_object.name}: at ({low_object.centre_metres[0]:.2f}, {low_object.centre_metres[1]:.2f}), "
              f"{low_object.top_height_metres:.2f} m tall.")
    return low_objects


def _squares_with_points(points, lowest_metres, highest_metres):
    """A grid over the course, seen from above. A square is True where the scan has a point
    between the two heights."""
    squares = int(2 * LOW_OBJECT_SEARCH_HALF_SIZE_METRES / LOW_OBJECT_GRID_SQUARE_METRES)
    half = LOW_OBJECT_SEARCH_HALF_SIZE_METRES
    chosen = points_in_box(points, (-half, half, -half, half), lowest_metres, highest_metres)
    square = np.clip(((chosen[:, :2] + half) / LOW_OBJECT_GRID_SQUARE_METRES).astype(int), 0, squares - 1)
    grid = np.zeros((squares, squares), dtype=bool)
    grid[square[:, 0], square[:, 1]] = True
    return grid
