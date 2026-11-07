"""Reads our phone scan of the real course and puts its points on the course's axes.

This runs on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.
make_map_from_phone_scan.py uses it, then measures walls and other shapes from the points.

What the scan is
----------------
A phone app (Scaniverse) recorded about 600,000 coloured points on the surfaces of the real
course on 2026-11-06. The points are in the phone's own axes: the phone does not know where
the course's corner is, or which way is east.

What this file does
-------------------
  1. Reads the scan file.
  2. Lines the scan up with the organizers' drawing. It tries many small turns, and for each
     turn every shift at once, and keeps the one that puts the most scanned points onto the
     drawing's walls and net.
  3. Measures the floor under every part of the scan, because the scan's floor is not quite
     flat (it drifts by up to about 25 cm across the room). Every height is then measured
     from the floor directly below.

What it does not do: it does not decide what the points are. That is make_map_from_phone_scan.py.
"""
import numpy as np
from scipy import ndimage, signal

from team_nav.course_map import distance_to_shapes_metres, shapes_blocking_between_heights

# ---- Lining up ---------------------------------------------------------------------------------
# Our own choice, found by eye from a picture of the scan: the scan is turned roughly half a turn
# from the drawing, and this scan point sits roughly at the course's centre.
SCAN_POINT_NEAR_COURSE_CENTRE_METRES = np.array([-1.0, -0.1])
TURNS_TO_TRY_DEGREES = np.arange(170.0, 190.01, 0.25)
# Our own choice. Only points at wall height are used for lining up, so the floor and boulders
# don't count.
LINING_UP_LOWEST_METRES = 0.7
LINING_UP_HIGHEST_METRES = 1.6
# Our own choice. How close a scanned point must be to a drawn wall to count as being on it.
LINING_UP_CLOSENESS_METRES = 0.04
LINING_UP_GRID_SQUARE_METRES = 0.01
# Our own choice. Half the size of the area searched, a little more than half the course's width.
LINING_UP_HALF_SIZE_METRES = 3.6

# ---- Floor ------------------------------------------------------------------------------------
# Our own choice. The floor's height is measured separately in square patches this big.
FLOOR_PATCH_METRES = 0.25
# Our own choice. Only points this low (before levelling) can be floor.
FLOOR_POINTS_ARE_BELOW_METRES = 0.3
# Our own choice. A patch needs this many low points before its floor height is trusted.
FLOOR_POINTS_NEEDED_IN_A_PATCH = 30


def read_scan_on_course_axes(path_to_scan_file, drawing):
    """Returns the scan's points as an N x 3 array on the course's axes: x east, y north, and
    height above the floor directly below, all in metres."""
    points = read_scan(path_to_scan_file)
    print(f"Read {len(points)} points from the scan.")
    points = line_up_with_drawing(points, drawing)
    return measure_heights_from_floor(points)


def read_scan(path_to_scan_file):
    """Reads a binary PLY file, which is a list of points, each with a position and a colour.
    Returns the positions as an N x 3 array. The colours are not used."""
    with open(path_to_scan_file, "rb") as scan_file:
        data = scan_file.read()
    header_end = data.index(b"end_header\n") + len(b"end_header\n")
    header = data[:header_end].decode("ascii").splitlines()
    if "format binary_little_endian 1.0" not in header:
        raise SystemExit("This reader only understands binary little-endian PLY files, which is what Scaniverse writes.")

    numpy_types = {"float": "<f4", "double": "<f8", "uchar": "u1"}
    fields = []
    for line in header:
        words = line.split()
        if words[:1] == ["property"]:
            fields.append((words[2], numpy_types[words[1]]))
    rows = np.frombuffer(data[header_end:], dtype=np.dtype(fields))
    return np.stack([rows["x"], rows["y"], rows["z"]], axis=1).astype(float)


def line_up_with_drawing(points, drawing):
    """Turns and moves the scan onto the course's axes. Returns the moved points."""
    wall_height = (points[:, 2] > LINING_UP_LOWEST_METRES) & (points[:, 2] < LINING_UP_HIGHEST_METRES)
    wall_height_points = points[wall_height, :2] - SCAN_POINT_NEAR_COURSE_CENTRE_METRES
    closeness_score = drawing_closeness_score(drawing)
    squares = closeness_score.shape[0]

    best_score = -1.0
    for turn_degrees in TURNS_TO_TRY_DEGREES:
        turned = wall_height_points @ _turn_matrix(turn_degrees).T
        counts = _count_points_per_square(turned, squares)
        # Sliding the counts over the score, every shift at once, gives the total score of each
        # shift. This is called cross-correlation, and the FFT makes it fast.
        total_scores = signal.fftconvolve(closeness_score, counts[::-1, ::-1], mode="same")
        best_square = np.unravel_index(np.argmax(total_scores), total_scores.shape)
        if total_scores[best_square] > best_score:
            best_score = total_scores[best_square]
            best_turn_degrees = turn_degrees
            best_shift_metres = (np.array(best_square) - squares // 2) * LINING_UP_GRID_SQUARE_METRES

    print(f"Lined up: turned {best_turn_degrees:.2f} degrees, then moved "
          f"{best_shift_metres[0]:+.2f} m in x and {best_shift_metres[1]:+.2f} m in y.")
    moved = points.copy()
    moved[:, :2] = (points[:, :2] - SCAN_POINT_NEAR_COURSE_CENTRE_METRES) @ _turn_matrix(best_turn_degrees).T + best_shift_metres
    return moved


def drawing_closeness_score(drawing):
    """A grid over the course: 1 on the drawing's walls and net, fading to 0 a few cm away."""
    squares = int(2 * LINING_UP_HALF_SIZE_METRES / LINING_UP_GRID_SQUARE_METRES)
    square_numbers = np.indices((squares, squares)).reshape(2, -1).T
    square_centres = -LINING_UP_HALF_SIZE_METRES + (square_numbers + 0.5) * LINING_UP_GRID_SQUARE_METRES
    shapes = shapes_blocking_between_heights(drawing, LINING_UP_LOWEST_METRES, LINING_UP_HIGHEST_METRES)
    distance_metres, _ = distance_to_shapes_metres(square_centres, shapes)
    score = np.exp(-0.5 * (distance_metres / LINING_UP_CLOSENESS_METRES) ** 2)
    return score.reshape(squares, squares)


def _count_points_per_square(points, squares):
    counts = np.zeros((squares, squares))
    square = np.floor((points + LINING_UP_HALF_SIZE_METRES) / LINING_UP_GRID_SQUARE_METRES).astype(int)
    inside = np.all((square >= 0) & (square < squares), axis=1)
    np.add.at(counts, (square[inside, 0], square[inside, 1]), 1)
    return counts


def _turn_matrix(turn_degrees):
    turn = np.radians(turn_degrees)
    return np.array([[np.cos(turn), -np.sin(turn)], [np.sin(turn), np.cos(turn)]])


def measure_heights_from_floor(points):
    """Replaces each point's height with its height above the scanned floor below it."""
    lowest_corner = points[:, :2].min(axis=0)
    patch = np.floor((points[:, :2] - lowest_corner) / FLOOR_PATCH_METRES).astype(int)
    patches_east, patches_north = patch.max(axis=0) + 1
    floor_height = np.full((patches_east, patches_north), np.nan)
    could_be_floor = points[:, 2] < FLOOR_POINTS_ARE_BELOW_METRES
    for patch_east in range(patches_east):
        for patch_north in range(patches_north):
            in_patch = could_be_floor & (patch[:, 0] == patch_east) & (patch[:, 1] == patch_north)
            if np.count_nonzero(in_patch) > FLOOR_POINTS_NEEDED_IN_A_PATCH:
                # The lowest tenth of the points, so that dust and small things don't lift the floor.
                floor_height[patch_east, patch_north] = np.percentile(points[in_patch, 2], 10)

    # Patches with no floor points take the height of the nearest patch that has some. Then
    # each patch is smoothed with its neighbours, so one odd patch can't make a step.
    nearest_known = ndimage.distance_transform_edt(np.isnan(floor_height), return_distances=False, return_indices=True)
    floor_height = ndimage.median_filter(floor_height[tuple(nearest_known)], size=3)
    print(f"The scanned floor's height varies from {floor_height.min():+.2f} m to {floor_height.max():+.2f} m. "
          "Heights are now measured from it.")
    levelled = points.copy()
    levelled[:, 2] -= floor_height[patch[:, 0], patch[:, 1]]
    return levelled
