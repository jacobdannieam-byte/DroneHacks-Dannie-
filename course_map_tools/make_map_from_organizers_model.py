"""Turns the organizers' 3D model of the course into one of our course map files.

This runs on a laptop, inside the simulator's Linux box, which has the libraries it needs.
Nothing here runs on the drone.

How to run it, from the top folder of the repository:

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 course_map_tools/make_map_from_organizers_model.py

What it does
------------
The 3D model is a list of named parts, each made of triangles. For each part, this script
decides what kind of shape it is from its name, and measures that shape:

  - A wall: the straight line through its middle seen from above, its thickness, its bottom and
    top heights, and any holes through it, such as the window.
  - The tube: which way it runs, and its width, height and length.
  - A boulder: its centre, its radius and its height.
  - The floor, the launch mark and the landing pad.

Parts painted on the floor, such as the guideline and the pad's inner rings, are left out,
because nothing can fly into them.
"""
import os

import numpy as np
from scipy import ndimage

from team_nav.course_map import CourseMap, LowObstacle, Opening, Tube, Wall, save_course_map
from checkpoint_line_rules import make_checkpoint_lines

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_FILE = os.path.join(REPOSITORY, "simulator", "course_model", "dronehacks-course-v3.obj")
MAP_FILE = os.path.join(REPOSITORY, "drone_code", "team_nav", "course_maps", "organizers_drawing.json")

# The organizers' part names that tell us what kind of shape a part is.
FLOOR_PART = "Floor"
LAUNCH_MARK_PART = "Launch mark"
LANDING_PAD_PART = "Landing pad 5 ft"
APPROACH_RING_PART = "Approach ring"
BOULDER_PARTS_START_WITH = "Boulder"
TUBE_PARTS_START_WITH = "Tube oval"
PARTS_TO_LEAVE_OUT_START_WITH = ("Net", "Guideline", "Landing pad ring", "Landing pad cross")

# Our own choice. The tunnel's name, which checkpoint_line_rules.py also uses.
TUNNEL = "Tunnel"

# Our own choices for measuring. Points are spread over each triangle this far apart, and a hole
# must be at least this many grid squares across to count as an opening.
POINT_SPACING_METRES = 0.01
GRID_SQUARE_METRES = 0.02
SMALLEST_OPENING_SQUARES = 5

SOURCE_START = "From the organizers' 3D model (dronehacks-course-v3.obj)"


def read_parts(path_to_model_file):
    """Returns {part name: array of triangles, each 3 corners of (x, y, z)}."""
    corners = []
    corner_numbers_by_part = {}
    part_name = None
    with open(path_to_model_file, encoding="utf-8") as model_file:
        for line in model_file:
            words = line.split()
            if not words:
                continue
            if words[0] == "v":
                corners.append([float(word) for word in words[1:4]])
            elif words[0] == "g":
                part_name = line[2:].strip()
            elif words[0] == "f" and part_name is not None:
                # The file counts corners from 1, Python from 0. "12//3" means corner 12.
                face = [int(word.split("/")[0]) - 1 for word in words[1:]]
                for index in range(1, len(face) - 1):
                    corner_numbers_by_part.setdefault(part_name, []).append((face[0], face[index], face[index + 1]))
    corners = np.array(corners)
    return {name: corners[np.array(numbers)] for name, numbers in corner_numbers_by_part.items()}


def points_covering(triangles):
    """Spreads points evenly over every triangle, POINT_SPACING_METRES apart."""
    all_points = []
    for corner_a, corner_b, corner_c in triangles:
        longest_side = max(np.linalg.norm(corner_b - corner_a), np.linalg.norm(corner_c - corner_a), np.linalg.norm(corner_c - corner_b))
        steps = max(1, int(np.ceil(longest_side / POINT_SPACING_METRES)))
        for step_b in range(steps + 1):
            for step_c in range(steps + 1 - step_b):
                fraction_b, fraction_c = step_b / steps, step_c / steps
                all_points.append(corner_a + fraction_b * (corner_b - corner_a) + fraction_c * (corner_c - corner_a))
    return np.array(all_points)


def measure_wall(name, triangles):
    corner_points = triangles.reshape(-1, 3)
    along, across, middle = _main_directions(corner_points[:, :2])
    along_values = (corner_points[:, :2] - middle) @ along
    across_values = (corner_points[:, :2] - middle) @ across
    centre_line_offset = (across_values.max() + across_values.min()) / 2.0
    start = middle + along * along_values.min() + across * centre_line_offset
    end = middle + along * along_values.max() + across * centre_line_offset
    wall = Wall(
        name=name,
        start_metres=start,
        end_metres=end,
        thickness_metres=float(across_values.max() - across_values.min()),
        bottom_height_metres=float(corner_points[:, 2].min()),
        top_height_metres=float(corner_points[:, 2].max()),
        openings=[],
        source=f"{SOURCE_START}, part '{name}'.",
    )
    wall.openings = find_openings(wall, triangles)
    return wall


def find_openings(wall, triangles):
    """Looks at the wall face-on, marks which squares of a grid are covered by the wall, and
    returns each uncovered patch that is surrounded by wall on all sides."""
    points = points_covering(triangles)
    distance_along = (points[:, :2] - wall.start_metres) @ wall.direction()
    height = points[:, 2] - wall.bottom_height_metres
    columns = int(np.ceil(wall.length_metres() / GRID_SQUARE_METRES))
    rows = int(np.ceil((wall.top_height_metres - wall.bottom_height_metres) / GRID_SQUARE_METRES))
    covered = np.zeros((columns, rows), dtype=bool)
    column = np.clip((distance_along / GRID_SQUARE_METRES).astype(int), 0, columns - 1)
    row = np.clip((height / GRID_SQUARE_METRES).astype(int), 0, rows - 1)
    covered[column, row] = True

    patches, patch_count = ndimage.label(~covered)
    openings = []
    for patch_number in range(1, patch_count + 1):
        squares = np.argwhere(patches == patch_number)
        touches_the_edge = squares[:, 0].min() == 0 or squares[:, 1].min() == 0 or squares[:, 0].max() == columns - 1 or squares[:, 1].max() == rows - 1
        if touches_the_edge or len(squares) < SMALLEST_OPENING_SQUARES ** 2:
            continue
        name = "window" if "window" in wall.name else f"{wall.name} opening {len(openings) + 1}"
        openings.append(Opening(
            name=name,
            start_along_wall_metres=float(squares[:, 0].min() * GRID_SQUARE_METRES),
            end_along_wall_metres=float((squares[:, 0].max() + 1) * GRID_SQUARE_METRES),
            bottom_height_metres=float(wall.bottom_height_metres + squares[:, 1].min() * GRID_SQUARE_METRES),
            top_height_metres=float(wall.bottom_height_metres + (squares[:, 1].max() + 1) * GRID_SQUARE_METRES),
            source=f"{SOURCE_START}, the hole in part '{wall.name}', measured to the nearest {GRID_SQUARE_METRES * 100:.0f} cm.",
        ))
    return openings


def measure_tube(part_name, triangles):
    """A tube is open at both ends. Cut through it at half its height, and its two side walls
    show up as two parallel lines. The tube runs along those lines."""
    points = points_covering(triangles)
    bottom, top = points[:, 2].min(), points[:, 2].max()
    middle_height = (bottom + top) / 2.0
    slice_points = points[np.abs(points[:, 2] - middle_height) < 0.05, :2]
    centre = (slice_points.max(axis=0) + slice_points.min(axis=0)) / 2.0

    best_score, best_along = -1.0, None
    for angle_degrees in range(0, 180):
        along = np.array([np.cos(np.radians(angle_degrees)), np.sin(np.radians(angle_degrees))])
        across_values = np.abs((slice_points - centre) @ np.array([-along[1], along[0]]))
        score = np.mean(across_values > across_values.max() - 0.05)
        if score > best_score:
            best_score, best_along = score, along
    along = best_along
    across = np.array([-along[1], along[0]])
    along_values = (points[:, :2] - centre) @ along
    across_at_middle = np.abs((slice_points - centre) @ across)
    outside_half_width = across_at_middle.max()
    inside_half_width = across_at_middle[across_at_middle > outside_half_width / 2.0].min()

    # From the rulebook: the drone reaches the tunnel from the maze, which is north of the long
    # barrier, so the tunnel's entrance is its north end.
    one_end = centre + along * along_values.max()
    other_end = centre + along * along_values.min()
    entrance, exit = (one_end, other_end) if one_end[1] > other_end[1] else (other_end, one_end)
    return Tube(
        name=TUNNEL,
        entrance_centre_metres=entrance,
        exit_centre_metres=exit,
        outside_width_metres=float(2.0 * outside_half_width),
        outside_height_metres=float(top - bottom),
        wall_thickness_metres=float(max(outside_half_width - inside_half_width, GRID_SQUARE_METRES)),
        source=f"{SOURCE_START}, part '{part_name}'. Its bottom is {bottom * 100:.0f} cm above the floor; the map treats it as standing on the floor.",
    )


def measure_boulder(name, triangles):
    corner_points = triangles.reshape(-1, 3)
    lowest, highest = corner_points.min(axis=0), corner_points.max(axis=0)
    return LowObstacle(
        name=name,
        centre_metres=(lowest[:2] + highest[:2]) / 2.0,
        radius_metres=float(max(highest[:2] - lowest[:2]) / 2.0),
        top_height_metres=float(highest[2]),
        source=f"{SOURCE_START}, part '{name}'.",
    )


def centre_and_half_width(triangles):
    corner_points = triangles.reshape(-1, 3)
    lowest, highest = corner_points.min(axis=0), corner_points.max(axis=0)
    return (lowest[:2] + highest[:2]) / 2.0, float(max(highest[:2] - lowest[:2]) / 2.0)


def _main_directions(points):
    """Returns the direction a set of points is longest in, the direction across it, and the
    middle of the points. The longest direction is chosen to point towards larger x, or larger
    y if it runs exactly north to south, so that walls always start at the same end."""
    middle = (points.max(axis=0) + points.min(axis=0)) / 2.0
    _, _, directions = np.linalg.svd(points - points.mean(axis=0))
    along = directions[0]
    if along[0] < -1e-6 or (abs(along[0]) <= 1e-6 and along[1] < 0):
        along = -along
    return along, np.array([-along[1], along[0]]), middle


def main():
    parts = read_parts(MODEL_FILE)
    floor_name = next(name for name in parts if name.startswith(FLOOR_PART))
    floor_corners = parts[floor_name].reshape(-1, 3)
    launch_centre, _ = centre_and_half_width(parts[LAUNCH_MARK_PART])
    pad_centre, pad_radius = centre_and_half_width(parts[LANDING_PAD_PART])
    ring_name = next(name for name in parts if name.startswith(APPROACH_RING_PART))
    _, ring_radius = centre_and_half_width(parts[ring_name])

    walls, tubes, low_obstacles = [], [], []
    for name, triangles in parts.items():
        if name.startswith(PARTS_TO_LEAVE_OUT_START_WITH) or name.startswith(FLOOR_PART):
            continue
        if name in (LAUNCH_MARK_PART, LANDING_PAD_PART, ring_name):
            continue
        if name.startswith(BOULDER_PARTS_START_WITH):
            low_obstacles.append(measure_boulder(name, triangles))
        elif name.startswith(TUBE_PARTS_START_WITH):
            tubes.append(measure_tube(name, triangles))
        else:
            walls.append(measure_wall(name, triangles))

    wall_tops = [round(wall.top_height_metres, 3) for wall in walls]
    course_map = CourseMap(
        name="Organizers' drawing",
        description=(
            "The course as the organizers drew it in their preliminary 3D model, dronehacks-course-v3.obj. "
            "Made by course_map_tools/make_map_from_organizers_model.py. Our phone scan of the real course "
            "shows several walls in different places, so treat this as the plan, not the built course."
        ),
        lowest_corner_metres=floor_corners[:, :2].min(axis=0),
        highest_corner_metres=floor_corners[:, :2].max(axis=0),
        wall_top_height_metres=max(set(wall_tops), key=wall_tops.count),
        edges_source=f"{SOURCE_START}: the floor part's edges, and the most common wall top height.",
        launch_mark_centre_metres=launch_centre,
        launch_mark_source=f"{SOURCE_START}, part '{LAUNCH_MARK_PART}'.",
        landing_pad_centre_metres=pad_centre,
        landing_pad_outer_radius_metres=pad_radius,
        approach_ring_radius_metres=ring_radius,
        landing_pad_source=f"{SOURCE_START}, parts '{LANDING_PAD_PART}' and '{ring_name}'.",
        walls=walls,
        tubes=tubes,
        low_obstacles=low_obstacles,
    )
    course_map.checkpoint_lines = make_checkpoint_lines(course_map)
    save_course_map(course_map, MAP_FILE)
    print(f"Saved {MAP_FILE}: {len(walls)} walls, {len(tubes)} tube, {len(low_obstacles)} low obstacles.")


if __name__ == "__main__":
    main()
