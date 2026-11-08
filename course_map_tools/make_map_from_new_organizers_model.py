"""Makes a course map for the organizers' changed course (2026-11-08), whose new 3D model is one
unnamed mesh in millimetres, so make_map_from_organizers_model.py (which reads named parts in
metres) cannot read it.

This runs on a laptop, inside the simulator's Linux box, which has the libraries it needs.
Nothing here runs on the drone. Copy the model into simulator/generated_files/ first (Git ignores
that folder), then, from the top folder of the repository:

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 course_map_tools/make_map_from_new_organizers_model.py \\
        simulator/generated_files/new_dronehacks_course.obj

What it does
------------
The organizers changed only the start area: the three maze walls are gone, and three new walls
stand there. So:
  1. It slices the model at flight heights (0.8 m and 1.5 m): every triangle that crosses that
     height gives a short line, and the lines together outline every wall seen from above.
  2. For each new wall, it starts from the team's rough line (NEW_WALLS_ROUGHLY), takes the
     slice points near it, and measures the wall from them: the line through its middle, its
     length and thickness, and its top height (the highest model corner on it).
  3. Everything else is copied from organizers_drawing.json (made from the old model), after
     checking it against the new model: each kept wall must show up in the slices along its
     whole length, each boulder near the floor, the launch mark and the pad at the same place,
     and the old maze walls must really be gone. Slice points that no map shape explains are
     reported.
  4. It writes the map, and warns if any wall's edge is closer than WARNING_ROOM_METRES to the
     launch mark's centre.

The checkpoint lines are copied unchanged. Two of them were placed from the old maze walls
(checkpoint_line_rules.py): "end of first corridor" and "Gate A". The painted guideline in the
new model is unchanged, so they are kept, but the team should check them against the organizers'
description of the new start.
"""
import copy
import os
import sys

import numpy as np
from scipy import ndimage

from team_nav.course_map import Wall, distance_to_line_piece_metres, load_course_map, save_course_map
from team_nav.drone_dimensions import REQUIRED_ROOM_METRES

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OLD_MAP_FILE = os.path.join(REPOSITORY, "drone_code", "team_nav", "course_maps", "organizers_drawing.json")
NEW_MAP_FILE = os.path.join(REPOSITORY, "drone_code", "team_nav", "course_maps", "organizers_new_start_2026-11-08.json")

# From the model's first line ("units: mm").
METRES_PER_MODEL_UNIT = 0.001
# Our own choices: heights at which the model is sliced. 0.8 m is the cruise height; 1.5 m is
# above the window's top, so the window wall shows solid there too; 0.15 m finds the boulders.
FLIGHT_SLICE_HEIGHTS_METRES = (0.8, 1.5)
LOW_SLICE_HEIGHT_METRES = 0.15
# Our own choice: the tunnel's cradles are saddles only 0.28 m tall, which a 0.15 m slice cuts
# only at their ends, so walls lower than 0.5 m are checked at this height.
FLOOR_SLICE_HEIGHT_METRES = 0.05
LOW_WALL_METRES = 0.5
# Our own choice: slice lines are turned into points this far apart.
SLICE_POINT_SPACING_METRES = 0.005

# From the team (2026-11-08), read off the new model by eye: roughly where each new wall is.
NEW_WALLS_ROUGHLY = {
    "S1 start diagonal wall": ((-2.38, 0.94), (-0.95, 1.69)),
    "S2 start east wall": ((0.10, 0.55), (1.62, 1.15)),
    "S3 north net wall": ((-2.65, 2.96), (-1.03, 2.96)),
}
OLD_WALLS_REMOVED = ("M1 maze divider", "M2 maze divider", "M3 maze divider")
# Our own choices for measuring a new wall: slice points within this distance of the rough line
# (extended by the same at each end) belong to it, unless they belong to a kept wall.
NEW_WALL_SEARCH_METRES = 0.25
# Thickness is measured between these percentiles across the wall, to ignore stray points.
THICKNESS_PERCENTILES = (1.0, 99.0)

# Our own choices for the checks. A kept wall "shows up" where a slice point lies within its half
# thickness plus this tolerance; it must show up along at least this share of its length.
MATCH_TOLERANCE_METRES = 0.03
SHARE_OF_LENGTH_REQUIRED = 0.9
CHECK_SPACING_METRES = 0.05
# The net lies along the course edge; slice points this close to it are the net.
NET_BAND_METRES = 0.03
# Floor markings are thin plates: corners between these heights above the floor.
MARKING_HEIGHTS_METRES = (0.005, 0.012)
PLATE_HEIGHT_TOLERANCE_METRES = 0.0005
# A floor marking may differ from the old map by this much before we call it moved.
MARKING_TOLERANCE_METRES = 0.02
# Our own choice, from the team's request: the drone needs 0.35 m from its centre to anything
# (drone_dimensions.py REQUIRED_ROOM_METRES: 0.30 m to the propeller tips, plus 0.05 m).
WARNING_ROOM_METRES = REQUIRED_ROOM_METRES
# Our own choice: unexplained slice points closer together than this form one piece.
UNEXPLAINED_GRID_METRES = 0.05
SMALLEST_UNEXPLAINED_POINTS = 20


def main():
    if len(sys.argv) != 2:
        sys.exit("Give the new model's .obj file, for example simulator/generated_files/new_dronehacks_course.obj.")
    triangles = read_mesh_in_metres(sys.argv[1])
    old_map = load_course_map(OLD_MAP_FILE)
    kept_walls = [wall for wall in old_map.walls if wall.name not in OLD_WALLS_REMOVED]
    flight_points = inside_the_net(np.vstack([slice_points(triangles, h) for h in FLIGHT_SLICE_HEIGHTS_METRES]), old_map)
    low_points = inside_the_net(slice_points(triangles, LOW_SLICE_HEIGHT_METRES), old_map)
    floor_points = inside_the_net(slice_points(triangles, FLOOR_SLICE_HEIGHT_METRES), old_map)
    corners = triangles.reshape(-1, 3)

    new_walls = [measure_new_wall(name, rough, flight_points, old_map, kept_walls, corners)
                 for name, rough in NEW_WALLS_ROUGHLY.items()]
    print("New walls measured from the model:")
    for wall in new_walls:
        print(f"  {wall.name}: ({wall.start_metres[0]:+.3f}, {wall.start_metres[1]:+.3f}) to ({wall.end_metres[0]:+.3f}, "
              f"{wall.end_metres[1]:+.3f}), {wall.length_metres():.2f} m long, {wall.thickness_metres:.3f} m thick, "
              f"from {wall.bottom_height_metres:.2f} m to {wall.top_height_metres:.2f} m above the floor")
    new_map = make_new_map(old_map, kept_walls, new_walls)
    problems = check_kept_parts(old_map, new_map, kept_walls, flight_points, low_points, floor_points, corners)
    problems += unexplained_pieces(flight_points, new_map, "at flight height")
    save_course_map(new_map, NEW_MAP_FILE)
    print(f"\nSaved {os.path.relpath(NEW_MAP_FILE, REPOSITORY)}.")
    print("\nChecks against the new model: " + ("all passed." if not problems else f"{len(problems)} to look at:"))
    for problem in problems:
        print(f"  - {problem}")
    warn_about_the_launch_mark(new_map)


# ---- Reading and slicing ------------------------------------------------------------------------

def read_mesh_in_metres(path):
    """Every triangle of the model, as an N x 3 x 3 array of (east, north, up) corners in metres."""
    corners, faces = [], []
    with open(path, encoding="utf-8") as model_file:
        for line in model_file:
            words = line.split()
            if not words:
                continue
            if words[0] == "v":
                corners.append([float(word) for word in words[1:4]])
            elif words[0] == "f":
                # The file counts corners from 1, Python from 0. "12//3" means corner 12.
                face = [int(word.split("/")[0]) - 1 for word in words[1:]]
                for index in range(1, len(face) - 1):
                    faces.append((face[0], face[index], face[index + 1]))
    return np.array(corners)[np.array(faces)] * METRES_PER_MODEL_UNIT


def slice_points(triangles, height_metres):
    """Points, seen from above, along where the model crosses this height."""
    heights = triangles[:, :, 2]
    crossing = triangles[(heights.min(axis=1) < height_metres) & (heights.max(axis=1) > height_metres)]
    points = []
    for triangle in crossing:
        ends = []
        for first, second in ((0, 1), (1, 2), (2, 0)):
            first_height, second_height = triangle[first, 2], triangle[second, 2]
            if (first_height - height_metres) * (second_height - height_metres) < 0:
                share = (height_metres - first_height) / (second_height - first_height)
                ends.append(triangle[first, :2] + share * (triangle[second, :2] - triangle[first, :2]))
        if len(ends) != 2:
            continue
        steps = max(2, int(np.linalg.norm(ends[1] - ends[0]) / SLICE_POINT_SPACING_METRES))
        for share in np.linspace(0.0, 1.0, steps):
            points.append(ends[0] + share * (ends[1] - ends[0]))
    return np.array(points)


def inside_the_net(points, course_map):
    """Leaves out the net itself and anything outside it."""
    low, high = course_map.lowest_corner_metres + NET_BAND_METRES, course_map.highest_corner_metres - NET_BAND_METRES
    return points[np.all((points > low) & (points < high), axis=1)]


# ---- The new walls ------------------------------------------------------------------------------

def measure_new_wall(name, rough_ends, flight_points, old_map, kept_walls, corners):
    start, end = np.array(rough_ends[0], float), np.array(rough_ends[1], float)
    direction = (end - start) / np.linalg.norm(end - start)
    extended_start, extended_end = start - direction * NEW_WALL_SEARCH_METRES, end + direction * NEW_WALL_SEARCH_METRES
    near = distance_to_line_piece_metres(flight_points, extended_start, extended_end) <= NEW_WALL_SEARCH_METRES
    near &= ~near_any_wall(flight_points, kept_walls) & ~near_a_tunnel(flight_points, old_map)
    points = flight_points[near]
    if len(points) < 50:
        raise SystemExit(f"Found almost nothing near the rough line of {name}. Check NEW_WALLS_ROUGHLY.")
    middle = points.mean(axis=0)
    _, _, axes = np.linalg.svd(points - middle, full_matrices=False)
    along_direction = axes[0] if axes[0] @ direction > 0 else -axes[0]
    along = (points - middle) @ along_direction
    across = (points - middle) @ np.array([-along_direction[1], along_direction[0]])
    low_across, high_across = np.percentile(across, THICKNESS_PERCENTILES)
    centre_offset = (low_across + high_across) / 2.0
    sideways = np.array([-along_direction[1], along_direction[0]])
    wall_start = middle + along_direction * along.min() + sideways * centre_offset
    wall_end = middle + along_direction * along.max() + sideways * centre_offset
    thickness = float(high_across - low_across)
    # The net is left out: S3 stands just inside it, and the net is taller than any wall.
    inside = corners[np.all((corners[:, :2] > old_map.lowest_corner_metres + NET_BAND_METRES)
                            & (corners[:, :2] < old_map.highest_corner_metres - NET_BAND_METRES), axis=1)]
    on_the_wall = distance_to_line_piece_metres(inside[:, :2], wall_start, wall_end) <= thickness / 2.0 + MATCH_TOLERANCE_METRES
    top_height = float(inside[on_the_wall, 2].max())
    # A wall may stand on short legs (S3's panel starts 0.10 m above the floor). Corners lower than
    # the floor markings' top are the floor itself.
    bottom_height = float(inside[on_the_wall, 2].min())
    if bottom_height < MARKING_HEIGHTS_METRES[1]:
        bottom_height = 0.0
    return Wall(name=name, start_metres=np.round(wall_start, 4), end_metres=np.round(wall_end, 4),
                thickness_metres=round(thickness, 4), bottom_height_metres=round(bottom_height, 4),
                top_height_metres=round(top_height, 4),
                openings=[],
                source="Measured from the organizers' changed 3D model of 2026-11-08 (one unnamed mesh, in mm) by "
                       "course_map_tools/make_map_from_new_organizers_model.py: slices at 0.8 m and 1.5 m, near the "
                       "team's rough line.")


def near_any_wall(points, walls):
    near = np.zeros(len(points), bool)
    for wall in walls:
        distance = distance_to_line_piece_metres(points, wall.start_metres, wall.end_metres)
        near |= distance <= wall.thickness_metres / 2.0 + MATCH_TOLERANCE_METRES
    return near


def near_a_tunnel(points, course_map):
    """True for points inside a tunnel's outline seen from above (plus the tolerance)."""
    near = np.zeros(len(points), bool)
    for tube in course_map.tubes:
        along = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
        relative = points - tube.entrance_centre_metres
        along_metres, sideways_metres = relative @ along, np.abs(relative[:, 0] * along[1] - relative[:, 1] * along[0])
        near |= ((along_metres > -MATCH_TOLERANCE_METRES) & (along_metres < tube.length_metres() + MATCH_TOLERANCE_METRES)
                 & (sideways_metres < tube.outside_width_metres / 2.0 + MATCH_TOLERANCE_METRES))
    return near


# ---- Checking what is kept ----------------------------------------------------------------------

def check_kept_parts(old_map, new_map, kept_walls, flight_points, low_points, floor_points, corners):
    problems = []
    for wall in kept_walls:
        points = flight_points if wall.top_height_metres > LOW_WALL_METRES else floor_points
        share = share_of_wall_found(wall, points)
        if share < SHARE_OF_LENGTH_REQUIRED:
            problems.append(f"{wall.name} shows up along only {share:.0%} of its length in the new model.")
    # Where an old maze wall's line crosses a wall that is there, the slices show that wall instead.
    not_explained = flight_points[~near_any_wall(flight_points, new_map.walls)]
    for name in OLD_WALLS_REMOVED:
        share = share_of_wall_found(old_map.wall_named(name), not_explained)
        if share > 1.0 - SHARE_OF_LENGTH_REQUIRED:
            problems.append(f"{name} was meant to be gone, but {share:.0%} of it is still in the new model.")
    for boulder in old_map.low_obstacles:
        distance = np.linalg.norm(low_points - boulder.centre_metres, axis=1)
        if not np.any(np.abs(distance - boulder.radius_metres) <= MATCH_TOLERANCE_METRES):
            problems.append(f"{boulder.name} is not where the old map has it.")
    problems += check_floor_markings(old_map, corners)
    return problems


def share_of_wall_found(wall, points):
    """The share of the wall's centre line that has a slice point within its half thickness."""
    steps = max(2, int(wall.length_metres() / CHECK_SPACING_METRES))
    found = 0
    for share in np.linspace(0.0, 1.0, steps):
        spot = wall.start_metres + share * (wall.end_metres - wall.start_metres)
        if np.any(np.linalg.norm(points - spot, axis=1) <= wall.thickness_metres / 2.0 + MATCH_TOLERANCE_METRES):
            found += 1
    return found / steps


def check_floor_markings(old_map, corners):
    """The launch mark and the pad: the middle of the thin plates around each must not have moved."""
    problems = []
    plates = corners[(corners[:, 2] > MARKING_HEIGHTS_METRES[0]) & (corners[:, 2] < MARKING_HEIGHTS_METRES[1])]
    for name, centre, reach in [("launch mark", old_map.launch_mark_centre_metres, 0.2),
                                ("landing pad", old_map.landing_pad_centre_metres, old_map.landing_pad_outer_radius_metres + 0.05)]:
        nearby = plates[np.linalg.norm(plates[:, :2] - centre, axis=1) <= reach]
        if len(nearby) == 0:
            problems.append(f"No {name} found near ({centre[0]:+.2f}, {centre[1]:+.2f}) in the new model.")
            continue
        # The marking is the topmost plate there; the painted guideline beside the launch mark is lower.
        nearby = nearby[nearby[:, 2] > nearby[:, 2].max() - PLATE_HEIGHT_TOLERANCE_METRES][:, :2]
        middle = (nearby.min(axis=0) + nearby.max(axis=0)) / 2.0
        print(f"The {name}'s middle in the new model: ({middle[0]:+.3f}, {middle[1]:+.3f}); the old map has "
              f"({centre[0]:+.3f}, {centre[1]:+.3f}).")
        if np.linalg.norm(middle - centre) > MARKING_TOLERANCE_METRES:
            problems.append(f"The {name} seems to have moved to ({middle[0]:+.2f}, {middle[1]:+.2f}).")
    return problems


def unexplained_pieces(points, course_map, where):
    """Pieces of the slices that no wall or tunnel in the map explains."""
    explained = near_any_wall(points, course_map.walls) | near_a_tunnel(points, course_map)
    left_over = points[~explained]
    if len(left_over) == 0:
        return []
    cells = np.floor((left_over - course_map.lowest_corner_metres) / UNEXPLAINED_GRID_METRES).astype(int)
    grid = np.zeros(cells.max(axis=0) + 1, bool)
    grid[cells[:, 0], cells[:, 1]] = True
    labels, count = ndimage.label(grid, structure=np.ones((3, 3)))
    problems = []
    for number in range(1, count + 1):
        piece = left_over[labels[cells[:, 0], cells[:, 1]] == number]
        if len(piece) >= SMALLEST_UNEXPLAINED_POINTS:
            low, high = piece.min(axis=0), piece.max(axis=0)
            problems.append(f"Something {where} that the map does not have, from ({low[0]:+.2f}, {low[1]:+.2f}) to "
                            f"({high[0]:+.2f}, {high[1]:+.2f}).")
    return problems


# ---- The new map --------------------------------------------------------------------------------

def make_new_map(old_map, kept_walls, new_walls):
    new_map = copy.deepcopy(old_map)
    new_map.name = "Organizers' changed course, 2026-11-08"
    new_map.description = (
        "The organizers' changed course of 2026-11-08: the three maze walls are replaced by three new walls in the "
        "start area, measured from their new 3D model. Everything else is copied from the organizers' drawing "
        "(organizers_drawing.json) after checking it against the new model. The checkpoint lines 'end of first "
        "corridor' and 'Gate A' were placed from the old maze walls and are kept unchanged; check them.")
    new_map.walls = copy.deepcopy(kept_walls) + new_walls
    return new_map


def warn_about_the_launch_mark(course_map):
    launch = np.asarray(course_map.launch_mark_centre_metres, float)[np.newaxis, :]
    print("\nRoom around the launch mark's centre (the drone needs 0.35 m to anything):")
    for wall in course_map.walls:
        room = float(distance_to_line_piece_metres(launch, wall.start_metres, wall.end_metres)[0]) - wall.thickness_metres / 2.0
        if room < 1.0:
            flag = "   <-- WARNING: closer than 0.35 m" if room < WARNING_ROOM_METRES else ""
            print(f"  {wall.name}: {room:.2f} m{flag}")


if __name__ == "__main__":
    main()
