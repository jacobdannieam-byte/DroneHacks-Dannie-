"""Makes real_course_2026-11-08.json from the corrected model, corrected_dronehacks.obj.

This runs on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.

    docker run --rm --volume "$PWD:/repository" --workdir /repository/course_map_tools \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 make_map_from_corrected_model.py

The model
---------
corrected_dronehacks.obj (copied from origin/hardcoded, commit 2b56750) is in millimetres, z up,
in the same east/north frame as the organizers' dronehacks-course-v3.obj. Unlike that file it has
no part names: the whole course is one welded mesh. So this script measures each part from the
mesh corners that lie inside a region of the floor plan that we chose by looking at horizontal
slices of the model (PARTS below). Each region holds one part and nothing else above the floor.

How each kind of part is measured
---------------------------------
  - A wall: the corners in its region, seen from above, give its direction (the long axis of the
    corners), its two ends and its thickness. Their lowest and highest heights give its bottom and
    top. A wall's opening (the window) is found by slicing the wall's corners by height.
  - The tunnel: the outside extents of its shell give its width, height, entrance and exit.
  - A boulder: the centre of its corners, their largest distance from that centre, and their top.
  - The launch mark and the pad: the painted rings on the floor (corners 1 to 3 mm above it).

Changed on purpose: the start diagonal wall is 0.30 m shorter at its south-west end than in the
model (START_WALL_SHORTENED_BY_METRES says why).

Left out on purpose: the net, the floor and its markings, the maze dividers (the corrected model
has none) and the tube cradles. The cradles hold the tunnel up, but the map stands the tunnel on
the floor, so they would become 0.28 m walls inside it. In Gazebo run 20261108_131203 the bottom
laser saw them as sudden steps and PX4 stopped using it for height.

Checkpoint lines: "end of first corridor" and "Gate A" are copied from organizers_drawing.json,
because the rules that place them (checkpoint_line_rules.py) need maze dividers this model lacks.
The other three are placed by checkpoint_line_rules.py from this map's walls.

We have not measured the built course against this map.
"""
import os

import numpy as np

from checkpoint_line_rules import gate_b, through_the_window, tunnel_exit
from team_nav.course_map import CourseMap, LowObstacle, Opening, Tube, Wall, load_course_map, save_course_map

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MODEL_FILE = os.path.join(REPOSITORY, "simulator", "course_model", "corrected_dronehacks.obj")
MAPS_FOLDER = os.path.join(REPOSITORY, "drone_code", "team_nav", "course_maps")
DRAWING_MAP_FILE = os.path.join(MAPS_FOLDER, "organizers_drawing.json")
MAP_FILE = os.path.join(MAPS_FOLDER, "real_course_2026-11-08.json")

# From the model file's first line: "units: mm".
METRES_PER_MODEL_UNIT = 0.001
# Measured in the model: the floor's top is at 6.3 mm and the painted rings at 7.6 to 8.9 mm.
FLOOR_TOP_METRES = 0.0063
HIGHEST_PAINT_METRES = 0.012
# Measured in the model: the net is at +/-3.048 m. Corners beyond this belong to the net or floor.
INSIDE_THE_NET_METRES = 3.04

# Our own choices, read from slices of the model: (name, west, east, south, north) in metres.
# Each region holds exactly one part above the floor.
WALL_REGIONS = (
    ("North-west wall", -2.80, -0.90, 2.80, 3.04),
    ("Start diagonal wall", -2.45, -0.85, 0.89, 1.80),
    ("Barrier", -3.045, 1.60, 0.40, 0.51),
    ("A1 approach wall", -1.10, -0.85, -0.70, 0.40),
    ("North-east diagonal wall", 0.05, 1.57, 0.515, 1.25),
    ("C1 chicane wall", 0.90, 1.572, -1.20, 0.39),
    ("C2a chicane wall (window)", -0.80, 0.00, -3.04, -1.15),
)
TUNNEL_REGION = ("Tunnel", 1.56, 3.04, -0.32, 1.23)
# Our own choice: only the tunnel's shell above the cradles' 0.30 m tops counts as the tunnel.
TUNNEL_SHELL_LOWEST_METRES = 0.30
# From the organizers' drawing: the boulders' centres, used only to find each one's corners.
BOULDER_CENTRES_METRES = {
    "Boulder 1": (2.0117, -2.4689), "Boulder 2": (2.5298, -1.5545), "Boulder 3": (0.7925, -2.347),
    "Boulder 4": (-2.5603, -2.4994), "Boulder 5": (-1.3411, -2.3774),
}
BOULDER_SEARCH_RADIUS_METRES = 0.35
# Our own choices for finding the window: 2 cm height slices, 2 cm steps along the wall, and a gap
# must be at least 0.30 m long to count as the window.
SLICE_METRES = 0.02
SMALLEST_WINDOW_METRES = 0.30

# The team's decision of 2026-11-08: the model's start diagonal wall ends 0.13 m east of the launch
# mark, so a north-facing drone on the mark would overlap it by 0.15 m, which cannot be how the
# course is built. We take the model's wall end as wrong and move it 0.30 m along the wall, the
# least that gives the propellers 0.10 m from it on the mark and on the first legs of the route
# (to (-2.70, 0.92), (-2.70, 1.60) and the first checkpoint). Check the real wall's end on the course.
START_WALL_NAME = "Start diagonal wall"
START_WALL_SHORTENED_BY_METRES = 0.30

SOURCE = "Measured from the corrected model (corrected_dronehacks.obj) by course_map_tools/make_map_from_corrected_model.py"


def read_corners_and_triangles(path):
    corners, triangles = [], []
    with open(path, encoding="utf-8") as model_file:
        for line in model_file:
            words = line.split()
            if not words:
                continue
            if words[0] == "v":
                corners.append([float(word) for word in words[1:4]])
            elif words[0] == "f":
                face = [int(word.split("/")[0]) - 1 for word in words[1:]]
                for index in range(1, len(face) - 1):
                    triangles.append((face[0], face[index], face[index + 1]))
    return np.array(corners) * METRES_PER_MODEL_UNIT, np.array(triangles)


def corners_in(corners, west, east, south, north, lowest=HIGHEST_PAINT_METRES):
    inside = ((corners[:, 0] >= west) & (corners[:, 0] <= east) & (corners[:, 1] >= south)
              & (corners[:, 1] <= north) & (corners[:, 2] > lowest))
    return corners[inside]


def wall_corners_in(corners, triangles, west, east, south, north):
    """Every corner of the triangles whose middle is in the region and above the paint. A wall's
    sides are tall triangles, so their middles are well above the floor, and their bottom corners,
    at floor level, come with them."""
    triangle_corners = corners[triangles]
    middles = triangle_corners.mean(axis=1)
    inside = ((middles[:, 0] >= west) & (middles[:, 0] <= east) & (middles[:, 1] >= south)
              & (middles[:, 1] <= north) & (middles[:, 2] > HIGHEST_PAINT_METRES))
    return triangle_corners[inside].reshape(-1, 3)


def measure_wall(name, wall_corners):
    """A Wall from its corners: the long axis seen from above, its ends, thickness, bottom and top."""
    middle = wall_corners[:, :2].mean(axis=0)
    _, _, directions = np.linalg.svd(wall_corners[:, :2] - middle)
    along = directions[0]
    across = np.array([-along[1], along[0]])
    along_metres = (wall_corners[:, :2] - middle) @ along
    across_metres = (wall_corners[:, :2] - middle) @ across
    line_middle = middle + across * (across_metres.max() + across_metres.min()) / 2.0
    wall = Wall(
        name=name,
        start_metres=line_middle + along * along_metres.min(),
        end_metres=line_middle + along * along_metres.max(),
        thickness_metres=float(across_metres.max() - across_metres.min()),
        bottom_height_metres=max(0.0, float(wall_corners[:, 2].min()) - FLOOR_TOP_METRES),
        top_height_metres=float(wall_corners[:, 2].max()) - FLOOR_TOP_METRES,
        openings=[],
        source=f"{SOURCE}, region {name}.",
    )
    if wall.start_metres[1] > wall.end_metres[1]:
        wall.start_metres, wall.end_metres = wall.end_metres, wall.start_metres
    return wall


def extend_to_the_tunnel(wall, tunnel):
    """The north-east diagonal wall runs into the tunnel's side. Its region stops short of the
    tunnel so as not to take tunnel corners, so its east end is moved along its line to the tunnel."""
    east_end, west_end = (wall.end_metres, wall.start_metres) if wall.end_metres[0] > wall.start_metres[0] else (
        wall.start_metres, wall.end_metres)
    direction = (east_end - west_end) / np.linalg.norm(east_end - west_end)
    tunnel_west_face = tunnel.exit_centre_metres[0] - tunnel.outside_width_metres / 2.0
    new_east_end = west_end + direction * (tunnel_west_face - west_end[0]) / direction[0]
    if wall.end_metres[0] > wall.start_metres[0]:
        wall.end_metres = new_east_end
    else:
        wall.start_metres = new_east_end
    wall.source += " East end moved along the wall's line to the tunnel's west side, which it meets."


def shorten_the_start_wall(wall):
    """Moves the start diagonal wall's south-west end START_WALL_SHORTENED_BY_METRES along the wall."""
    wall.start_metres = wall.start_metres + wall.direction() * START_WALL_SHORTENED_BY_METRES
    wall.source += (f" South-west end moved {START_WALL_SHORTENED_BY_METRES:.2f} m north-east along the wall: the team's "
                    "decision of 2026-11-08, because the model's end would overlap a drone on the launch mark.")


def find_window(wall, corners, triangles):
    """The window: the longest stretch along the wall that is empty over a range of heights,
    found by slicing the wall's triangles by height. None if there is none."""
    along = wall.direction()
    across = np.array([-along[1], along[0]])
    triangle_corners = corners[triangles]
    centres = triangle_corners.mean(axis=1)
    near = np.abs((centres[:, :2] - wall.start_metres) @ across) <= wall.thickness_metres
    near &= centres[:, 2] > HIGHEST_PAINT_METRES
    wall_triangles = triangle_corners[near]
    length = wall.length_metres()
    steps = np.arange(0.0, length, SLICE_METRES)
    heights = np.arange(wall.bottom_height_metres, wall.top_height_metres, SLICE_METRES) + FLOOR_TOP_METRES
    solid = np.zeros((len(heights), len(steps)), dtype=bool)
    for row, height in enumerate(heights):
        for triangle in wall_triangles:
            low, high = triangle[:, 2].min(), triangle[:, 2].max()
            if low > height or high < height:
                continue
            spread = (triangle[:, :2] - wall.start_metres) @ along
            solid[row, (steps >= spread.min() - SLICE_METRES) & (steps <= spread.max() + SLICE_METRES)] = True
    best_row, best_gap = None, None
    for row in range(len(heights)):
        for gap_start, gap_end in _empty_stretches(solid[row], steps):
            interior = 0.0 < gap_start and gap_end < length
            if interior and (best_gap is None or gap_end - gap_start > best_gap[1] - best_gap[0]):
                best_row, best_gap = row, (gap_start, gap_end)
    if best_gap is None or best_gap[1] - best_gap[0] < SMALLEST_WINDOW_METRES:
        return None
    # Grow up and down from that slice while the middle of the gap stays empty. The window's ends
    # are where every one of those slices is empty.
    middle_column = int(np.argmin(np.abs(steps - (best_gap[0] + best_gap[1]) / 2.0)))
    lowest_row = highest_row = best_row
    while lowest_row > 0 and not solid[lowest_row - 1, middle_column]:
        lowest_row -= 1
    while highest_row < len(heights) - 1 and not solid[highest_row + 1, middle_column]:
        highest_row += 1
    window_start, window_end = best_gap
    for row in range(lowest_row, highest_row + 1):
        for gap_start, gap_end in _empty_stretches(solid[row], steps):
            if gap_start <= steps[middle_column] <= gap_end:
                window_start, window_end = max(window_start, gap_start), min(window_end, gap_end)
    return Opening(name="window", start_along_wall_metres=float(window_start), end_along_wall_metres=float(window_end),
                   bottom_height_metres=float(heights[lowest_row] - FLOOR_TOP_METRES),
                   top_height_metres=float(heights[highest_row] - FLOOR_TOP_METRES),
                   source=f"{SOURCE}: the empty stretch of the wall, sliced every 2 cm.")


def _empty_stretches(solid_row, steps):
    stretches, start = [], None
    for is_solid, step in zip(solid_row, steps):
        if not is_solid and start is None:
            start = step
        if is_solid and start is not None:
            stretches.append((start, step))
            start = None
    return stretches


def measure_tunnel(corners):
    name, west, east, south, north = TUNNEL_REGION
    shell = corners_in(corners, west, east, south, north, lowest=TUNNEL_SHELL_LOWEST_METRES)
    centre_east = (shell[:, 0].min() + shell[:, 0].max()) / 2.0
    return Tube(
        name=name,
        entrance_centre_metres=np.array([centre_east, shell[:, 1].max()]),
        exit_centre_metres=np.array([centre_east, shell[:, 1].min()]),
        outside_width_metres=float(shell[:, 0].max() - shell[:, 0].min()),
        outside_height_metres=float(shell[:, 2].max() - FLOOR_TOP_METRES),
        wall_thickness_metres=0.0341,
        source=f"{SOURCE}: the shell's outside extents above the cradles. The map stands it on the floor. "
               "Wall thickness copied from organizers_drawing.json.",
    )


def measure_boulder(name, corners):
    centre_guess = np.array(BOULDER_CENTRES_METRES[name])
    near = corners[(np.linalg.norm(corners[:, :2] - centre_guess, axis=1) < BOULDER_SEARCH_RADIUS_METRES)
                   & (corners[:, 2] > HIGHEST_PAINT_METRES)]
    centre = (near[:, :2].min(axis=0) + near[:, :2].max(axis=0)) / 2.0
    return LowObstacle(name=name, centre_metres=centre,
                       radius_metres=float(np.linalg.norm(near[:, :2] - centre, axis=1).max()),
                       top_height_metres=float(near[:, 2].max() - FLOOR_TOP_METRES), source=f"{SOURCE}.")


def painted_ring(corners, guess, smallest_radius, largest_radius):
    """(centre, outer radius) of the painted ring around guess with corners in this radius range."""
    paint = corners[(corners[:, 2] > FLOOR_TOP_METRES + 0.0005) & (corners[:, 2] < HIGHEST_PAINT_METRES)]
    distance = np.linalg.norm(paint[:, :2] - guess, axis=1)
    ring = paint[(distance >= smallest_radius) & (distance <= largest_radius)]
    ring = ring[(np.abs(ring[:, 0]) < INSIDE_THE_NET_METRES) & (np.abs(ring[:, 1]) < INSIDE_THE_NET_METRES)]
    centre = (ring[:, :2].min(axis=0) + ring[:, :2].max(axis=0)) / 2.0
    return centre, float(np.linalg.norm(ring[:, :2] - centre, axis=1).max())


def main():
    corners, triangles = read_corners_and_triangles(MODEL_FILE)
    drawing = load_course_map(DRAWING_MAP_FILE)
    walls = []
    for name, west, east, south, north in WALL_REGIONS:
        walls.append(measure_wall(name, wall_corners_in(corners, triangles, west, east, south, north)))
    window_wall = walls[-1]
    window = find_window(window_wall, corners, triangles)
    if window is not None:
        window_wall.openings.append(window)
    tunnel = measure_tunnel(corners)
    extend_to_the_tunnel(walls[4], tunnel)
    shorten_the_start_wall(walls[1])
    boulders = [measure_boulder(name, corners) for name in BOULDER_CENTRES_METRES]
    # The launch mark is a small painted disc; the pad's outer ring is about 0.76 m across.
    launch_centre, _ = painted_ring(corners, drawing.launch_mark_centre_metres, 0.0, 0.10)
    pad_centre, pad_radius = painted_ring(corners, drawing.landing_pad_centre_metres, 0.70, 0.80)
    course_map = CourseMap(
        name="Real course 2026-11-08",
        description=("The corrected model of the course (corrected_dronehacks.obj), without the tube cradles. Made by "
                     "course_map_tools/make_map_from_corrected_model.py. Not yet measured against the built course."),
        lowest_corner_metres=drawing.lowest_corner_metres, highest_corner_metres=drawing.highest_corner_metres,
        wall_top_height_metres=drawing.wall_top_height_metres,
        edges_source="Copied from organizers_drawing.json; the corrected model's net is in the same place.",
        launch_mark_centre_metres=launch_centre, launch_mark_source=f"{SOURCE}: the painted launch mark.",
        landing_pad_centre_metres=pad_centre, landing_pad_outer_radius_metres=pad_radius,
        approach_ring_radius_metres=drawing.approach_ring_radius_metres,
        landing_pad_source=f"{SOURCE}: the painted outer ring. Approach ring copied from organizers_drawing.json.",
        walls=walls, tubes=[tunnel], low_obstacles=boulders,
    )
    course_map.checkpoint_lines = [
        drawing.checkpoint_line_named("end of first corridor"),
        drawing.checkpoint_line_named("Gate A"),
        tunnel_exit(course_map),
        through_the_window(course_map),
        gate_b(course_map),
    ]
    save_course_map(course_map, MAP_FILE)
    print(f"Saved {MAP_FILE}.")
    for wall in walls:
        print(f"  {wall.name}: {np.round(wall.start_metres, 3).tolist()} to {np.round(wall.end_metres, 3).tolist()}, "
              f"{wall.thickness_metres:.3f} m thick, {wall.bottom_height_metres:.3f} to {wall.top_height_metres:.3f} m, "
              f"openings {[(round(o.start_along_wall_metres, 2), round(o.end_along_wall_metres, 2), round(o.bottom_height_metres, 2), round(o.top_height_metres, 2)) for o in wall.openings]}")
    print(f"  Tunnel: entrance {np.round(tunnel.entrance_centre_metres, 3).tolist()}, exit "
          f"{np.round(tunnel.exit_centre_metres, 3).tolist()}, {tunnel.outside_width_metres:.3f} wide, "
          f"{tunnel.outside_height_metres:.3f} high")
    for boulder in boulders:
        print(f"  {boulder.name}: {np.round(boulder.centre_metres, 3).tolist()} r {boulder.radius_metres:.3f} "
              f"top {boulder.top_height_metres:.3f}")
    print(f"  Launch {np.round(launch_centre, 4).tolist()}, pad {np.round(pad_centre, 4).tolist()} r {pad_radius:.3f}")
    for line in course_map.checkpoint_lines:
        print(f"  {line.name}: {np.round(line.start_metres, 3).tolist()} to {np.round(line.end_metres, 3).tolist()}")


if __name__ == "__main__":
    main()
