"""Works out where the rulebook's checkpoint lines are, from the walls in a course map.

This runs on a laptop while a map file is being made. Nothing here runs on the drone, which
reads the finished lines from the map file.

THESE RULES ARE OUR DRAFT
-------------------------
The organizers' website describes the checkpoints in words, not coordinates:
  1. "Reach the end of the first corridor, where the line turns east."
  2. "Leave the maze where the line ends, through Gate A."
  3. "Come out of the south end of the dark tunnel."
  4. "Enter the landing area through Gate B at the north end or through the window in the
     lower wall."
  5. Land inside the pad's outer ring (that one needs no line).
"The whole drone crosses" each line. The lines below are our reading of those words, placed
using the walls of whichever map is being made. Check them against the organizers' drawing of
the checkpoints before relying on them.

Directions: "north" means towards larger y, and "east" towards larger x, on the axes of the
organizers' 3D model.
"""
import numpy as np

from team_nav.course_map import CheckpointLine

# Our own choice. How far beyond a line we put the point that marks its far side.
FAR_SIDE_DISTANCE_METRES = 0.5

# Our own choice: the names both map makers give these shapes.
FIRST_MAZE_DIVIDER = "M1 maze divider"
THIRD_MAZE_DIVIDER = "M3 maze divider"
TUNNEL = "Tunnel"
APPROACH_WALL = "A1 approach wall"
# The chicane walls that close the landing area, north to south. A map may lack one of them.
CHICANE_WALLS_NORTH_TO_SOUTH = ("C2b chicane wall", "C2a chicane wall (window)")
WINDOW = "window"

SOURCE = (
    "Our draft reading of the rulebook's words, placed using this map's walls by "
    "course_map_tools/checkpoint_line_rules.py. The rulebook gives no coordinates."
)


def make_checkpoint_lines(course_map):
    """Returns the checkpoint lines, in the order the drone must cross them. Gate B and the
    window are two ways to clear the same checkpoint."""
    return [
        end_of_first_corridor(course_map),
        gate_a(course_map),
        tunnel_exit(course_map),
        through_the_window(course_map),
        gate_b(course_map),
    ]


def end_of_first_corridor(course_map):
    """The first corridor runs north from the launch mark, between the west net and the first
    maze divider. It ends level with that divider's north end."""
    divider = course_map.wall_named(FIRST_MAZE_DIVIDER)
    north_end = _north_end(divider)
    start = np.array([course_map.lowest_corner_metres[0], north_end[1]])
    end = north_end
    far_side = (start + end) / 2.0 + np.array([0.0, FAR_SIDE_DISTANCE_METRES])
    return CheckpointLine("end of first corridor", start, end, far_side, SOURCE)


def gate_a(course_map):
    """The maze ends at Gate A: the line along the east face of the third maze divider, from
    its south end to the north net."""
    divider = course_map.wall_named(THIRD_MAZE_DIVIDER)
    east_face_x = max(divider.start_metres[0], divider.end_metres[0]) + divider.thickness_metres / 2.0
    south_end_y = min(divider.start_metres[1], divider.end_metres[1])
    start = np.array([east_face_x, south_end_y])
    end = np.array([east_face_x, course_map.highest_corner_metres[1]])
    far_side = (start + end) / 2.0 + np.array([FAR_SIDE_DISTANCE_METRES, 0.0])
    return CheckpointLine("Gate A", start, end, far_side, SOURCE)


def tunnel_exit(course_map):
    """The tunnel's exit: the line across the tube's far end, as wide as the tube."""
    tube = _tube_named(course_map, TUNNEL)
    along = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
    to_the_left = np.array([-along[1], along[0]])
    half_width = tube.outside_width_metres / 2.0
    start = tube.exit_centre_metres - to_the_left * half_width
    end = tube.exit_centre_metres + to_the_left * half_width
    far_side = tube.exit_centre_metres + along * FAR_SIDE_DISTANCE_METRES
    return CheckpointLine("tunnel exit", start, end, far_side, SOURCE)


def through_the_window(course_map):
    """The window's line: across the window, along the middle of its wall. The drone crosses
    it heading into the landing area, where the landing pad is."""
    wall, opening = course_map.opening_named(WINDOW)
    start = wall.point_along_wall_metres(opening.start_along_wall_metres)
    end = wall.point_along_wall_metres(opening.end_along_wall_metres)
    far_side = _point_beyond_line_towards(start, end, course_map.landing_pad_centre_metres)
    return CheckpointLine("through the window", start, end, far_side, SOURCE, opening_name=WINDOW)


def gate_b(course_map):
    """Gate B is the gap at the north end of the chicane walls: from the approach wall's south
    end to the north end of the northernmost chicane wall. The drone crosses it heading into
    the landing area, where the landing pad is."""
    approach_wall = course_map.wall_named(APPROACH_WALL)
    start = _south_end(approach_wall)
    end = _north_end(_northernmost_chicane_wall(course_map))
    far_side = _point_beyond_line_towards(start, end, course_map.landing_pad_centre_metres)
    return CheckpointLine("Gate B", start, end, far_side, SOURCE)


def _northernmost_chicane_wall(course_map):
    wall_names = [wall.name for wall in course_map.walls]
    for name in CHICANE_WALLS_NORTH_TO_SOUTH:
        if name in wall_names:
            return course_map.wall_named(name)
    raise ValueError(f"The map has none of the chicane walls {CHICANE_WALLS_NORTH_TO_SOUTH}, so Gate B can't be placed.")


def _point_beyond_line_towards(start, end, target):
    """A point FAR_SIDE_DISTANCE_METRES from the middle of the line, on the target's side."""
    middle = (start + end) / 2.0
    direction = (end - start) / np.linalg.norm(end - start)
    sideways = np.array([-direction[1], direction[0]])
    if np.dot(target - middle, sideways) < 0:
        sideways = -sideways
    return middle + sideways * FAR_SIDE_DISTANCE_METRES


def _north_end(wall):
    if wall.start_metres[1] > wall.end_metres[1]:
        return wall.start_metres.copy()
    return wall.end_metres.copy()


def _south_end(wall):
    if wall.start_metres[1] < wall.end_metres[1]:
        return wall.start_metres.copy()
    return wall.end_metres.copy()


def _tube_named(course_map, name):
    for tube in course_map.tubes:
        if tube.name == name:
            return tube
    raise ValueError(f"The map has no tube called '{name}', so the tunnel exit line can't be placed.")
