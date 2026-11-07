"""Makes a 3D model of the course from a course map file. SIMULATOR ONLY.

world.py calls this when the simulator flies in a course described by a map file, such as our
phone scan, instead of in the organizers' own 3D model. The result is a plain-text OBJ file, the
same format as the organizers' model, so course_model.py and the flight viewer read it the same
way.

What it makes
-------------
  - Each wall as one or more boxes. A wall with an opening, such as the window, is cut into the
    boxes around the opening.
  - The tunnel as an oval ring, stretched from its entrance to its exit.
  - Each low obstacle, such as a boulder, as an upright cylinder.
  - The floor, the launch mark, the landing pad and its approach ring, all flat on the floor.
  - The safety net, as four upright sheets.

The part names start with the same words as in the organizers' model ("Floor", "Net", "Landing",
"Launch", "Approach ring"), because course_model.py uses those words to tell floor markings and
the net apart from obstacles.
"""
import math

import numpy as np

# From the organizers' website: the net is 10 feet (3.048 m) tall.
NET_HEIGHT_METRES = 3.048
# Our own choice. Curved shapes are drawn with this many flat sides, which is smooth to about
# 1 cm on the tunnel.
SIDES_ON_A_CURVE = 48
# Our own choices, only for drawing. The map gives where the launch mark is, not how big it is.
LAUNCH_MARK_HALF_SIZE_METRES = 0.15
APPROACH_RING_LINE_WIDTH_METRES = 0.03
# Our own choice. Floor markings sit this far above the floor, so that the viewer draws them on
# top of the floor rather than flickering between the two.
MARKING_HEIGHT_METRES = 0.001


def write_course_mesh(course_map, path_to_model_file):
    """Writes the OBJ file for the map."""
    mesh = _MeshWriter()
    _add_floor_and_markings(mesh, course_map)
    _add_net(mesh, course_map)
    for wall in course_map.walls:
        _add_wall(mesh, wall)
    for tube in course_map.tubes:
        _add_tube(mesh, tube)
    for obstacle in course_map.low_obstacles:
        _add_cylinder(mesh, f"Low obstacle: {obstacle.name}", obstacle.centre_metres, obstacle.radius_metres, obstacle.top_height_metres)
    with open(path_to_model_file, "w", encoding="utf-8") as model_file:
        model_file.write(mesh.as_text(f"Made by course_mesh_from_map.py from the course map '{course_map.name}'."))


class _MeshWriter:
    """Collects corner points and faces, part by part, and writes them as OBJ text."""

    def __init__(self):
        self.lines = []
        self.corner_count = 0

    def start_part(self, name):
        self.lines.append(f"g {name}")

    def add_face(self, corners):
        """Adds one flat face with three or more corners, given in order around its edge."""
        for corner in corners:
            self.lines.append(f"v {corner[0]:.4f} {corner[1]:.4f} {corner[2]:.4f}")
        first_number = self.corner_count + 1    # OBJ files count corners from 1.
        self.corner_count += len(corners)
        self.lines.append("f " + " ".join(str(first_number + index) for index in range(len(corners))))

    def as_text(self, comment):
        return f"# {comment}\n" + "\n".join(self.lines) + "\n"


def _add_box(mesh, bottom_corners, height_bottom_metres, height_top_metres):
    """A box standing upright, given its four corners seen from above, in order around it."""
    bottom = [np.append(corner, height_bottom_metres) for corner in bottom_corners]
    top = [np.append(corner, height_top_metres) for corner in bottom_corners]
    mesh.add_face(bottom)
    mesh.add_face(top)
    for side in range(4):
        following = (side + 1) % 4
        mesh.add_face([bottom[side], bottom[following], top[following], top[side]])


def _add_wall(mesh, wall):
    mesh.start_part(f"Wall: {wall.name}")
    across = np.array([-wall.direction()[1], wall.direction()[0]]) * wall.thickness_metres / 2.0
    for along_start, along_end, bottom_metres, top_metres in _solid_blocks(wall):
        start = wall.point_along_wall_metres(along_start)
        end = wall.point_along_wall_metres(along_end)
        _add_box(mesh, [start - across, end - across, end + across, start + across], bottom_metres, top_metres)


def _solid_blocks(wall):
    """Cuts the wall into blocks around its openings. Returns (start along, end along, bottom,
    top) for each block."""
    cut_points = {0.0, wall.length_metres()}
    for opening in wall.openings:
        cut_points.update([opening.start_along_wall_metres, opening.end_along_wall_metres])
    cut_points = sorted(cut_points)

    blocks = []
    for along_start, along_end in zip(cut_points[:-1], cut_points[1:]):
        middle_along = (along_start + along_end) / 2.0
        solid_heights = [(wall.bottom_height_metres, wall.top_height_metres)]
        for opening in wall.openings:
            if opening.start_along_wall_metres < middle_along < opening.end_along_wall_metres:
                solid_heights = _remove_height_range(solid_heights, opening.bottom_height_metres, opening.top_height_metres)
        for bottom_metres, top_metres in solid_heights:
            blocks.append((along_start, along_end, bottom_metres, top_metres))
    return blocks


def _remove_height_range(height_ranges, lowest_removed, highest_removed):
    remaining = []
    for bottom_metres, top_metres in height_ranges:
        if bottom_metres < lowest_removed:
            remaining.append((bottom_metres, min(top_metres, lowest_removed)))
        if top_metres > highest_removed:
            remaining.append((max(bottom_metres, highest_removed), top_metres))
    return remaining


def _add_tube(mesh, tube):
    """An oval ring lying on the floor, stretched from the entrance to the exit."""
    mesh.start_part(f"Tube: {tube.name}")
    along = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
    to_the_left = np.append(np.array([-along[1], along[0]]), 0.0)
    middle_height_metres = tube.outside_height_metres / 2.0
    outside = (tube.outside_width_metres / 2.0, tube.outside_height_metres / 2.0)
    inside = (outside[0] - tube.wall_thickness_metres, outside[1] - tube.wall_thickness_metres)

    def oval_point(end_centre, half_sizes, angle):
        centre = np.append(end_centre, middle_height_metres)
        return centre + to_the_left * half_sizes[0] * math.cos(angle) + np.array([0.0, 0.0, half_sizes[1] * math.sin(angle)])

    for side in range(SIDES_ON_A_CURVE):
        angle = 2.0 * math.pi * side / SIDES_ON_A_CURVE
        next_angle = 2.0 * math.pi * (side + 1) / SIDES_ON_A_CURVE
        for half_sizes in (outside, inside):
            mesh.add_face([
                oval_point(tube.entrance_centre_metres, half_sizes, angle),
                oval_point(tube.exit_centre_metres, half_sizes, angle),
                oval_point(tube.exit_centre_metres, half_sizes, next_angle),
                oval_point(tube.entrance_centre_metres, half_sizes, next_angle),
            ])
        for end_centre in (tube.entrance_centre_metres, tube.exit_centre_metres):
            mesh.add_face([
                oval_point(end_centre, outside, angle),
                oval_point(end_centre, outside, next_angle),
                oval_point(end_centre, inside, next_angle),
                oval_point(end_centre, inside, angle),
            ])


def _add_cylinder(mesh, part_name, centre_metres, radius_metres, top_height_metres):
    mesh.start_part(part_name)
    rim = []
    for side in range(SIDES_ON_A_CURVE):
        angle = 2.0 * math.pi * side / SIDES_ON_A_CURVE
        rim.append(np.asarray(centre_metres) + radius_metres * np.array([math.cos(angle), math.sin(angle)]))
    mesh.add_face([np.append(point, top_height_metres) for point in rim])
    for side in range(SIDES_ON_A_CURVE):
        following = (side + 1) % SIDES_ON_A_CURVE
        mesh.add_face([
            np.append(rim[side], 0.0), np.append(rim[following], 0.0),
            np.append(rim[following], top_height_metres), np.append(rim[side], top_height_metres),
        ])


def _add_flat_disc(mesh, part_name, centre_metres, radius_metres):
    mesh.start_part(part_name)
    rim = []
    for side in range(SIDES_ON_A_CURVE):
        angle = 2.0 * math.pi * side / SIDES_ON_A_CURVE
        rim.append(np.append(np.asarray(centre_metres) + radius_metres * np.array([math.cos(angle), math.sin(angle)]), MARKING_HEIGHT_METRES))
    mesh.add_face(rim)


def _add_flat_ring(mesh, part_name, centre_metres, outer_radius_metres, line_width_metres):
    mesh.start_part(part_name)
    for side in range(SIDES_ON_A_CURVE):
        corners = []
        for angle_step, radius_metres in ((side, outer_radius_metres), (side + 1, outer_radius_metres),
                                          (side + 1, outer_radius_metres - line_width_metres), (side, outer_radius_metres - line_width_metres)):
            angle = 2.0 * math.pi * angle_step / SIDES_ON_A_CURVE
            point = np.asarray(centre_metres) + radius_metres * np.array([math.cos(angle), math.sin(angle)])
            corners.append(np.append(point, MARKING_HEIGHT_METRES))
        mesh.add_face(corners)


def _add_floor_and_markings(mesh, course_map):
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    mesh.start_part("Floor")
    mesh.add_face([np.array([low_x, low_y, 0.0]), np.array([high_x, low_y, 0.0]), np.array([high_x, high_y, 0.0]), np.array([low_x, high_y, 0.0])])

    mesh.start_part("Launch mark")
    centre = course_map.launch_mark_centre_metres
    half = LAUNCH_MARK_HALF_SIZE_METRES
    mesh.add_face([np.append(centre + offset, MARKING_HEIGHT_METRES) for offset in ([-half, -half], [half, -half], [half, half], [-half, half])])

    _add_flat_disc(mesh, "Landing pad", course_map.landing_pad_centre_metres, course_map.landing_pad_outer_radius_metres)
    _add_flat_ring(mesh, "Approach ring", course_map.landing_pad_centre_metres, course_map.approach_ring_radius_metres, APPROACH_RING_LINE_WIDTH_METRES)


def _add_net(mesh, course_map):
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    corners = [np.array(corner) for corner in ((low_x, low_y), (high_x, low_y), (high_x, high_y), (low_x, high_y))]
    for side, side_name in enumerate(("south", "east", "north", "west")):
        start, end = corners[side], corners[(side + 1) % 4]
        mesh.start_part(f"Net {side_name}")
        mesh.add_face([np.append(start, 0.0), np.append(end, 0.0), np.append(end, NET_HEIGHT_METRES), np.append(start, NET_HEIGHT_METRES)])
