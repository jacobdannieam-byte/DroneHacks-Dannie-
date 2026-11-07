"""Checks the Gazebo worlds made from every course map. TRIAL ONLY, SIMULATOR ONLY.

What it does
------------
For every map file in drone_code/team_nav/course_maps/, it:

  1. makes the world file with make_gazebo_world.py, with the drone in it;
  2. asks Gazebo's own checker ("gz sdf -k") whether Gazebo can read the file, if Gazebo is
     installed here;
  3. reads every box and cylinder back out of the world file, and tests points against the
     map:
       - each wall: a grid of points every 2 cm over its face, at its middle and just inside
         both faces, must be solid except inside an opening; points just outside its faces
         must be empty;
       - the tunnel: points in the middle of its curved wall must be solid, and points in its
         hollow middle must be empty;
       - each low obstacle: its middle must be solid, and points just beyond its side or top
         must be empty;
       - the net: a point just outside each side, and just above the roof, must be solid, and a
         point just inside each side must be empty.

It reads the shapes back out of the file, instead of trusting course_world_sdf.py's own
numbers, so that a shape turned the wrong way or put in the wrong place is caught.

It prints one line per map, and stops with an error if any map fails. Run it inside the
drone-simulator-gazebo box so that step 2 runs too:

    docker run --rm --volume "$PWD:/repository" drone-simulator-gazebo \\
        python3 /repository/simulator/gazebo_trial/check_worlds.py
"""
import glob
import math
import os
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

import numpy as np

FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(FOLDER_OF_THIS_FILE))
MAP_FILES_PATTERN = os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav", "course_maps", "*.json")

# Where Gazebo's checker finds the drone model and the PX4 model it is built on.
OUR_MODELS_FOLDER = os.path.join(FOLDER_OF_THIS_FILE, "models")
PX4_MODELS_FOLDER = "/opt/PX4-Autopilot/Tools/simulation/gz/models"

# See make_gazebo_world.py for why we add this folder to Python's search list.
sys.path.insert(0, os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav"))
from team_nav.course_map import load_course_map  # noqa: E402

from course_world_sdf import NET_HEIGHT_METRES, model_name_for  # noqa: E402
from make_gazebo_world import make_world_file  # noqa: E402

# Our own choice: test points every 2 cm over each wall's face.
GRID_SPACING_METRES = 0.02

# Our own choice. Points closer than this to an edge are where "solid" and "empty" meet, so
# we skip them, and points "just inside" or "just outside" a surface are this far from it.
EDGE_ALLOWANCE_METRES = 0.005

# Our own choice: how many places along the tunnel, and around it, to test.
TUNNEL_PLACES_ALONG = 9
TUNNEL_PLACES_AROUND = 360


# ---- Reading shapes back out of a world file -------------------------------------------------

class Box:
    def __init__(self, pose, size_metres):
        self.centre_metres = np.array(pose[:3])
        roll_radians, _, yaw_radians = pose[3:]
        # Gazebo applies roll about x first, then yaw about z (see course_world_sdf.py).
        self.rotation = _turn_about_z(yaw_radians) @ _turn_about_x(roll_radians)
        self.half_size_metres = np.array(size_metres) / 2.0

    def contains(self, point_metres):
        point_in_box_directions = self.rotation.T @ (np.asarray(point_metres) - self.centre_metres)
        return bool(np.all(np.abs(point_in_box_directions) <= self.half_size_metres))


class Cylinder:
    """An upright cylinder. The world file never tilts one."""

    def __init__(self, pose, radius_metres, length_metres):
        self.centre_metres = np.array(pose[:3])
        self.radius_metres = radius_metres
        self.half_length_metres = length_metres / 2.0

    def contains(self, point_metres):
        offset = np.asarray(point_metres) - self.centre_metres
        return bool(math.hypot(offset[0], offset[1]) <= self.radius_metres and abs(offset[2]) <= self.half_length_metres)


def solid_shapes_by_model(world_file_path):
    """Returns {model name: [Box or Cylinder, ...]} for every solid shape in the world file."""
    world = ElementTree.parse(world_file_path).getroot().find("world")
    shapes_by_model = {}
    for model in world.findall("model"):
        shapes = []
        for collision in model.iter("collision"):
            shape = _shape_from_collision(collision)
            if shape is not None:
                shapes.append(shape)
        shapes_by_model[model.get("name")] = shapes
    return shapes_by_model


def _shape_from_collision(collision):
    pose = [float(number) for number in collision.find("pose").text.split()]
    box_size = collision.find("geometry/box/size")
    if box_size is not None:
        return Box(pose, [float(number) for number in box_size.text.split()])
    cylinder = collision.find("geometry/cylinder")
    if cylinder is not None:
        return Cylinder(pose, float(cylinder.find("radius").text), float(cylinder.find("length").text))
    # The only other shape is the floor, which is an endless plane and needs no test.
    return None


def _turn_about_x(angle_radians):
    cosine, sine = math.cos(angle_radians), math.sin(angle_radians)
    return np.array([[1.0, 0.0, 0.0], [0.0, cosine, -sine], [0.0, sine, cosine]])


def _turn_about_z(angle_radians):
    cosine, sine = math.cos(angle_radians), math.sin(angle_radians)
    return np.array([[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]])


def is_solid(shapes, point_metres):
    for shape in shapes:
        if shape.contains(point_metres):
            return True
    return False


# ---- Tests, one kind of map shape at a time --------------------------------------------------
# Each returns a list of sentences, one per problem found. An empty list means it passed.

def problems_with_walls(course_map, shapes_by_model):
    problems = []
    for wall in course_map.walls:
        shapes = shapes_by_model[model_name_for(wall.name)]
        half_thickness_metres = wall.thickness_metres / 2.0
        # Sideways offsets from the wall's centre line: middle, just inside each face, and just
        # outside each face.
        inside_offsets_metres = [0.0, half_thickness_metres - EDGE_ALLOWANCE_METRES, -(half_thickness_metres - EDGE_ALLOWANCE_METRES)]
        outside_offsets_metres = [half_thickness_metres + EDGE_ALLOWANCE_METRES, -(half_thickness_metres + EDGE_ALLOWANCE_METRES)]
        sideways = np.array([-wall.direction()[1], wall.direction()[0]])
        wrong_point_count = 0
        for along_metres, height_metres in _grid_over_wall_face(wall):
            on_centre_line = wall.point_along_wall_metres(along_metres)
            should_be_solid = not _inside_an_opening(wall, along_metres, height_metres)
            for offset_metres in inside_offsets_metres + outside_offsets_metres:
                point = [*(on_centre_line + sideways * offset_metres), height_metres]
                expected = should_be_solid and offset_metres in inside_offsets_metres
                if is_solid(shapes, point) != expected:
                    wrong_point_count += 1
        if wrong_point_count:
            problems.append(f"wall '{wall.name}' has {wrong_point_count} test points that are solid where the map has none, or the other way round")
    return problems


def _grid_over_wall_face(wall):
    """Points every GRID_SPACING_METRES over the wall's face, as (along, height) pairs,
    leaving out points right on an edge of the wall or of an opening."""
    points = []
    for along_metres in np.arange(GRID_SPACING_METRES / 2.0, wall.length_metres(), GRID_SPACING_METRES):
        for height_metres in np.arange(wall.bottom_height_metres + GRID_SPACING_METRES / 2.0, wall.top_height_metres, GRID_SPACING_METRES):
            if not _near_an_edge(wall, along_metres, height_metres):
                points.append((along_metres, height_metres))
    return points


def _near_an_edge(wall, along_metres, height_metres):
    along_edges = [0.0, wall.length_metres()]
    height_edges = [wall.bottom_height_metres, wall.top_height_metres]
    for opening in wall.openings:
        along_edges += [opening.start_along_wall_metres, opening.end_along_wall_metres]
        height_edges += [opening.bottom_height_metres, opening.top_height_metres]
    near_along_edge = min(abs(along_metres - edge) for edge in along_edges) < EDGE_ALLOWANCE_METRES
    near_height_edge = min(abs(height_metres - edge) for edge in height_edges) < EDGE_ALLOWANCE_METRES
    return near_along_edge or near_height_edge


def _inside_an_opening(wall, along_metres, height_metres):
    for opening in wall.openings:
        inside_along = opening.start_along_wall_metres < along_metres < opening.end_along_wall_metres
        inside_height = opening.bottom_height_metres < height_metres < opening.top_height_metres
        if inside_along and inside_height:
            return True
    return False


def problems_with_tubes(course_map, shapes_by_model):
    problems = []
    for tube in course_map.tubes:
        shapes = shapes_by_model[model_name_for(tube.name)]
        axis = (tube.exit_centre_metres - tube.entrance_centre_metres) / tube.length_metres()
        to_the_left = np.array([-axis[1], axis[0]])
        middle_height_metres = tube.outside_height_metres / 2.0
        # Half-sizes of the oval through the middle of the tube's wall, and of its hollow
        # inside. Our own choice: test the hollow at 90% of its size, well clear of the wall.
        wall_middle_half_sizes = (tube.outside_width_metres / 2.0 - tube.wall_thickness_metres / 2.0,
                                  tube.outside_height_metres / 2.0 - tube.wall_thickness_metres / 2.0)
        hollow_half_sizes = (0.9 * (tube.outside_width_metres / 2.0 - tube.wall_thickness_metres),
                             0.9 * (tube.outside_height_metres / 2.0 - tube.wall_thickness_metres))
        missing_wall_count = 0
        blocked_hollow_count = 0
        for fraction_along in np.linspace(0.05, 0.95, TUNNEL_PLACES_ALONG):
            on_axis = tube.entrance_centre_metres + axis * tube.length_metres() * fraction_along
            for angle_radians in np.linspace(0.0, 2.0 * math.pi, TUNNEL_PLACES_AROUND, endpoint=False):
                for half_sizes, should_be_solid in ((wall_middle_half_sizes, True), (hollow_half_sizes, False)):
                    sideways_metres = half_sizes[0] * math.cos(angle_radians)
                    up_metres = half_sizes[1] * math.sin(angle_radians)
                    point = [*(on_axis + to_the_left * sideways_metres), middle_height_metres + up_metres]
                    if is_solid(shapes, point) != should_be_solid:
                        if should_be_solid:
                            missing_wall_count += 1
                        else:
                            blocked_hollow_count += 1
        if missing_wall_count or blocked_hollow_count:
            problems.append(f"tunnel '{tube.name}' has {missing_wall_count} gaps in its wall and {blocked_hollow_count} blocked points inside")
    return problems


def problems_with_low_obstacles(course_map, shapes_by_model):
    problems = []
    for obstacle in course_map.low_obstacles:
        shapes = shapes_by_model[model_name_for(obstacle.name)]
        centre_x, centre_y = obstacle.centre_metres
        half_height_metres = obstacle.top_height_metres / 2.0
        just_beyond_side = [centre_x + obstacle.radius_metres + EDGE_ALLOWANCE_METRES, centre_y, half_height_metres]
        just_above_top = [centre_x, centre_y, obstacle.top_height_metres + EDGE_ALLOWANCE_METRES]
        if not is_solid(shapes, [centre_x, centre_y, half_height_metres]):
            problems.append(f"low obstacle '{obstacle.name}' is not solid in its middle")
        if is_solid(shapes, just_beyond_side) or is_solid(shapes, just_above_top):
            problems.append(f"low obstacle '{obstacle.name}' is bigger than the map says")
    return problems


def problems_with_net(course_map, shapes_by_model):
    shapes = shapes_by_model["safety_net"]
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    middle_x, middle_y = (low_x + high_x) / 2.0, (low_y + high_y) / 2.0
    # Our own choice of test height: 1 m, about where the drone flies.
    test_height_metres = 1.0
    gap = EDGE_ALLOWANCE_METRES
    # Each side: (name, a point just outside it, a point just inside it).
    sides = [
        ("west", [low_x - gap, middle_y, test_height_metres], [low_x + gap, middle_y, test_height_metres]),
        ("east", [high_x + gap, middle_y, test_height_metres], [high_x - gap, middle_y, test_height_metres]),
        ("south", [middle_x, low_y - gap, test_height_metres], [middle_x, low_y + gap, test_height_metres]),
        ("north", [middle_x, high_y + gap, test_height_metres], [middle_x, high_y - gap, test_height_metres]),
    ]
    problems = []
    for side_name, just_outside, just_inside in sides:
        if not is_solid(shapes, just_outside) or is_solid(shapes, just_inside):
            problems.append(f"the {side_name} side of the net is not on the course edge")
    if not is_solid(shapes, [middle_x, middle_y, NET_HEIGHT_METRES + gap]):
        problems.append("the net roof is missing")
    return problems


# ---- Gazebo's own check ----------------------------------------------------------------------

def problems_from_gazebo_checker(world_file_path):
    """Runs "gz sdf -k", which reads the file the way Gazebo will. Returns None if Gazebo is
    not installed here."""
    if shutil.which("gz") is None:
        return None
    environment = dict(os.environ)
    # SDF_PATH and GZ_SIM_RESOURCE_PATH tell Gazebo where to look for "model://" names.
    model_folders = f"{OUR_MODELS_FOLDER}:{PX4_MODELS_FOLDER}"
    environment["SDF_PATH"] = model_folders
    environment["GZ_SIM_RESOURCE_PATH"] = model_folders
    result = subprocess.run(["gz", "sdf", "-k", world_file_path], capture_output=True, text=True, env=environment)
    if result.returncode == 0 and "Valid" in result.stdout:
        return []
    return [f"Gazebo's checker rejected the file: {(result.stdout + result.stderr).strip()}"]


# ---- Putting it together ---------------------------------------------------------------------

def check_one_map(map_file_path):
    """Returns (list of problems, whether Gazebo's own checker ran)."""
    course_map = load_course_map(map_file_path)
    world_file_path = make_world_file(map_file_path, include_drone=True)
    shapes_by_model = solid_shapes_by_model(world_file_path)
    problems = []
    problems += problems_with_walls(course_map, shapes_by_model)
    problems += problems_with_tubes(course_map, shapes_by_model)
    problems += problems_with_low_obstacles(course_map, shapes_by_model)
    problems += problems_with_net(course_map, shapes_by_model)
    gazebo_problems = problems_from_gazebo_checker(world_file_path)
    if gazebo_problems is not None:
        problems += gazebo_problems
    return problems, gazebo_problems is not None


def main():
    map_file_paths = sorted(glob.glob(MAP_FILES_PATTERN))
    if not map_file_paths:
        raise SystemExit(f"Found no map files matching {MAP_FILES_PATTERN}. Run this from inside the repository.")
    any_failed = False
    gazebo_checker_ran = True
    for map_file_path in map_file_paths:
        problems, checker_ran = check_one_map(map_file_path)
        gazebo_checker_ran = gazebo_checker_ran and checker_ran
        map_name = os.path.basename(map_file_path)
        if problems:
            any_failed = True
            print(f"FAIL  {map_name}")
            for problem in problems:
                print(f"        {problem}")
        else:
            print(f"PASS  {map_name}")
    if not gazebo_checker_ran:
        print("Gazebo is not installed here, so Gazebo's own file check was skipped. Run this in the drone-simulator-gazebo box to include it.")
    if any_failed:
        raise SystemExit("At least one world does not match its map. The lines above say which shapes are wrong.")


if __name__ == "__main__":
    main()
