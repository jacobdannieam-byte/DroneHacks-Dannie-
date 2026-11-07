"""Which course the simulator flies in. SIMULATOR ONLY.

The simulated course, the "world", is described by a course map file: the same kind of file the
drone's route planner reads (drone_code/team_nav/course_maps/). To choose one, set the
environment variable SIMULATOR_WORLD_MAP to the map file's path inside the box.
run_test_flight.sh does this for you with its --world option. Without it, the simulator uses
the organizers' drawing.

The world map is the truth in the simulation: where the walls really are, where the referee's
checkpoint lines are, and where the drone starts. The drone's own map may differ on purpose, to
test how the drone copes with a course that has changed.

The pretend sensors and the referee also need a full 3D model of the course (course_model.py):
  - For the organizers' drawing, that is the organizers' own 3D model, which has more detail
    than the map, such as the window frame and the line on the floor.
  - For any other map, course_mesh_from_map.py makes a 3D model from the map's shapes the first
    time it is needed, in generated_files/worlds/. The folder's name includes a fingerprint of
    the map file's contents, so a changed map gets a fresh model and fresh distance tables.

whole_simulator.launch.py calls prepare_world before it starts any simulator program, so that
the model and its tables are ready before the drone flies.
"""
import hashlib
import os

from team_nav.course_map import load_course_map

from course_simulator.course_mesh_from_map import write_course_mesh
from course_simulator.course_model import CourseModel
from course_simulator.paths import (
    COURSE_MODEL_FILE, FOLDER_FOR_GENERATED_WORLDS, FOLDER_FOR_SAVED_TABLES, ORGANIZERS_DRAWING_MAP_FILE,
)

WORLD_MAP_SETTING = "SIMULATOR_WORLD_MAP"


def world_map_file():
    path = os.environ.get(WORLD_MAP_SETTING, ORGANIZERS_DRAWING_MAP_FILE)
    if not os.path.exists(path):
        raise FileNotFoundError(
            f"{WORLD_MAP_SETTING} is set to {path}, but there is no such file in the box. Give the map's path "
            "from the top of the repository, for example drone_code/team_nav/course_maps/phone_scan_2026-11-06.json."
        )
    return path


def load_world_map():
    return load_course_map(world_map_file())


def prepare_world():
    """Makes the current world's 3D model and distance tables, if they don't exist yet.

    Without this, every simulator program that needs the tables builds them itself, all at the
    same moment, and the referee starts recording late. Building tables is heavy work. The first
    time we flew in a new world that way, the flight went wrong: the drone climbed to 3.6 m and
    the test never saw it touch down. The same flight with the tables built beforehand was normal.
    """
    CourseModel(*course_model_file_and_tables_folder())


def course_model_file_and_tables_folder():
    """Returns (the 3D model file, the folder for its distance tables) for the current world."""
    map_file = world_map_file()
    if os.path.abspath(map_file) == os.path.abspath(ORGANIZERS_DRAWING_MAP_FILE):
        return COURSE_MODEL_FILE, FOLDER_FOR_SAVED_TABLES

    world_folder = os.path.join(FOLDER_FOR_GENERATED_WORLDS, _folder_name_for(map_file))
    model_file = os.path.join(world_folder, "course.obj")
    if not os.path.exists(model_file):
        os.makedirs(world_folder, exist_ok=True)
        # Several simulator programs start at once. Each writes to its own temporary name and
        # then renames it, so none of them can read a half-written file.
        temporary_file = f"{model_file}.{os.getpid()}.tmp"
        write_course_mesh(load_course_map(map_file), temporary_file)
        os.replace(temporary_file, model_file)
    return model_file, os.path.join(world_folder, "course_tables")


def _folder_name_for(map_file):
    with open(map_file, "rb") as contents:
        fingerprint = hashlib.sha1(contents.read()).hexdigest()[:10]
    map_name = os.path.splitext(os.path.basename(map_file))[0]
    return f"{map_name}_{fingerprint}"
