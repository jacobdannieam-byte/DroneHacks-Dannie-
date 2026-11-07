"""Makes a Gazebo world file from a course map file. TRIAL ONLY, SIMULATOR ONLY.

What it does
------------
It reads one course map (a file from drone_code/team_nav/course_maps/), turns the course into
Gazebo's world format with course_world_sdf.py, puts our simulated drone on the launch mark,
and saves the result in simulator/gazebo_trial/generated/worlds/.

For example, from the top folder of the repository:

    python3 simulator/gazebo_trial/make_gazebo_world.py drone_code/team_nav/course_maps/organizers_drawing.json

It needs Python 3 and numpy, which the simulator boxes have. It does not need Gazebo itself.

What it does not do
-------------------
It does not start Gazebo. run_px4_in_gazebo.sh and inside_the_box/start_px4_in_gazebo.sh do
that. It also does not check the world; check_worlds.py does that, for every map file.

Two kinds of world
------------------
The hover test (run_px4_in_gazebo.sh) uses a plain world: Gazebo's (0, 0, 0) is the centre of
the course on the floor, and the drone is PX4's x500 with our camera ("drone_with_oak_d_s2" in
the models folder). Use --without-drone for a world with only the course.

The full simulator (run_full_route_in_gazebo.sh) uses --for-full-simulator, which changes two
things:
  - The drone is "drone_16_with_oak_d_s2": a drone with drone 16's motor positions, weight and
    thrust (make_drone_16_model.py makes it), with our camera.
  - The whole course is shifted so that Gazebo's (0, 0, 0) is where the drone's centre rests on
    the launch mark. PX4 in Gazebo reports the drone's true position measured from Gazebo's
    (0, 0, 0). The normal simulator's referee and pretend sensors expect it the way PX4's own
    physics reports it: measured from where the drone started, with height 0 while it stands
    on the floor. The shift makes the two the same.
"""
import argparse
import math
import os
import sys

FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(FOLDER_OF_THIS_FILE))
FOLDER_FOR_WORLDS = os.path.join(FOLDER_OF_THIS_FILE, "generated", "worlds")

# The map reader lives in our drone package, which is not installed as a Python package outside
# the drone's ROS setup. Adding its folder to Python's search list lets us import it directly,
# so this script always reads maps with exactly the same code as the route planner.
sys.path.insert(0, os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav"))
from team_nav.course_map import load_course_map  # noqa: E402

from course_world_sdf import world_sdf  # noqa: E402
from make_drone_16_model import HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES  # noqa: E402

# Our assumption, the same one the normal simulator makes (see DEFAULT_START_HEADING_DEGREES in
# simulator/ros_packages/course_simulator/course_simulator/true_drone_state.py): the drone
# starts facing 90 degrees from the course's x axis, which is straight down the first corridor.
# The rules do not say which way it faces.
START_HEADING_DEGREES = 90.0

# PX4 finds the drone in the world by this name. The scripts that start PX4 pass the same name.
# Our own choice.
DRONE_NAME_IN_WORLD = "drone"

# The folder names of our two drone models, in the models folder next to this file.
X500_WITH_CAMERA_MODEL = "drone_with_oak_d_s2"
DRONE_16_WITH_CAMERA_MODEL = "drone_16_with_oak_d_s2"


def drone_sdf(model_folder_name, drone_x_metres, drone_y_metres):
    """The SDF text that places a drone at this spot in Gazebo's world, at height 0."""
    heading_radians = math.radians(START_HEADING_DEGREES)
    return f"""    <include>
      <uri>model://{model_folder_name}</uri>
      <name>{DRONE_NAME_IN_WORLD}</name>
      <pose>{drone_x_metres:.4f} {drone_y_metres:.4f} 0 0 0 {heading_radians:.5f}</pose>
    </include>"""


def world_file_path_for(map_file_path, for_full_simulator=False):
    map_file_name = os.path.splitext(os.path.basename(map_file_path))[0]
    ending = "_full_simulator.sdf" if for_full_simulator else ".sdf"
    return os.path.join(FOLDER_FOR_WORLDS, map_file_name + ending)


def make_world_file(map_file_path, include_drone=True, for_full_simulator=False):
    """Reads the map, writes the world file, and returns the world file's path. The module's
    description explains for_full_simulator."""
    course_map = load_course_map(map_file_path)
    launch_x, launch_y = course_map.launch_mark_centre_metres
    if for_full_simulator:
        # The course point that must land on Gazebo's (0, 0, 0): the drone's centre as it stands
        # on the launch mark. Gazebo places the course's own origin at minus that point.
        course_origin_in_world_metres = (-launch_x, -launch_y, -HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES)
        drone = drone_sdf(DRONE_16_WITH_CAMERA_MODEL, 0.0, 0.0)
    else:
        course_origin_in_world_metres = (0.0, 0.0, 0.0)
        drone = drone_sdf(X500_WITH_CAMERA_MODEL, launch_x, launch_y)

    world_file_path = world_file_path_for(map_file_path, for_full_simulator)
    os.makedirs(FOLDER_FOR_WORLDS, exist_ok=True)
    with open(world_file_path, "w", encoding="utf-8") as world_file:
        world_file.write(world_sdf(course_map, drone if include_drone else "", course_origin_in_world_metres))
    return world_file_path


def main():
    parser = argparse.ArgumentParser(description="Makes a Gazebo world file from a course map file.")
    parser.add_argument("map_file", help="A course map, for example drone_code/team_nav/course_maps/organizers_drawing.json")
    parser.add_argument("--without-drone", action="store_true", help="Leave the drone out, so the world has only the course.")
    parser.add_argument("--for-full-simulator", action="store_true",
                        help="Use the drone 16 model, and put Gazebo's origin where the drone rests on the launch mark. "
                             "run_full_route_in_gazebo.sh uses this.")
    arguments = parser.parse_args()

    world_file_path = make_world_file(arguments.map_file, include_drone=not arguments.without_drone,
                                      for_full_simulator=arguments.for_full_simulator)
    print(f"Wrote {os.path.relpath(world_file_path)}")


if __name__ == "__main__":
    main()
