"""Makes the Gazebo world for the full Gazebo simulator, with textured surfaces. SIMULATOR ONLY.

What it does
------------
It builds the course from a course map file, the same way the Gazebo trial does
(simulator/gazebo_trial/course_world_sdf.py), and then changes two things for the stereo cameras:

  1. Every surface gets a picture ("texture") of random blotches instead of a flat colour. Stereo
     depth works by finding the same small patch in the left and the right picture. On a flat
     colour every patch looks the same, so no depth can be found. The blotches come in three
     sizes, so that there is detail to match both close up and far away.
  2. The floor inside the net is covered by visible tiles of about 1 m. Gazebo stretches one
     texture over a whole surface, and the floor plane is about 16 m across, so one texture over
     it would be too coarse to match. The tiles are only seen, not felt: the drone stands on the
     floor plane underneath them.

The drone is "drone_16_with_oak_d_stereo" (models/), placed on the launch mark, and the course is
shifted so that Gazebo's (0, 0, 0) is where the drone's centre rests on the launch mark. The
trial's make_gazebo_world.py explains why that shift is needed.

Run it inside the drone-simulator-gazebo-full box, from the top folder of the repository:

    python3 simulator/gazebo_full/make_textured_world.py drone_code/team_nav/course_maps/phone_scan_2026-11-06.json OUTPUT_FOLDER

It writes OUTPUT_FOLDER/course_world.sdf and the texture pictures next to it, and prints the
world file's path.

What it does not do
-------------------
The textures are not photos of the real course. Real walls may be plainer (worse for stereo) or
have more detail. The net is drawn as a solid textured sheet, but the real net is see-through.
"""
import argparse
import math
import os
import re
import sys

import numpy as np
from matplotlib import pyplot
from scipy.ndimage import gaussian_filter

FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(FOLDER_OF_THIS_FILE))
TRIAL_FOLDER = os.path.join(REPOSITORY_FOLDER, "simulator", "gazebo_trial")

# The map reader is the drone's own, so the world matches exactly what the route planner reads.
sys.path.insert(0, os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav"))
sys.path.insert(0, TRIAL_FOLDER)
from team_nav.course_map import load_course_map  # noqa: E402

from course_world_sdf import COURSE_FRAME_NAME, world_sdf  # noqa: E402
from make_drone_16_model import HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES  # noqa: E402
from make_gazebo_world import DRONE_NAME_IN_WORLD, START_HEADING_DEGREES  # noqa: E402

# The folder name of our drone with two cameras, in models/ next to this file.
DRONE_MODEL_NAME = "drone_16_with_oak_d_stereo"

# Our own choice. 1024 pixels over a 6 m wall is about 6 mm per pixel, finer than the cameras
# can see beyond about 1 m (one camera pixel covers about 3 mm at 1 m and 1 cm at 3 m).
TEXTURE_SIZE_PIXELS = 1024

# Our own choice: the blotch sizes, as the blur width in texture pixels. On a 3 m wall, 2, 7 and
# 25 pixels are about 0.6, 2 and 7 cm, so some detail is a few camera pixels wide at any
# distance in the course.
BLOTCH_SIZES_PIXELS = (2.0, 7.0, 25.0)

# Our own choice. How strongly the blotches vary in brightness: 0.6 means from 40% to 100% of
# the surface's colour. Strong enough to match well, while the surfaces keep their colours.
BLOTCH_CONTRAST = 0.6

# Our own choice. The floor tiles inside the net. About 1 m, so the floor texture has about
# 1 mm per texture pixel. They sit this far above the real floor so that the floor plane doesn't
# show through them.
FLOOR_TILE_TARGET_SIZE_METRES = 1.0
FLOOR_TILE_HEIGHT_ABOVE_FLOOR_METRES = 0.002

# Our own choice: one texture per kind of surface, each from its own random numbers, so that two
# different walls never show exactly the same pattern side by side. The colours are the same as
# the trial's (course_world_sdf.py).
TEXTURE_SEEDS = {"floor": 1, "wall": 2, "tunnel": 3, "low_obstacle": 4, "net": 5}

# Matches the material that course_world_sdf.py writes for every shape, and captures its colour.
PLAIN_MATERIAL_PATTERN = re.compile(r"<material><ambient>([^<]*)</ambient><diffuse>([^<]*)</diffuse></material>")


def make_texture(file_path, seed):
    """Writes one grey texture picture of random blotches, brightness 1 - contrast to 1."""
    random_numbers = np.random.default_rng(seed)
    brightness = np.zeros((TEXTURE_SIZE_PIXELS, TEXTURE_SIZE_PIXELS))
    for blotch_size_pixels in BLOTCH_SIZES_PIXELS:
        noise = random_numbers.standard_normal((TEXTURE_SIZE_PIXELS, TEXTURE_SIZE_PIXELS))
        # "wrap" makes the picture join up with itself at its edges.
        blotches = gaussian_filter(noise, blotch_size_pixels, mode="wrap")
        brightness += blotches / blotches.std()
    lowest, highest = np.percentile(brightness, (1, 99))
    brightness = np.clip((brightness - lowest) / (highest - lowest), 0.0, 1.0)
    brightness = 1.0 - BLOTCH_CONTRAST + BLOTCH_CONTRAST * brightness
    grey_picture = np.repeat(brightness[:, :, np.newaxis], 3, axis=2)
    pyplot.imsave(file_path, grey_picture)


def textured_material(colour_text, texture_path):
    """A Gazebo material whose colour is the texture picture multiplied by colour_text."""
    return (f"<material><ambient>{colour_text}</ambient><diffuse>{colour_text}</diffuse>"
            f"<pbr><metal><albedo_map>{texture_path}</albedo_map>"
            f"<metalness>0</metalness><roughness>1</roughness></metal></pbr></material>")


def kind_of_surface(model_name, course_map):
    """Which texture a model of the course gets, from its name."""
    if model_name == "floor":
        return "floor"
    if model_name.startswith("safety_net"):
        return "net"
    for tube in course_map.tubes:
        if model_name.startswith(model_name_safe(tube.name)):
            return "tunnel"
    for obstacle in course_map.low_obstacles:
        if model_name.startswith(model_name_safe(obstacle.name)):
            return "low_obstacle"
    return "wall"


def model_name_safe(name):
    """The same name change as course_world_sdf.model_name_for."""
    safe_characters = []
    for character in name:
        if character.isalnum() or character == "_":
            safe_characters.append(character)
        else:
            safe_characters.append("_")
    return "".join(safe_characters)


def add_textures(world_text, course_map, texture_paths):
    """Replaces every flat colour with a textured one. The world text is a list of models, one
    after another; each model's surfaces get the texture for that model's kind."""
    pieces = re.split(r'(<model name="[^"]*">)', world_text)
    current_kind = "wall"
    changed_pieces = []
    for piece in pieces:
        model_start = re.match(r'<model name="([^"]*)">', piece)
        if model_start:
            current_kind = kind_of_surface(model_start.group(1), course_map)
        texture_path = texture_paths[current_kind]
        changed_pieces.append(PLAIN_MATERIAL_PATTERN.sub(
            lambda match: textured_material(match.group(2), texture_path), piece))
    return "".join(changed_pieces)


def floor_tiles_sdf(course_map, texture_path):
    """Thin flat boxes covering the floor inside the net, seen by cameras but not solid."""
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    tiles_across = max(1, math.ceil((high_x - low_x) / FLOOR_TILE_TARGET_SIZE_METRES))
    tiles_along = max(1, math.ceil((high_y - low_y) / FLOOR_TILE_TARGET_SIZE_METRES))
    tile_width_metres = (high_x - low_x) / tiles_across
    tile_length_metres = (high_y - low_y) / tiles_along
    material = textured_material("0.55 0.55 0.55 1.0", texture_path)
    visuals = []
    for column in range(tiles_across):
        for row in range(tiles_along):
            centre_x = low_x + (column + 0.5) * tile_width_metres
            centre_y = low_y + (row + 0.5) * tile_length_metres
            visuals.append(f"""        <visual name="tile_{column}_{row}">
          <pose>{centre_x:.4f} {centre_y:.4f} {FLOOR_TILE_HEIGHT_ABOVE_FLOOR_METRES / 2.0:.4f} 0 0 0</pose>
          <geometry><box><size>{tile_width_metres:.4f} {tile_length_metres:.4f} {FLOOR_TILE_HEIGHT_ABOVE_FLOOR_METRES:.4f}</size></box></geometry>
          {material}
        </visual>
""")
    return f"""    <model name="floor_tiles_seen_only">
      <static>true</static>
      <pose relative_to="{COURSE_FRAME_NAME}">0 0 0 0 0 0</pose>
      <link name="link">
{"".join(visuals)}      </link>
    </model>"""


def drone_sdf():
    """Our drone on Gazebo's (0, 0, 0), which is the launch mark, facing the start heading."""
    return f"""    <include>
      <uri>model://{DRONE_MODEL_NAME}</uri>
      <name>{DRONE_NAME_IN_WORLD}</name>
      <pose>0 0 0 0 0 {math.radians(START_HEADING_DEGREES):.5f}</pose>
    </include>"""


def make_world(map_file_path, output_folder):
    course_map = load_course_map(map_file_path)
    os.makedirs(output_folder, exist_ok=True)
    texture_paths = {}
    for kind, seed in TEXTURE_SEEDS.items():
        texture_paths[kind] = os.path.abspath(os.path.join(output_folder, f"texture_{kind}.png"))
        make_texture(texture_paths[kind], seed)

    launch_x, launch_y = course_map.launch_mark_centre_metres
    course_origin_in_world_metres = (-launch_x, -launch_y, -HEIGHT_OF_CENTRE_ABOVE_FLOOR_WHEN_STANDING_METRES)
    extra_sdf = drone_sdf() + "\n" + floor_tiles_sdf(course_map, texture_paths["floor"])
    plain_world = world_sdf(course_map, extra_sdf, course_origin_in_world_metres)
    textured_world = add_textures(plain_world, course_map, texture_paths)

    world_file_path = os.path.join(output_folder, "course_world.sdf")
    with open(world_file_path, "w", encoding="utf-8") as world_file:
        world_file.write(textured_world)
    return world_file_path


def main():
    parser = argparse.ArgumentParser(description="Makes the textured Gazebo world for the full Gazebo simulator.")
    parser.add_argument("map_file", help="The world map: what is really there.")
    parser.add_argument("output_folder", help="Where to write course_world.sdf and its textures.")
    arguments = parser.parse_args()
    print(make_world(arguments.map_file, arguments.output_folder))


if __name__ == "__main__":
    main()
