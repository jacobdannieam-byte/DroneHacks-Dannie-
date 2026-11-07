"""Measures how fast Gazebo can draw camera pictures in our box. TRIAL ONLY, SIMULATOR ONLY.

Why this matters
----------------
The box has no graphics card, so the processor draws every camera picture in software. That
might be too slow to be useful. This script measures it rather than guessing.

What it does
------------
For each camera setup in CAMERA_SETUPS, it:

  1. makes a world from a course map, with a camera on a fixed stand on the launch mark,
     at flying height, looking down the first corridor;
  2. starts Gazebo with that world, with no screen;
  3. waits for the first pictures, then counts pictures for MEASURING_SECONDS;
  4. reads Gazebo's simulated clock and the processor time Gazebo used;
  5. stops Gazebo.

It then prints a table. "Simulation speed" is simulated seconds per real second: 1.0 means
the simulation keeps up with real time; 0.5 means everything runs in slow motion, at half
speed. A slow simulation is still correct, because PX4 follows Gazebo's clock, but a flight
takes longer to run.

There is no drone and no PX4 here, only the course and the camera, so this measures the
drawing alone. run_px4_in_gazebo.sh measures the whole thing with a flying drone.

Run it inside the drone-simulator-gazebo box, for example:

    docker run --rm --volume "$PWD:/repository" drone-simulator-gazebo \\
        python3 /repository/simulator/gazebo_trial/measure_rendering_speed.py

It takes about 4 minutes.
"""
import argparse
import math
import os
import subprocess
import sys
import threading
import time
from typing import NamedTuple, Optional

from gz.msgs10.image_pb2 import Image as GazeboImage
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node as GazeboNode

FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(FOLDER_OF_THIS_FILE))
FOLDER_FOR_TEST_WORLDS = os.path.join(FOLDER_OF_THIS_FILE, "generated", "speed_test_worlds")
DEFAULT_MAP_FILE = os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav", "course_maps", "organizers_drawing.json")

# See make_gazebo_world.py for why we add this folder to Python's search list.
sys.path.insert(0, os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav"))
from team_nav.course_map import load_course_map  # noqa: E402

from course_world_sdf import WORLD_NAME, world_sdf  # noqa: E402

# Our own choice: long enough that one slow picture barely changes the average.
MEASURING_SECONDS = 20.0

# Our own choice: Gazebo needs some seconds to start and draw its first picture. If nothing
# arrives in this time, the drawing does not work at all.
LONGEST_WAIT_FOR_FIRST_PICTURE_SECONDS = 60.0

# The camera stand: on the launch mark, at 0.8 m, which is the cruise height of the normal
# simulator's landing test, facing 90 degrees from the x axis, down the first corridor.
CAMERA_HEIGHT_METRES = 0.8
CAMERA_HEADING_DEGREES = 90.0

# The same settings as our pretend OAK-D S2 (models/oak_d_s2_camera/model.sdf, which says where
# each number comes from): the width of the view, 2 * atan(512 / 634.5) radians, and the
# nearest and farthest distances the camera reports.
CAMERA_VIEW_WIDTH_RADIANS = 1.3579
NEAREST_DISTANCE_METRES = 0.35
FARTHEST_DISTANCE_METRES = 10.0

# From docs/ARCHITECTURE.md (drone 16's camera settings): the real camera's two eyes are
# 7.5 cm apart.
DISTANCE_BETWEEN_EYES_METRES = 0.075

# Our own choice: the first few pictures are often slow while the drawing warms up, so we wait
# this long after the first one before we start counting.
WARM_UP_SECONDS = 5.0

# Our own choice: how long to give Gazebo to shut down cleanly after each setup.
LONGEST_WAIT_FOR_GAZEBO_TO_STOP_SECONDS = 30.0

# Our own choice: how often to check whether Gazebo has started.
SECONDS_BETWEEN_CHECKS = 0.5

# Our own choice, only for the --use-pretend-screen test: the size of the pretend screen, as
# width x height x colour bits per pixel. Gazebo shows no window, so any ordinary size works.
PRETEND_SCREEN_SIZE = "1280x1024x24"

# How many clock ticks Linux counts per second of processor time, read from Linux itself. We
# need it to turn Linux's record of a program's processor use into seconds.
LINUX_CLOCK_TICKS_PER_SECOND = os.sysconf("SC_CLK_TCK")

class CameraSetup(NamedTuple):
    """One camera setup to measure.

    sensor_type is Gazebo's name for the kind of camera. "depth_camera" gives distances.
    "camera" gives an ordinary picture; the real camera's two black-and-white eyes are two of
    these, with pixel format L8 (one byte of brightness). R_FLOAT32 is one 4-byte decimal
    number per pixel, the distance in metres."""
    description: str
    sensor_type: Optional[str]
    width_pixels: int
    height_pixels: int
    pictures_per_second: int
    camera_count: int
    pixel_format: Optional[str]


# Our own choices of what to compare. The sizes are the real camera's (1024x640 at 30 per
# second, docs/ARCHITECTURE.md), the organizers' own simulator (640x400 at 15 per second,
# AI_CONTEXT.md) and our depth plan (512x320 at about 10 per second, docs/ARCHITECTURE.md).
CAMERA_SETUPS = [
    CameraSetup("no camera (for comparison)", None, 0, 0, 0, 0, None),
    CameraSetup("depth 512x320 at 10/s (our depth plan)", "depth_camera", 512, 320, 10, 1, "R_FLOAT32"),
    CameraSetup("depth 512x320 at 30/s", "depth_camera", 512, 320, 30, 1, "R_FLOAT32"),
    CameraSetup("depth 640x400 at 15/s (organizers' lab)", "depth_camera", 640, 400, 15, 1, "R_FLOAT32"),
    CameraSetup("depth 1024x640 at 30/s (real camera size)", "depth_camera", 1024, 640, 30, 1, "R_FLOAT32"),
    CameraSetup("two eyes 640x400 at 15/s", "camera", 640, 400, 15, 2, "L8"),
    CameraSetup("two eyes 1024x640 at 30/s (real camera)", "camera", 1024, 640, 30, 2, "L8"),
]


# ---- Making a test world ---------------------------------------------------------------------

def camera_stand_sdf(course_map, setup):
    """A fixed, floating stand with setup.camera_count cameras on it, side by side, 7.5 cm
    apart like the real camera's two eyes."""
    if setup.sensor_type is None:
        return ""
    sensors = []
    for number in range(setup.camera_count):
        sideways_metres = (number - (setup.camera_count - 1) / 2.0) * DISTANCE_BETWEEN_EYES_METRES
        sensors.append(f"""        <sensor name="camera_{number}" type="{setup.sensor_type}">
          <pose>0 {sideways_metres:.4f} 0 0 0 0</pose>
          <always_on>true</always_on>
          <update_rate>{setup.pictures_per_second}</update_rate>
          <topic>speed_test/camera_{number}</topic>
          <camera>
            <horizontal_fov>{CAMERA_VIEW_WIDTH_RADIANS}</horizontal_fov>
            <image><width>{setup.width_pixels}</width><height>{setup.height_pixels}</height><format>{setup.pixel_format}</format></image>
            <clip><near>{NEAREST_DISTANCE_METRES}</near><far>{FARTHEST_DISTANCE_METRES}</far></clip>
          </camera>
        </sensor>
""")
    launch_x, launch_y = course_map.launch_mark_centre_metres
    heading_radians = math.radians(CAMERA_HEADING_DEGREES)
    return f"""    <model name="camera_stand">
      <static>true</static>
      <pose>{launch_x:.4f} {launch_y:.4f} {CAMERA_HEIGHT_METRES} 0 0 {heading_radians:.5f}</pose>
      <link name="link">
{"".join(sensors)}      </link>
    </model>"""


def write_test_world(course_map, setup_number, setup):
    stand = camera_stand_sdf(course_map, setup)
    os.makedirs(FOLDER_FOR_TEST_WORLDS, exist_ok=True)
    path = os.path.join(FOLDER_FOR_TEST_WORLDS, f"setup_{setup_number}.sdf")
    with open(path, "w", encoding="utf-8") as world_file:
        world_file.write(world_sdf(course_map, stand))
    return path


# ---- Listening to Gazebo ---------------------------------------------------------------------

class GazeboListener:
    """Counts pictures from every camera, and keeps Gazebo's latest simulated time and its own
    estimate of the simulation speed. Gazebo calls these functions from its own thread."""

    def __init__(self, camera_count):
        self.lock = threading.Lock()
        self.picture_count = 0
        self.simulated_seconds = None
        self.gazebo_node = GazeboNode()
        for number in range(camera_count):
            self.gazebo_node.subscribe(GazeboImage, f"/speed_test/camera_{number}", self.on_picture)
        self.gazebo_node.subscribe(WorldStatistics, f"/world/{WORLD_NAME}/stats", self.on_world_statistics)

    def on_picture(self, _picture):
        with self.lock:
            self.picture_count += 1

    def on_world_statistics(self, statistics):
        with self.lock:
            self.simulated_seconds = statistics.sim_time.sec + statistics.sim_time.nsec * 1e-9

    def snapshot(self):
        with self.lock:
            return self.picture_count, self.simulated_seconds


def processor_seconds_used_by(process_id):
    """The processor time a running program has used so far, adding up all its threads."""
    with open(f"/proc/{process_id}/stat", encoding="utf-8") as stat_file:
        # The program's name is in brackets and may contain spaces, so we split after it.
        fields_after_name = stat_file.read().rsplit(")", 1)[1].split()
    # Fields 14 and 15 of the file (counting from 1) are the time spent in the program itself
    # and in the operating system on its behalf. After the name we are at field 3.
    user_ticks = int(fields_after_name[14 - 3])
    system_ticks = int(fields_after_name[15 - 3])
    return (user_ticks + system_ticks) / LINUX_CLOCK_TICKS_PER_SECOND


# ---- Measuring one setup ---------------------------------------------------------------------

def measure_one_setup(world_file_path, camera_count, use_pretend_screen):
    """Starts Gazebo on the world, measures, stops Gazebo, and returns the results."""
    gazebo = start_gazebo(world_file_path, use_pretend_screen)
    try:
        listener = GazeboListener(camera_count)
        wait_until_running(listener, camera_count, gazebo)
        start_wall_seconds = time.monotonic()
        start_pictures, start_simulated_seconds = listener.snapshot()
        start_processor_seconds = processor_seconds_used_by(gazebo.pid)
        time.sleep(MEASURING_SECONDS)
        wall_seconds = time.monotonic() - start_wall_seconds
        end_pictures, end_simulated_seconds = listener.snapshot()
        processor_seconds = processor_seconds_used_by(gazebo.pid) - start_processor_seconds
    finally:
        gazebo.terminate()
        gazebo.wait(timeout=LONGEST_WAIT_FOR_GAZEBO_TO_STOP_SECONDS)

    simulated_seconds = end_simulated_seconds - start_simulated_seconds
    pictures_per_camera = (end_pictures - start_pictures) / max(camera_count, 1)
    return {
        "pictures per real second, per camera": pictures_per_camera / wall_seconds,
        "pictures per simulated second, per camera": pictures_per_camera / simulated_seconds if simulated_seconds > 0 else 0.0,
        "simulation speed": simulated_seconds / wall_seconds,
        "processor cores busy": processor_seconds / wall_seconds,
    }


def start_gazebo(world_file_path, use_pretend_screen):
    """Starts Gazebo's simulation server with no window. -s means "server only", -r means
    "start running at once". --headless-rendering draws pictures without any screen (through
    EGL). With use_pretend_screen, Gazebo draws on Xvfb, a pretend screen, instead."""
    command = ["gz", "sim", "-s", "-r", "-v", "1", world_file_path]
    if use_pretend_screen:
        command = ["xvfb-run", "--auto-servernum", f"--server-args=-screen 0 {PRETEND_SCREEN_SIZE}"] + command
    else:
        command.insert(3, "--headless-rendering")
    return subprocess.Popen(command)


def wait_until_running(listener, camera_count, gazebo):
    """Waits until the simulated clock is moving and, if there are cameras, a picture has
    arrived. Then waits a few more seconds, so start-up work does not count."""
    deadline = time.monotonic() + LONGEST_WAIT_FOR_FIRST_PICTURE_SECONDS
    while time.monotonic() < deadline:
        if gazebo.poll() is not None:
            raise RuntimeError("Gazebo stopped while starting. Its messages above say why.")
        picture_count, simulated_seconds = listener.snapshot()
        clock_is_running = simulated_seconds is not None and simulated_seconds > 0.0
        pictures_arrived = camera_count == 0 or picture_count > 0
        if clock_is_running and pictures_arrived:
            time.sleep(WARM_UP_SECONDS)
            return
        time.sleep(SECONDS_BETWEEN_CHECKS)
    raise RuntimeError(
        f"No picture arrived within {LONGEST_WAIT_FOR_FIRST_PICTURE_SECONDS:.0f} seconds, so Gazebo cannot draw "
        "camera pictures in this box. Try again with --use-pretend-screen, and read Gazebo's messages above."
    )


# ---- Putting it together ---------------------------------------------------------------------

def print_table(results):
    print()
    print(f"{'Camera setup':45} {'pictures/s':>11} {'pictures per':>13} {'simulation':>11} {'cores':>6}")
    print(f"{'':45} {'(real time)':>11} {'simulated s':>13} {'speed':>11} {'busy':>6}")
    for description, result in results:
        print(
            f"{description:45} {result['pictures per real second, per camera']:11.1f} "
            f"{result['pictures per simulated second, per camera']:13.1f} "
            f"{result['simulation speed']:11.2f} {result['processor cores busy']:6.2f}"
        )


def main():
    parser = argparse.ArgumentParser(description="Measures how fast Gazebo draws camera pictures in this box.")
    parser.add_argument("--map-file", default=DEFAULT_MAP_FILE, help="The course map to build the test world from.")
    parser.add_argument("--use-pretend-screen", action="store_true", help="Draw on Xvfb instead of drawing with no screen at all.")
    arguments = parser.parse_args()

    course_map = load_course_map(arguments.map_file)
    results = []
    for setup_number, setup in enumerate(CAMERA_SETUPS):
        print(f"Measuring: {setup.description}", flush=True)
        world_file_path = write_test_world(course_map, setup_number, setup)
        result = measure_one_setup(world_file_path, setup.camera_count, arguments.use_pretend_screen)
        results.append((setup.description, result))
    print_table(results)


if __name__ == "__main__":
    main()
