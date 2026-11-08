"""SIMULATOR ONLY. After a Gazebo run, finds how close the drone's true propellers came to anything.

    python3 check_footprint_clearance.py <run folder in the box> <world map file>

fly_course_in_gazebo.sh runs it after every flight. It reads the referee's record of the true
path (referee/flight_*.csv: true position, height and heading 30 times a second, from Gazebo) and,
for every moment the drone is off the floor, measures the gap between the drone's outline seen
from above and everything in the world map at the drone's height: walls, the tunnel, boulders and
the net. The outline is the one full_course's route checks use (astar_geometry.py: the four motors,
each with its 7-inch propeller).

It adds two lines to summary.txt: the smallest gap, where and against what, and PASS or FAIL.
A run FAILS if the gap is ever under LEAST_ALLOWED_CLEARANCE_METRES. That is stricter than the
referee's touch rule (1 cm), so a near miss shows up before it becomes a touch.

What it does not do: look at the drone's legs or body above and below the propellers, or judge
anything during takeoff or touchdown on the floor (the floor itself is not an obstacle).
"""
import csv
import glob
import math
import os
import sys

import numpy as np

from team_nav.course_map import BlockingCircle, BlockingStrip, load_course_map
from team_nav.practice_flights.astar_geometry import RouteGeometry

# The team's requirement of 2026-11-08.
LEAST_ALLOWED_CLEARANCE_METRES = 0.08
# Our own choice: off the floor means the true height is above this.
OFF_THE_FLOOR_METRES = 0.03
# Our own choice: obstacles are taken at the drone's height to the nearest 5 cm, +/- 0.20 m,
# the same band the route checks use.
HEIGHT_STEP_METRES = 0.05


def main():
    run_folder, world_map_file = sys.argv[1], sys.argv[2]
    world = load_course_map(world_map_file)
    samples = read_true_path(run_folder)
    if not samples:
        write_summary(run_folder, ["Footprint clearance: no airborne samples in the referee's record. FAIL."])
        return
    clearances = clearance_of_each_sample(world, samples)
    worst = int(np.argmin(clearances))
    seconds, east, north, height, heading = samples[worst]
    nearest = name_of_nearest_thing(world, samples[worst])
    verdict = "PASS" if clearances[worst] >= LEAST_ALLOWED_CLEARANCE_METRES else "FAIL"
    write_summary(run_folder, [
        f"Footprint clearance (true path): smallest {clearances[worst]:.3f} m, from {nearest}, at {seconds:.1f} s after "
        f"takeoff, east {east:.3f} m, north {north:.3f} m, height {height:.2f} m, heading {heading:.1f} degrees.",
        f"Footprint clearance verdict: {verdict} (a run fails under {LEAST_ALLOWED_CLEARANCE_METRES:.2f} m).",
    ])


def read_true_path(run_folder):
    """(seconds after the first automatic sample, east, north, height, heading) while off the floor."""
    files = sorted(path for path in glob.glob(os.path.join(run_folder, "referee", "flight_*.csv"))
                   if not path.endswith("_summary.csv"))
    if not files:
        return []
    samples, start_seconds = [], None
    with open(files[-1], encoding="utf-8") as record:
        for row in csv.DictReader(record):
            if row["automatic_flight"] != "True":
                continue
            seconds = float(row["seconds"])
            if start_seconds is None:
                start_seconds = seconds
            height = float(row["height_metres"])
            if height > OFF_THE_FLOOR_METRES:
                samples.append((seconds - start_seconds, float(row["course_x_metres"]), float(row["course_y_metres"]),
                                height, float(row["heading_degrees"])))
    return samples


def geometry_at(world, height_metres):
    """RouteGeometry's shapes at this height, with the tunnel's strips given flat ends.

    The map's distance function rounds every strip's ends by half its width. That is right for
    thin walls, but near the floor the tunnel's strip is its full 1.47 m width, and rounded ends
    would put a 0.73 m bulge past its open exit, where the floor is clear. So each tunnel strip is
    replaced by small strips across it, 2 cm apart, whose rounded ends stay within 1 cm of the
    tunnel's real flat end."""
    geometry = RouteGeometry(world, height_metres)
    tunnel_names = {tube.name for tube in world.tubes}
    shapes = []
    for shape in geometry.shapes:
        if not (isinstance(shape, BlockingStrip) and shape.name in tunnel_names):
            shapes.append(shape)
            continue
        along = (shape.end_metres - shape.start_metres) / np.linalg.norm(shape.end_metres - shape.start_metres)
        across = np.array([-along[1], along[0]])
        for offset in np.arange(-shape.half_thickness_metres + 0.01, shape.half_thickness_metres, 0.02):
            shapes.append(BlockingStrip(shape.name, shape.start_metres + across * offset + along * 0.01,
                                        shape.end_metres + across * offset - along * 0.01, 0.01))
    geometry.shapes = shapes
    return geometry


def clearance_of_each_sample(world, samples):
    clearances = np.empty(len(samples))
    geometries = {}
    for number, (_, east, north, height, heading) in enumerate(samples):
        height_step = round(height / HEIGHT_STEP_METRES) * HEIGHT_STEP_METRES
        if height_step not in geometries:
            geometries[height_step] = geometry_at(world, height_step)
        clearances[number] = geometries[height_step].footprint_clearance(np.array([[east, north]]),
                                                                          math.radians(heading))[0]
    return clearances


def name_of_nearest_thing(world, sample):
    _, east, north, height, heading = sample
    geometry = geometry_at(world, round(height / HEIGHT_STEP_METRES) * HEIGHT_STEP_METRES)
    best_name, best_clearance = "nothing", math.inf
    for shape in list(geometry.shapes):
        geometry.shapes = [shape]
        clearance = geometry.footprint_clearance(np.array([[east, north]]), math.radians(heading))[0]
        if clearance < best_clearance:
            best_name = shape.name if not isinstance(shape, BlockingCircle) else f"{shape.name} (boulder)"
            best_clearance = clearance
    return best_name


def write_summary(run_folder, lines):
    with open(os.path.join(run_folder, "summary.txt"), "a", encoding="utf-8") as summary:
        for line in lines:
            summary.write(line + "\n")
    for line in lines:
        print(line)


if __name__ == "__main__":
    main()
