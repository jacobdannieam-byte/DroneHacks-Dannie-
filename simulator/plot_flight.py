"""Draws a recorded simulator flight, seen from above, on top of the course. SIMULATOR ONLY.

It reads one of the flight records that the referee writes, and saves a picture next to it.
The Mac may not have the Python libraries it needs, so the easy way is to run it inside the box,
from the top folder of the repository. If you give no file name, it uses the newest record:

    docker run --rm --volume "$PWD:/repository" drone-simulator python3 /repository/simulator/plot_flight.py

The outlined shapes are the course's obstacles at the height the drone flies at.
"""
import csv
import glob
import os
import sys

import matplotlib

matplotlib.use("Agg")  # Draw straight to a file, with no window.
import matplotlib.pyplot as plt  # noqa: E402  (must come after choosing the drawing method)
import numpy as np  # noqa: E402

FOLDER_OF_THIS_SCRIPT = os.path.dirname(os.path.abspath(__file__))
FLIGHT_RECORDS_FOLDER = os.path.join(FOLDER_OF_THIS_SCRIPT, "generated_files", "flight_records")
OBSTACLE_TABLE = os.path.join(FOLDER_OF_THIS_SCRIPT, "generated_files", "course_tables", "distance_to_obstacles.npy")

# These must match the course table's layout in course_model.py.
GRID_CELL_SIZE_METRES = 0.02
GRID_LOWEST_CORNER_METRES = (-3.10, -3.10, -0.10)

# The height of the slice through the course that we draw.
DRAWING_HEIGHT_METRES = 0.8

# A grid corner this close to an obstacle is drawn as part of it.
DRAWN_THICKNESS_METRES = 0.02

COURSE_EDGE_METRES = 3.048


def find_newest_record():
    records = sorted(glob.glob(os.path.join(FLIGHT_RECORDS_FOLDER, "flight_*.csv")))
    if not records:
        raise FileNotFoundError(f"No flight records in {FLIGHT_RECORDS_FOLDER}. Run a test flight first.")
    return records[-1]


def read_positions(path_to_record):
    x_values, y_values = [], []
    with open(path_to_record, newline="", encoding="utf-8") as record_file:
        for row in csv.DictReader(record_file):
            x_values.append(float(row["course_x_metres"]))
            y_values.append(float(row["course_y_metres"]))
    return x_values, y_values


def draw_course_slice(axes):
    distance_table = np.load(OBSTACLE_TABLE, mmap_mode="r")
    height_index = round((DRAWING_HEIGHT_METRES - GRID_LOWEST_CORNER_METRES[2]) / GRID_CELL_SIZE_METRES)
    obstacle_here = np.asarray(distance_table[:, :, height_index]) < DRAWN_THICKNESS_METRES

    table_width = obstacle_here.shape[0] * GRID_CELL_SIZE_METRES
    table_depth = obstacle_here.shape[1] * GRID_CELL_SIZE_METRES
    left, bottom = GRID_LOWEST_CORNER_METRES[0], GRID_LOWEST_CORNER_METRES[1]
    # The table is indexed by (x, y), but pictures are indexed by (row, column), so we swap.
    axes.imshow(obstacle_here.T, origin="lower", cmap="Greys", alpha=0.8,
                extent=(left, left + table_width, bottom, bottom + table_depth))


def main():
    path_to_record = sys.argv[1] if len(sys.argv) > 1 else find_newest_record()
    x_values, y_values = read_positions(path_to_record)

    figure, axes = plt.subplots(figsize=(8, 8))
    draw_course_slice(axes)
    axes.plot(x_values, y_values, color="tab:red", linewidth=2, label="True flight path")
    axes.plot(x_values[0], y_values[0], "o", color="tab:green", label="Start")
    axes.plot(x_values[-1], y_values[-1], "s", color="tab:blue", label="End")

    axes.set_xlim(-COURSE_EDGE_METRES, COURSE_EDGE_METRES)
    axes.set_ylim(-COURSE_EDGE_METRES, COURSE_EDGE_METRES)
    axes.set_aspect("equal")
    axes.set_xlabel("Course x (metres)")
    axes.set_ylabel("Course y (metres)")
    axes.set_title(os.path.basename(path_to_record))
    axes.legend()

    path_to_picture = path_to_record.replace(".csv", ".png")
    figure.savefig(path_to_picture, dpi=120)
    print(f"Saved the picture to {path_to_picture}")


if __name__ == "__main__":
    main()
