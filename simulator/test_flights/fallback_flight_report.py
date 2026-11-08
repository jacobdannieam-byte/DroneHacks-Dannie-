"""Reports what happened at every wall crossing of a fallback-mission simulator flight: the drone's
speed, its true height, PX4's height estimate and the bottom laser's reading, and how the flight
ended. SIMULATOR TEST CODE; nothing here runs on the drone.

Run it after a flight, from the top folder of the repository:

    python3 simulator/test_flights/fallback_flight_report.py --world <the world's map file>

It reads the newest referee record (simulator/generated_files/flight_records/flight_*.csv and its
summary) and the newest course_run records folder (simulator/generated_files/team_runs/), unless
--referee-record and --team-run name others. Crossings are found from the drone's true path
against the world's walls, so they are right even when the drone's map differs from the world.

For each crossing it prints, for the 3 seconds before and after:
  - the true sideways speed where the drone crossed the wall's centre line;
  - the true height there, and PX4's estimate's largest error in that window;
  - the laser's raw reading at its lowest in that window (it drops to about 0.5 m over a wall top),
    and whether PX4 reported it stopped using the laser in that window.
The referee's clock and course_run's clock differ, so course_run's rows are matched to the
crossing by position (the camera tracking's position nearest the crossing point).
"""
import argparse
import csv
import glob
import json
import os
import sys

import numpy as np

REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPOSITORY, "drone_code", "team_nav"))
from team_nav.course_map import line_pieces_cross, load_course_map  # noqa: E402
from team_nav.start_frame import StartFrame  # noqa: E402

# Our own choices.
WINDOW_SECONDS = 3.0
START_HEADING_DEGREES = 90.0      # course_run's default (course_run.py).
# The referee writes about 20 rows a second; speeds are measured over this many rows either side.
SPEED_ROWS_EITHER_SIDE = 3


def main():
    arguments = parse_arguments()
    world = load_course_map(arguments.world)
    referee_rows = read_referee_record(arguments.referee_record)
    team_rows = read_csv(os.path.join(arguments.team_run, "drone_condition.csv"))
    print(f"World: {world.name}\nReferee record: {arguments.referee_record}\nTeam run: {arguments.team_run}\n")
    print_how_it_ended(arguments, world, referee_rows)
    for crossing in crossings_on_true_path(world, referee_rows):
        print_crossing(crossing, referee_rows, team_rows, world)


def parse_arguments():
    parser = argparse.ArgumentParser(description="Report a fallback flight's wall crossings.")
    parser.add_argument("--world", required=True)
    parser.add_argument("--referee-record", default=newest("simulator/generated_files/flight_records/flight_*[0-9].csv"))
    parser.add_argument("--team-run", default=newest("simulator/generated_files/team_runs/*course_run"))
    return parser.parse_args()


def newest(pattern):
    found = sorted(glob.glob(os.path.join(REPOSITORY, pattern)))
    return found[-1] if found else None


def read_csv(path):
    with open(path, encoding="utf-8") as opened:
        return list(csv.DictReader(opened))


def number(text):
    try:
        return float(text)
    except (TypeError, ValueError):
        return None


def read_referee_record(path):
    rows = []
    for row in read_csv(path):
        rows.append({"seconds": float(row["seconds"]), "east": float(row["course_x_metres"]),
                     "north": float(row["course_y_metres"]), "height": float(row["height_metres"]),
                     "px4_height": number(row["px4_estimated_height_metres"]), "armed": row["armed"] == "True"})
    return rows


# ---- How it ended --------------------------------------------------------------------------------

def print_how_it_ended(arguments, world, referee_rows):
    summary_path = arguments.referee_record.replace(".csv", "_summary.json")
    summary = json.load(open(summary_path, encoding="utf-8")) if os.path.exists(summary_path) else {}
    flying = [row for row in referee_rows if row["height"] > 0.05]
    last = referee_rows[-1]
    pad_distance = float(np.hypot(last["east"] - world.landing_pad_centre_metres[0],
                                  last["north"] - world.landing_pad_centre_metres[1]))
    print(f"Touches: {summary.get('touches')}. Went above the wall tops: {summary.get('went_above_walls', 'see summary')}.")
    if flying:
        print(f"Flying time (true height above 5 cm): {flying[-1]['seconds'] - flying[0]['seconds']:.1f} s.")
    on_pad = pad_distance <= world.landing_pad_outer_radius_metres
    print(f"Ended {pad_distance:.2f} m from the pad's centre ({'ON' if on_pad else 'NOT ON'} the pad, whose radius is "
          f"{world.landing_pad_outer_radius_metres:.2f} m), height {last['height']:.2f} m, armed {last['armed']}.")
    for line in team_result_lines(arguments.team_run):
        print(line)
    print()


def team_result_lines(team_run):
    lines = []
    with open(os.path.join(team_run, "events.jsonl"), encoding="utf-8") as opened:
        for text in opened:
            event = json.loads(text)
            if event.get("kind") in ("result", "stop", "error", "control_lost", "exit", "refused"):
                lines.append(f"course_run: {event.get('text', '')[:300]}")
    return lines


# ---- Crossings -----------------------------------------------------------------------------------

def crossings_on_true_path(world, rows):
    """(wall name, row index) wherever the true path crosses a wall's centre line."""
    found = []
    for index in range(1, len(rows)):
        before = np.array([rows[index - 1]["east"], rows[index - 1]["north"]])
        after = np.array([rows[index]["east"], rows[index]["north"]])
        if np.allclose(before, after):
            continue
        for wall in world.walls:
            if line_pieces_cross(before, after, wall.start_metres, wall.end_metres):
                found.append((wall.name, index))
    return found


def print_crossing(crossing, referee_rows, team_rows, world):
    wall_name, index = crossing
    row = referee_rows[index]
    window = [other for other in referee_rows if abs(other["seconds"] - row["seconds"]) <= WINDOW_SECONDS]
    errors = [abs(other["px4_height"] - other["height"]) for other in window if other["px4_height"] is not None]
    print(f"{wall_name}, {row['seconds'] - referee_rows[0]['seconds']:.1f} s into the record, at "
          f"({row['east']:+.2f}, {row['north']:+.2f}):")
    print(f"  true speed {true_speed(referee_rows, index):.2f} m/s, true height {row['height']:.2f} m, PX4's height "
          f"{row['px4_height']:.2f} m; PX4's largest error within {WINDOW_SECONDS:.0f} s: {max(errors):.2f} m")
    print("  " + laser_text(team_rows, np.array([row["east"], row["north"]]), world))


def true_speed(rows, index):
    first, last = max(index - SPEED_ROWS_EITHER_SIDE, 0), min(index + SPEED_ROWS_EITHER_SIDE, len(rows) - 1)
    distance = np.hypot(rows[last]["east"] - rows[first]["east"], rows[last]["north"] - rows[first]["north"])
    return float(distance / (rows[last]["seconds"] - rows[first]["seconds"]))


def laser_text(team_rows, crossing_point, world):
    start_frame = StartFrame(world.launch_mark_centre_metres, START_HEADING_DEGREES)
    nearest_index, nearest_distance = None, np.inf
    for index, row in enumerate(team_rows):
        forward, left = number(row["from_launch_forward_metres"]), number(row["from_launch_left_metres"])
        if forward is None or number(row["from_launch_up_metres"]) is None or number(row["from_launch_up_metres"]) < 1.0:
            continue
        on_course = start_frame.course_from_start_frame((forward, left, 0.0))[:2]
        distance = float(np.linalg.norm(on_course - crossing_point))
        if distance < nearest_distance:
            nearest_index, nearest_distance = index, distance
    if nearest_index is None:
        return "course_run's records have no high position near this crossing."
    time = float(team_rows[nearest_index]["seconds_since_start"])
    window = [row for row in team_rows if abs(float(row["seconds_since_start"]) - time) <= WINDOW_SECONDS]
    lasers = [number(row["raw_laser_metres"]) for row in window if number(row["raw_laser_metres"]) is not None]
    gaps = [abs(number(row["px4_height_risen_metres"]) - number(row["from_launch_up_metres"])) for row in window
            if number(row["px4_height_risen_metres"]) is not None and number(row["from_launch_up_metres"]) is not None]
    stopped = any(row["px4_uses_laser_for_height"] == "False" for row in window)
    return (f"laser: lowest raw reading {min(lasers):.2f} m, highest {max(lasers):.2f} m; PX4 vs camera height largest "
            f"gap {max(gaps):.2f} m; PX4 reported it stopped using the laser: {stopped}"
            if lasers else "laser: no raw readings in the window.")


if __name__ == "__main__":
    main()
