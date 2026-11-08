"""Draws one full-Gazebo run seen from above, and writes its short summary. SIMULATOR ONLY.

What it reads, all from one run folder made by fly_course_in_gazebo.sh:
  - referee/flight_*.csv and *_summary.json: the referee's record. The true path comes from
    here: PX4 in Gazebo reports the drone's true position from Gazebo's own physics, and the
    referee puts it in course positions. The referee also counts touches: a touch is any moment
    when a propeller comes within 1 cm of an obstacle of the WORLD map (see the simulator's
    referee.py).
  - course_run_records/*_course_run/: course_run's own records. drone_condition.csv holds the
    camera tracking's position as the drone saw it, and targets.csv every target it sent. Both
    are in the "start frame": forward, left and up from where the drone sat on the launch mark.
  - flight_program_exit_code.txt, camera_rate_*.txt, stereo_check/stereo_depth_check.txt,
    camera_stamps.csv (record_camera_stamps.py) and course_run_records/*_depth_camera/
    depth_camera.csv: the picture and depth rates during the flight.

What it writes into the run folder:
  - path_from_above.png: the world map's walls, tunnel and boulders, the true path, the camera
    tracking path, the targets, every touch, and, if course_run saved live maps
    (course_run_records/*/live_maps/), the squares it believed blocked at the end and its last
    planned path.
  - summary.txt: did it finish, touches, times, the simulation speed and the camera rate.

Run it inside the drone-simulator-gazebo-full box (fly_course_in_gazebo.sh does):

    python3 /repository/simulator/gazebo_full/draw_run.py RUN_FOLDER WORLD_MAP_FILE

What it does not do: it does not judge the run. The referee does that during the flight.
"""
import csv
import glob
import json
import math
import os
import sys
import textwrap

import matplotlib

matplotlib.use("Agg")  # Draw straight to a file, with no window.
import matplotlib.pyplot as plt  # noqa: E402  (must come after choosing the drawing method)
import numpy as np  # noqa: E402
from matplotlib.patches import Circle, Polygon  # noqa: E402

FOLDER_OF_THIS_FILE = os.path.dirname(os.path.abspath(__file__))
REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(FOLDER_OF_THIS_FILE))
sys.path.insert(0, os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav"))
from team_nav.course_map import load_course_map  # noqa: E402

# From the simulator's referee.py: a propeller this close to an obstacle is a touch.
TOUCHING_DISTANCE_METRES = 0.01
# From the simulator's referee.py: contact within this time of the last counts as the same touch.
SECONDS_OF_CLEARANCE_BEFORE_NEXT_TOUCH_COUNTS = 1.0

# course_run's default, and the Gazebo world's: the nose points 90 degrees from the course's x
# axis, down the first corridor. course_run records the value it used, and we read that first.
DEFAULT_START_HEADING_DEGREES = 90.0

# From drone_code/team_nav (clearance_grid.py and the live map files): the live map's squares
# are 5 cm, and a score above 1.0 means the drone believes the square is blocked.
LIVE_MAP_SQUARE_METRES = 0.05
LIVE_MAP_BLOCKED_SCORE = 1.0

# Our own choice of colours.
WALL_COLOUR = "#5f5e5a"
TUNNEL_COLOUR = "#7f9cc7"
BOULDER_COLOUR = "#9c7b5b"
TRUE_PATH_COLOUR = "#1b6ac9"
TRACKED_PATH_COLOUR = "#e07b00"
TARGET_COLOUR = "#2a9d4b"
TOUCH_COLOUR = "#d62828"
LIVE_MAP_COLOUR = (0.85, 0.15, 0.55)
LIVE_PATH_COLOUR = "#8e44ad"


# ---- Reading the records ---------------------------------------------------------------------

def read_table(path):
    with open(path, newline="", encoding="utf-8") as table_file:
        return list(csv.DictReader(table_file))


def number_or_none(text):
    if text in (None, ""):
        return None
    return float(text)


def newest_file(pattern):
    matches = sorted(glob.glob(pattern))
    return matches[-1] if matches else None


def read_text_or(path, missing_text):
    if not os.path.exists(path):
        return missing_text
    with open(path, encoding="utf-8") as text_file:
        return text_file.read().strip()


def course_run_folder(run_folder):
    return newest_file(os.path.join(run_folder, "course_run_records", "*_course_run"))


def course_run_events(folder):
    events = []
    if folder is None:
        return events
    with open(os.path.join(folder, "events.jsonl"), encoding="utf-8") as events_file:
        for line in events_file:
            events.append(json.loads(line))
    return events


def start_heading_degrees(events):
    for event in events:
        if event["kind"] == "start_heading":
            return float(event["start_heading_degrees"])
    return DEFAULT_START_HEADING_DEGREES


# ---- Turning start-frame positions into course positions -------------------------------------

def start_frame_to_course(forward_metres, left_metres, launch_mark_metres, heading_degrees):
    """The start frame's x is the nose's direction at the start, y is to its left. Both are
    turned by the start heading and moved to the launch mark."""
    heading_radians = math.radians(heading_degrees)
    course_x = launch_mark_metres[0] + forward_metres * math.cos(heading_radians) - left_metres * math.sin(heading_radians)
    course_y = launch_mark_metres[1] + forward_metres * math.sin(heading_radians) + left_metres * math.cos(heading_radians)
    return course_x, course_y


def tracked_path_on_course(folder, launch_mark_metres, heading_degrees):
    xs, ys = [], []
    if folder is None or not os.path.exists(os.path.join(folder, "drone_condition.csv")):
        return xs, ys
    for row in read_table(os.path.join(folder, "drone_condition.csv")):
        forward = number_or_none(row.get("tracked_forward_metres"))
        left = number_or_none(row.get("tracked_left_metres"))
        if forward is None or left is None:
            continue
        x, y = start_frame_to_course(forward, left, launch_mark_metres, heading_degrees)
        xs.append(x)
        ys.append(y)
    return xs, ys


def position_targets_on_course(folder, launch_mark_metres, heading_degrees):
    xs, ys = [], []
    if folder is None or not os.path.exists(os.path.join(folder, "targets.csv")):
        return xs, ys
    for row in read_table(os.path.join(folder, "targets.csv")):
        if row["kind"] != "position":
            continue
        x, y = start_frame_to_course(float(row["forward_or_speed"]), float(row["left_or_speed"]),
                                     launch_mark_metres, heading_degrees)
        xs.append(x)
        ys.append(y)
    return xs, ys


def newest_live_map(folder):
    """The last live map course_run saved, and the last planned path in any of them, both in
    course positions. Either is None if there is none. The live map is what the drone believed
    about the course at the end, built from its own depth pictures."""
    if folder is None:
        return None, None
    map_files = sorted(glob.glob(os.path.join(folder, "live_maps", "live_map_*.npz")))
    if not map_files:
        return None, None
    last_map = np.load(map_files[-1])
    last_path = None
    for map_file in map_files:
        path_metres = np.load(map_file)["path_metres"]
        if len(path_metres) > 0:
            last_path = path_metres
    return last_map, last_path


def touches_from_referee_record(rows):
    """Each touch as (seconds since the record began, x, y, height), using the referee's rule."""
    touches = []
    last_contact_seconds = None
    first_seconds = float(rows[0]["seconds"]) if rows else 0.0
    for row in rows:
        if float(row["nearest_obstacle_metres"]) >= TOUCHING_DISTANCE_METRES:
            continue
        seconds = float(row["seconds"])
        is_new = last_contact_seconds is None or seconds - last_contact_seconds > SECONDS_OF_CLEARANCE_BEFORE_NEXT_TOUCH_COUNTS
        last_contact_seconds = seconds
        if is_new:
            touches.append((seconds - first_seconds, float(row["course_x_metres"]), float(row["course_y_metres"]),
                            float(row["height_metres"])))
    return touches


# ---- Drawing -----------------------------------------------------------------------------------

def strip_corners(start_metres, end_metres, thickness_metres):
    """The four corners, seen from above, of a straight strip of this thickness."""
    direction = (end_metres - start_metres) / np.linalg.norm(end_metres - start_metres)
    across = np.array([-direction[1], direction[0]]) * thickness_metres / 2.0
    return [start_metres - across, end_metres - across, end_metres + across, start_metres + across]


def draw_world_map(axes, world_map):
    low_x, low_y = world_map.lowest_corner_metres
    high_x, high_y = world_map.highest_corner_metres
    axes.plot([low_x, high_x, high_x, low_x, low_x], [low_y, low_y, high_y, high_y, low_y], color=WALL_COLOUR, linewidth=2)
    for wall in world_map.walls:
        axes.add_patch(Polygon(strip_corners(wall.start_metres, wall.end_metres, wall.thickness_metres),
                               facecolor=WALL_COLOUR, edgecolor="none", alpha=0.85))
    for tube in world_map.tubes:
        corners = strip_corners(tube.entrance_centre_metres, tube.exit_centre_metres, tube.outside_width_metres)
        axes.add_patch(Polygon(corners, facecolor=TUNNEL_COLOUR, edgecolor=TUNNEL_COLOUR, alpha=0.35))
    for obstacle in world_map.low_obstacles:
        axes.add_patch(Circle(obstacle.centre_metres, obstacle.radius_metres, facecolor=BOULDER_COLOUR, alpha=0.6))
    pad = world_map.landing_pad_centre_metres
    axes.add_patch(Circle(pad, world_map.landing_pad_outer_radius_metres, facecolor="none", edgecolor="#888", linestyle="--"))
    axes.plot(*world_map.launch_mark_centre_metres, marker="s", markersize=8, color="#444", label="launch mark")


def draw_live_map(axes, live_map, live_path):
    """The squares the drone believed blocked, and its last planned path."""
    if live_map is not None:
        # scores[east_index, north_index], so it is turned to rows of north for the picture.
        blocked = (live_map["scores"] > LIVE_MAP_BLOCKED_SCORE).T
        low_x, low_y = live_map["lowest_corner_metres"]
        north_count, east_count = blocked.shape
        overlay = np.zeros((north_count, east_count, 4))
        overlay[blocked] = (*LIVE_MAP_COLOUR, 0.35)
        axes.imshow(overlay, origin="lower", interpolation="nearest", zorder=1,
                    extent=(low_x, low_x + east_count * LIVE_MAP_SQUARE_METRES,
                            low_y, low_y + north_count * LIVE_MAP_SQUARE_METRES))
        axes.scatter([], [], marker="s", color=LIVE_MAP_COLOUR, alpha=0.35, label="live map: blocked (drone's belief, at the end)")
    if live_path is not None:
        axes.plot(live_path[:, 0], live_path[:, 1], color=LIVE_PATH_COLOUR, linewidth=1.2, linestyle="--",
                  label="last live path planned", zorder=4)


def draw_picture(path, world_map, true_path, tracked_path, targets, touches, title, live_map=None, live_path=None):
    figure, axes = plt.subplots(figsize=(9, 10.5))
    draw_world_map(axes, world_map)
    draw_live_map(axes, live_map, live_path)
    if targets[0]:
        axes.scatter(targets[0], targets[1], s=6, color=TARGET_COLOUR, label="targets sent by course_run", zorder=3)
    if tracked_path[0]:
        axes.plot(tracked_path[0], tracked_path[1], color=TRACKED_PATH_COLOUR, linewidth=1.2,
                  label="camera tracking path (what the drone believed)", zorder=4)
    if true_path[0]:
        axes.plot(true_path[0], true_path[1], color=TRUE_PATH_COLOUR, linewidth=1.6,
                  label="true path (Gazebo)", zorder=5)
    for number, (_, x, y, _) in enumerate(touches):
        axes.scatter([x], [y], s=120, marker="x", color=TOUCH_COLOUR, linewidths=3, zorder=6,
                     label="touch" if number == 0 else None)
    axes.set_aspect("equal")
    axes.set_xlabel("course x, metres")
    axes.set_ylabel("course y, metres")
    axes.set_title(title, fontsize=10)
    # Below the course, so it never hides part of the path.
    axes.legend(loc="upper center", bbox_to_anchor=(0.5, -0.07), ncol=2, fontsize=8)
    axes.grid(alpha=0.2)
    figure.tight_layout()
    figure.savefig(path, dpi=120)
    plt.close(figure)


# ---- Camera and depth rates in flight --------------------------------------------------------

def flight_window_seconds(referee_rows):
    """The first and last wall-clock time at which the referee saw automatic flight (Offboard),
    or None. The referee's "seconds" are this computer's clock, like the picture stamps."""
    flying_times = [float(row["seconds"]) for row in referee_rows if row["automatic_flight"] == "True"]
    if not flying_times:
        return None
    return min(flying_times), max(flying_times)


def rate_per_second(times, window):
    inside = [time_seconds for time_seconds in times if window[0] <= time_seconds <= window[1]]
    duration_seconds = window[1] - window[0]
    return len(inside) / duration_seconds if duration_seconds > 0 else 0.0


def camera_lines_in_flight(run_folder, course_run_records_folder, window):
    """Picture rates of each eye, the left-right stamp gap and the team's depth rate, in flight."""
    if window is None:
        return ["In flight: the drone never flew automatically, so nothing was measured."]
    lines = [f"In flight: {window[1] - window[0]:.0f} s of automatic flight."]
    stamps_file = os.path.join(run_folder, "camera_stamps.csv")
    if os.path.exists(stamps_file):
        rows = read_table(stamps_file)
        left = np.array([float(row["stamp_seconds"]) for row in rows if row["eye"] == "left"])
        right = np.array([float(row["stamp_seconds"]) for row in rows if row["eye"] == "right"])
        lines.append(f"In flight: left pictures {rate_per_second(left, window):.1f}/s, "
                     f"right pictures {rate_per_second(right, window):.1f}/s (arrival stamps).")
        if len(left) and len(right):
            # For each left picture, the nearest right picture's stamp. The eyes are drawn at the
            # same simulated moment; the gap is how far apart their pictures reach ROS.
            right_sorted = np.sort(right)
            positions = np.clip(np.searchsorted(right_sorted, left), 1, len(right_sorted) - 1)
            gaps = np.minimum(np.abs(left - right_sorted[positions - 1]), np.abs(left - right_sorted[positions]))
            lines.append(f"Left-right stamp gap: median {1000 * np.median(gaps):.1f} ms, "
                         f"95th percentile {1000 * np.percentile(gaps, 95):.1f} ms.")
    depth_file = newest_file(os.path.join(run_folder, "course_run_records", "*_depth_camera", "depth_camera.csv"))
    if depth_file:
        rows = read_table(depth_file)
        times = [float(row["picture_time_seconds"]) for row in rows]
        in_flight = [row for row in rows if window[0] <= float(row["picture_time_seconds"]) <= window[1]]
        calculation = [float(row["calculation_seconds"]) for row in in_flight]
        median_text = f"{np.median(calculation):.2f} s" if calculation else "none"
        lines.append(f"In flight: team depth_camera {rate_per_second(times, window):.1f} depth pictures/s, "
                     f"median calculation {median_text} each.")
    if course_run_records_folder:
        summary_file = os.path.join(course_run_records_folder, "live_navigation_summary.json")
        if os.path.exists(summary_file):
            live_summary = json.load(open(summary_file, encoding="utf-8"))
            lines.append(f"Live navigation: {json.dumps(live_summary)}")
    return lines


# ---- The summary -----------------------------------------------------------------------------

def camera_rate_line(run_folder, side):
    text = read_text_or(os.path.join(run_folder, f"camera_rate_{side}.txt"), "")
    rates = [line.strip() for line in text.splitlines() if "average rate" in line]
    return rates[-1] if rates else "not measured"


def summary_lines(run_folder, world_map_file, referee_summary, touches, events, true_path_length_metres):
    exit_code = read_text_or(os.path.join(run_folder, "flight_program_exit_code.txt"), "unknown (the run was stopped early)")
    # "result" is a finished run; "stop" is course_run stopping the run itself, for example after
    # a suspected touch; "refused" and "error" end it before the flight.
    result_events = [event["text"] for event in events if event["kind"] in ("result", "stop", "refused", "error")]
    lines = [
        f"World map: {world_map_file}",
        f"course_run exit code: {exit_code}",
        f"course_run's own verdict: {result_events[-1] if result_events else 'none recorded'}",
    ]
    if referee_summary:
        lines += [
            f"Referee: checkpoints passed {referee_summary.get('checkpoint_seconds')}",
            f"Referee: landed on the pad: {referee_summary.get('landed_on_pad')}, motors off at "
            f"{referee_summary.get('motors_off_seconds')} s after takeoff",
            f"Referee: touches {referee_summary.get('touches')}, penalty {referee_summary.get('penalty_seconds')} s",
            f"Referee: went above the walls {referee_summary.get('went_above_walls')}, left the course "
            f"{referee_summary.get('left_the_course')}",
            f"Referee: largest PX4 height error {referee_summary.get('largest_height_estimate_error_metres')} m",
            f"Simulation speed (PX4 clock seconds per real second): "
            f"{referee_summary.get('px4_clock_speed_compared_with_real_time')}",
        ]
    else:
        lines.append("Referee: no summary written, so the simulator probably stopped before the referee started.")
    for number, (seconds, x, y, height) in enumerate(touches):
        lines.append(f"Touch {number + 1}: {seconds:.1f} s into the record, at ({x:.2f}, {y:.2f}), {height:.2f} m up")
    lines += [
        f"True path length: {true_path_length_metres:.1f} m",
        f"Left camera rate (ros2 topic hz): {camera_rate_line(run_folder, 'left')}",
        f"Right camera rate (ros2 topic hz): {camera_rate_line(run_folder, 'right')}",
    ]
    stereo_text = read_text_or(os.path.join(run_folder, "stereo_check", "stereo_depth_check.txt"), "")
    stereo_lines = [line for line in stereo_text.splitlines() if line.startswith(("Grid points", "Pixels with"))]
    lines += [f"Stereo check: {line}" for line in stereo_lines] or ["Stereo check: not run"]
    return lines


def main():
    if len(sys.argv) != 3:
        print("Give the run folder and the world map file, for example: draw_run.py RUN_FOLDER WORLD_MAP_FILE")
        sys.exit(2)
    run_folder, world_map_file = sys.argv[1], sys.argv[2]
    world_map = load_course_map(world_map_file)

    referee_record = newest_file(os.path.join(run_folder, "referee", "flight_*[0-9].csv"))
    referee_rows = read_table(referee_record) if referee_record else []
    referee_summary_file = newest_file(os.path.join(run_folder, "referee", "flight_*_summary.json"))
    referee_summary = json.load(open(referee_summary_file, encoding="utf-8")) if referee_summary_file else None

    folder = course_run_folder(run_folder)
    events = course_run_events(folder)
    heading_degrees = start_heading_degrees(events)
    launch_mark = world_map.launch_mark_centre_metres

    true_path = ([float(row["course_x_metres"]) for row in referee_rows],
                 [float(row["course_y_metres"]) for row in referee_rows])
    tracked_path = tracked_path_on_course(folder, launch_mark, heading_degrees)
    targets = position_targets_on_course(folder, launch_mark, heading_degrees)
    touches = touches_from_referee_record(referee_rows)
    true_path_length_metres = float(np.sum(np.hypot(np.diff(true_path[0]), np.diff(true_path[1])))) if referee_rows else 0.0

    lines = summary_lines(run_folder, world_map_file, referee_summary, touches, events, true_path_length_metres)
    lines += camera_lines_in_flight(run_folder, folder, flight_window_seconds(referee_rows))
    verdict = textwrap.fill(lines[2], width=110)
    title = f"{os.path.basename(run_folder.rstrip('/'))}: {world_map.name}\n{verdict}"
    live_map, live_path = newest_live_map(folder)
    draw_picture(os.path.join(run_folder, "path_from_above.png"), world_map, true_path, tracked_path, targets, touches,
                 title, live_map, live_path)
    with open(os.path.join(run_folder, "summary.txt"), "w", encoding="utf-8") as summary_file:
        summary_file.write("\n".join(lines) + "\n")


if __name__ == "__main__":
    main()
