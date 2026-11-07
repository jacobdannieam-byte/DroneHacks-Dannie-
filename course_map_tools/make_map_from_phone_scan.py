"""Turns our phone scan of the real course into two course map files.

This runs on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.

How to run it, from the top folder of the repository, with the scan file copied into
simulator/generated_files/ (Git ignores that folder, so the scan is never published):

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 course_map_tools/make_map_from_phone_scan.py \\
        simulator/generated_files/dronehacks-course-scan-2026-11-06.ply

It takes about a minute. phone_scan_points.py reads the scan and puts it on the course's axes,
measure_phone_scan.py measures each shape, and this file decides which measurements to trust
and writes:

  phone_scan_2026-11-06.json
      Our best map. Walls we doubt are copied from the organizers' drawing instead.
  phone_scan_2026-11-06_trusting_every_wall.json
      Uses the scan for every wall. The route planner's tests use it as a "changed course".

Why some walls are copied from the drawing
------------------------------------------
A phone scan sometimes draws part of a room twice when the phone loses track of where it is.
Two of our scan's biggest differences from the drawing, A1 and C1, look like copies of that
kind, so we only trust them after checking a photo. DOUBTED_WALLS below gives the reason for
each. The third big difference, M3, is used from the scan; WALLS_TO_MEASURE says why.

Left out of both maps
---------------------
The C2b chicane wall: the scan has no points where the drawing puts it. Either it was not built
when we scanned, or the phone missed it. Ask the organizers.
The tunnel's cradles: they are under the tunnel, where the drone never flies.
"""
import dataclasses
import os
import sys

import numpy as np

from team_nav.course_map import CourseMap, load_course_map, save_course_map
from checkpoint_line_rules import make_checkpoint_lines
from measure_phone_scan import (
    find_low_objects, measure_tunnel, measure_wall, measure_window, where_lines_meet,
)
from phone_scan_points import read_scan_on_course_axes

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS_FOLDER = os.path.join(REPOSITORY, "drone_code", "team_nav", "course_maps")
DRAWING_MAP_FILE = os.path.join(MAPS_FOLDER, "organizers_drawing.json")
BEST_MAP_FILE = os.path.join(MAPS_FOLDER, "phone_scan_2026-11-06.json")
TRUSTING_MAP_FILE = os.path.join(MAPS_FOLDER, "phone_scan_2026-11-06_trusting_every_wall.json")
SCAN_DATE = "2026-11-06"

# Where to look for each wall, as (smallest x, largest x, smallest y, largest y) in metres on the
# course's axes. Our own choice, made by looking at pictures of the lined-up scan: each box holds
# one wall and nothing else. "start" and "end" say where each end of the wall is: "measured"
# means where the scan's points end; any other name means "extend the wall until it meets that",
# because the scan doesn't show clearly where a wall joins the net or another wall.
# Walls are measured in this order, so a wall can only be extended to meet one listed above it.
WALLS_TO_MEASURE = {
    # The barrier's two faces are scanned about 15 cm apart, at y = 0.24 m and 0.39 m. The box
    # leaves out a line of points about 20 cm further south, which belongs with the doubtful A1.
    # The scan's barrier stops about 15 cm short of the tunnel. That gap is far too narrow for
    # the drone, and the rulebook says the tunnel is the only way through, so it is closed.
    "Long barrier": {"box": (-3.0, 1.25, 0.19, 0.45), "start": "west net", "end": "tunnel's west side"},
    "M1 maze divider": {"box": (-2.05, -1.75, 0.55, 2.2), "start": "Long barrier", "end": "measured"},
    "M2 maze divider": {"box": (-0.8, -0.3, 1.45, 2.95), "start": "measured", "end": "north net"},
    "C2a chicane wall (window)": {"box": (-0.85, -0.1, -3.0, -1.35), "start": "south net", "end": "measured"},
    # We first doubted the scan's M3, which is about 70 cm east of the drawing's and turned by
    # 9 degrees. But the scan's M2, one of its clearest walls, is about 30 cm east of the
    # drawing's, and the drawing's M3 would leave only 65 cm between them, too narrow for the
    # drone. The scan's M3 keeps the three maze walls about evenly spaced (1.4 and 1.5 m apart,
    # where the drawing's are 1.1 m apart). So we use it, but its 9 degree turn may still be a
    # scan error: the tunnel's west side nearby is scanned with the same turn, although its east
    # side is straight. Check with a photo.
    "M3 maze divider": {"box": (0.6, 1.25, 0.65, 2.3), "start": "Long barrier", "end": "measured"},
    # These two are copied from the drawing in our best map. See DOUBTED_WALLS.
    "A1 approach wall": {"box": (-1.7, -1.35, -1.05, -0.1), "start": "measured", "end": "measured"},
    "C1 chicane wall": {"box": (0.85, 1.75, -1.35, 0.2), "start": "measured", "end": "measured"},
}
WINDOW_WALL = "C2a chicane wall (window)"

# Scanned walls we use, but whose source note in the map should carry a warning.
WARNINGS_FOR_SCANNED_WALLS = {
    "M3 maze divider": " It is turned by 9 degrees, which may be a scan error. Check with a photo.",
}

# Walls our best map copies from the drawing instead of the scan, and why.
DOUBTED_WALLS = {
    "A1 approach wall": (
        "Copied from the organizers' drawing. The scan shows this wall about 65 cm from where the "
        "drawing puts it, joined to an extra line of points just south of the barrier where nothing "
        "should be. The wall and the extra line are shifted by about the same amount, which suggests "
        "the phone lost track and drew them a second time."
    ),
    "C1 chicane wall": (
        "Copied from the organizers' drawing until a photo confirms it. The scan shows this wall slanted "
        "the other way (turned by about 36 degrees). Its points are smeared over about 30 cm, where the "
        "other walls show two sharp faces 10 cm apart, so the phone may have lost track here."
    ),
}


def main():
    if len(sys.argv) != 2:
        raise SystemExit("Give the path of the scan file, for example simulator/generated_files/dronehacks-course-scan-2026-11-06.ply.")
    drawing = load_course_map(DRAWING_MAP_FILE)
    points = read_scan_on_course_axes(sys.argv[1], drawing)

    tunnel = measure_tunnel(points, drawing.tubes[0])
    scanned_walls = measure_every_wall(points, drawing, tunnel)
    window_wall = scanned_walls[WINDOW_WALL]
    window_wall.openings = [measure_window(points, window_wall)]
    low_objects = find_low_objects(points)

    best_map = build_map(drawing, walls_for_best_map(scanned_walls, drawing), tunnel, low_objects,
        "Phone scan of the real course", (
            f"The course as our phone scan recorded it on {SCAN_DATE}. A1 and C1 are copied from the "
            "organizers' drawing until a photo confirms them, because the scan may have drawn them twice. "
            "M3 comes from the scan, but its exact angle needs a photo. "
            "C2b and the tunnel's cradles are left out because the scan does not show them. The course may "
            "have changed since the scan."
        ))
    trusting_map = build_map(drawing, list(scanned_walls.values()), tunnel, low_objects,
        "Phone scan, trusting every wall", (
            f"Like phone_scan_{SCAN_DATE}.json, but every wall comes from the scan, including A1 and C1, "
            "which may be copies the phone drew by mistake. Used to test the route planner on a changed course."
        ))
    save_course_map(best_map, BEST_MAP_FILE)
    save_course_map(trusting_map, TRUSTING_MAP_FILE)
    print(f"Saved {BEST_MAP_FILE} and {TRUSTING_MAP_FILE}.")


def measure_every_wall(points, drawing, tunnel):
    """Measures each wall in WALLS_TO_MEASURE and extends its ends as the table says."""
    lines_to_meet = net_side_lines(drawing)
    lines_to_meet.update(tunnel_side_lines(tunnel))
    scanned_walls = {}
    for name, plan in WALLS_TO_MEASURE.items():
        wall = measure_wall(points, name, plan["box"])
        wall.start_metres = extend_to(wall.start_metres, wall.direction(), plan["start"], lines_to_meet)
        wall.end_metres = extend_to(wall.end_metres, wall.direction(), plan["end"], lines_to_meet)
        wall.source += f" Its start is {describe_end(plan['start'])}; its end is {describe_end(plan['end'])}."
        wall.source += WARNINGS_FOR_SCANNED_WALLS.get(name, "")
        scanned_walls[name] = wall
        lines_to_meet[name] = (wall.start_metres, wall.direction())
        print(f"{name}: from ({wall.start_metres[0]:.2f}, {wall.start_metres[1]:.2f}) to "
              f"({wall.end_metres[0]:.2f}, {wall.end_metres[1]:.2f}), {wall.thickness_metres:.2f} m thick, "
              f"{wall.top_height_metres:.2f} m tall.")
    return scanned_walls


def walls_for_best_map(scanned_walls, drawing):
    walls = []
    for name, wall in scanned_walls.items():
        if name in DOUBTED_WALLS:
            walls.append(dataclasses.replace(drawing.wall_named(name), source=DOUBTED_WALLS[name]))
        else:
            walls.append(wall)
    return walls


def net_side_lines(drawing):
    """Each side of the safety net, as (a point on it, its direction)."""
    west, south = drawing.lowest_corner_metres
    east, north = drawing.highest_corner_metres
    north_direction = np.array([0.0, 1.0])
    east_direction = np.array([1.0, 0.0])
    return {
        "west net": (np.array([west, 0.0]), north_direction),
        "east net": (np.array([east, 0.0]), north_direction),
        "south net": (np.array([0.0, south]), east_direction),
        "north net": (np.array([0.0, north]), east_direction),
    }


def tunnel_side_lines(tunnel):
    """The outside of each of the tunnel's sides, as (a point on it, its direction)."""
    along = (tunnel.exit_centre_metres - tunnel.entrance_centre_metres) / tunnel.length_metres()
    to_the_left = np.array([-along[1], along[0]])
    half_width_metres = tunnel.outside_width_metres / 2.0
    left_side = tunnel.entrance_centre_metres + to_the_left * half_width_metres
    right_side = tunnel.entrance_centre_metres - to_the_left * half_width_metres
    if left_side[0] < right_side[0]:
        return {"tunnel's west side": (left_side, along), "tunnel's east side": (right_side, along)}
    return {"tunnel's west side": (right_side, along), "tunnel's east side": (left_side, along)}


def extend_to(end_point, wall_direction, target, lines_to_meet):
    if target == "measured":
        return end_point
    if target not in lines_to_meet:
        raise SystemExit(f"A wall is meant to meet '{target}', which is not a net side, the tunnel, or a wall measured before it. Check WALLS_TO_MEASURE.")
    line_point, line_direction = lines_to_meet[target]
    return where_lines_meet(end_point, wall_direction, line_point, line_direction)


def describe_end(target):
    if target == "measured":
        return "where the scan's points end"
    return f"extended to meet the {target}"


def build_map(drawing, walls, tunnel, low_objects, name, description):
    course_map = CourseMap(
        name=name,
        description=description,
        lowest_corner_metres=drawing.lowest_corner_metres,
        highest_corner_metres=drawing.highest_corner_metres,
        wall_top_height_metres=drawing.wall_top_height_metres,
        edges_source=(
            "From the organizers' drawing. The scanned net lines up with the drawing's net to within "
            "about 10 cm; a net sags, so it can't be placed more closely than that."
        ),
        launch_mark_centre_metres=drawing.launch_mark_centre_metres,
        launch_mark_source="From the organizers' drawing. The scan does not show floor markings clearly.",
        landing_pad_centre_metres=drawing.landing_pad_centre_metres,
        landing_pad_outer_radius_metres=drawing.landing_pad_outer_radius_metres,
        approach_ring_radius_metres=drawing.approach_ring_radius_metres,
        landing_pad_source="From the organizers' drawing. The scan does not show floor markings clearly.",
        walls=walls,
        tubes=[tunnel],
        low_obstacles=low_objects,
    )
    course_map.checkpoint_lines = make_checkpoint_lines(course_map)
    return course_map


if __name__ == "__main__":
    main()
