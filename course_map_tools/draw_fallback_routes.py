"""Draws the two fallback routes (high over the walls) on top views of the course, so the team can
check them by eye.

This runs on a laptop, inside the simulator's Linux box, which has the drawing library
(matplotlib). Nothing here runs on the drone. From the top folder of the repository:

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 course_map_tools/draw_fallback_routes.py

It saves one picture per case in simulator/generated_files/fallback_route_pictures/ (Git ignores
that folder): both routes on our phone scan, on the organizers' drawing and on their changed course of
2026-11-08, and the tunnel route
planned on the scan drawn over the changed maze worlds that the simulator flights use
(simulator/test_flights/make_fallback_test_worlds.py), to show that it does not depend on the maze walls.

What a picture shows, seen from above with north up, on top of draw_planned_routes.py's course
drawing (walls, the tunnel, low obstacles, checkpoint lines):
  - The route in blue. Small dots are waypoints flown straight through (the arcs); large open
    circles are stops, which are only where the height changes and at the pad.
  - The height written where it changes: the climb on the launch mark, the descent before the
    tunnel and the climb after it.
  - Each wall crossing as an orange cross, with how far off square it is.
  - The rule check's cautions in the title. A rule break would also be listed there.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")   # Draw straight to files; the box has no screen.
import matplotlib.pyplot as plt
import numpy as np

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPOSITORY, "course_map_tools"))
sys.path.insert(0, os.path.join(REPOSITORY, "simulator", "test_flights"))
import draw_planned_routes as drawing  # noqa: E402  (must come after the lines above)
from make_fallback_test_worlds import changed_maze_worlds  # noqa: E402
from our_files import CHANGED_COURSE_MAP, DRAWING_MAP, SCAN_MAP, load_our_map  # noqa: E402  (draw_planned_routes adds its folder)
from team_nav.fallback_routes import FALLBACK_PAD, FALLBACK_TUNNEL, HIGH_HEIGHT_METRES, plan_fallback_route  # noqa: E402
from team_nav.high_path_rules import check_high_path  # noqa: E402
from team_nav.stopping_spots import find_stopping_spots  # noqa: E402

PICTURES_FOLDER = os.path.join(REPOSITORY, "simulator", "generated_files", "fallback_route_pictures")
CROSSING_COLOUR = drawing.CHECKPOINT_LINE_COLOUR
# For drawing only: walls are drawn dark as at the normal 0.8 m cruise height, so they stand out,
# even though the fallback routes fly above all of them.
WALL_DRAWING_HEIGHT_METRES = 0.8


def main():
    os.makedirs(PICTURES_FOLDER, exist_ok=True)
    scan = load_our_map(SCAN_MAP)
    cases = [
        ("1_fallback_pad_phone_scan", scan, scan, FALLBACK_PAD),
        ("2_fallback_tunnel_phone_scan", scan, scan, FALLBACK_TUNNEL),
        ("3_fallback_pad_organizers_drawing", load_our_map(DRAWING_MAP), load_our_map(DRAWING_MAP), FALLBACK_PAD),
        ("4_fallback_tunnel_organizers_drawing", load_our_map(DRAWING_MAP), load_our_map(DRAWING_MAP), FALLBACK_TUNNEL),
    ]
    changed_course = load_our_map(CHANGED_COURSE_MAP)
    cases += [("8_fallback_pad_changed_course_2026-11-08", changed_course, changed_course, FALLBACK_PAD),
              ("9_fallback_tunnel_changed_course_2026-11-08", changed_course, changed_course, FALLBACK_TUNNEL)]
    for number, (world_name, world) in enumerate(changed_maze_worlds(scan)):
        cases.append((f"{5 + number}_fallback_tunnel_scan_route_over_{world_name}", world, scan, FALLBACK_TUNNEL))
    for file_name, world, drone_map, route_name in cases:
        path = os.path.join(PICTURES_FOLDER, f"{file_name}.png")
        draw_one_case(world, drone_map, route_name, path)
        print(f"Saved {path}")


def draw_one_case(world, drone_map, route_name, path):
    """Draws the route planned on drone_map over the course world (usually the same map)."""
    figure, axes = plt.subplots(figsize=(drawing.PICTURE_SIZE_INCHES, drawing.PICTURE_SIZE_INCHES),
                                dpi=drawing.DOTS_PER_INCH)
    figure.patch.set_facecolor(drawing.BACKGROUND_COLOUR)
    axes.set_facecolor(drawing.BACKGROUND_COLOUR)
    drawing.draw_course(axes, world, WALL_DRAWING_HEIGHT_METRES)
    route = plan_fallback_route(drone_map, route_name)
    draw_route(axes, route)
    for high_stretch in stretches_at_the_high_height(route.waypoints_metres):
        for crossing in check_high_path(world, high_stretch, HIGH_HEIGHT_METRES).crossings:
            draw_crossing(axes, crossing)
    drawing.draw_title(axes, f"{route_name} on {drawing.plain_words(world.name)}", describe(route, world, drone_map))
    drawing.set_up_axes(axes, world)
    figure.savefig(path, facecolor=drawing.BACKGROUND_COLOUR, bbox_inches="tight")
    plt.close(figure)


def stretches_at_the_high_height(waypoints):
    """The (east, north) points of each stretch of the route flown at HIGH_HEIGHT_METRES."""
    stretches, current = [], []
    for waypoint in waypoints:
        if np.isclose(waypoint[2], HIGH_HEIGHT_METRES):
            current.append(waypoint[:2])
        elif current:
            stretches.append(current)
            current = []
    if current:
        stretches.append(current)
    return [stretch for stretch in stretches if len(stretch) > 1]


def draw_route(axes, route):
    waypoints = route.waypoints_metres
    colour = drawing.ROUTE_COLOUR
    axes.plot(waypoints[:, 0], waypoints[:, 1], color=colour, linewidth=2.5, solid_capstyle="round", zorder=5)
    axes.plot(waypoints[:, 0], waypoints[:, 1], linestyle="none", marker="o", markersize=3,
              markerfacecolor=colour, markeredgecolor="none", zorder=6)
    stops = waypoints[np.array(find_stopping_spots(waypoints))]
    axes.plot(stops[:, 0], stops[:, 1], linestyle="none", marker="o", markersize=12,
              markerfacecolor="none", markeredgecolor=colour, markeredgewidth=1.5, zorder=6)
    for number in range(1, len(waypoints)):
        if not np.isclose(waypoints[number][2], waypoints[number - 1][2]):
            text = f"{waypoints[number - 1][2]:.2f} -> {waypoints[number][2]:.2f} m"
            axes.text(waypoints[number][0] + 0.12, waypoints[number][1] + 0.12, text, fontsize=8,
                      color=drawing.TEXT_COLOUR, zorder=7,
                      bbox={"facecolor": drawing.BACKGROUND_COLOUR, "edgecolor": "none", "alpha": 0.85, "pad": 1})


def draw_crossing(axes, crossing):
    point = crossing.crossing_point_metres
    axes.plot(point[0], point[1], marker="x", markersize=11, markeredgewidth=2.5, color=CROSSING_COLOUR, zorder=8)
    axes.text(point[0] + 0.08, point[1] - 0.22, f"{crossing.degrees_off_perpendicular:.0f} deg off square",
              fontsize=7, color=CROSSING_COLOUR, zorder=8)


def describe(route, world, drone_map):
    paragraphs = [f"{route.length_metres():.1f} m at 2.3 m over the walls. Stops (open circles) only where the height "
                  f"changes and at the pad. Orange crosses: walls crossed in this course."]
    if world is not drone_map:
        paragraphs.append(f"Planned on the drone's map ({drawing.plain_words(drone_map.name)}), drawn over a changed "
                          f"course.")
    for _, check in route.path_checks:
        paragraphs += [drawing.plain_words("Rule broken: " + text) for text in check.rule_breaks]
        paragraphs += [drawing.plain_words("Caution: " + text) for text in check.cautions]
    return paragraphs


if __name__ == "__main__":
    main()
