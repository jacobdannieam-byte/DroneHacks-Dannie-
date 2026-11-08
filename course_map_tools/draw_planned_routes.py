"""Draws the route planner's routes on top views of the course, so the team can check them by eye.

This runs on a laptop, inside the simulator's Linux box, which has the drawing library
(matplotlib). Nothing here runs on the drone. From the top folder of the repository:

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 course_map_tools/draw_planned_routes.py

It takes about ten seconds and saves one picture per case in simulator/generated_files/route_pictures/
(Git ignores that folder). The first picture is the route the team chose for the first competition
run, through the side gap into the landing area; the second, the same mission on the organizers' changed
course of 2026-11-08 (which has no route from the launch mark: a new wall is 0.08 m from it). The others try the window first: our three map
files, and a few changed courses. The changes are the same ones the route planner's tests make
(drone_code/team_nav/test/course_changes.py), so the pictures show what the tests check.

What a picture shows, seen from above with north up:
  - Walls in dark grey. Walls lower than the drone's body at its usual height are light grey.
  - Openings, such as the window, in aqua. The title gives their height above the floor.
  - The tunnel as a light grey band between its outer edges.
  - Low obstacles as circles: filled if they reach the drone's body, outlined if the drone
    passes over them. Each is labelled with its height.
  - Anything a test added, such as a pole, in magenta.
  - Checkpoint lines as dashed orange lines, each with a number near one end. The title says
    which number is which line.
  - Walls and lines are named in plain words, such as "third maze wall" or "maze exit". The map
    files use the organizers' short codes (M3, Gate A), which mean nothing to a newcomer.
  - The planned route in blue, with its height wherever it climbs or descends from the usual height.
    Small dots are waypoints the drone flies straight through. Large open circles are where it
    stops, at corners (drone_code/team_nav/team_nav/stopping_spots.py).
  - At the route's tightest point, a dashed circle as wide as the drone plus its safety
    margin. It just touches the nearest thing there.
"""
import os
import sys

import matplotlib
matplotlib.use("Agg")   # Draw straight to files; the box has no screen.
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle, Polygon

from team_nav.drone_dimensions import REQUIRED_ROOM_METRES, heights_spanned_metres
from team_nav.route_planner import RoutePlanningError, plan_route
from team_nav.stopping_spots import find_stopping_spots

# The changed courses and the route checks live with the planner's tests. We use them here
# so that the pictures and the tests describe exactly the same cases.
REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPOSITORY, "drone_code", "team_nav", "test"))
import course_changes  # noqa: E402  (must come after the line above)
from our_files import (  # noqa: E402
    CHANGED_COURSE_MAP, DRAWING_MAP, SCAN_MAP, SCAN_MAP_TRUSTING_EVERY_WALL, load_our_map, load_rulebook_mission, load_side_gap_first_mission,
)
from route_checks import least_room_along_route, points_along_route  # noqa: E402

PICTURES_FOLDER = os.path.join(REPOSITORY, "simulator", "generated_files", "route_pictures")

# Our own choices, for drawing only. Colours are from a palette chosen to stay distinct for
# readers with colour blindness.
ROUTE_COLOUR = "#2a78d6"            # blue
CHECKPOINT_LINE_COLOUR = "#eb6834"  # orange
OPENING_COLOUR = "#1baf7a"          # aqua
ADDED_BY_A_TEST_COLOUR = "#e87ba4"  # magenta
WALL_COLOUR = "#52514e"
LOW_WALL_COLOUR = "#c3c2b7"
TUNNEL_COLOUR = "#dedcd5"
TEXT_COLOUR = "#0b0b0b"
QUIET_TEXT_COLOUR = "#52514e"
BACKGROUND_COLOUR = "#fcfcfb"
PICTURE_SIZE_INCHES = 9.0
DOTS_PER_INCH = 110
# The source note the tests give to every shape they add.
ADDED_BY_A_TEST = "Added by a test."

# Plain names for the pictures, in place of the organizers' short codes used in the map files.
PLAIN_NAMES = {
    "M1 maze divider": "first maze wall",
    "M2 maze divider": "second maze wall",
    "M3 maze divider": "third maze wall",
    "C1 chicane wall": "first chicane wall",
    "C2a chicane wall (window)": "wall with the window",
    "C2b chicane wall": "wall beside the window wall",
    "A1 approach wall": "wall beside the landing approach",
    "Gate A": "maze exit",
    "Gate B": "side gap into the landing area",
}
# Walls that get a label on the picture. The barriers and the tunnel's cradles are easy to
# recognise without one.
LABELLED_WALLS = [name for name in PLAIN_NAMES if not name.startswith("Gate")]


def main():
    os.makedirs(PICTURES_FOLDER, exist_ok=True)
    first_run_path = os.path.join(PICTURES_FOLDER, "0_phone_scan_first_run_through_the_side_gap.png")
    draw_one_case(load_our_map(SCAN_MAP), load_side_gap_first_mission(), first_run_path)
    print(f"Saved {first_run_path}")
    changed_course_path = os.path.join(PICTURES_FOLDER, "0_changed_course_2026-11-08_side_gap.png")
    draw_one_case(load_our_map(CHANGED_COURSE_MAP), load_side_gap_first_mission(), changed_course_path)
    print(f"Saved {changed_course_path}")

    mission = load_rulebook_mission()
    for file_name, course_map in cases_to_draw(mission):
        path = os.path.join(PICTURES_FOLDER, f"{file_name}.png")
        draw_one_case(course_map, mission, path)
        print(f"Saved {path}")


def cases_to_draw(mission):
    """Returns (file name, course map) for each picture."""
    drawing = load_our_map(DRAWING_MAP)
    scan = load_our_map(SCAN_MAP)
    scan_route = plan_route(scan, mission)
    pole_centre_metres = course_changes.point_halfway_from_tunnel_to_window(scan_route)
    return [
        ("1_organizers_drawing", drawing),
        ("2_phone_scan", scan),
        ("3_phone_scan_trusting_every_wall", load_our_map(SCAN_MAP_TRUSTING_EVERY_WALL)),
        ("4_phone_scan_window_too_small", course_changes.with_window_too_small(scan)),
        ("5_phone_scan_pole_in_the_way", course_changes.with_low_obstacle(scan, "pole", pole_centre_metres, 0.10, 1.7)),
        ("6_phone_scan_without_the_second_maze_wall", course_changes.without_wall(scan, "M2 maze divider")),
        ("7_phone_scan_window_and_side_gap_blocked", course_changes.with_gate_b_closed(course_changes.with_window_too_small(scan))),
    ]


def draw_one_case(course_map, mission, path):
    figure, axes = plt.subplots(figsize=(PICTURE_SIZE_INCHES, PICTURE_SIZE_INCHES), dpi=DOTS_PER_INCH)
    figure.patch.set_facecolor(BACKGROUND_COLOUR)
    axes.set_facecolor(BACKGROUND_COLOUR)
    draw_course(axes, course_map, mission.cruise_height_metres)
    try:
        route = plan_route(course_map, mission)
    except RoutePlanningError as reason:
        route_description = "No route. " + str(reason)
    else:
        draw_route(axes, course_map, route, mission.cruise_height_metres)
        route_description = describe_route(route, course_map)
    draw_title(axes, plain_words(course_map.name), [plain_words(route_description), plain_words(describe_course(course_map))])
    set_up_axes(axes, course_map)
    figure.savefig(path, facecolor=BACKGROUND_COLOUR, bbox_inches="tight")
    plt.close(figure)


# ---- The course -------------------------------------------------------------------------------

def draw_course(axes, course_map, cruise_height_metres):
    draw_floor_markings(axes, course_map)
    for tube in course_map.tubes:
        draw_tube(axes, tube)
    for wall in course_map.walls:
        draw_wall(axes, wall, cruise_height_metres)
    for obstacle in course_map.low_obstacles:
        draw_low_obstacle(axes, obstacle, cruise_height_metres)
    for number, line in enumerate(course_map.checkpoint_lines):
        draw_checkpoint_line(axes, line, number + 1)


def draw_floor_markings(axes, course_map):
    pad = course_map.landing_pad_centre_metres
    axes.add_patch(Circle(pad, course_map.landing_pad_outer_radius_metres, facecolor="#ecebe7", edgecolor="none"))
    axes.add_patch(Circle(pad, course_map.approach_ring_radius_metres, facecolor="none", edgecolor=LOW_WALL_COLOUR, linewidth=1))
    axes.text(pad[0], pad[1] - 0.35, "landing pad", ha="center", fontsize=8, color=QUIET_TEXT_COLOUR)
    launch = course_map.launch_mark_centre_metres
    axes.plot(*launch, marker="s", markersize=9, color=QUIET_TEXT_COLOUR)
    axes.text(launch[0], launch[1] - 0.25, "launch", ha="center", fontsize=8, color=QUIET_TEXT_COLOUR)


def strip_corners(start_metres, end_metres, thickness_metres):
    """The four corners, seen from above, of a straight strip of this thickness."""
    direction = (end_metres - start_metres) / np.linalg.norm(end_metres - start_metres)
    across = np.array([-direction[1], direction[0]]) * thickness_metres / 2.0
    return [start_metres - across, end_metres - across, end_metres + across, start_metres + across]


def draw_wall(axes, wall, cruise_height_metres):
    blocks_the_drone = bool(wall.solid_pieces_between_heights(*heights_spanned_metres(cruise_height_metres)))
    colour = WALL_COLOUR if blocks_the_drone else LOW_WALL_COLOUR
    if wall.source == ADDED_BY_A_TEST:
        colour = ADDED_BY_A_TEST_COLOUR
    axes.add_patch(Polygon(strip_corners(wall.start_metres, wall.end_metres, wall.thickness_metres), facecolor=colour, edgecolor="none"))
    for opening in wall.openings:
        draw_opening(axes, wall, opening)
    if wall.name in LABELLED_WALLS:
        draw_wall_label(axes, wall)


def draw_wall_label(axes, wall):
    """The wall's plain name, written along the wall, just beside it."""
    direction = wall.direction()
    beside = np.array([-direction[1], direction[0]]) * (wall.thickness_metres / 2.0 + 0.1)
    label_point = (wall.start_metres + wall.end_metres) / 2.0 + beside
    angle_degrees = np.degrees(np.arctan2(direction[1], direction[0]))
    # Keep the writing the right way up.
    if angle_degrees > 90.0:
        angle_degrees -= 180.0
    if angle_degrees <= -90.0:
        angle_degrees += 180.0
    axes.text(label_point[0], label_point[1], PLAIN_NAMES[wall.name], fontsize=7, color=TEXT_COLOUR,
              ha="center", va="center", rotation=angle_degrees, rotation_mode="anchor", zorder=8)


def plain_words(text):
    """The text with the organizers' short codes replaced by plain names."""
    for code_name, plain_name in PLAIN_NAMES.items():
        text = text.replace(code_name, plain_name)
    return text


def draw_opening(axes, wall, opening):
    start = wall.point_along_wall_metres(opening.start_along_wall_metres)
    end = wall.point_along_wall_metres(opening.end_along_wall_metres)
    axes.add_patch(Polygon(strip_corners(start, end, wall.thickness_metres * 1.6), facecolor=OPENING_COLOUR, edgecolor="none"))


def draw_tube(axes, tube):
    axes.add_patch(Polygon(strip_corners(tube.entrance_centre_metres, tube.exit_centre_metres, tube.outside_width_metres),
                           facecolor=TUNNEL_COLOUR, edgecolor=WALL_COLOUR, linewidth=1))
    middle = (tube.entrance_centre_metres + tube.exit_centre_metres) / 2.0
    axes.text(middle[0], middle[1] + 0.15, tube.name.lower(), ha="center", fontsize=8, color=QUIET_TEXT_COLOUR)


def draw_low_obstacle(axes, obstacle, cruise_height_metres):
    lowest_body_height_metres, _ = heights_spanned_metres(cruise_height_metres)
    reaches_the_drone = obstacle.top_height_metres > lowest_body_height_metres
    colour = ADDED_BY_A_TEST_COLOUR if obstacle.source == ADDED_BY_A_TEST else WALL_COLOUR
    axes.add_patch(Circle(obstacle.centre_metres, obstacle.radius_metres, edgecolor=colour, linewidth=1.5,
                          facecolor=colour if reaches_the_drone else "none"))
    axes.text(obstacle.centre_metres[0], obstacle.centre_metres[1] - obstacle.radius_metres - 0.12,
              f"{obstacle.top_height_metres:.2f} m tall", ha="center", fontsize=7, color=QUIET_TEXT_COLOUR)


def draw_checkpoint_line(axes, line, number):
    """The line, and its number near its start, where it meets a wall or the net. The route
    usually crosses near the middle, so a name there would be hidden."""
    axes.plot([line.start_metres[0], line.end_metres[0]], [line.start_metres[1], line.end_metres[1]],
              linestyle=(0, (4, 3)), color=CHECKPOINT_LINE_COLOUR, linewidth=1.5)
    along = (line.end_metres - line.start_metres) / np.linalg.norm(line.end_metres - line.start_metres)
    towards_far_side = line.far_side_point_metres - line.start_metres
    towards_far_side = towards_far_side - along * np.dot(towards_far_side, along)
    towards_far_side = towards_far_side / np.linalg.norm(towards_far_side)
    tag_point = line.start_metres + along * 0.2 + towards_far_side * 0.15
    axes.text(tag_point[0], tag_point[1], str(number), ha="center", va="center", fontsize=8, color=TEXT_COLOUR,
              bbox={"boxstyle": "round", "facecolor": BACKGROUND_COLOUR, "edgecolor": CHECKPOINT_LINE_COLOUR})


# ---- The route ---------------------------------------------------------------------------------

def draw_route(axes, course_map, route, cruise_height_metres):
    waypoints = route.waypoints_metres
    axes.plot(waypoints[:, 0], waypoints[:, 1], color=ROUTE_COLOUR, linewidth=2.5, solid_capstyle="round", zorder=5)
    axes.plot(waypoints[:, 0], waypoints[:, 1], linestyle="none", marker="o", markersize=5,
              markerfacecolor=ROUTE_COLOUR, markeredgecolor=BACKGROUND_COLOUR, markeredgewidth=1.5, zorder=6)
    stops = waypoints[np.array(find_stopping_spots(waypoints))]
    axes.plot(stops[:, 0], stops[:, 1], linestyle="none", marker="o", markersize=10,
              markerfacecolor="none", markeredgecolor=ROUTE_COLOUR, markeredgewidth=1.5, zorder=6)
    # Label the first waypoint of each stretch flown at another height, such as through the window.
    for number in range(1, len(waypoints)):
        at_another_height = not np.isclose(waypoints[number][2], cruise_height_metres)
        previous_at_same_height = np.isclose(waypoints[number][2], waypoints[number - 1][2])
        if at_another_height and not previous_at_same_height:
            axes.text(waypoints[number][0], waypoints[number][1] + 0.15, f"{waypoints[number][2]:.2f} m up", ha="center",
                      fontsize=8, color=TEXT_COLOUR, zorder=7,
                      bbox={"facecolor": BACKGROUND_COLOUR, "edgecolor": "none", "alpha": 0.8, "pad": 1})

    _, _, tightest_point = least_room_along_route(course_map, points_along_route(waypoints))
    axes.add_patch(Circle(tightest_point[:2], REQUIRED_ROOM_METRES, facecolor="none", edgecolor=TEXT_COLOUR,
                          linestyle=(0, (2, 2)), linewidth=1, zorder=7))


def describe_route(route, course_map):
    lines_crossed = ", ".join(line_name for _, line_name, _ in route.checkpoints_cleared)
    return (
        f"{route.length_metres:.1f} m long. Least room {route.least_room_metres:.2f} m, next to the "
        f"{route.least_room_near} (dashed circle). Crosses: {lines_crossed}. Open circles are stops; "
        f"the closest stop to a boulder is {_closest_stop_to_a_boulder_metres(route, course_map):.2f} m from its edge."
    )


def _closest_stop_to_a_boulder_metres(route, course_map):
    """How close the nearest stop comes to the edge of a boulder in the map."""
    closest_metres = np.inf
    for waypoint, is_stop in zip(route.waypoints_metres, find_stopping_spots(route.waypoints_metres)):
        if not is_stop:
            continue
        for boulder in course_map.low_obstacles:
            distance_metres = np.linalg.norm(waypoint[:2] - boulder.centre_metres) - boulder.radius_metres
            closest_metres = min(closest_metres, distance_metres)
    return closest_metres


def describe_course(course_map):
    numbered_lines = ", ".join(f"{number + 1} {line.name}" for number, line in enumerate(course_map.checkpoint_lines))
    openings = []
    for wall in course_map.walls:
        for opening in wall.openings:
            openings.append(f"The {opening.name} is {opening.bottom_height_metres:.2f} to {opening.top_height_metres:.2f} m above the floor.")
    return f"Checkpoint lines: {numbered_lines}. " + " ".join(openings)


def draw_title(axes, map_name, paragraphs):
    wrapped = [wrap(paragraph, 110) for paragraph in paragraphs]
    axes.set_title(map_name + "\n" + "\n".join(wrapped), loc="left", fontsize=10, color=TEXT_COLOUR)


def wrap(text, longest_line_characters):
    lines = [""]
    for word in text.split(" "):
        if len(lines[-1]) + len(word) + 1 > longest_line_characters:
            lines.append("")
        lines[-1] = f"{lines[-1]} {word}".strip()
    return "\n".join(lines)


def set_up_axes(axes, course_map):
    low_x, low_y = course_map.lowest_corner_metres
    high_x, high_y = course_map.highest_corner_metres
    axes.plot([low_x, high_x, high_x, low_x, low_x], [low_y, low_y, high_y, high_y, low_y], color=WALL_COLOUR, linewidth=1)
    margin_metres = 0.15
    axes.set_xlim(low_x - margin_metres, high_x + margin_metres)
    axes.set_ylim(low_y - margin_metres, high_y + margin_metres)
    axes.set_aspect("equal")
    axes.set_xlabel("metres east", color=QUIET_TEXT_COLOUR)
    axes.set_ylabel("metres north", color=QUIET_TEXT_COLOUR)
    axes.tick_params(colors=QUIET_TEXT_COLOUR, labelsize=8)
    for edge in axes.spines.values():
        edge.set_visible(False)


if __name__ == "__main__":
    main()
