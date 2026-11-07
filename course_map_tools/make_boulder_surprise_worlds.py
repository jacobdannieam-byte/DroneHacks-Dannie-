"""Makes simulator worlds in which the boulders are not where the drone's map says.

This runs on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.

Boulders are easy to move, so on the day they may not be where our map puts them. In the
simulator, the world (what is really there) and the drone's map (what the drone believes) are
separate files, so we can test this. Each world made here is our best scan map with the boulders
changed. The drone still plans with the unchanged scan map, so it meets boulders it doesn't
expect. Flying simulator/test_flights/route_test.py in each world shows whether the drone keeps
moving over a boulder, whether PX4's height estimate stays right, and, if it doesn't, whether
the flight program notices and lands.

The boulders are placed on the route the first competition run will fly, through the side gap
into the landing area (missions/dronehacks_2026_side_gap_first.json), except the one under the
window's line-up, which is placed on the window route.

The worlds
----------
  boulder_in_the_first_corridor
      A boulder added halfway up the first corridor, right on the planned route. The drone
      flies straight over it at its cruising speed.
  boulder_under_a_corner_after_the_tunnel
      One of the drawing's boulders moved under the first corner of the route after the tunnel.
      The drone pauses at every corner, so it stops briefly right above the boulder.
  boulder_under_the_window_line_up
      The boulder in front of the window moved under the point where the drone lines up with
      the window. The drone pauses there, then climbs to the window's height. Fly this one with
      the window mission; the printed command says how.
  drawing_boulders_gone
      The drawing's five boulders removed, leaving only the three low things our scan found.
      The drone keeps away from boulders that aren't there, which should change nothing.

How to run it, from the top folder of the repository:

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 course_map_tools/make_boulder_surprise_worlds.py

It saves the worlds in simulator/generated_files/boulder_surprise_worlds/ (Git ignores that
folder) and prints the command that flies each one.
"""
import os
import sys

from team_nav.course_map import save_course_map
from team_nav.route_planner import plan_route

# The changed courses live with the planner's tests. We use them here so that the simulator
# flights and the tests describe the same kinds of change.
REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPOSITORY, "drone_code", "team_nav", "test"))
import course_changes  # noqa: E402  (must come after the line above)
from our_files import SCAN_MAP, load_our_map, load_rulebook_mission, load_side_gap_first_mission  # noqa: E402

WORLDS_FOLDER = os.path.join(REPOSITORY, "simulator", "generated_files", "boulder_surprise_worlds")

# From the organizers' drawing: the size of each of its boulders.
BOULDER_RADIUS_METRES = 0.2286
BOULDER_TOP_HEIGHT_METRES = 0.5334
# The names of two of the drawing's boulders, as they appear in our scan map.
BOULDER_EAST_OF_THE_TUNNEL_EXIT = "Boulder 2"
BOULDER_IN_FRONT_OF_THE_WINDOW = "Boulder 3"
DRAWING_BOULDER_NAMES = ("Boulder 1", "Boulder 2", "Boulder 3", "Boulder 4", "Boulder 5")
WINDOW_MISSION_FILE = "drone_code/team_nav/missions/dronehacks_2026.json"


def main():
    drone_map = load_our_map(SCAN_MAP)
    route = plan_route(drone_map, load_side_gap_first_mission())
    window_route = plan_route(drone_map, load_rulebook_mission())
    os.makedirs(WORLDS_FOLDER, exist_ok=True)

    worlds = {
        "boulder_in_the_first_corridor": course_changes.with_low_obstacle(
            drone_map, "surprise boulder", course_changes.point_halfway_up_the_first_corridor(route),
            BOULDER_RADIUS_METRES, BOULDER_TOP_HEIGHT_METRES,
        ),
        "boulder_under_a_corner_after_the_tunnel": course_changes.with_low_obstacle_moved(
            drone_map, BOULDER_EAST_OF_THE_TUNNEL_EXIT, first_corner_after_the_tunnel(route),
        ),
        "boulder_under_the_window_line_up": course_changes.with_low_obstacle_moved(
            drone_map, BOULDER_IN_FRONT_OF_THE_WINDOW, window_line_up_point(window_route),
        ),
        "drawing_boulders_gone": without_the_drawings_boulders(drone_map),
    }
    for name, world in worlds.items():
        path = os.path.join(WORLDS_FOLDER, f"{name}.json")
        save_course_map(world, path)
        mission_option = f" --mission {WINDOW_MISSION_FILE}" if name == "boulder_under_the_window_line_up" else ""
        print(f"Saved {name}. Fly it with:")
        print(f"    simulator/run_test_flight.sh --world {os.path.relpath(path, REPOSITORY)} route_test.py{mission_option}")


def first_corner_after_the_tunnel(route):
    """The first waypoint after the one where the route clears the tunnel's exit, seen from above."""
    _, _, tunnel_cleared_at = route.checkpoints_cleared[-2]
    return route.waypoints_metres[tunnel_cleared_at + 1][:2]


def window_line_up_point(route):
    """The waypoint where the route lines up to fly straight through the window, seen from above."""
    _, _, window_cleared_at = route.checkpoints_cleared[-1]
    return route.waypoints_metres[window_cleared_at - 2][:2]   # The two waypoints after it are the level part.


def without_the_drawings_boulders(course_map):
    original_name = course_map.name
    for name in DRAWING_BOULDER_NAMES:
        course_map = course_changes.without_low_obstacle(course_map, name)
    course_map.name = f"{original_name}, without the drawing's boulders"
    return course_map


if __name__ == "__main__":
    main()
