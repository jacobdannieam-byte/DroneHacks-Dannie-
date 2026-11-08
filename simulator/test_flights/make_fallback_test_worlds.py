"""Makes the changed-course worlds for the fallback missions' simulator flights: our phone scan with
the maze walls changed, as the organizers may change them (one removed, one slanted). SIMULATOR
TEST CODE; nothing here runs on the drone.

From the top folder of the repository (any Python with numpy will do; the box has one):

    docker run --rm --volume "$PWD:/repository" --workdir /repository \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 simulator/test_flights/make_fallback_test_worlds.py

It writes one map file per world into simulator/generated_files/fallback_worlds/ (Git ignores it)
and prints the run_test_flight.sh command for each. The drone itself still gets the unchanged
scan as its map, so these flights show what happens when the real maze differs from the map.

The changes use the route planner's own test changes (drone_code/team_nav/test/course_changes.py)
where they exist:
  - without the second maze wall (M2), which the tunnel route's east leg crosses;
  - the third maze wall (M3) moved 0.4 m west;
  - the first maze wall (M1) removed and the second (M2) slanted by 20 degrees, which is what the
    team expects for 2026-11-08.
"""
import os
import sys

import numpy as np

REPOSITORY = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPOSITORY, "drone_code", "team_nav", "test"))
import course_changes  # noqa: E402  (must come after the line above)
from our_files import SCAN_MAP, load_our_map  # noqa: E402
from team_nav.course_map import save_course_map  # noqa: E402

WORLDS_FOLDER = os.path.join(REPOSITORY, "simulator", "generated_files", "fallback_worlds")
# Our own choices, for the tests.
THIRD_WALL_SHIFT_WEST_METRES = 0.4
SLANT_DEGREES = 20.0


def changed_maze_worlds(scan):
    """(file name, course map) for each changed world, made from the scan."""
    slanted = with_wall_slanted(course_changes.without_wall(scan, "M1 maze divider"), "M2 maze divider", SLANT_DEGREES)
    slanted.name = "Phone scan without the first maze wall, second maze wall slanted 20 degrees"
    return [
        ("scan_without_second_maze_wall", course_changes.without_wall(scan, "M2 maze divider")),
        ("scan_third_maze_wall_moved_west",
         course_changes.with_wall_moved(scan, "M3 maze divider", np.array([-THIRD_WALL_SHIFT_WEST_METRES, 0.0]))),
        ("scan_first_wall_removed_second_slanted", slanted),
    ]


def with_wall_slanted(course_map, wall_name, degrees):
    """A copy with this wall turned about its middle by this many degrees (anticlockwise seen from
    above), clipped so that it stays inside the net."""
    changed = course_changes.copy_with_new_name(course_map, f"{wall_name} slanted by {degrees:.0f} degrees")
    wall = changed.wall_named(wall_name)
    middle = (wall.start_metres + wall.end_metres) / 2.0
    angle = np.radians(degrees)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    wall.start_metres = np.clip(middle + rotation @ (wall.start_metres - middle),
                                changed.lowest_corner_metres, changed.highest_corner_metres)
    wall.end_metres = np.clip(middle + rotation @ (wall.end_metres - middle),
                              changed.lowest_corner_metres, changed.highest_corner_metres)
    wall.source = f"Slanted by {degrees:.0f} degrees by make_fallback_test_worlds.py, for a simulator test."
    return changed


def main():
    os.makedirs(WORLDS_FOLDER, exist_ok=True)
    for file_name, world in changed_maze_worlds(load_our_map(SCAN_MAP)):
        path = os.path.join(WORLDS_FOLDER, f"{file_name}.json")
        save_course_map(world, path)
        relative_path = os.path.relpath(path, REPOSITORY)
        print(f"Saved {relative_path}. Fly it with:")
        print(f"  simulator/run_test_flight.sh --world {relative_path} real_program_test.py "
              f"--env TEAM_NAV_MISSION=fallback_tunnel -- "
              f"/repository/simulator/generated_files/ros_install/team_nav/lib/team_nav/course_run")


if __name__ == "__main__":
    main()
