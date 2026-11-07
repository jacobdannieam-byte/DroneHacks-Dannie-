"""Makes a SIMULATOR-ONLY copy of a practice map that the simulator's referee can load.
SIMULATOR TEST CODE. Nothing here runs on the real drone, and the drone never reads its output.

Why it is needed
----------------
The referee (simulator/ros_packages/course_simulator/course_simulator/referee.py) always judges
the rulebook's five checkpoints, so it stops with an error if the world's map has no line called
"end of first corridor", "Gate A", "tunnel exit", "through the window" or "Gate B". A practice
course such as course_maps/practice_mock_gap_and_window.json has none of them, and without the
referee the simulator would not count touches.

So this file copies the map and adds any of those five lines that are missing, as tiny lines in
the course's south-west corner, far from any route. The referee then loads, counts touches as
usual, and simply never sees those checkpoints cleared. The drone's own map stays untouched: the
practice program is still given the real file with --map, so these pretend lines never reach it.

Run it on the Mac (it needs only Python), then fly in the world it wrote:

    python3 simulator/test_flights/make_world_for_the_referee.py drone_code/team_nav/course_maps/practice_mock_gap_and_window.json
    simulator/run_test_flight.sh --world simulator/generated_files/practice_worlds/practice_mock_gap_and_window.json real_program_test.py ...
"""
import json
import os
import sys

REPOSITORY_FOLDER = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RULEBOOK_MISSION_FILE = os.path.join(REPOSITORY_FOLDER, "drone_code", "team_nav", "missions", "dronehacks_2026.json")
WORLDS_FOLDER = os.path.join(REPOSITORY_FOLDER, "simulator", "generated_files", "practice_worlds")

# Our own choices: the pretend lines are 5 cm long, 10 cm in from the course's south-west corner,
# 10 cm apart, so no route goes near them.
PRETEND_LINE_LENGTH_METRES = 0.05
PRETEND_LINE_INSET_METRES = 0.10
PRETEND_LINE_SPACING_METRES = 0.10


def rulebook_line_names():
    with open(RULEBOOK_MISSION_FILE, encoding="utf-8") as mission_file:
        mission = json.load(mission_file)
    names = []
    for checkpoint in mission["checkpoints"]:
        names += checkpoint["cross_one_of"]
    return names


def pretend_line(name, number, lowest_corner_metres):
    east_metres = lowest_corner_metres[0] + PRETEND_LINE_INSET_METRES + number * PRETEND_LINE_SPACING_METRES
    north_metres = lowest_corner_metres[1] + PRETEND_LINE_INSET_METRES
    return {
        "name": name,
        "start_metres": [east_metres, north_metres],
        "end_metres": [east_metres, north_metres + PRETEND_LINE_LENGTH_METRES],
        "far_side_point_metres": [east_metres + PRETEND_LINE_SPACING_METRES / 2, north_metres],
        "source": "SIMULATOR ONLY: a pretend line added by simulator/test_flights/make_world_for_the_referee.py so the "
                  "referee, which always judges the rulebook's checkpoints, can load this practice world.",
    }


def make_world(map_path):
    with open(map_path, encoding="utf-8") as map_file:
        course = json.load(map_file)
    names_present = [line["name"] for line in course["checkpoint_lines"]]
    added = 0
    for name in rulebook_line_names():
        if name not in names_present:
            course["checkpoint_lines"].append(pretend_line(name, added, course["course_edges"]["lowest_corner_metres"]))
            added += 1
    course["name"] = course["name"] + " (simulator world for the referee)"
    os.makedirs(WORLDS_FOLDER, exist_ok=True)
    world_path = os.path.join(WORLDS_FOLDER, os.path.basename(map_path))
    with open(world_path, "w", encoding="utf-8") as world_file:
        json.dump(course, world_file, indent=2)
        world_file.write("\n")
    print(f"Wrote {os.path.relpath(world_path, REPOSITORY_FOLDER)}, with {added} pretend rulebook line(s) added.")


def main():
    if len(sys.argv) != 2:
        print("Give one map file, for example: python3 simulator/test_flights/make_world_for_the_referee.py "
              "drone_code/team_nav/course_maps/practice_mock_gap_and_window.json")
        return 2
    make_world(sys.argv[1])
    return 0


if __name__ == "__main__":
    sys.exit(main())
