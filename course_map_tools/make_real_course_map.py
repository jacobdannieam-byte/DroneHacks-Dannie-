"""Makes real_course_2026-11-08.json: the organizers' model with our knowledge of the start area.

This runs on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.

    docker run --rm --volume "$PWD:/repository" --workdir /repository/course_map_tools \\
        --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator \\
        python3 make_real_course_map.py

What it does
------------
  1. Measures the organizers' 3D model (dronehacks-course-v3.obj) exactly as
     make_map_from_organizers_model.py does for organizers_drawing.json: the barrier, the tunnel
     and its cradles, the chicane walls C1, C2a (with its window) and C2b, the approach wall A1,
     the boulders, the launch mark, the pad and the checkpoint lines.
  2. Removes the maze dividers M1, M2 and M3. On 2026-11-08 the team saw that the maze is not built
     as drawn. Also removes the three tube cradles: the model holds the tunnel up on them, but the
     map stands the tunnel on the floor, so they would become 0.28 m walls inside the tunnel. There
     the bottom laser would see each one as a sudden step, which in Gazebo run 20261108_131203 made
     PX4 stop using the laser for height. In the real tunnel the laser sees the tunnel's own floor.
  3. Adds the walls and boulders the team sketched in the start area on 2026-11-08 (every part of
     sketch_2026-11-08.json whose name starts with "Sketch").

The checkpoint lines are worked out in step 1, while M1 and M3 still exist, so "end of first
corridor" and "Gate A" stay where the organizers' drawing puts them. That is our draft reading;
the real lines may differ.

We have not measured the built course against this map. Treat it as the best current guess.
"""
import os

from make_map_from_organizers_model import course_map_from_model
from team_nav.course_map import load_course_map, save_course_map

REPOSITORY = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MAPS_FOLDER = os.path.join(REPOSITORY, "drone_code", "team_nav", "course_maps")
SKETCH_MAP_FILE = os.path.join(MAPS_FOLDER, "sketch_2026-11-08.json")
MAP_FILE = os.path.join(MAPS_FOLDER, "real_course_2026-11-08.json")

# Our own choices: which parts to drop from the model and which to take from the sketch.
REMOVED_WALLS_START_WITH = ("M1 ", "M2 ", "M3 ", "Tube cradle")
SKETCH_PARTS_START_WITH = "Sketch"


def main():
    course_map = course_map_from_model()
    sketch = load_course_map(SKETCH_MAP_FILE)
    kept_walls = []
    for wall in course_map.walls:
        if not wall.name.startswith(REMOVED_WALLS_START_WITH):
            kept_walls.append(wall)
    for wall in sketch.walls:
        if wall.name.startswith(SKETCH_PARTS_START_WITH):
            kept_walls.append(wall)
    course_map.walls = kept_walls
    for obstacle in sketch.low_obstacles:
        if obstacle.name.startswith(SKETCH_PARTS_START_WITH):
            course_map.low_obstacles.append(obstacle)
    course_map.name = "Real course 2026-11-08"
    course_map.description = (
        "The organizers' 3D model (dronehacks-course-v3.obj) without the maze dividers M1 to M3 and the tube "
        "cradles, plus the "
        "start-area walls and boulders from the team's sketch of 2026-11-08. Made by "
        "course_map_tools/make_real_course_map.py. Not yet measured against the built course.")
    save_course_map(course_map, MAP_FILE)
    print(f"Saved {MAP_FILE}: {len(course_map.walls)} walls, {len(course_map.tubes)} tube, "
          f"{len(course_map.low_obstacles)} low obstacles.")


if __name__ == "__main__":
    main()
