# Tools that make and check course maps

These programs run on a laptop, inside the simulator's Linux box. Nothing here runs on the drone.

A **course map** is a file that describes the course: where the walls, the tunnel, the window and the boulders are, where the launch mark and landing pad are, and where each checkpoint line is. The drone's route planner reads one at startup and plans the whole route from it. The maps live in [`../drone_code/team_nav/course_maps/`](../drone_code/team_nav/course_maps/), and the code that reads them is [`../drone_code/team_nav/team_nav/course_map.py`](../drone_code/team_nav/team_nav/course_map.py).

This folder makes those map files, and draws the routes planned on them.

## Which file to read first

1. **`make_map_from_organizers_model.py`** turns the organizers' 3D model into `organizers_drawing.json`. It's the simplest of the map makers.
2. **`checkpoint_line_rules.py`** says where each checkpoint line goes, from the walls around it. Both map makers use it, so the lines follow the walls if a wall moves.
3. **`make_map_from_phone_scan.py`** turns our phone scan into two maps. Its header says which scanned walls we trust and why. It uses:
   - `phone_scan_points.py`, which reads the scan and turns it to line up with the course's axes;
   - `measure_phone_scan.py`, which measures each wall, the tunnel, the window and the low objects in the scan.
4. **`draw_planned_routes.py`** plans a route on each map, and on a few changed courses, and saves a picture of each.
5. **`make_map_from_new_organizers_model.py`** makes the map of the organizers' changed course (2026-11-08) from their new model, which is one unnamed mesh in millimetres: it slices the model at flight height to measure the three new walls, keeps everything else from the drawing after checking it against the model, and warns about walls too close to the launch mark.
6. **`draw_fallback_routes.py`** draws the two fallback routes high over the walls (`fallback_pad`, `fallback_tunnel`), on our maps and over the changed maze worlds the simulator flies them in.
7. **`make_boulder_surprise_worlds.py`** makes simulator worlds where the boulders aren't where our scan map says, so we can fly the drone into boulders it doesn't expect.

## How to run them

Run these from the top folder of the repository. Each command starts the simulator's box, runs one program, and closes the box.

Make the map from the organizers' model:

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 course_map_tools/make_map_from_organizers_model.py
```

Make the maps from our phone scan. Copy the scan into `simulator/generated_files/` first. Git ignores that folder, so the scan is never published.

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 course_map_tools/make_map_from_phone_scan.py simulator/generated_files/dronehacks-course-scan-2026-11-06.ply
```

Make the map of the changed course. Copy the new model into `simulator/generated_files/` first.

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 course_map_tools/make_map_from_new_organizers_model.py simulator/generated_files/new_dronehacks_course.obj
```

Draw the planned routes. The pictures go to `simulator/generated_files/route_pictures/`.

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 course_map_tools/draw_planned_routes.py
```

Draw the fallback routes. The pictures go to `simulator/generated_files/fallback_route_pictures/`.
Run `simulator/test_flights/make_fallback_test_worlds.py` first (see its folder's README).

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 course_map_tools/draw_fallback_routes.py
```

Make the boulder surprise worlds. They go to `simulator/generated_files/boulder_surprise_worlds/`, and the program prints the command that flies each one.

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 course_map_tools/make_boulder_surprise_worlds.py
```

## What we still need to check by eye

The phone scan differs from the organizers' drawing in a few big ways. A wrong wall in the map could make the planner avoid a route that is actually open, or fly too close to a wall, so please compare these with a photo or video of the course:

- **M3 maze divider.** The scan puts it about 70 cm east of the drawing and turned by 9 degrees. We use the scan's position.
- **C1 chicane wall.** The scan shows it slanted the other way, with blurry edges. We use the scan's position.
- **A1 approach wall.** The scan seems to have drawn it twice. We use the drawing's position.
- **C2b chicane wall.** The scan has no points where the drawing puts it, so our scan map leaves it out.
- **The window.** The scan puts it 0.81 to 1.56 m above the floor. The drawing put it 0.32 to 1.20 m.
- **The boulders.** The scan found three low things, 27 to 36 cm tall, but not where the drawing puts its five 53 cm boulders. Both our scan maps keep both sets, because boulders are easy to move.
