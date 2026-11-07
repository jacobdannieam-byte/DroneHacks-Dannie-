# team_nav

This is our team's ROS package for the real drone. It has three parts today:

- **The launch file, which still starts nothing on purpose.** It replaces the organizers' demo program, so that the drone can't fly the demo's 5 m square into the course walls. Read [`launch/team_nav.launch.py`](launch/team_nav.launch.py) to see what starts on the drone and why that is safe.
- **The route planner.** Given a map of the course and a mission, it plans the drone's whole route.
- **The route follower.** It slides a target point along the planned route ahead of the drone, which is how the drone will fly the route. So far only the simulator's test flight (`simulator/test_flights/route_test.py`) uses the planner and the follower; nothing on the drone does.

To install this package on the drone, see [`../README.md`](../README.md).

## The route planner

Nothing about the route is written into the code. It comes from two files:

- A **map file** in `course_maps/` says where everything is: walls, openings, the tunnel, boulders, the launch mark, the landing pad, and the checkpoint lines.
- The **mission file** in `missions/` says which checkpoint lines to cross, in what order. A checkpoint can list more than one line, such as the window or Gate B, in order of preference.

The planner reads both at startup, and the flight program will call it again whenever the drone changes its map because its cameras saw something new.

Read the files in `team_nav/` in this order:

| File | What it does |
|---|---|
| `course_map.py` | Reads and writes map files, and says what blocks the drone at a given height. |
| `mission.py` | Reads the mission file. |
| `drone_dimensions.py` | The drone's size and the safety margins the planner keeps. |
| `boulder_distances.py` | Measures how close the drone's centre, where the bottom laser is, comes to each boulder. |
| `clearance_grid.py` | Covers the course with 5 cm squares and works out how much room the drone has in each, and how far each is from a boulder. |
| `opening_crossings.py` | Works out how the drone flies straight through an opening such as the window. |
| `route_planner.py` | Searches the grid for the best route that crosses the checkpoint lines in order, keeps the laser off boulders where it can, and straightens it. |
| `start_frame.py` | Converts between the map's directions and the camera tracking's, which start from the launch mark. |
| `route_follower.py` | Slides a target along the route ahead of the drone, pausing at each corner. |

### The maps we keep

| File | What it is |
|---|---|
| `course_maps/organizers_drawing.json` | Made from the organizers' 3D model. |
| `course_maps/phone_scan_2026-11-06.json` | Made from our phone scan of the real course. This is our best map. One wall we doubt (A1) is copied from the drawing instead. It has the scan's three low things and the drawing's five boulders, because boulders are easy to move. |
| `course_maps/phone_scan_2026-11-06_trusting_every_wall.json` | The scan with every wall taken from the scan. The tests use it as a changed course. |

The programs that make these maps are in [`../../course_map_tools/`](../../course_map_tools/README.md). Every shape in a map has a `source` note saying where its numbers came from.

### Where the planned routes go

| Map | Length | Least room | Chicane | Closest to a boulder's edge |
|---|---|---|---|---|
| Organizers' drawing | 17.9 m | 0.36 m, in the window | through the window, at 0.76 m | 0.12 m, in front of the window |
| Our phone scan | 18.8 m | 0.35 m, next to the third maze wall | through the window, at 1.19 m | 0.09 m, just past the window |
| Scan, trusting every wall | 18.9 m | 0.35 m, next to the third maze wall | through the window, at 1.19 m | 0.09 m, just past the window |

"Least room" is the closest the drone's centre comes to anything it could hit. The drone needs at least 0.35 m: 0.30 m to its propeller tips, plus 0.05 m to spare. On the scan, the tightest place is the gap between the north end of the third maze wall and the net.

The planner tries to keep the drone's centre 0.25 m from every boulder's edge, because a boulder under the bottom laser makes the drone's height reading jump. It's a strong preference, not a ban, so the route can still get through if a boulder blocks the only way. Near the window it can't keep the full 0.25 m, so the route lists those legs, and the flight program must keep moving there. `route_planner.py`'s header explains this.

To see the routes, run `course_map_tools/draw_planned_routes.py` (see its folder's README).

## The tests

The tests in `test/` plan routes on our three maps and on changed copies of the drawing and the scan: the window too small, something behind the window, Gate B closed as well, the tunnel blocked, the first corridor closed or narrowed, a pole in the way, boulders added, moved or removed, a maze wall moved or removed, and replanning from partway through the course. Each test either checks that the route is safe or checks that the planner explains clearly why there is no route. `test/test_route_follower.py` flies a pretend drone along a route to check the follower's rules, including that it holds the window's height while any part of it is over the window's lower wall.

A route's safety is checked by `test/route_checks.py`, which measures the route again from the map rather than trusting the planner's own numbers. It also checks that every leg near a boulder is listed, so the flight program knows to keep moving there. `test/course_changes.py` makes the changed courses.

Run them from the top folder of the repository. They take about 30 seconds.

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 -m pytest drone_code/team_nav/test -q -p no:cacheprovider
```

## The other files

`package.xml` and `setup.py` describe the package to ROS. `setup.py` also installs the map and mission files, so the drone can find them. `resource/team_nav` lets ROS find the package, and `team_nav/__init__.py` is empty.
