# team_nav

This is our team's ROS package for the real drone. It has four parts today:

- **Three launch files**, which say what starts on the drone: [`watch_only`](launch/watch_only.launch.py) (nothing can fly; for checks and hand tests), [`test_flights`](launch/test_flights.launch.py) (hand-over on, each test started by hand) and [`course_run`](launch/course_run.launch.py) (the competition). All three start our flight recorder. [`team_nav/launch_pieces.py`](team_nav/launch_pieces.py) compares them. Any of them replaces the organizers' demo program, which would fly a 5 m square into the course walls.
- **The route planner.** Given a map of the course and a mission, it plans the drone's whole route.
- **The route follower.** It slides a target point along the planned route ahead of the drone, which is how the drone will fly the route.
- **Two safety checks that stop the run.** One watches for a touch (a sudden tilt or spin). The other watches for PX4's height going wrong (PX4's height and the bottom laser's disagree over clear floor). When either fires, the flight program lands where the drone is.

`course_run.py` uses them as the real drone program. So far it has flown only in the simulator, through `simulator/test_flights/real_program_test.py`. On the drone, `launch/course_run.launch.py` starts it, and even then it can't take control until the team turns on the hand-over in that file.

To install this package on the drone, see [`../README.md`](../README.md).

## The route planner

Nothing about the route is written into the code. It comes from two files:

- A **map file** in `course_maps/` says where everything is: walls, openings, the tunnel, boulders, the launch mark, the landing pad, and the checkpoint lines.
- The **mission file** in `missions/` says which checkpoint lines to cross, in what order. A checkpoint can list more than one line, such as the window or the side gap into the landing area (called "Gate B" in the map files), in order of preference. There are two missions:
  - `missions/dronehacks_2026_side_gap_first.json` prefers the side gap. The team chose it for the first competition run on 2026-11-07: the laser never passes over a wall edge there, so PX4's height isn't put at risk, and a finish is worth 35 points against the window's extra 5.
  - `missions/dronehacks_2026.json` prefers the window, for later runs.

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
| `stopping_spots.py` | Says where on a route the drone stops (at corners), and moves those stops away from every place a boulder might be. |
| `start_frame.py` | Converts between the map's directions and the camera tracking's, which start from the launch mark. |
| `route_follower.py` | Slides a target along the route ahead of the drone, pausing at each corner. |
| `touch_detector.py` | Flags a suspected touch when the drone tilts or turns much more than in normal flight. |
| `height_check.py` | Flags PX4's height as untrustworthy when it disagrees with the bottom laser over clear floor. |
| `run_mode.py` | Reads the one setting that chooses competition or practice mode. Competition if it is missing. |
| `run_log.py` | Writes each program's records to disk as it goes, so a crash or power cut loses at most half a second. |
| `drone_reports.py` | Listens to everything the drone reports. It never sends anything. |
| `run_safety.py` | Decides each cycle whether the run carries on, holds still, lands or has been taken away, and refuses targets outside the course. |
| `target_sender.py` | Sends position and speed targets to the organizers' flight_interface; nothing else. |
| `flight_steps.py` | The connection to the drone and the steps every flight uses: wait until ready, wait for the start switch, climb, hold, land. |
| `course_run.py` | The competition program: plans the route and flies the whole course. |
| `high_path_rules.py` | Checks a path flown at 2.3 m, over the walls, against the bottom laser's rules: walls crossed square-on and at speed, no turning or stopping over a wall, a boulder or the tunnel. |
| `fallback_routes.py` | Plans the two fallback routes high over the walls (`fallback_pad`, `fallback_tunnel`) from the map. |
| `height_hold.py` | Makes the fallback missions hold camera tracking's height, never more than 0.15 m above the route, changing by at most 0.1 m/s, and ignoring camera height jumps. |
| `position_health.py` | Decides whether camera tracking's position can still be trusted: no silence, no jump, no PX4 reset. |
| `fallback_abort.py` | When a fallback run must stop with a healthy position: where to fly at the current height (the pad, or the nearest open floor). |
| `fallback_run.py` | Makes `course_run` fly a fallback route when `TEAM_NAV_MISSION` names one (`../TEST_SEQUENCE.md`, step 9F). |
| `tunnel_first_route.py` | Plans and checks the `tunnel_first` route on the drone's map: S above open floor before the tunnel, the high leg there, the GO line through the tunnel. |
| `heading_turn.py` | The slow 180 degree turn (0.5 degrees every 0.05 s, pausing if PX4 lags, never catching up) and the heading guard that compares PX4's heading with camera tracking's. |
| `tunnel_look.py` | Finds the tunnel's opening in a picture (edges) and in stereo depth points, and judges whether five frames agree. |
| `tunnel_look_camera.py` | Collects the left picture, its camera_info and depth_camera's points for the LOOK. |
| `tunnel_first_look.py` | The LOOK step: look, slide if an edge is cut off, correct sideways, or fall back to the map's line. |
| `tunnel_first_run.py` | Makes `course_run` fly `tunnel_first` when `TEAM_NAV_MISSION=tunnel_first` (`../TEST_SEQUENCE.md`, step 9T). |
| `launch_pieces.py`, `launch/*.launch.py` | What starts on the drone: `watch_only`, `test_flights` or `course_run`. |
| `drone.sh` | The one command typed on the drone's computer (`../TEST_SEQUENCE.md`). |

### The maps we keep

| File | What it is |
|---|---|
| `course_maps/organizers_drawing.json` | Made from the organizers' 3D model. |
| `course_maps/phone_scan_2026-11-06.json` | Made from our phone scan of the real course. This is our best map. One wall we doubt (A1) is copied from the drawing instead. It has the scan's three low things and the drawing's five boulders, because boulders are easy to move. |
| `course_maps/phone_scan_2026-11-06_trusting_every_wall.json` | The scan with every wall taken from the scan. The tests use it as a changed course. |
| `course_maps/organizers_new_start_2026-11-08.json` | The organizers' changed course of 2026-11-08: the three maze walls replaced by three new walls in the start area, measured from their new 3D model by `course_map_tools/make_map_from_new_organizers_model.py`; everything else from the drawing, checked against the new model. **The new diagonal wall is 0.08 m from the launch mark's centre, so no route can start from the mark** (the drone needs 0.35 m). The checkpoint lines "end of first corridor" and "Gate A" are kept from the drawing, which placed them from the old maze walls. |
| `course_maps/practice_mock_gap_and_window.json` | The practice course we build in the net: a gap, then a window over a 0.4 m box, then a pad. Its sizes are a proposal until measured. Its mission is `missions/practice_mock_gap_and_window.json`; `practice_flights/README.md` describes it. |

The programs that make these maps are in [`../../course_map_tools/`](../../course_map_tools/README.md). Every shape in a map has a `source` note saying where its numbers came from.

### Where the planned routes go

Measured on 2026-11-07.

| Map | Mission | Length | Least room | Closest a moving drone comes to a boulder's edge | Closest stop to a boulder's edge |
|---|---|---|---|---|---|
| Organizers' drawing | side gap first | 17.8 m | 0.35 m, next to the wall with the window | 0.25 m or more | 0.43 m |
| Our phone scan | side gap first | 17.3 m | 0.35 m, next to the third maze wall | 0.25 m or more | 0.38 m |
| Scan, trusting every wall | side gap first | 18.2 m | 0.35 m, next to the third maze wall | 0.25 m or more | 0.38 m |
| Organizers' drawing | window first | 17.2 m | 0.38 m, next to the first maze wall | 0.17 m, just past the window | 0.41 m |
| Our phone scan | window first | 18.3 m | 0.35 m, next to the third maze wall | 0.09 m, just past the window | 0.38 m |
| Scan, trusting every wall | window first | 18.4 m | 0.35 m, next to the third maze wall | 0.09 m, just past the window | 0.38 m |

"Least room" is the closest the drone's centre comes to anything it could hit. The drone needs at least 0.35 m: 0.30 m to its propeller tips, plus 0.05 m to spare. On the scan, the tightest place is the gap between the north end of the third maze wall and the net.

The planner tries to keep the drone's centre 0.25 m from every boulder's edge while it moves, because a boulder under the bottom laser makes the drone's height reading jump. It's a strong preference, not a ban, so the route can still get through if a boulder blocks the only way. Near the window it can't keep the full 0.25 m, so the route lists those legs, and the flight program must keep moving there. `route_planner.py`'s header explains this.

Stops are kept further away, 0.40 m, because a slow drone is when PX4 is most likely to stop trusting the laser. The flight program also gives the planner the boulders from the organizers' drawing, because boulder positions are uncertain (`stopping_spots.py`'s header explains this). At the window, the drone climbs to the window's height on the spot, 0.56 m in front of the wall, so both of its stops at the window keep at least 0.40 m from every boulder (`opening_crossings.py`).

To see the routes, run `course_map_tools/draw_planned_routes.py` (see its folder's README).

## The tests

The tests in `test/` plan routes on our three maps and on changed copies of the drawing and the scan: the window too small, something behind the window, Gate B closed as well, the tunnel blocked, the first corridor closed or narrowed, a pole in the way, boulders added, moved or removed, a maze wall moved or removed, and replanning from partway through the course. Each test either checks that the route is safe or checks that the planner explains clearly why there is no route. `test/test_route_follower.py` flies a pretend drone along a route to check the follower's rules, including that it holds the window's height while any part of it is over the window's lower wall. `test/test_stopping_spots.py` checks how stops are moved away from boulders, on made-up routes and on our maps. `test/test_fallback_routes.py` checks the two fallback routes on our maps and on changed mazes, and the laser rules on made-up paths; `test/test_fallback_safety.py` checks the height hold's limits (camera height jumping up, down, or freezing), the position health, the two-way abort and the launch pose check. `test/test_tunnel_first.py` checks the turn ramp, the heading guard, the start check, that a map target reaches PX4 unchanged whatever the drone's heading, and the route on our maps. `test/test_touch_detector.py` and `test/test_height_check.py` check the two safety checks with made-up readings.

A route's safety is checked by `test/route_checks.py`, which measures the route again from the map rather than trusting the planner's own numbers. It also checks that every leg near a boulder is listed, so the flight program knows to keep moving there. `test/course_changes.py` makes the changed courses.

Run them from the top folder of the repository. They take about 45 seconds.

```bash
docker run --rm --volume "$PWD:/repository" --workdir /repository --env PYTHONPATH=/repository/drone_code/team_nav drone-simulator python3 -m pytest drone_code/team_nav/test -q -p no:cacheprovider
```

## The other files

`package.xml` and `setup.py` describe the package to ROS. `setup.py` also installs the map and mission files, so the drone can find them. `resource/team_nav` lets ROS find the package, and `team_nav/__init__.py` is empty.
