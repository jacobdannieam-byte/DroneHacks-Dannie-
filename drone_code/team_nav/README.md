# team_nav

This is our team's ROS 2 package for the real drone. It holds the mission program. That program flies either the course stages or a route of waypoints, and lands with the bottom laser sensor (section 7 of [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)). A position watchdog runs for the whole flight.

[`launch/team_nav.launch.py`](launch/team_nav.launch.py) starts these programs on the drone:
- the organizers' flight interface, exactly one, with `auto_offboard_on_arm` on and `target_system` set to 16, as their example launch file starts it (two would fight over PX4's control flags);
- our mission program;
- our corner counter (`corner_count`, on by default), which counts FAST corners in the left camera's picture for the pre-start check;
- our depth program (`depth_points`, on by default), which makes stereo depth points on `/team/depth_points`.

The last two are separate programs, so a crash in either can't stop the mission or the flight.

`target_system` is drone 16's PX4 system ID (`MAV_SYS_ID` is 16, measured in `~/skybeta/px4/params/fleet/drone16-vio-2026-11-03.params`). The flight interface in skybeta 0.3.2 defaults it to 1, and then its Offboard request would go to a system that isn't there.

Nothing moves until a person acts. Only the pilot can arm the drone, on the remote control. Control passes to the mission only after the pilot arms on the ground and then flips the switch to Position. The pilot can take the drone back at any moment, and the mission then stops sending targets. Rehearse the start with the propellers off before the first flight.

The `mission` launch argument chooses what flies:
- `mission:=stages`, the default: the course stages.
- `mission:=route`: the waypoints in [`missions/route_waypoints.json`](missions/route_waypoints.json).
- `mission:=none`: only the flight interface starts.

On the drone, where launch arguments can't be given, [`launch/team_nav_route.launch.py`](launch/team_nav_route.launch.py) starts the route mission. It is the same launch file with `mission` defaulting to `route`; point `COMPETITOR_LAUNCH` at it.

Other settings work the same way. For example, `cruise_height_metres:=0.6` changes the cruise height, and every watchdog threshold and route speed is a launch argument too.

## Before the start (fixed after drone 16 on 2026-11-07)

On 2026-11-07 the drone sat still on the takeoff spot, and the mission flapped once a second between "not ready: camera tracking is frozen" and "ready". While "not ready" it sent no target, so the hand-over never happened. A drone standing still can report bit-identical positions, and that is normal. Now:

- While camera tracking reports keep arriving (none missing for 0.5 s), the mission sends a hold target at the drone's position 10 times a second, whatever else is wrong. The organizers' code only hands over control if it has heard a target in the last second.
- `prestart_check.py` says **not ready** only if the reports' own timestamps stop advancing for 0.5 s, or if the position has been bit-identical for over 3 s while the left camera is also blind (fewer than 30 FAST corners, from `corner_count`). Once ready, it stays ready for at least 2 s. If Offboard starts while it is not ready, the start is refused and a gentle descent is commanded.
- A missing corner count never blocks a start on its own. In flight, watchdog rule (c) still catches tracking that is frozen while the drone is asked to move.
- Every 5 s while waiting, the log shows camera tracking's position, heading and corner count. If the drone is more than 1.0 m sideways or 0.3 m up or down from where the stack started (`origin_check_horizontal_metres`, `origin_check_vertical_metres`), it logs `THE DRONE WAS MOVED AFTER THE STACK STARTED`. That only warns. Targets are always held at least 0.3 m above the stack's start point, so a drone carried to the net climbs to 0.3 m above wherever the stack started. Restart the stack on the takeoff spot ([`../restart_drone_16.sh`](../README.md)).

The blind threshold of 30 corners is an estimate. Check the real count with the drone's camera covered and uncovered.

## Depth points on the drone (`depth_points`)

`depth_points` reads `/oak/left/image_raw` and `/oak/right/image_raw` (topic names from the organizers' guide), and each camera's lens numbers from `/oak/left/camera_info` and `/oak/right/camera_info` when it starts. It publishes `/team/depth_points` about 10 times a second, in exactly the simulator's format: `sensor_msgs/PointCloud2`, x, y, z float32 = forward, left, up in metres from the drone's centre.

How it works, using the numbers measured on this drone earlier:
1. It rectifies both pictures and shrinks them to 512 x 320, where the focal length is about 317.25 px.
2. It matches them with OpenCV's StereoSGBM, numDisparities 64, about 15 ms. Depth = 23.79 / disparity in metres.
3. It keeps 0.37 to 4.0 m at every 4th pixel, and adds the camera's place on the drone (launch arguments `camera_forward_metres` 0.116, `camera_left_metres` 0.037, `camera_up_metres` 0.029).

If camera_info has no baseline, it uses the OAK-D-S2's 0.075 m. It assumes the camera looks straight along the nose. Turn it off with `depth_points:=false`. Every topic name is a launch argument.

**It needs OpenCV (`python3-opencv`).** The Ubuntu 22.04 package works: it was checked under ROS 2 Humble in the simulator image, with OpenCV installed. On synthetic pictures of a wall 1.19 m from the camera, it gave 1.190 m at 9.8 clouds per second. We don't know yet whether the drone's stack has OpenCV. If it doesn't, `depth_points` logs one error line saying so and stops, and nothing else is affected. To add it, put `RUN apt-get install -y python3-opencv` in `~/team_ws/Dockerfile` and run `host/run.sh build` (organizers' guide, section 5). The corner counter doesn't need OpenCV.

Check it against a tape measure with `drone_code/depth_check_16.sh skybeta-16.local`, holding the drone 1.0, 1.5 and 2.0 m from a textured wall.

## Reading order

All flight decisions are plain Python with no ROS in them, so they run and are tested on a laptop. Read them in this order:

1. [`team_nav/mission.py`](team_nav/mission.py) covers the whole flight: waiting on the launch mark, the takeoff, the course, the landing, and the rules checked at every step.
2. [`team_nav/position_watchdog.py`](team_nav/position_watchdog.py) decides when the position can no longer be trusted, which lands the drone at once.
   - [`team_nav/prestart_check.py`](team_nav/prestart_check.py) decides, before the start, whether a start would be accepted, and warns if the drone was moved after the stack started.
3. [`team_nav/course_stages.py`](team_nav/course_stages.py) covers the stages between takeoff and landing: follow_line, gate_a, tunnel_align, tunnel_cross, tunnel_exit, chicane and find_pad.
4. [`team_nav/route_flight.py`](team_nav/route_flight.py) is the route mission. It flies the waypoints one leg at a time at 0.5 m/s. At a corner that turns by 25° or more it stops and turns in place at up to 25°/s; at a shallower one it flies straight through and lets the heading blend round. A waypoint can carry a height: the leg into it is flown at that height, and the height changes while the drone is stopped before that leg, never during a leg.
   - [`team_nav/drift_correction.py`](team_nav/drift_correction.py) and [`team_nav/depth_ahead.py`](team_nav/depth_ahead.py) are the drift correction at route corners (below).
   - [`team_nav/stereo_depth.py`](team_nav/stereo_depth.py) turns the two cameras' pictures into depth points, and [`team_nav/depth_points_node.py`](team_nav/depth_points_node.py) is the program that runs it on the drone. [`team_nav/depth_check_node.py`](team_nav/depth_check_node.py) prints the depth straight ahead for a tape-measure check.
5. [`team_nav/landing.py`](team_nav/landing.py) is the landing from section 7.
6. [`team_nav/position_watch.py`](team_nav/position_watch.py) spots frozen tracking before a start, and measures the speed actually flown.
7. [`team_nav/mission_inputs.py`](team_nav/mission_inputs.py) holds what the detectors report, and decides whether each report can still be trusted.
8. [`team_nav/mission_settings.py`](team_nav/mission_settings.py) has every number, with its unit and where it came from.
9. [`team_nav/mission_node.py`](team_nav/mission_node.py) is the thin ROS 2 program that passes messages in and out, 20 times a second.
10. [`team_nav/camera_exposure.py`](team_nav/camera_exposure.py) caps the camera's automatic exposure, so the pictures camera tracking depends on stay sharp.
    - [`team_nav/corner_count_node.py`](team_nav/corner_count_node.py) is the corner counter program. It uses [`team_nav/camera_images.py`](team_nav/camera_images.py) to read camera pictures and [`team_nav/fast_corners.py`](team_nav/fast_corners.py) to count corners, both plain numpy, so they work even if OpenCV is missing on the drone.
11. [`launch/team_nav.launch.py`](launch/team_nav.launch.py) says what starts on the drone, and why that is safe.

The route's waypoints live in files in [`missions/`](missions/), not in code, because the judges take points off for routes written into code. The launch argument `route_file` picks one by name:

| File | What it is |
|---|---|
| `route_waypoints.json` (the default) | The team's hand-made 20-waypoint route. It keeps 0.40 m from walls except at Gate B. |
| `route_waypoints_drawing.json` | Planned by Kenny's route planner (`origin/classical`) on the organizers' drawing, at 0.8 m. |
| `route_waypoints_scan.json` | Planned by the same planner on the phone scan of the real course (2026-11-06), at 0.8 m. |

A waypoint is `[forward, left]`, or `[forward, left, height]` when the leg into it must be flown at a height other than cruise. The height is in metres above the launch floor. Each planned file sets the planner's window crossing height on its two window waypoints: **0.76 m** on the drawing (waypoints 19 and 20) and **1.19 m** on the scan (waypoints 22 and 23). The window on the scan's map is 0.81 to 1.56 m high, so 0.8 m there would hit its lower edge. The files also keep every height the planner wanted, in `planned_heights_metres`, for information only. A waypoint where the height changes is never flown through.

## The drift correction (route mission, off by default)

Camera tracking has no loop closure, so its error grows with distance flown. In the simulator, with its pretend drift of 1% of the distance flown, the drone ended about 0.2 m from where it thought it was. With `drift_correction:=true`, each time the route stops at a corner, before it turns:

1. `depth_ahead.py` takes the median depth of the points within 3° of straight ahead, from the depth topic `depth_topic`. With fewer than 5 points there, or a reading older than 0.5 s, there is no good reading.
2. `drift_correction.py` measures the distance to the nearest wall straight ahead in the course map (`course_map_file`), from where the drone thinks it is, along its heading, at its height.
3. If the map's wall is 0.4 to 2.5 m away and the depth differs from it by less than 0.4 m, the position estimate for the rest of the route moves along the heading by the difference, at most 0.2 m per correction. A depth shorter than the map moves the estimate forward. The safety net is skipped, because a stereo camera can't see a net.
4. Every correction, and every skip with its reason, goes in the log ("drift correction at waypoint …").

The correction is added to camera tracking's position before the route sees it, and taken off every target sent. PX4's own estimate is never touched. The watchdog and the landing never see the correction, and it is cleared at every start.

On the drone the depth comes from `depth_points` (below), on `/team/depth_points`, the default `depth_topic`. The simulator passes its pretend depth camera's topic instead. The correction also needs the right start heading: `start_heading_degrees` (default 90, north up the first corridor, as the simulator and Kenny's planner assume). The course maps and the code that reads them (`course_map.py`, `start_frame.py`, `course_maps/`) are Kenny's, copied unchanged from `origin/classical` at commit `8d848da` (the commit the planned routes came from). That branch has since added `line_pieces_cross` to `course_map.py`; we don't use it.

## What it hears and sends

| | Topic | Why |
|---|---|---|
| hears | `/visual_slam/tracking/odometry` | The drone's position, heading and timestamp from camera tracking. |
| hears | `/skybeta/state` | Whether Offboard is active and whether the drone is armed or landed. This is JSON from the organizers' code. |
| hears | `/fmu/out/vehicle_local_position` | `dist_bottom` (the bottom laser distance), plus `xy_reset_counter` and `heading_reset_counter` for watchdog rule (e). It is read only. This name has no version suffix on drone 16 (measured). Set the parameter `px4_local_position_topic` to empty, and the organizers' code works the name out. |
| hears | `/fmu/out/vehicle_status_v1` | `nav_state`, PX4's own mode, for watchdog rule (f). It is read only. PX4's status message is at version 1, hence `_v1` (measured on drone 16 and in the simulator). |
| hears | `depth_topic` (only with `drift_correction:=true`) | Depth points for the drift correction, as `sensor_msgs/PointCloud2`. By default `/team/depth_points`, from `depth_points`. |
| hears | `/team/left_corner_count` | The FAST corner count in the left camera (`corner_count`), for the pre-start check. |
| hears | `/visual_slam/status` | Camera tracking's own `vo_state`. It is only logged and never lands the drone, because on drone 16 it stayed at 1 ("tracking") while the camera was covered. |
| sends | `/skybeta/setpoint/pose` | Position targets: 10 times a second while waiting (a hold at the drone), then every step while climbing and cruising. |
| sends | `/skybeta/setpoint/twist` | Velocity commands while landing, because position targets can't go below 0.3 m. They are in world directions (x East, y North, z Up), whatever way the nose points. |
| sets | camera parameters on `/oak` | The automatic exposure cap: `r_auto_exposure_limit` 8000 on the left and right cameras first, then the switches that turn it on. It is applied again if the camera program restarts. The node name is a parameter (`camera_node_name`); check it on the drone with `ros2 param list /oak`. |

It never publishes to `/fmu/in/*`. It never arms the drone or asks for Offboard.

## Rules from the organizer code audit

These come from reading the organizers' flight interface. Each one has a test.

- **The fallback trap.** When a velocity command expires, or `/skybeta/hold` is called, the drone flies back to the last position target it was sent, wherever that was. Two things guard against this. Just before the landing's first velocity command, the mission sends a position target at the drone's current position. After that, velocity commands go out every step, 20 times a second, until PX4 switches the motors off. The 170 s limit only starts a landing. It never interrupts one.
- **No speed limit on position targets.** PX4 will fly to a position target at up to 12 m/s. So no target is ever more than 0.5 m from the drone, and the log records each new peak horizontal speed.
- **The flight state repeats old values.** Before anything advances, the mission checks three things separately: camera tracking reports have arrived recently, PX4's position reports have arrived recently, and the flight state is fresh and says the PX4 link is up. If any check fails, the drone holds, and it lands if that lasts 5 s.
- **Touchdown needs three signs together.** The mission must be commanding a descent, position reports must show the drone has stopped moving down, and the laser must be at the floor reading. The laser never reads below 0.1 m, so it can't confirm touchdown alone. Without a usable laser or position reports, the drone keeps descending gently, and PX4 lands it a few seconds later.
- **Frozen camera tracking counts as unhealthy in flight.** On drone 16, odometry once kept arriving 30 times a second with a bit-identical position for many seconds after tracking died. While a target asks the drone to move, a freeze trips watchdog rule (c). While hovering, more than 10 identical positions in a row means hold, then land after 5 s. The landing never treats a frozen position as stillness. Before the start, the rule is different (see "Before the start" above), because a drone standing still can report identical positions.
- **Each run starts clean.** When a run starts, everything learned in the previous run is cleared: the pad position, the space seen free, the counts, the fault, the timers and the detector reports. Once PX4 has disarmed the drone on the ground, the mission is ready for another run. A new run starts the same way as the first: the pilot arms, then flips the switch to Position. PX4 can still report Offboard after disarming. That is never treated as a start, and the drone gets no target while it lasts.

## The position watchdog

From takeoff until the motors stop, `position_watchdog.py` checks these rules. When one trips, the drone lands at once using the normal landing, and the log names the rule. During a landing, a trip is only logged, because the landing is already the response.

| Rule | Trips when | Default threshold (launch argument) |
|---|---|---|
| (a) | No new camera tracking report arrives. | 0.5 s (`watchdog_odometry_timeout_seconds`). Measured on drone 16, `ros2 topic hz` on `/visual_slam/tracking/odometry` showed gaps of up to 0.332 s at a normal 30 Hz, so 0.3 s would land the drone for nothing. The simulator's reports also pause for 0.21 to 0.3 s at times. PX4 itself stops using the reports after about 0.4 s, so this rule trips after PX4 has already stopped using them. |
| (b) | A report moved further from the one before than a believable speed allows. The time between them comes from the reports' own timestamps. | 1.5 m/s × time + 0.05 m (`watchdog_jump_speed_metres_per_second`, `watchdog_jump_margin_metres`) |
| (c) | Position and orientation stay bit-identical while a target asks the drone to move. | 0.5 s (`watchdog_frozen_seconds`). "Asks to move" means a target over 0.05 m away, over 2° round, or a velocity over 0.02 m/s (`watchdog_motion_request_*`). |
| (e) | `xy_reset_counter` or `heading_reset_counter` changes after takeoff. | No threshold. |
| (f) | PX4's status leaves Offboard after showing it. This stops all targets instead of landing. | No threshold. |

The team's list has no rule (d). No topic we receive carries a feature (FAST corner) count, so there is nothing to build one on yet.

## What it does today

The line detector, pad detector, depth clearances and camera tracking health don't exist yet, so those inputs start out missing, and missing counts as invalid. With no detectors running, a flight goes like this:

1. It takes off when the drone hands over control.
2. It climbs to 0.8 m.
3. It holds for 5 s, because camera tracking health is missing.
4. It lands using the section 7 landing.

This matches test steps 7 and 8 of the plan: a short low hover, then an automatic landing. As each detector starts writing to `Mission.inputs`, the stages that use it begin to work.

## Camera tracking on drone 16

Both of these were measured on the drone:
- cuVSLAM runs with `enable_imu_fusion` false. It's pure stereo visual odometry, and no motion sensor carries the estimate through darkness or a blank view. That is why the tunnel stage is off by default.
- It also runs with `enable_localization_n_mapping` false. There's no loop closure, so the tracker never corrects drift itself.

So it can freeze, jump or slowly drift, and the tracker won't notice. `position_watch.py` watches for the first two.

## Tests

From the top of the repository, run:

```bash
python3 -m pytest drone_code/team_nav/test
```

Four helper files in `test/` exist only for the tests and never run on the drone. `fake_drone.py` stands in for PX4 and the organizers' flight interface. `fake_ros.py` stands in for ROS 2, including the parameter client, so `mission_node.py` and `camera_exposure.py` can be tested without ROS. `fake_launch.py` does the same for the launch file. `conftest.py` lets the tests find the package. To fly the package in the simulator, with this launch file, run `simulator/run_team_nav.sh` from the top of the repository. It launches with three simulator-only settings: `target_system:=1`, `vision_check:=false` and `depth_topic:=/simulator_only/depth_points`. The simulator has no camera images, so nothing reports camera tracking health. `vision_check` defaults to true, and the mission refuses false unless that script started it inside the simulator box. None of this has flown on the drone yet.

## Other files

`package.xml`, `setup.py` and `setup.cfg` describe the package to ROS. `resource/team_nav` lets ROS find it. To install the package on the drone, see [`../README.md`](../README.md).
