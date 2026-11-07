# team_nav

This is our team's ROS 2 package for the real drone. It holds the mission program. That program flies either the course stages or a route of waypoints, and lands with the bottom laser sensor (section 7 of [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)). A position watchdog runs for the whole flight.

[`launch/team_nav.launch.py`](launch/team_nav.launch.py) starts two programs on the drone, the same way the organizers' example launch file does:
- the organizers' flight interface, with `auto_offboard_on_arm` on and `target_system` set to 16;
- our mission program.

`target_system` is drone 16's PX4 system ID (`MAV_SYS_ID` is 16, measured in `~/skybeta/px4/params/fleet/drone16-vio-2026-11-03.params`). The flight interface in skybeta 0.3.2 defaults it to 1, and then its Offboard request would go to a system that isn't there.

Nothing moves until a person acts. Only the pilot can arm the drone, on the remote control. Control passes to the mission only after the pilot arms on the ground and then flips the switch to Position. The pilot can take the drone back at any moment, and the mission then stops sending targets. Rehearse the start with the propellers off before the first flight.

The `mission` launch argument chooses what flies:
- `mission:=stages`, the default: the course stages.
- `mission:=route`: the waypoints in [`missions/route_waypoints.json`](missions/route_waypoints.json).
- `mission:=none`: only the flight interface starts.

Other settings work the same way. For example, `cruise_height_metres:=0.6` changes the cruise height, and every watchdog threshold and route speed is a launch argument too.

## Reading order

All flight decisions are plain Python with no ROS in them, so they run and are tested on a laptop. Read them in this order:

1. [`team_nav/mission.py`](team_nav/mission.py) covers the whole flight: waiting on the launch mark, the takeoff, the course, the landing, and the rules checked at every step.
2. [`team_nav/position_watchdog.py`](team_nav/position_watchdog.py) decides when the position can no longer be trusted, which lands the drone at once.
3. [`team_nav/course_stages.py`](team_nav/course_stages.py) covers the stages between takeoff and landing: follow_line, gate_a, tunnel_align, tunnel_cross, tunnel_exit, chicane and find_pad.
4. [`team_nav/route_flight.py`](team_nav/route_flight.py) is the route mission. It flies the waypoints one leg at a time, stops at each corner, and turns in place no faster than 15°/s.
5. [`team_nav/landing.py`](team_nav/landing.py) is the landing from section 7.
6. [`team_nav/position_watch.py`](team_nav/position_watch.py) spots frozen tracking before a start, and measures the speed actually flown.
7. [`team_nav/mission_inputs.py`](team_nav/mission_inputs.py) holds what the detectors report, and decides whether each report can still be trusted.
8. [`team_nav/mission_settings.py`](team_nav/mission_settings.py) has every number, with its unit and where it came from.
9. [`team_nav/mission_node.py`](team_nav/mission_node.py) is the thin ROS 2 program that passes messages in and out, 20 times a second.
10. [`team_nav/camera_exposure.py`](team_nav/camera_exposure.py) caps the camera's automatic exposure, so the pictures camera tracking depends on stay sharp.
11. [`launch/team_nav.launch.py`](launch/team_nav.launch.py) says what starts on the drone, and why that is safe.

The route's waypoints live in [`missions/route_waypoints.json`](missions/route_waypoints.json), not in code, because the judges take points off for routes written into code.

## What it hears and sends

| | Topic | Why |
|---|---|---|
| hears | `/visual_slam/tracking/odometry` | The drone's position, heading and timestamp from camera tracking. |
| hears | `/skybeta/state` | Whether Offboard is active and whether the drone is armed or landed. This is JSON from the organizers' code. |
| hears | `/fmu/out/vehicle_local_position` | `dist_bottom` (the bottom laser distance), plus `xy_reset_counter` and `heading_reset_counter` for watchdog rule (e). It is read only. This name has no version suffix on drone 16 (measured). Set the parameter `px4_local_position_topic` to empty, and the organizers' code works the name out. |
| hears | `/fmu/out/vehicle_status_v1` | `nav_state`, PX4's own mode, for watchdog rule (f). It is read only. PX4's status message is at version 1, hence `_v1` (measured on drone 16 and in the simulator). |
| hears | `/visual_slam/status` | Camera tracking's own `vo_state`. It is only logged and never lands the drone, because on drone 16 it stayed at 1 ("tracking") while the camera was covered. |
| sends | `/skybeta/setpoint/pose` | Position targets while waiting, climbing and cruising. |
| sends | `/skybeta/setpoint/twist` | Velocity commands while landing, because position targets can't go below 0.3 m. They are in world directions (x East, y North, z Up), whatever way the nose points. |
| sets | camera parameters on `/oak` | The automatic exposure cap: `r_auto_exposure_limit` 8000 on the left and right cameras first, then the switches that turn it on. It is applied again if the camera program restarts. The node name is a parameter (`camera_node_name`); check it on the drone with `ros2 param list /oak`. |

It never publishes to `/fmu/in/*`. It never arms the drone or asks for Offboard.

## Rules from the organizer code audit

These come from reading the organizers' flight interface. Each one has a test.

- **The fallback trap.** When a velocity command expires, or `/skybeta/hold` is called, the drone flies back to the last position target it was sent, wherever that was. Two things guard against this. Just before the landing's first velocity command, the mission sends a position target at the drone's current position. After that, velocity commands go out every step, 20 times a second, until PX4 switches the motors off. The 170 s limit only starts a landing. It never interrupts one.
- **No speed limit on position targets.** PX4 will fly to a position target at up to 12 m/s. So no target is ever more than 0.5 m from the drone, and the log records each new peak horizontal speed.
- **The flight state repeats old values.** Before anything advances, the mission checks three things separately: camera tracking reports have arrived recently, PX4's position reports have arrived recently, and the flight state is fresh and says the PX4 link is up. If any check fails, the drone holds, and it lands if that lasts 5 s.
- **Touchdown needs three signs together.** The mission must be commanding a descent, position reports must show the drone has stopped moving down, and the laser must be at the floor reading. The laser never reads below 0.1 m, so it can't confirm touchdown alone. Without a usable laser or position reports, the drone keeps descending gently, and PX4 lands it a few seconds later.
- **Frozen camera tracking counts as unhealthy.** On drone 16, odometry once kept arriving 30 times a second with a bit-identical position for many seconds after tracking died. While a target asks the drone to move, a freeze trips watchdog rule (c). While hovering, more than 10 identical positions in a row means hold, then land after 5 s. The landing never treats a frozen position as stillness. Before a start, no target is sent while tracking is frozen, so the organizers' code can't hand over control, and the log says why. If Offboard still begins in the moment before the freeze is spotted, the start is refused. The mission then commands a gentle descent, which keeps the drone on the floor until PX4 switches the motors off.
- **Each run starts clean.** When a run starts, everything learned in the previous run is cleared: the pad position, the space seen free, the counts, the fault, the timers and the detector reports. Once PX4 has disarmed the drone on the ground, the mission is ready for another run. A new run starts the same way as the first: the pilot arms, then flips the switch to Position. PX4 can still report Offboard after disarming. That is never treated as a start, and the drone gets no target while it lasts.

## The position watchdog

From takeoff until the motors stop, `position_watchdog.py` checks these rules. When one trips, the drone lands at once using the normal landing, and the log names the rule. During a landing, a trip is only logged, because the landing is already the response.

| Rule | Trips when | Default threshold (launch argument) |
|---|---|---|
| (a) | No new camera tracking report arrives. | 0.2 s (`watchdog_odometry_timeout_seconds`) |
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

Four helper files in `test/` exist only for the tests and never run on the drone. `fake_drone.py` stands in for PX4 and the organizers' flight interface. `fake_ros.py` stands in for ROS 2, including the parameter client, so `mission_node.py` and `camera_exposure.py` can be tested without ROS. `fake_launch.py` does the same for the launch file. `conftest.py` lets the tests find the package. To fly the package in the simulator, with this launch file, run `simulator/run_team_nav.sh` from the top of the repository. It launches with two simulator-only settings: `target_system:=1` and `vision_check:=false`. The simulator has no camera images, so nothing reports camera tracking health. `vision_check` defaults to true, and the mission refuses false unless that script started it inside the simulator box. None of this has flown on the drone yet.

## Other files

`package.xml`, `setup.py` and `setup.cfg` describe the package to ROS. `resource/team_nav` lets ROS find it. To install the package on the drone, see [`../README.md`](../README.md).
