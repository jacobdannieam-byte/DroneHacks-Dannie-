# team_nav

This is our team's ROS 2 package for the real drone. It holds the mission program, which flies the course stages and lands with the bottom laser sensor (section 7 of [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)).

[`launch/team_nav.launch.py`](launch/team_nav.launch.py) starts two programs on the drone, the same way the organizers' example launch file does:
- the organizers' flight interface, with `auto_offboard_on_arm` on and `target_system` set to 16;
- our mission program.

`target_system` is drone 16's PX4 system ID (`MAV_SYS_ID` is 16, measured in `~/skybeta/px4/params/fleet/drone16-vio-2026-11-03.params`). The flight interface in skybeta 0.3.2 defaults it to 1, and then its Offboard request would go to a system that isn't there.

Nothing moves until a person acts. Only the pilot can arm the drone, on the remote control. Control passes to the mission only after the pilot arms on the ground and then flips the switch to Position. The pilot can take the drone back at any moment, and the mission then stops sending targets. Rehearse the start with the propellers off before the first flight.

To bring up only the flight interface, launch with `mission:=false`. To change the cruise height, launch with `cruise_height_metres:=0.6`.

## Reading order

All flight decisions are plain Python with no ROS in them, so they run and are tested on a laptop. Read them in this order:

1. [`team_nav/mission.py`](team_nav/mission.py) covers the whole flight: waiting on the launch mark, the takeoff, the course, the landing, and the rules checked at every step.
2. [`team_nav/course_stages.py`](team_nav/course_stages.py) covers the stages between takeoff and landing: follow_line, gate_a, tunnel_align, tunnel_cross, tunnel_exit, chicane and find_pad.
3. [`team_nav/landing.py`](team_nav/landing.py) is the landing from section 7.
4. [`team_nav/position_watch.py`](team_nav/position_watch.py) spots sudden position jumps and measures the speed actually flown.
5. [`team_nav/mission_inputs.py`](team_nav/mission_inputs.py) holds what the detectors report, and decides whether each report can still be trusted.
6. [`team_nav/mission_settings.py`](team_nav/mission_settings.py) has every number, with its unit and where it came from.
7. [`team_nav/mission_node.py`](team_nav/mission_node.py) is the thin ROS 2 program that passes messages in and out, 20 times a second.
8. [`team_nav/camera_exposure.py`](team_nav/camera_exposure.py) caps the camera's automatic exposure, so the pictures camera tracking depends on stay sharp.
9. [`launch/team_nav.launch.py`](launch/team_nav.launch.py) says what starts on the drone, and why that is safe.

## What it hears and sends

| | Topic | Why |
|---|---|---|
| hears | `/visual_slam/tracking/odometry` | The drone's position and heading from camera tracking. |
| hears | `/skybeta/state` | Whether Offboard is active and whether the drone is armed or landed. This is JSON from the organizers' code. |
| hears | `/fmu/out/vehicle_local_position` | Only `dist_bottom`, the bottom laser distance. It is read only. This name has no version suffix on drone 16 (measured). Set the parameter `px4_local_position_topic` to empty, and the organizers' code works the name out. |
| sends | `/skybeta/setpoint/pose` | Position targets while waiting, climbing and cruising. |
| sends | `/skybeta/setpoint/twist` | Velocity commands while landing, because position targets can't go below 0.3 m. They are in world directions (x East, y North, z Up), whatever way the nose points. |
| sets | camera parameters on `/oak` | The automatic exposure cap: `r_auto_exposure_limit` 8000 on the left and right cameras first, then the switches that turn it on. It is applied again if the camera program restarts. The node name is a parameter (`camera_node_name`); check it on the drone with `ros2 param list /oak`. |

It never publishes to `/fmu/in/*`. It never arms the drone or asks for Offboard.

## Rules from the organizer code audit

These come from reading the organizers' flight interface. Each one has a test.

- **The fallback trap.** When a velocity command expires, or `/skybeta/hold` is called, the drone flies back to the last position target it was sent, wherever that was. Two things guard against this. Just before the landing's first velocity command, the mission sends a position target at the drone's current position. After that, velocity commands go out every step, 20 times a second, until PX4 switches the motors off. The 170 s limit only starts a landing. It never interrupts one.
- **No speed limit on position targets.** PX4 will fly to a position target at up to 12 m/s. So no target is ever more than 0.5 m from the drone, and the log records each new peak horizontal speed.
- **The flight state repeats old values.** Before anything advances, the mission checks three things separately: camera tracking reports have arrived recently, PX4's position reports have arrived recently, and the flight state is fresh and says the PX4 link is up. If any check fails, the drone holds, and it lands if that lasts 5 s.
- **After a position jump, the pilot recovers the drone.** The mission stops advancing and sends only a zero velocity. It sends no position hold, because the position no longer means anything. The pilot then takes over.
- **Touchdown needs three signs together.** The mission must be commanding a descent, position reports must show the drone has stopped moving down, and the laser must be at the floor reading. The laser never reads below 0.1 m, so it can't confirm touchdown alone. Without a usable laser or position reports, the drone keeps descending gently, and PX4 lands it a few seconds later.
- **Frozen camera tracking counts as unhealthy.** On drone 16, odometry once kept arriving 30 times a second with a bit-identical position for many seconds after tracking died. More than 10 identical positions in a row now counts as frozen. In flight, a freeze means hold, then land, as when reports stop. The landing never treats a frozen position as stillness. Before a start, no target is sent while tracking is frozen, so the organizers' code can't hand over control, and the log says why. If Offboard still begins in the moment before the freeze is spotted, the start is refused. The mission then commands a gentle descent, which keeps the drone on the floor until PX4 switches the motors off.
- **Each run starts clean.** When a run starts, everything learned in the previous run is cleared: the pad position, the space seen free, the counts, the fault, the timers and the detector reports. Once PX4 has disarmed the drone on the ground, the mission is ready for another run. A new run starts the same way as the first: the pilot arms, then flips the switch to Position. PX4 can still report Offboard after disarming. That is never treated as a start, and the drone gets no target while it lasts.

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
