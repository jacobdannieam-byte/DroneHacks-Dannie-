# team_nav

This is our team's ROS 2 package for the real drone. It holds the mission program, which flies the course stages and lands with the bottom laser sensor (section 7 of [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)).

[`launch/team_nav.launch.py`](launch/team_nav.launch.py) starts two programs on the drone, the same way the organizers' example launch file does:
- the organizers' flight interface, with `auto_offboard_on_arm` on;
- our mission program.

Nothing moves until a person acts. Only the pilot can arm the drone, on the remote control. Control passes to the mission only after the pilot arms on the ground and then flips the switch to Position. The pilot can take the drone back at any moment, and the mission then stops sending targets. Rehearse the start with the propellers off before the first flight.

To bring up only the flight interface, launch with `mission:=false`. To change the cruise height, launch with `cruise_height_metres:=0.6`.

## Reading order

All flight decisions are plain Python with no ROS in them, so they run and are tested on a laptop. Read them in this order:

1. [`team_nav/mission.py`](team_nav/mission.py) covers the whole flight: waiting on the launch mark, the takeoff, the course, the landing, and the rules checked at every step.
2. [`team_nav/course_stages.py`](team_nav/course_stages.py) covers the stages between takeoff and landing: follow_line, gate_a, tunnel_align, tunnel_cross, tunnel_exit, chicane and find_pad.
3. [`team_nav/landing.py`](team_nav/landing.py) is the landing from section 7.
4. [`team_nav/mission_inputs.py`](team_nav/mission_inputs.py) holds what the detectors report, and decides whether each report can still be trusted.
5. [`team_nav/mission_settings.py`](team_nav/mission_settings.py) has every number, with its unit and where it came from.
6. [`team_nav/mission_node.py`](team_nav/mission_node.py) is the thin ROS 2 program that passes messages in and out.
7. [`launch/team_nav.launch.py`](launch/team_nav.launch.py) says what starts on the drone, and why that is safe.

## What it hears and sends

| | Topic | Why |
|---|---|---|
| hears | `/visual_slam/tracking/odometry` | The drone's position and heading from camera tracking. |
| hears | `/skybeta/state` | Whether Offboard is active and whether the drone is armed or landed. This is JSON from the organizers' code. |
| hears | `/fmu/out/vehicle_local_position` | Only `dist_bottom`, the bottom laser distance. It is read only. This name has no version suffix on drone 16 (measured). Set the parameter `px4_local_position_topic` to empty, and the organizers' code works the name out. |
| sends | `/skybeta/setpoint/pose` | Position targets while waiting, climbing and cruising. |
| sends | `/skybeta/setpoint/twist` | Velocity commands while landing, because position targets can't go below 0.3 m. |

It never publishes to `/fmu/in/*`. It never arms the drone or asks for Offboard.

## What it does today

The line detector, pad detector, depth clearances and camera tracking health don't exist yet, so those inputs start out missing, and missing counts as invalid. With no detectors running, a flight goes like this:

1. It takes off when the drone hands over control.
2. It climbs to 0.8 m.
3. It holds for 5 s, because camera tracking health is missing.
4. It lands using the section 7 landing.

This matches test steps 7 and 8 of the plan: a short low hover, then an automatic landing. As each detector starts writing to `Mission.inputs`, the stages that use it begin to work.

## Tests

From the top of the repository, run:

```bash
python3 -m pytest drone_code/team_nav/test
```

Four helper files in `test/` exist only for the tests and never run on the drone. `fake_drone.py` stands in for PX4 and the organizers' flight interface. `fake_ros.py` stands in for ROS 2, so `mission_node.py` can be tested without ROS. `fake_launch.py` does the same for the launch file. `conftest.py` lets the tests find the package. A real flight check still has to happen in the simulator and then on the drone. None of this has flown yet.

## Other files

`package.xml`, `setup.py` and `setup.cfg` describe the package to ROS. `resource/team_nav` lets ROS find it. To install the package on the drone, see [`../README.md`](../README.md).
