"""Actual RefNav callbacks and publishers, remapped to private ROS test topics.

Vehicle state is a fixture, not a flight or PX4 failsafe qualification.
"""
import json, time
from pathlib import Path
import rclpy
from px4_msgs.msg import VehicleStatus, TrajectorySetpoint, VehicleCommand
from refnav.node import RefNav, PX4_QOS
from refnav.core import NAV_OFFBOARD

args=['--ros-args','-p','route:=[0.0,0.0]','-p','exit_when_done:=false']
for name in ('offboard_control_mode','trajectory_setpoint','vehicle_command'):
    args+=['-r',f'/fmu/in/{name}:=/qualification/{name}']
args+=['-r','/fmu/out/vehicle_status_v1:=/qualification/status']
rclpy.init(args=args)
fixture=rclpy.create_node('authority_fixture')
status=fixture.create_publisher(VehicleStatus,'/qualification/status',PX4_QOS)
messages=[]; commands=[]
subs=[fixture.create_subscription(TrajectorySetpoint,'/qualification/trajectory_setpoint',lambda msg:messages.append(msg.timestamp),PX4_QOS),
      fixture.create_subscription(VehicleCommand,'/qualification/vehicle_command',lambda msg:commands.append(msg.command),PX4_QOS)]
results=[]
for case in ('native_disarm','native_takeover'):
    node=RefNav()
    for timer in node.timers: timer.cancel()
    end=time.monotonic()+5
    while status.get_subscription_count()<1 or node.pub_sp.get_subscription_count()<1:
        assert time.monotonic()<end
        rclpy.spin_once(node,timeout_sec=.01)
    now=node._now()
    node.mission.phase='hover'; node.mission.t_phase=now; node.mission.offboard_seen=True
    node.mission.p.hover_s=100.
    node.mission.sp=[0.,0.,-.8,0.]; node.mission.home=(0.,0.,0.,0.)
    node.mission.on_health(now,True); node.mission.depth_t=now
    node.v.pos=(0.,0.,-.8); node.v.pos_ok=True; node.v.heading=0.; node.v.landed=False
    node.v.armed=True; node.v.nav_state=NAV_OFFBOARD
    before=len(messages); node._tick()
    assert node.mission.phase=='hover'
    end=time.monotonic()+3
    while len(messages)==before:
        assert time.monotonic()<end
        rclpy.spin_once(fixture,timeout_sec=.01)
    event=VehicleStatus(); event.arming_state=1 if case=='native_disarm' else 2
    event.nav_state=NAV_OFFBOARD if case=='native_disarm' else 2
    event.timestamp=fixture.get_clock().now().nanoseconds//1000
    status.publish(event)
    end=time.monotonic()+3
    while node.v.armed!=(case!='native_disarm') or node.v.nav_state!=event.nav_state:
        assert time.monotonic()<end
        rclpy.spin_once(node,timeout_sec=.01)
    before=len(messages); cmd_before=len(commands)
    for _ in range(10):
        node._tick(); rclpy.spin_once(fixture,timeout_sec=.01)
    assert len(messages)==before and len(commands)==cmd_before
    assert node.mission.result.startswith('yielded:')
    results.append({'case':case,'result':'PASS','post_loss_ticks':10,'post_loss_targets':0,'post_loss_commands':0})
    node.destroy_node()
assert not any(name.startswith('/fmu/in/') for name,_ in fixture.get_topic_names_and_types())
Path('/evidence/ros-authority.json').write_text(json.dumps({'scope':'actual ROS owner with synthetic state; no PX4 or simulator','results':results},indent=2))
fixture.destroy_node(); rclpy.shutdown()
print(json.dumps(results))
