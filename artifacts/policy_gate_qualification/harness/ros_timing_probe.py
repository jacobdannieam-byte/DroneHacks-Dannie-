"""Private ROS callbacks only; no flight, no live PX4 interfaces."""
import json
from pathlib import Path
from types import SimpleNamespace
import rclpy
from refnav.node import RefNav
from refnav.owner_timing import OwnerTiming

args=['--ros-args','-p','route:=[0.0,0.0]','-p','exit_when_done:=false']
for topic in ('offboard_control_mode','trajectory_setpoint','vehicle_command'):
    args+=['-r',f'/fmu/in/{topic}:=/qualification/{topic}']
rclpy.init(args=args)
node=RefNav()
for timer in node.timers: timer.cancel()
invalidations=[]
node._hybrid_runtime=SimpleNamespace(invalidate=lambda:invalidations.append(True),
    loop=SimpleNamespace(replay=SimpleNamespace(hybrid=SimpleNamespace(clock=SimpleNamespace(
        capture=lambda stamp,now: stamp*1000, max_error_ns=0)))))
node._owner_timing=OwnerTiming(authority_ns=750_000_000,landed_ns=1_500_000_000)
rows=[]
for key,age,expected in [('authority',.504,True),('authority',.8,False),
    ('landed',1.,True),('landed',1.6,False),('position',.31,False)]:
    node._hybrid_reports.clear()
    now=node.get_clock().now().nanoseconds
    node._note_hybrid_report(key,True,int((now-age*1e9)//1000))
    accepted=key in node._hybrid_reports
    assert accepted==expected
    rows.append(dict(stream=key,age_s=age,accepted=accepted))
assert len(invalidations)>=3
assert not any(name.startswith('/fmu/in/') for name,_ in node.get_topic_names_and_types())
Path('/evidence/ros-owner-timing.json').write_text(json.dumps(dict(result='PASS',
    scope='actual ROS report ingestion with fixture stamps; no flight or PX4 fallback qualification',cases=rows),indent=2))
node._hybrid_runtime=None
node.destroy_node(); rclpy.shutdown()
