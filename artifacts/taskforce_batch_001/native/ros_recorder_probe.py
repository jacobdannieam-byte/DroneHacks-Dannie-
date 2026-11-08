import json,time
from pathlib import Path
from types import SimpleNamespace
import rclpy
from px4_msgs.msg import VehicleLocalPosition
from refnav.node import PX4_QOS
from examples.qualification_recorder import TrialRecording
rclpy.init()
node=rclpy.create_node('recorder_private_probe')
record=TrialRecording('fixture_system','ned')
pub=node.create_publisher(VehicleLocalPosition,'/qualification/position',PX4_QOS)
sub=node.create_subscription(VehicleLocalPosition,'/qualification/position',
    lambda m:record.sample(m,node.get_clock().now().nanoseconds),PX4_QOS)
deadline=time.monotonic()+5
while pub.get_subscription_count()!=1:
    assert time.monotonic()<deadline
    rclpy.spin_once(node,timeout_sec=.01)
message=VehicleLocalPosition()
message.timestamp_sample=node.get_clock().now().nanoseconds//1000
message.x=0.;message.y=0.;message.z=-.8
message.vx=.2;message.vy=0.;message.vz=0.
message.xy_valid=message.z_valid=message.v_xy_valid=message.v_z_valid=True
pub.publish(message)
while not record.document['samples']:
    assert time.monotonic()<deadline
    rclpy.spin_once(node,timeout_sec=.01)
now=node.get_clock().now().nanoseconds
record.status(SimpleNamespace(arming_state=1,timestamp=now//1000),now)
assert 'disarm_s' not in record.document
record.status(SimpleNamespace(arming_state=2,timestamp=now//1000+1),now+1000)
record.status(SimpleNamespace(arming_state=1,timestamp=now//1000+2),now+2000)
assert 'disarm_s' in record.document
assert not any(n.startswith('/fmu/in/') for n,_ in node.get_topic_names_and_types())
Path('/evidence/ros-recorder.json').write_text(json.dumps(dict(result='PASS',
    scope='private ROS position delivery plus status fixtures; no vehicle or command publisher',
    recording=record.result()),indent=2))
node.destroy_node();rclpy.shutdown()
