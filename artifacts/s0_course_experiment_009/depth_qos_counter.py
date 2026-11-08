import json,time
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Image
from rosgraph_msgs.msg import Clock
rclpy.init(); node=rclpy.create_node('s0_depth_qos_ab_counter',enable_rosout=False,start_parameter_services=False)
start=time.monotonic(); begin=start+3; end=begin+12; rows={'best_effort':[],'reliable':[]}; clock=[]
def cb(key):
 def receive(m):
  wall=time.monotonic()
  if wall>=begin: rows[key].append((m.header.stamp.sec*10**9+m.header.stamp.nanosec,wall,len(m.data)))
 return receive
for key,reliability in [('best_effort',ReliabilityPolicy.BEST_EFFORT),('reliable',ReliabilityPolicy.RELIABLE)]:
 node.create_subscription(Image,'/simulator_only/oak/depth/image',cb(key),QoSProfile(depth=1,reliability=reliability))
node.create_subscription(Clock,'/clock',lambda m:clock.append(m.clock.sec*10**9+m.clock.nanosec),QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
while rclpy.ok() and time.monotonic()<end: rclpy.spin_once(node,timeout_sec=.02)
report={'scope':'simultaneous read-only image QoS counter; no command publishers','window_wall_s':12,'clock_span_s':None if len(clock)<2 else (clock[-1]-clock[0])/1e9,'publishers':[{'name':x.node_name,'reliability':str(x.qos_profile.reliability)} for x in node.get_publishers_info_by_topic('/simulator_only/oak/depth/image')],'subscribers':{}}
for key,values in rows.items():
 stamps=[x[0] for x in values]; gaps=[(b-a)/1e9 for a,b in zip(stamps,stamps[1:])]
 report['subscribers'][key]={'count':len(values),'unique_captures':len(set(stamps)),'first_capture_ns':stamps[0] if stamps else None,'last_capture_ns':stamps[-1] if stamps else None,'maximum_capture_gap_s':max(gaps) if gaps else None,'message_bytes':values[-1][2] if values else None}
print(json.dumps(report,indent=2)); node.destroy_node(); rclpy.shutdown()
