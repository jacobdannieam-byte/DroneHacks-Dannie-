"""Offline deserialization only; never initializes ROS."""
import json,sqlite3,hashlib,sys,math
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
root=Path(sys.argv[1]); bag=root/'observations'; path=next(bag.glob('*.db3'))
assert (bag/'metadata.yaml').exists()
assert not any(p.stat().st_size for p in bag.glob('*-wal'))
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
initial=sha(path)
db=sqlite3.connect(path.resolve().as_uri()+'?mode=ro&immutable=1',uri=True)
assert db.execute('pragma quick_check').fetchone()[0]=='ok'
rows={}
for i,n,t in db.execute('select id,name,type from topics'):
 if n in ['/fmu/out/vehicle_local_position','/fmu/out/vehicle_status_v1','/gazebo_hybrid/simulated_estimator/odometry'] or n.startswith('/fmu/in/') and n!='/fmu/in/vehicle_visual_odometry':
  cls=get_message(t)
  rows[n]=[(ts,deserialize_message(b,cls)) for ts,b in db.execute('select timestamp,data from messages where topic_id=? order by id',(i,))]
pos=rows['/fmu/out/vehicle_local_position'];odom=rows['/gazebo_hybrid/simulated_estimator/odometry'];status=rows['/fmu/out/vehicle_status_v1']
p=min(pos,key=lambda x:abs(x[1].timestamp_sample-27500000));o=min(odom,key=lambda x:abs(x[0]-p[0]))
q=o[1].pose.pose.position
mapped=[p[1].x,-p[1].y,-p[1].z]
report=dict(scope='independent pre-arm S0 refusal audit; no flight qualification',raw_sha256=initial,
 status_count=len(status),arming_states=sorted(set(m.arming_state for _,m in status)),
 control_topic_counts={n:len(v) for n,v in rows.items() if n.startswith('/fmu/in/')},
 native_near_failure=dict(receipt_ns=p[0],source_us=p[1].timestamp_sample,position_ned_m=[p[1].x,p[1].y,p[1].z],velocity_ned_mps=[p[1].vx,p[1].vy,p[1].vz]),
 odom_near_failure=dict(receipt_ns=o[0],source_ns=o[1].header.stamp.sec*10**9+o[1].header.stamp.nanosec,position_m=[q.x,q.y,q.z]),
 assumed_ned_to_odom_m=mapped,measured_odom_minus_assumed_m=[a-b for a,b in zip([q.x,q.y,q.z],mapped)],
 all_native_position_ranges={k:[min(getattr(m,k) for _,m in pos),max(getattr(m,k) for _,m in pos)] for k in ['x','y','z']},
 all_odom_position_ranges={k:[min(getattr(m.pose.pose.position,k) for _,m in odom),max(getattr(m.pose.pose.position,k) for _,m in odom)] for k in ['x','y','z']})
db.close();assert sha(path)==initial
print(json.dumps(report,indent=2))
