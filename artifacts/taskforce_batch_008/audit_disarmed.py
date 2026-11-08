"""Decode retained bags read-only; initializes no ROS nodes or publishers."""
import hashlib
import json
from pathlib import Path
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

root=Path('/raw')
reports=[]
for name in ('c1-check','r1-check'):
    folder=root/name/'observations'
    bags=list(folder.glob('*.db3'))
    assert len(bags)==1 and (folder/'metadata.yaml').is_file(),name
    assert not any(p.stat().st_size for p in folder.glob('*-wal')),name
    digest=hashlib.sha256(bags[0].read_bytes()).hexdigest()
    db=sqlite3.connect(bags[0].resolve().as_uri()+'?mode=ro&immutable=1',uri=True)
    assert db.execute('pragma quick_check').fetchone()[0]=='ok'
    controls={}
    arming=[]
    for ident,topic,kind in db.execute('select id,name,type from topics'):
        if topic.startswith('/fmu/in/') and topic!='/fmu/in/vehicle_visual_odometry':
            controls[topic]=db.execute('select count(*) from messages where topic_id=?',(ident,)).fetchone()[0]
        if topic=='/fmu/out/vehicle_status_v1':
            cls=get_message(kind)
            arming=[deserialize_message(blob,cls).arming_state for blob, in db.execute('select data from messages where topic_id=? order by id',(ident,))]
    db.close()
    assert arming and set(arming)=={1},(name,set(arming))
    assert not any(controls.values()),(name,controls)
    assert hashlib.sha256(bags[0].read_bytes()).hexdigest()==digest
    reports.append(dict(run=name,raw_bag_sha256=digest,native_status_count=len(arming),
        native_arming_states=sorted(set(arming)),control_message_counts=controls,result='PASS'))
Path('/output/disarmed-raw-audit.json').write_text(json.dumps({'scope':'raw native disarmed state and absence of control publication; no flight qualification','runs':reports},indent=2)+'\n')
print(json.dumps(reports))
