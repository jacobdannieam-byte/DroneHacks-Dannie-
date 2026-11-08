"""Read-only timestamp audit; run with pinned ROS/px4_msgs, no ROS nodes."""
import json
import sqlite3
from collections import defaultdict
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

db_path = Path('/raw/observations_0.db3')
results = {}
with sqlite3.connect(db_path.as_uri() + '?mode=ro&immutable=1', uri=True) as db:
    for topic_id, name, kind in db.execute('select id,name,type from topics'):
        if name not in ('/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_local_position', '/clock'):
            continue
        offsets, gaps, states, invalid = [], [], set(), 0
        previous = None
        for receipt, data in db.execute('select timestamp,data from messages where topic_id=? order by id', (topic_id,)):
            message = deserialize_message(data, get_message(kind))
            if name == '/clock':
                stamp = message.clock.sec * 10**9 + message.clock.nanosec
            else:
                stamp = getattr(message, 'timestamp_sample', message.timestamp) * 1000
                if name.endswith('vehicle_status_v1'):
                    states.add(int(message.arming_state))
                else:
                    invalid += not all(getattr(message, key) for key in ('xy_valid','z_valid','v_xy_valid','v_z_valid'))
            offsets.append(receipt-stamp)
            if previous is not None:
                gaps.append(stamp-previous)
            previous = stamp
        values = sorted(offsets)
        results[name] = {'count':len(values), 'receipt_minus_source_ns':
            {'min':min(values),'max':max(values),'median':values[len(values)//2],
             'negative_count':sum(v<0 for v in values)},
            'source_gap_ns':{'min':min(gaps),'max':max(gaps),'nonpositive_count':sum(g<=0 for g in gaps)},
            'arming_states':sorted(states), 'invalid_state_count':invalid}
print(json.dumps({'scope':'recorded disarmed bag timestamps; no live clock mapping approval', 'topics':results},indent=2))
