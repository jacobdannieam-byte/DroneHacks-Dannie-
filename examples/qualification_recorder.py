"""Subscriber-only qualification recorder. Never creates command publishers.

Events must be recorded by the single trial owner at publication time, in the
same ROS clock as the telemetry. Raw rosbag recording remains mandatory.
"""
import argparse
import json
import math
from pathlib import Path
from time import monotonic


class TrialRecording:
    def __init__(self, clock_id, frame_id):
        self.document = dict(clock_id=clock_id, frame_id=frame_id, samples=[], events=[])
        self.faults = []
        self.last_time = None
        self.seen_armed = False
        self.last_status_stamp = None

    def status(self, message, receipt_ns):
        stamp = int(message.timestamp) * 1000
        if not 0 <= receipt_ns - stamp <= 100_000_000:
            self.faults.append('status_clock_or_age'); return
        if self.last_status_stamp is not None and stamp <= self.last_status_stamp:
            self.faults.append('nonadvancing_status'); return
        self.last_status_stamp = stamp
        if message.arming_state == 2:
            self.seen_armed = True
        elif message.arming_state == 1:
            if self.seen_armed and 'disarm_s' not in self.document:
                self._record_event('disarm', stamp, receipt_ns, 'native_vehicle_status')
        else:
            self.faults.append('unknown_arming_state')

    def _record_event(self, name, stamp, receipt_ns, source):
        if not 0 <= receipt_ns-stamp <= 100_000_000 or name+'_s' in self.document:
            raise ValueError('invalid or duplicate event')
        self.document[name+'_s'] = stamp/1e9
        self.document['events'].append(dict(event=name, stamp_ns=stamp,
                                            receipt_ns=receipt_ns, source=source))

    def sample(self, message, receipt_ns):
        values = [message.x, message.y, message.z, message.vx, message.vy, message.vz]
        stamp = int(message.timestamp_sample) * 1000
        if (not all(math.isfinite(v) for v in values) or
                not all(getattr(message, field, False) for field in ('xy_valid','z_valid','v_xy_valid','v_z_valid'))):
            self.faults.append('invalid_position_or_velocity'); return
        if stamp > receipt_ns or receipt_ns-stamp > 100_000_000:
            self.faults.append('sample_clock_or_age'); return
        if self.last_time is not None and stamp <= self.last_time:
            self.faults.append('nonadvancing_sample'); return
        self.last_time = stamp
        self.document['samples'].append(dict(time_s=stamp/1e9, receipt_ns=receipt_ns,
            position_m=values[:3], velocity_mps=values[3:]))

    def event(self, raw, receipt_ns):
        try:
            event=json.loads(raw)
            name=event['event']; stamp=event['stamp_ns']
            if name not in ('brake_command','land_command') or type(stamp) is not int:
                raise ValueError()
            if not 0 <= receipt_ns-stamp <= 100_000_000 or name+'_s' in self.document:
                raise ValueError()
            self._record_event(name, stamp, receipt_ns, 'owner_event_topic')
        except (ValueError,KeyError,TypeError): self.faults.append('invalid_or_duplicate_event')

    def result(self):
        return dict(**self.document, recording_faults=list(self.faults),
                    recording_complete=not self.faults and all(k+'_s' in self.document for k in ('brake_command','land_command','disarm')))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=90)
    args=parser.parse_args()
    if args.output.exists() or not 1 <= args.seconds <= 120:
        raise ValueError('new output and bounded duration required')
    import rclpy
    from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
    from std_msgs.msg import String
    from refnav.node import PX4_QOS
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('qualification_recorder')
    record=TrialRecording('ros_sim_time','px4_local_ned')
    node.create_subscription(VehicleLocalPosition,'/fmu/out/vehicle_local_position',
        lambda m: record.sample(m,node.get_clock().now().nanoseconds),PX4_QOS)
    node.create_subscription(String,'/qualification/events',
        lambda m: record.event(m.data,node.get_clock().now().nanoseconds),10)
    node.create_subscription(VehicleStatus,'/fmu/out/vehicle_status_v1',
        lambda m: record.status(m,node.get_clock().now().nanoseconds),PX4_QOS)
    try:
        deadline=monotonic()+args.seconds
        while rclpy.ok() and monotonic()<deadline: rclpy.spin_once(node,timeout_sec=.1)
    finally:
        with args.output.open('x') as stream: json.dump(record.result(),stream,indent=2,allow_nan=False)
        node.destroy_node(); rclpy.shutdown()

if __name__=='__main__': main()
