"""Wait for a delivered simulation clock before ingesting ahead-of-clock PX4 data.

Original callback and source stamps are retained. The H1 ingestion timestamp is
the later processing time, never a fabricated callback arrival. Only the
explicit simulated-estimator adapter may use this fence. No offset is fitted.
"""
from collections import deque

class SimulationClockFence:
    def __init__(self, mapping, max_wait_ns=50_000_000, capacity=32):
        self.mapping=mapping; self.max_wait_ns=max_wait_ns; self.capacity=capacity
        self.pending=[]; self.audit=deque(maxlen=2048); self.deferred=0; self.released=0; self.last_clock=None

    def accept(self, topic, message, received_ns, wall_ns):
        for item in self.pending:
            if wall_ns-item[3]>self.max_wait_ns or received_ns-item[2]>self.max_wait_ns:
                self.pending.clear()
                raise ValueError('sim_clock_wait_expired')
        if topic=='/clock':
            clock=message.clock.sec*10**9+message.clock.nanosec
            if self.last_clock is not None and clock<self.last_clock:
                self.pending.clear()
                raise ValueError('sim_clock_rewind')
            self.last_clock=clock
            output=[(topic,message,received_ns,wall_ns)]
            waiting=[]
            for old_topic,old_message,arrival,wall,target in self.pending:
                if target<=min(clock,received_ns):
                    self.released+=1
                    self.audit.append({'topic':old_topic,'callback_ros_ns':arrival,
                        'callback_wall_ns':wall,'ingestion_ros_ns':received_ns,
                        'ingestion_wall_ns':wall_ns,'mapped_capture_upper_ns':target})
                    output.append((old_topic,old_message,received_ns,wall_ns))
                else: waiting.append((old_topic,old_message,arrival,wall,target))
            self.pending=waiting
            return output
        if topic.startswith('/fmu/out/') and hasattr(message,'timestamp_sample'):
            target=max(int(message.timestamp_sample),int(getattr(message,'timestamp',0)))*1000+self.mapping.offset_ns+self.mapping.max_error_ns
            if target>received_ns or any(item[0]==topic for item in self.pending):
                if target-received_ns>self.max_wait_ns or len(self.pending)>=self.capacity:
                    self.pending.clear()
                    raise ValueError('sim_clock_future_or_overload')
                self.pending.append((topic,message,received_ns,wall_ns,target)); self.deferred+=1
                return []
        return [(topic,message,received_ns,wall_ns)]

    def close(self):
        self.pending.clear()

    def report(self):
        return {'deferred':self.deferred,'released':self.released,'pending':len(self.pending),
            'audit_complete':self.released<=len(self.audit),'audit':list(self.audit)}
