"""Record actual independent Gazebo overhead RGB frames; no ROS/control access."""
import json
from pathlib import Path
import queue
import signal
import struct
import sys
import threading
import time
import zlib

from gz.msgs10.image_pb2 import Image, RGB_INT8
from gz.transport13 import Node


def png(width,height,data,step):
    def chunk(kind,body):
        return struct.pack('!I',len(body))+kind+body+struct.pack('!I',zlib.crc32(kind+body)&0xffffffff)
    rows=b''.join(b'\x00'+data[y*step:y*step+width*3] for y in range(height))
    return (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',width,height,8,2,0,0,0))
            +chunk(b'IDAT',zlib.compress(rows,1))+chunk(b'IEND',b''))


def main():
    root=Path(sys.argv[1]);root.mkdir(exist_ok=False,parents=True)
    pending=queue.Queue(maxsize=32);stop=threading.Event();dropped=[0];count=0
    def receive(message):
        if message.pixel_format_type!=RGB_INT8:
            print('Unsupported image format',message.pixel_format_type,flush=True);return
        stamp=message.header.stamp.sec+message.header.stamp.nsec/1e9
        try:pending.put_nowait((stamp,message.width,message.height,message.step,bytes(message.data)))
        except queue.Full:dropped[0]+=1
    signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    node=Node()
    if not node.subscribe(Image,'/course_observer/image',receive):raise RuntimeError('Camera subscription refused')
    with (root/'frames.jsonl').open('x') as index:
        while not stop.is_set() or not pending.empty():
            try:stamp,w,h,step,data=pending.get(timeout=.2)
            except queue.Empty:continue
            name=f'{count:06d}.png';(root/name).write_bytes(png(w,h,data,step))
            index.write(json.dumps(dict(file=name,source_ros_s=stamp,width=w,height=h))+'\n');index.flush();count+=1
            if count%50==0:print('Recorded',count,'actual overhead frames',flush=True)
    (root/'capture-summary.json').write_text(json.dumps(dict(frames=count,dropped_queue_frames=dropped[0],source='Gazebo independent overhead RGB camera',synthetic_reconstruction=False),indent=2))


if __name__=='__main__':main()
