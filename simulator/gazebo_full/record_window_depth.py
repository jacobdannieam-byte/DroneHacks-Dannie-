"""Record stereo cloud/pose pairs for offline Gazebo window-alignment debugging.

SIMULATOR ANALYSIS ONLY. Subscribes to ordinary sensor messages, publishes nothing.
Run inside the simulation container with a destination folder argument. A file every
second limits disk use; pose association must be within 0.10 seconds of exposure.
"""
import sys
from pathlib import Path
from collections import deque
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'drone_code/team_nav'))
from team_nav.depth_camera import points_from_message,OBSTACLE_POINTS_TOPIC


def main():
    destination = Path(sys.argv[1]);destination.mkdir(parents=True,exist_ok=True)
    rclpy.init();node = rclpy.create_node('window_depth_observer')
    history = deque(maxlen=150)
    last = [0.]
    def stamp(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9
    def pose(message):
        p,q = message.pose.pose.position,message.pose.pose.orientation
        history.append((stamp(message),[p.x,p.y,p.z],[q.x,q.y,q.z,q.w]))
    def cloud(message):
        seconds = stamp(message)
        if not history or seconds-last[0]<1.:
            return
        match = min(history,key=lambda row:abs(row[0]-seconds))
        if abs(match[0]-seconds)>.10:
            return
        np.savez_compressed(destination/f'{seconds:.3f}.npz',points=points_from_message(message),
                            position=match[1],quaternion=match[2],stamp=seconds)
        last[0] = seconds
    node.create_subscription(Odometry,'/visual_slam/tracking/odometry',pose,10)
    node.create_subscription(PointCloud2,OBSTACLE_POINTS_TOPIC,cloud,2)
    rclpy.spin(node)


if __name__=='__main__':
    main()
