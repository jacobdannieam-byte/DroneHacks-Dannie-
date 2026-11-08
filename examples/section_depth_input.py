"""Capture-time depth adapter for the native owner; ROS topics supplied by launcher."""
from collections import deque
import numpy as np
from refnav.depth import decode_depth, quat_to_mat
from refnav.world_model.observations import Observation, rotation_ok


class SectionDepthInput:
    def __init__(self, policy, mapping):
        self.policy, self.mapping = policy, mapping
        self.positions, self.attitudes = deque(maxlen=120), deque(maxlen=120)
        self.info = self.extrinsics = self.pending = None
        self.transforms_by_child = {}
        self.resets = None
        self.attitude_reset = None
        self.last_depth_stamp = None
        self.rejected = {}

    def reject(self, reason):
        self.rejected[reason] = self.rejected.get(reason, 0)+1

    def position(self, msg):
        if not all(getattr(msg,k,False) for k in ('xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control')):
            return
        resets = tuple(getattr(msg,k) for k in ('xy_reset_counter','z_reset_counter','heading_reset_counter'))
        if self.resets is not None and resets != self.resets:
            self.policy.fault = 'section_pose_reset'
        self.resets = resets
        t = msg.timestamp_sample*1e-6+self.mapping.offset_ns/1e9
        error = max(float(msg.eph),float(msg.epv))
        angular_variance = max(float(msg.heading_var),float(msg.tilt_var))
        if not np.isfinite([error,angular_variance]).all() or error<=0 or angular_variance<=0:
            self.reject('position_uncertainty_unavailable'); return
        self.positions.append((t, np.array([msg.x,msg.y,msg.z]),error,float(np.sqrt(angular_variance))))

    def attitude(self, msg):
        q = np.array(msg.q)
        if not np.isfinite(q).all() or abs(np.linalg.norm(q)-1)>.001:
            return
        reset = getattr(msg,'quat_reset_counter',None)
        if reset is None:
            self.reject('attitude_reset_counter_missing'); return
        if self.attitude_reset is not None and reset!=self.attitude_reset:
            self.policy.fault='section_attitude_reset'
        self.attitude_reset=reset
        t = msg.timestamp_sample*1e-6+self.mapping.offset_ns/1e9
        self.attitudes.append((t, quat_to_mat(*q) @ np.diag([1.,-1.,-1.])))

    def camera_info(self, msg):
        p = tuple(msg.p)
        if any(msg.d) or p[3]!=0 or p[7]!=0:
            self.reject('unrectified_camera'); return
        k = (p[0],p[5],p[2],p[6]) if p[0]>0 else (msg.k[0],msg.k[4],msg.k[2],msg.k[5])
        info = (msg.header.frame_id,msg.width,msg.height,k)
        if self.info is not None and info != self.info:
            self.policy.fault = 'section_calibration_changed'
        self.info = info

    def transforms(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id != 'base_link':
                continue
            q = transform.transform.rotation
            rotation = quat_to_mat(q.w,q.x,q.y,q.z)
            t = transform.transform.translation
            if not rotation_ok(rotation):
                continue
            value = (transform.child_frame_id,rotation,np.array([t.x,t.y,t.z]))
            previous = self.transforms_by_child.get(value[0])
            if previous is not None and (not np.array_equal(value[1],previous[1]) or not np.array_equal(value[2],previous[2])):
                self.policy.fault = 'section_extrinsics_changed'
            self.transforms_by_child[value[0]] = value

    def depth(self, msg):
        self.pending = msg

    def update(self, now_s):
        if self.info is not None:
            self.extrinsics = self.transforms_by_child.get(self.info[0])
        msg = self.pending
        if msg is None or self.info is None or self.extrinsics is None or not self.positions or not self.attitudes:
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if not 0<=now_s-stamp<=.25:
            self.reject('depth_age'); self.pending=None; return
        if self.last_depth_stamp is not None and stamp<=self.last_depth_stamp:
            self.policy.fault='section_depth_rewind'; self.pending=None; return
        position = min(self.positions,key=lambda p:abs(p[0]-stamp))
        attitude = min(self.attitudes,key=lambda p:abs(p[0]-stamp))
        if max(abs(position[0]-stamp),abs(attitude[0]-stamp))>.03:
            self.reject('capture_pose_missing'); return
        frame,width,height,k = self.info
        if (msg.header.frame_id!=frame or frame!=self.extrinsics[0] or (msg.width,msg.height)!=(width,height)
                or msg.is_bigendian):
            self.reject('depth_calibration_mismatch'); self.pending=None; return
        try:
            depth = decode_depth(msg.encoding,height,width,msg.step,msg.data)
            obs = Observation(depth,k,self.extrinsics[1],self.extrinsics[2],attitude[1],position[1],
                              stamp,position[0],f'depth-{stamp:.9f}',frame_id='px4_local_ned',
                              clock_id=self.mapping.ros_clock_id,position_sigma_m=position[2],angle_sigma_rad=position[3])
            self.policy.offer(obs)
            self.last_depth_stamp = stamp
        except (ValueError,TypeError):
            self.reject('invalid_depth')
        self.pending = None


def attach(node, policy, mapping, position_topic, attitude_topic, depth_topic, info_topic):
    from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
    from sensor_msgs.msg import Image, CameraInfo
    from tf2_msgs.msg import TFMessage
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from refnav.node import PX4_QOS
    adapter = SectionDepthInput(policy,mapping)
    for kind,topic,callback,qos in (
        (VehicleLocalPosition,position_topic,adapter.position,PX4_QOS),
        (VehicleAttitude,attitude_topic,adapter.attitude,PX4_QOS),
        (Image,depth_topic,adapter.depth,qos_profile_sensor_data),
        (CameraInfo,info_topic,adapter.camera_info,qos_profile_sensor_data),
        (TFMessage,'/tf_static',adapter.transforms,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))):
        node.create_subscription(kind,topic,callback,qos)
    return adapter
