from types import SimpleNamespace as S
import numpy as np
from examples.section_depth_input import SectionDepthInput
from refnav.section_policy import SectionPolicy


def position(**changes):
    values=dict(timestamp_sample=1_000_000,xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
                heading_good_for_control=True,xy_reset_counter=0,z_reset_counter=0,heading_reset_counter=0,
                x=0,y=0,z=-.8,eph=.02,epv=.03,heading_var=.0004,tilt_var=.0001)
    return S(**dict(values,**changes))


def adapter():
    policy=SectionPolicy([('Tunnel','exit',2)])
    reader=SectionDepthInput(policy,S(offset_ns=0,ros_clock_id='fixture'))
    reader.position(position())
    reader.attitude(S(timestamp_sample=1_000_000,q=[1,0,0,0],quat_reset_counter=0))
    reader.camera_info(S(header=S(frame_id='camera'),width=2,height=2,d=[],p=[1,0,1,0,0,1,1,0,0,0,1,0]))
    reader.transforms(S(transforms=[S(header=S(frame_id='base_link'),child_frame_id='camera',
        transform=S(rotation=S(w=1,x=0,y=0,z=0),translation=S(x=.1,y=0,z=0)))]))
    reader.depth(S(header=S(frame_id='camera',stamp=S(sec=1,nanosec=0)),width=2,height=2,
                   encoding='32FC1',step=8,is_bigendian=False,data=np.ones((2,2),np.float32).tobytes()))
    return policy,reader


def test_capture_pose_and_reported_uncertainty_are_preserved():
    policy,reader=adapter();reader.update(1.01)
    assert policy.observation is not None
    assert policy.observation.position_sigma_m==.03
    assert policy.observation.angle_sigma_rad==.02
    assert policy.observation.pose_stamp==1.
    assert not reader.rejected


def test_pose_and_attitude_resets_latch_rejection():
    policy,reader=adapter()
    reader.position(position(xy_reset_counter=1))
    assert policy.fault=='section_pose_reset'
    policy,reader=adapter()
    reader.attitude(S(timestamp_sample=1_020_000,q=[1,0,0,0],quat_reset_counter=1))
    assert policy.fault=='section_attitude_reset'


def test_stale_depth_does_not_create_observation():
    policy,reader=adapter();reader.update(1.3)
    assert policy.observation is None and reader.rejected=={'depth_age':1}
