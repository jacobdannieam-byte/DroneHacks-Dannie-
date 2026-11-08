from dataclasses import replace
import numpy as np
from refnav.section_policy import SectionPolicy
from refnav.world_model.observations import Observation


def room(stamp=1., roof=.7):
    h,w=120,160
    u,v=np.meshgrid(np.arange(w),np.arange(h))
    right,down=(u-80)/30,(v-60)/30
    depth=np.minimum(4.,np.minimum(.8/np.maximum(abs(right),1e-9),
                                  roof/np.maximum(abs(down),1e-9)))
    return Observation(depth,(30.,30.,80.,60.),np.array([[0,0,1],[-1,0,0],[0,-1,0]]),
                       np.zeros(3),np.diag([1.,-1.,-1.]),np.zeros(3),stamp,stamp,str(stamp),frame_id='ned')


def test_tunnel_requires_independent_frames_and_observed_volume():
    policy=SectionPolicy([('Tunnel','exit',2)])
    for i in range(3):
        obs=room(1+i*.05);policy.offer(obs)
        result=policy.assess(0,np.zeros(3),[1,0,0],obs.stamp)
        assert result['allowed'] == (i==2)
    assert result['tunnel_checked']


def test_unknown_roof_and_low_roof_never_allow_tunnel():
    for missing in (True,False):
        policy=SectionPolicy([('Tunnel','exit',2)])
        for i in range(3):
            obs=room(1+i*.05,roof=.15 if not missing else .7)
            if missing:obs.depth_m[:60]=np.nan
            policy.offer(obs)
            assert not policy.assess(0,np.zeros(3),[1,0,0],obs.stamp)['allowed']


def test_stale_reset_and_duplicate_captures_fail_closed():
    policy=SectionPolicy([('Maze','exit',2)])
    policy.offer(room())
    assert not policy.assess(0,np.zeros(3),[1,0,0],2.)['allowed']
    policy.offer(replace(room(1.1),epoch=1))
    assert policy.fault=='section_reference_changed'
    policy=SectionPolicy([('Maze','exit',2)])
    policy.offer(room());policy.offer(room())
    assert policy.fault=='section_depth_rewind_or_duplicate'


def test_open_space_cannot_be_classified_as_tunnel():
    policy=SectionPolicy([('Tunnel','exit',2)])
    obs=room();obs.depth_m[:]=4
    policy.offer(obs)
    result=policy.assess(0,np.zeros(3),[1,0,0],1.)
    assert not result['allowed']
    assert result['reason']=='tunnel_floor_or_roof_unobserved'
