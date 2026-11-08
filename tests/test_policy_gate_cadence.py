from dataclasses import replace
import pytest
from examples.h1_fixture import profile, setup, tick, MS
from examples.h1_observation import HybridBagReplay, TRACKING

@pytest.mark.parametrize('source,age,expected',[
    ('simulated_estimator',400,True),('visual',400,False),
    ('simulated_estimator',1250,True),('simulated_estimator',1251,False),
    ('simulated_estimator',-1,False)])
def test_integrated_flag_cadence_is_profile_specific(source,age,expected):
    selected=replace(profile(offset_rig=True),tracking_source=source)
    replay=HybridBagReplay(selected,monotonic=lambda:0.)
    class Driver:
        profile=selected.spatial
        def feed(self,topic,msg,stamp):
            if topic==TRACKING: msg.status[0].message=source
            replay.feed(topic,msg,stamp)
    setup(Driver()); tick(Driver(),1500,0.)
    replay._flags=((1503-age)*MS,True)
    reason,inputs=replay.prepare(1503*MS)
    assert (inputs is not None)==expected,reason
    replay._flags=(1503*MS,False)
    assert replay.prepare(1503*MS)[0]=='estimator_health_missing_or_stale'
