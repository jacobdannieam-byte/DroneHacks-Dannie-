from copy import deepcopy
import pytest
from examples.calibration_bag_audit import correlate


def fixture():
    identity=dict(trial_id='test',owner_id='/owner',profile_sha256='a'*64)
    event=dict(**identity,event='land_command',stamp_ns=1000,px4_issue_us=1)
    record=dict(**identity,provenance_scope='reviewed_calibration',recording_complete=True,
                trial_kind='hover_land',recording_faults=[],events=[event])
    return record,[dict(event)],[dict(timestamp=1,command=21,target_system=1,target_component=1)],[]


def test_c0_land_correlation_without_fabricated_brake():
    assert correlate(*fixture())['events_matched']==1


@pytest.mark.parametrize('fault',['stamp','identity','duplicate','wrong_command','missing','target','faulted'])
def test_mismatch_rejected(fault):
    rec,events,commands,targets=fixture()
    if fault=='stamp': commands[0]['timestamp']=2
    if fault=='identity': events[0]['trial_id']='other'
    if fault=='duplicate': events.append(dict(events[0]))
    if fault=='wrong_command': commands[0]['command']=400
    if fault=='missing': commands=[]
    if fault=='target': commands[0]['target_system']=2
    if fault=='faulted': rec['recording_faults']=['failed']
    with pytest.raises(ValueError): correlate(rec,events,commands,targets)


@pytest.mark.parametrize('position_mode', [False, True])
def test_stopping_brake_requires_velocity_control(position_mode):
    record,events,commands,targets=fixture()
    record['trial_kind']='stopping'
    brake=dict(events[0],event='brake_command',stamp_ns=500,px4_issue_us=2)
    record['events'].insert(0,brake)
    events.insert(0,dict(brake))
    targets=[dict(timestamp=2,velocity=[0.,0.,0.],position=[float('nan')]*3)]
    modes=[dict(timestamp=2,position=position_mode,velocity=not position_mode)]
    if position_mode:
        with pytest.raises(ValueError,match='zero-velocity'): correlate(record,events,commands,targets,modes)
    else:
        assert correlate(record,events,commands,targets,modes)['events_matched']==2
