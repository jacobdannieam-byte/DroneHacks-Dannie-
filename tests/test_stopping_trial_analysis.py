import pytest
from examples.stopping_trial_analysis import analyze

def recording():
    return dict(clock_id='fixture',frame_id='fixture',brake_command_s=1.,land_command_s=3.,disarm_s=4.,
        samples=[dict(time_s=i/20,position_m=[min(i/20,1.5)*.2,0.,0.],
                      velocity_mps=[.2 if i<30 else 0.,0.,0.]) for i in range(81)])

def test_sustained_stop_and_landing():
    result=analyze(recording())
    assert result['reaction_s']==.5
    assert result['braking_distance_m']==pytest.approx(.1)
    assert result['landing_duration_s']==1.

@pytest.mark.parametrize('fault', ['gap','speed','nan','no_stop','no_start','coverage','landing'])
def test_bad_recording_cannot_qualify(fault):
    doc=recording()
    if fault=='gap': del doc['samples'][20:25]
    if fault=='speed': doc['samples'][10]['velocity_mps'][0]=.51
    if fault=='nan': doc['samples'][10]['position_m'][0]=float('nan')
    if fault=='no_stop':
        for r in doc['samples']: r['velocity_mps'][0]=.2
    if fault=='no_start':
        for r in doc['samples']: r['velocity_mps'][0]=0.
    if fault=='coverage': doc['samples']=doc['samples'][:-5]
    if fault=='landing': doc['disarm_s']=34.
    with pytest.raises(ValueError): analyze(doc)
