import pytest
from types import SimpleNamespace
from dataclasses import asdict
import json
from examples.stopping_trial_analysis import analyze

def recording():
    return dict(clock_id='fixture',frame_id='fixture',brake_command_s=1.,land_command_s=3.,disarm_s=4.,
        recording_complete=True, recording_faults=[],
        events=[dict(event=n,stamp_ns=t,receipt_ns=t,source=source) for n,t,source in
                [('brake_command',1_000_000_000,'owner_event_topic'),('land_command',3_000_000_000,'owner_event_topic'),('disarm',4_000_000_000,'native_vehicle_status')]],
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

@pytest.mark.parametrize('fault', ['no_complete','no_events','spoof_disarm','mismatch','duplicate'])
def test_missing_or_spoofed_provenance_refused(fault):
    doc=recording()
    if fault=='no_complete': del doc['recording_complete']
    if fault=='no_events': del doc['events']
    if fault=='spoof_disarm': doc['events'][-1]['source']='owner_event_topic'
    if fault=='mismatch': doc['events'][-1]['stamp_ns']-=1
    if fault=='duplicate': doc['events'].append(dict(doc['events'][-1]))
    with pytest.raises(ValueError): analyze(doc)


def test_motion_resuming_after_initial_settle_is_not_a_qualified_stop():
    doc=recording()
    for row in doc['samples']:
        t=row['time_s']
        row['position_m'][0]+=.4*max(0,min(t,3)-2.1)
        if 2.1<t<3: row['velocity_mps'][0]=.4
    with pytest.raises(ValueError,match='motion resumed'):
        analyze(doc)


def native_recording(error=0):
    from examples.h1_observation import ClockMapping, RESET_FIELDS
    from examples.qualification_recorder import TrialRecording
    mapping = ClockMapping('px4_fixture', 'ros_sim_time', 0, error, 0, 10_000_000_000, 'synthetic-test')
    record = TrialRecording('ros_sim_time', 'px4_local_ned', clock_mapping=mapping,
        trial_id='test', owner_id='/calibration_trial_owner', profile_sha256='a'*64)
    for i in range(20, 101):
        ns = i * 50_000_000
        record.sample(SimpleNamespace(timestamp=ns//1000, timestamp_sample=ns//1000,
            x=min(i/20,2.5)*.2, y=0., z=-1., vx=.2 if i<50 else 0., vy=0., vz=0.,
            xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True,
            **{k:0 for k in RESET_FIELDS}), ns+error, wall_ns=ns+error)
        if ns in (1_000_000_000,5_000_000_000):
            armed = 2 if ns==1_000_000_000 else 1
            record.status(SimpleNamespace(timestamp=ns//1000,arming_state=armed),ns+error,wall_ns=ns+error)
    for name,ns in (('brake_command',2_000_000_000),('land_command',4_000_000_000)):
        record.event(json.dumps(dict(event=name,stamp_ns=ns,px4_issue_us=ns//1000,
            trial_id='test',owner_id='/calibration_trial_owner',profile_sha256='a'*64)), ns,
            publisher=dict(publisher_node='/calibration_trial_owner',publisher_gid='01'*24,
                           publisher_binding='singleton_graph_only'))
    return record.result()


def test_strict_native_mapping_and_non_grid_uncertainty_window():
    doc = native_recording(error=1_000_000)
    result = analyze(doc, require_reviewed=True)
    assert result['reaction_s'] == pytest.approx(.501)
    assert result['landing_duration_s'] == pytest.approx(1.001)
    assert result['review_status'] == 'unreviewed'


@pytest.mark.parametrize('fault', ['identity','capture','reset','status','publisher','wait','hover'])
def test_strict_provenance_cannot_be_forged_or_omitted(fault):
    doc = native_recording()
    if fault == 'identity': doc['events'][-1]['profile_sha256']='b'*64
    if fault == 'capture': doc['samples'][0]['mapped_stamp_ns']+=1000
    if fault == 'reset': doc['samples'][-1]['reset_counters']['xy_reset_counter']=1
    if fault == 'status': doc['status_reports']=[]
    if fault == 'publisher': doc['events'][-1]['publisher_gid']='00'*24
    if fault == 'wait': doc['samples'][0]['ingestion_wall_ns']+=51_000_000
    if fault == 'hover': doc['trial_kind']='hover_land'
    with pytest.raises(ValueError): analyze(doc, require_reviewed=True)


def test_legacy_fixture_cannot_be_used_for_measurement_cli():
    with pytest.raises(ValueError,match='reviewed calibration'):
        analyze(recording(), require_reviewed=True)
