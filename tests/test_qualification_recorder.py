from types import SimpleNamespace
import json
from examples.qualification_recorder import TrialRecording

def message(**changes):
    values=dict(timestamp_sample=1_000_000,x=0.,y=0.,z=-.8,vx=.2,vy=0.,vz=0.,
                xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True)
    values.update(changes); return SimpleNamespace(**values)

def test_preserves_capture_and_receipt():
    record=TrialRecording('fixture','ned'); record.sample(message(),1_004_000_000)
    row=record.result()['samples'][0]
    assert row['time_s']==1. and row['receipt_ns']==1_004_000_000
    assert not record.result()['recording_complete']

def test_invalid_telemetry_is_retained_as_failure():
    for changes in ({'vx':float('nan')},{'v_xy_valid':False},{'timestamp_sample':2_000_000}):
        record=TrialRecording('fixture','ned'); record.sample(message(**changes),1_004_000_000)
        assert record.result()['recording_faults'] and not record.result()['samples']

def test_event_duplicates_and_future_rejected():
    record=TrialRecording('fixture','ned')
    event=json.dumps(dict(event='brake_command',stamp_ns=1_000_000_000))
    record.event(event,1_000_000_000); record.event(event,1_000_000_000)
    assert record.result()['recording_faults']==['invalid_or_duplicate_event']


def test_event_topic_cannot_claim_native_disarm():
    record=TrialRecording('fixture','ned')
    record.event(json.dumps(dict(event='disarm',stamp_ns=1_000_000_000)),1_000_000_000)
    assert 'disarm_s' not in record.document
    assert record.faults


def test_only_fresh_native_disarmed_transition_counts():
    record=TrialRecording('fixture','ned')
    record.status(SimpleNamespace(arming_state=1,timestamp=1_000_000),1_000_000_000)
    assert 'disarm_s' not in record.document
    record.status(SimpleNamespace(arming_state=2,timestamp=1_100_000),1_100_000_000)
    record.status(SimpleNamespace(arming_state=0,timestamp=1_200_000),1_200_000_000)
    assert 'disarm_s' not in record.document
    record.status(SimpleNamespace(arming_state=1,timestamp=1_300_000),1_300_000_000)
    assert record.document['disarm_s']==1.3
    assert record.document['events'][-1]['source']=='native_vehicle_status'


def test_stale_native_disarm_refused():
    record=TrialRecording('fixture','ned')
    record.status(SimpleNamespace(arming_state=2,timestamp=1_000_000),1_000_000_000)
    record.status(SimpleNamespace(arming_state=1,timestamp=1_100_000),2_000_000_000)
    assert 'disarm_s' not in record.document
    assert 'status_clock_or_age' in record.faults


def test_rewound_native_status_does_not_create_disarm():
    record=TrialRecording('fixture','ned')
    record.status(SimpleNamespace(arming_state=2,timestamp=1_000_000),1_000_000_000)
    record.status(SimpleNamespace(arming_state=1,timestamp=990_000),1_010_000_000)
    assert 'disarm_s' not in record.document
    assert 'nonadvancing_status' in record.faults
