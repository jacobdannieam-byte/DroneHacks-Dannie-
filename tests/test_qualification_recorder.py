from types import SimpleNamespace
import json
from dataclasses import asdict
from hashlib import sha256
import pytest
from examples.h1_observation import ClockMapping
from onboard.diagnostics.phase1_probe import RESET_FIELDS
from examples.qualification_recorder import TrialRecording, event_publisher_identity, load_contract, write_ready, drain_clock_pending

def message(**changes):
    values=dict(timestamp_sample=1_000_000,x=0.,y=0.,z=-.8,vx=.2,vy=0.,vz=0.,
                xy_valid=True,z_valid=True,v_xy_valid=True,v_z_valid=True)
    values.update({field: 0 for field in RESET_FIELDS})
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


def mapped_record(*, error=0, until=20_000_000_000):
    mapping = ClockMapping('px4_boot', 'ros_sim_time', 2_000_000_000, error,
                           1_000_000_000, until, 'reviewed-clock-fixture')
    return TrialRecording('ros_sim_time', 'px4_local_ned', clock_mapping=mapping,
                          trial_id='trial-1', owner_id='/calibration_trial_owner', profile_sha256='a'*64)


def test_shutdown_finishes_pending_sample_without_rewriting_receipt():
    record = mapped_record(error=4_000_000)
    record.sample(message(timestamp=1_000_000), 2_996_000_000, wall_ns=1_000_000_000)
    wall = [1_000_000_000]
    def spin():
        wall[0] += 4_000_000
    drain_clock_pending(record, lambda: 3_004_000_000, spin, lambda: wall[0])
    assert not record.pending and not record.faults
    assert record.document['samples'][0]['receipt_ns'] == 2_996_000_000
    assert record.document['samples'][0]['ingestion_ns'] == 3_004_000_000


def test_shutdown_stalled_clock_preserves_failure_instead_of_dropping_sample():
    record = mapped_record()
    record.sample(message(timestamp=1_000_000), 2_996_000_000, wall_ns=1_000_000_000)
    wall = [1_000_000_000]
    def spin():
        wall[0] += 10_000_000
    drain_clock_pending(record, lambda: 2_996_000_000, spin, lambda: wall[0])
    assert record.faults and not record.document['samples']
    assert not record.result()['recording_complete']


def owner_event(record, name='brake_command', stamp=3_000_000_000):
    return dict(event=name, stamp_ns=stamp, px4_issue_us=record.mapping.source_stamp_us(stamp),
                **{key: record.document[key] for key in ('trial_id','owner_id','profile_sha256')})


def test_explicit_offset_preserves_native_mapped_and_receipt_times():
    record = mapped_record()
    record.sample(message(timestamp=1_001_000),3_004_000_000,wall_ns=100)
    row, = record.document['samples']
    assert row['source_stamp_us'] == 1_000_000 and row['source_publish_us'] == 1_001_000
    assert row['mapped_stamp_ns'] == 3_000_000_000 and row['time_s'] == 3.
    assert row['receipt_ns'] == row['ingestion_ns'] == 3_004_000_000
    assert record.document['clock_mapping']['offset_ns'] == 2_000_000_000
    assert record.document['provenance_scope'] == 'reviewed_calibration'
    assert TrialRecording('fixture','ned').document['provenance_scope'] == 'legacy_synthetic'


def test_ahead_of_clock_sample_and_status_keep_original_callback_times():
    record = mapped_record()
    record.sample(message(timestamp=1_000_000),2_996_000_000,wall_ns=1_000_000_000)
    record.status(SimpleNamespace(timestamp=1_000_000,arming_state=2),
                  2_996_000_000,wall_ns=1_000_000_000)
    assert not record.seen_armed and not record.document['samples'] and len(record.pending) == 2
    record.clock(3_000_000_000,wall_ns=1_004_000_000)
    assert record.seen_armed and not record.pending and not record.faults
    sample, = record.document['samples']
    assert sample['receipt_ns'] == 2_996_000_000 < sample['mapped_stamp_ns']
    assert sample['ingestion_ns'] == 3_000_000_000
    record.status(SimpleNamespace(timestamp=1_100_000,arming_state=1),
                  3_096_000_000,wall_ns=1_100_000_000)
    record.clock(3_100_000_000,wall_ns=1_104_000_000)
    event, = record.document['events']
    assert event['source'] == 'native_vehicle_status'
    assert event['source_stamp_us'] == 1_100_000
    assert event['receipt_ns'] < event['stamp_ns'] == event['ingestion_ns']


@pytest.mark.parametrize('fault', ['timeout','rewind','large_future','invalid_stamp','publication_before_capture','epoch'])
def test_clock_failures_prevent_measured_evidence(fault):
    record = mapped_record()
    record.clock(2_990_000_000,wall_ns=0)
    if fault == 'large_future':
        record.sample(message(timestamp=1_000_000),2_900_000_000,wall_ns=0)
    elif fault == 'invalid_stamp':
        record.sample(message(timestamp_sample=True,timestamp=1_000_000),3_000_000_000,wall_ns=0)
    elif fault == 'publication_before_capture':
        record.sample(message(timestamp=999_000),3_000_000_000,wall_ns=0)
    elif fault == 'epoch':
        record.sample(message(timestamp=1_000_000),21_000_000_000,wall_ns=0)
    else:
        record.sample(message(timestamp=1_000_000),2_996_000_000,wall_ns=0)
        record.clock(2_980_000_000 if fault == 'rewind' else 3_000_000_000,
                     wall_ns=50_000_001 if fault == 'timeout' else 1)
    assert record.faults and not record.document['samples']
    assert not record.result()['recording_complete']


def test_clock_error_bound_waits_for_latest_possible_capture():
    record = mapped_record(error=2_000_000)
    record.sample(message(timestamp=1_000_000),3_000_000_000,wall_ns=0)
    record.clock(3_001_000_000,wall_ns=1_000_000)
    assert not record.document['samples']
    record.clock(3_002_000_000,wall_ns=2_000_000)
    assert len(record.document['samples']) == 1 and not record.faults


@pytest.mark.parametrize('key,value', [('trial_id','other'),('owner_id','/other'),
                                     ('profile_sha256','b'*64),('px4_issue_us',1)])
def test_owner_event_bound_to_exact_trial_and_outbound_stamp(key,value):
    record = mapped_record()
    event = owner_event(record)
    event[key] = value
    record.event(json.dumps(event),3_000_000_000)
    assert record.faults and not record.document['events']


def test_bound_owner_event_and_pending_completion():
    record = mapped_record()
    record.event(json.dumps(owner_event(record)),3_000_000_000)
    assert record.document['events'][0]['px4_issue_us'] == 1_000_000
    record.sample(message(timestamp=1_001_000),3_000_000_000,wall_ns=0)
    assert record.result()['pending_clock_samples'] == 1
    assert not record.result()['recording_complete']


def test_native_entry_requires_pinned_reviewed_contract(tmp_path):
    record = mapped_record()
    contract = dict(clock_id='ros_sim_time',frame_id='px4_local_ned',review_status='reviewed',
        trial_kind='stopping',
        clock_mapping=asdict(record.mapping),trial_id='trial-1',owner_id='/calibration_trial_owner',
        profile_sha256='a'*64,topics=dict(vehicle_local_position='/fmu/out/vehicle_local_position',
        vehicle_status='/fmu/out/vehicle_status_v1',events='/qualification/events'))
    path = tmp_path/'contract.json'
    path.write_text(json.dumps(contract))
    loaded, topics = load_contract(path,sha256(path.read_bytes()).hexdigest())
    assert loaded.mapping == record.mapping and topics == contract['topics']
    with pytest.raises(ValueError,match='digest'):
        load_contract(path,'0'*64)
    contract['review_status'] = 'draft'
    path.write_text(json.dumps(contract))
    with pytest.raises(ValueError,match='reviewed'):
        load_contract(path,sha256(path.read_bytes()).hexdigest())


def test_reset_cannot_shrink_measured_excursion():
    record = mapped_record()
    record.sample(message(timestamp=1_000_000),3_000_000_000,wall_ns=0)
    record.sample(message(timestamp_sample=1_010_000,timestamp=1_010_000,xy_reset_counter=1),
                  3_010_000_000,wall_ns=10_000_000)
    assert len(record.document['samples']) == 1
    assert record.document['samples'][0]['reset_counters'] == {key:0 for key in RESET_FIELDS}
    assert 'estimator_reset_or_missing_counters' in record.faults


def test_event_callback_requires_exact_single_owner_endpoint():
    endpoint = SimpleNamespace(node_namespace='/',node_name='calibration_trial_owner',endpoint_gid=b'1'*16)
    node = SimpleNamespace(get_publishers_info_by_topic=lambda topic:[endpoint])
    expected = dict(publisher_node='/calibration_trial_owner',publisher_gid=(b'1'*16).hex(),
                    publisher_binding='singleton_graph_only')
    assert event_publisher_identity(node,'/qualification/events','/calibration_trial_owner') == expected
    record = mapped_record()
    record.event(json.dumps(owner_event(record)),3_000_000_000,publisher=expected)
    assert record.document['events'][0]['publisher_gid'] == expected['publisher_gid']
    for peers, owner in (([], '/calibration_trial_owner'),
        ([endpoint,endpoint], '/calibration_trial_owner'), ([endpoint], '/different_owner')):
        node.get_publishers_info_by_topic = lambda topic, peers=peers:peers
        with pytest.raises(ValueError):
            event_publisher_identity(node,'/qualification/events',owner)
    changed = dict(expected,publisher_gid=(b'2'*16).hex())
    record.event(json.dumps(owner_event(record,'land_command',3_100_000_000)),3_100_000_000,publisher=changed)
    assert record.faults and 'land_command_s' not in record.document


def test_ready_handshake_requires_fresh_disarmed_native_data_and_is_exclusive(tmp_path):
    record = mapped_record()
    record.document['contract_sha256'] = 'b'*64
    assert record.readiness(3_000_000_000,pid=42) is None
    record.sample(message(timestamp=1_000_000),3_000_000_000,wall_ns=0)
    record.status(SimpleNamespace(timestamp=1_000_000,arming_state=1),3_000_000_000,wall_ns=0)
    report = record.readiness(3_000_000_000,pid=42)
    assert report['recorder_pid'] == 42 and report['contract_sha256'] == 'b'*64
    assert report['trial_id'] == 'trial-1'
    path = tmp_path/'ready.json'
    write_ready(path,report)
    assert json.loads(path.read_text()) == report
    with pytest.raises(FileExistsError):
        write_ready(path,report)
    assert record.readiness(3_101_000_000,pid=42) is None
    record.faults.append('fixture_fault')
    assert record.readiness(3_000_000_000,pid=42) is None


def test_hover_land_completeness_does_not_fabricate_braking_measurement():
    record = mapped_record()
    record.document['trial_kind'] = 'hover_land'
    record.sample(message(timestamp=1_000_000),3_000_000_000,wall_ns=0)
    record.status(SimpleNamespace(timestamp=1_000_000,arming_state=2),3_000_000_000,wall_ns=0)
    record.event(json.dumps(owner_event(record,'land_command',3_100_000_000)),3_100_000_000)
    record.status(SimpleNamespace(timestamp=1_200_000,arming_state=1),3_200_000_000,wall_ns=200_000_000)
    assert record.result()['recording_complete']
    assert 'brake_command_s' not in record.document
    record.document['trial_kind'] = 'stopping'
    assert not record.result()['recording_complete']


def ready_record():
    record = mapped_record()
    record.sample(message(timestamp=1_000_000),3_000_000_000,wall_ns=1_000_000_000)
    record.status(SimpleNamespace(timestamp=1_000_000,arming_state=1),3_000_000_000,wall_ns=1_000_000_000)
    assert record.readiness(3_000_000_000,pid=42) is not None
    record.monitor(3_000_000_000,1_000_000_000)
    return record


def test_after_ready_position_silence_fails_without_new_callback():
    record = ready_record()
    record.monitor(3_100_000_000,1_100_000_000)
    assert not record.faults
    record.monitor(3_100_000_001,1_100_000_001)
    assert record.faults == ['recorder_position_silent']


def test_after_ready_status_silence_fails_despite_advancing_position():
    record = ready_record()
    for step in range(1, 16):
        source = 1_000_000 + step*50_000
        now = source*1000 + record.mapping.offset_ns
        wall = 1_000_000_000 + step*50_000_000
        record.sample(message(timestamp_sample=source,timestamp=source),now,wall_ns=wall)
        record.monitor(now,wall)
    assert not record.faults
    record.monitor(3_750_000_001,1_750_000_001)
    assert record.faults == ['recorder_status_silent']


@pytest.mark.parametrize('resumed', [False, True])
def test_after_ready_wall_clock_stall_fails_even_if_ros_resumes(resumed):
    record = ready_record()
    record.monitor(3_000_000_001 if resumed else 3_000_000_000,1_300_000_000)
    assert record.faults == ['recorder_clock_stalled']


def test_after_ready_wall_receipt_limit_is_not_renewed_by_clock():
    record = ready_record()
    # Advancing ROS by a nanosecond per callback does not refresh receipt wall age.
    for step in range(1, 32):
        record.monitor(3_000_000_000+step,1_000_000_000+step*100_000_000)
    assert record.faults == ['recorder_position_silent']
