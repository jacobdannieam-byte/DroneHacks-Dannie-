import gzip
import json

from examples.audit_classical_telemetry import audit


def write(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    with gzip.open(path, 'wt', newline='') as f:
        f.write(text)


def test_malformed_records_are_counted_not_repaired(tmp_path):
    session = tmp_path / 'flight'
    write(session / 'session.json.gz', json.dumps({'run_mode': 'practice'}))
    write(session / 'topics/fmu__out__vehicle_status_v1.csv.gz',
          'seconds_since_start,timestamp,arming_state\n1,1000000,1\n2,2000000,2\n\x00\x00\n')
    write(tmp_path / 'depth/depth_camera.csv.gz',
          'picture_time_seconds,calculation_seconds,trusted_fraction,points_published\n'
          '1,.05,.2,123\n2,.07\n')
    report = audit(tmp_path)
    status = report['sessions'][0]['streams']['fmu__out__vehicle_status_v1']
    assert status['rows'] == 2
    assert status['malformed_rows_rejected'] == 1
    assert report['sessions_with_recorded_armed_status'] == 1
    assert report['depth_sessions'][0]['malformed_rows_rejected'] == 1
    assert report['training_ready'] is False


def test_reset_and_rewound_clock_remain_visible(tmp_path):
    session = tmp_path / 'flight'
    write(session / 'session.json.gz', '{}')
    write(session / 'topics/fmu__out__vehicle_local_position.csv.gz',
          'seconds_since_start,timestamp,xy_reset_counter\n1,1000000,255\n2,2000000,0\n3,1500000,0\n')
    report = audit(tmp_path)
    state = report['sessions'][0]['streams']['fmu__out__vehicle_local_position']
    assert state['reset_counter_transitions']['xy_reset_counter'] == 1
    assert state['source_nonincreasing_steps'] == 1
    assert state['all_position_velocity_heading_valid_rows'] == 0
