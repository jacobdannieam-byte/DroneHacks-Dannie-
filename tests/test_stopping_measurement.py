import pytest
from refnav.stopping_envelope import StoppingEnvelope
from refnav.stopping_measurement import validate_measurement

def fixture():
    envelope = StoppingEnvelope(.35, .15, .5, .2, .3, 'a'*64)
    report = dict(schema='dronehacks.stopping.v1', review_status='reviewed', reviewer='test fixture',
                  scope='isolated_sitl', candidate_commit='b'*40, vehicle_configuration='fixture',
                  raw_evidence_sha256='c'*64, horizontal_radius_m=.35, half_height_m=.15,
                  trials=[dict(result='PASS', initial_speed_mps=.5, peak_speed_mps=.5,
                               reaction_s=.15, braking_distance_m=.25, landing_duration_s=20)])
    return report, envelope

def test_valid_report():
    report, envelope = fixture()
    validate_measurement(report, envelope)

@pytest.mark.parametrize('field,value', [('initial_speed_mps', .3), ('peak_speed_mps', .51),
    ('reaction_s', .21), ('braking_distance_m', .31), ('landing_duration_s',31),
    ('reaction_s',float('nan')), ('result','FAIL')])
def test_outside_coverage_refused(field,value):
    report,envelope=fixture(); report['trials'][0][field]=value
    with pytest.raises(ValueError): validate_measurement(report,envelope)

@pytest.mark.parametrize('field,value', [('trials', []), ('reviewer',''),
    ('scope','hardware'), ('horizontal_radius_m',.36), ('half_height_m',None),
    ('review_status','draft'), ('schema','unknown')])
def test_missing_or_incompatible_evidence_refused(field,value):
    report,envelope=fixture(); report[field]=value
    with pytest.raises(ValueError): validate_measurement(report,envelope)
