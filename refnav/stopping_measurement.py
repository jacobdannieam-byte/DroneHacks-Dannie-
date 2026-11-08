"""Validate a reviewed stopping report without treating its hash as proof."""
import math
import re


def validate_measurement(report, envelope):
    if report.get('schema') != 'dronehacks.stopping.v1':
        raise ValueError('unsupported stopping measurement schema')
    if report.get('review_status') != 'reviewed' or not str(report.get('reviewer', '')).strip():
        raise ValueError('stopping measurements require independent review')
    if report.get('scope') != 'isolated_sitl':
        raise ValueError('this activation accepts isolated SITL measurements only')
    for field in ('candidate_commit', 'vehicle_configuration', 'raw_evidence_sha256'):
        if not isinstance(report.get(field), str) or not report[field].strip():
            raise ValueError('missing stopping provenance: ' + field)
    if not re.fullmatch('[0-9a-f]{40}', report['candidate_commit']):
        raise ValueError('full measured candidate commit required')
    if not re.fullmatch('[0-9a-f]{64}', report['raw_evidence_sha256']):
        raise ValueError('raw stopping evidence SHA256 required')
    trials = report.get('trials')
    if not isinstance(trials, list) or not trials:
        raise ValueError('stopping trials required')
    covered_speed = 0.
    for trial in trials:
        if trial.get('result') != 'PASS':
            raise ValueError('failed stopping trial in reviewed report')
        values = {}
        for field in ('initial_speed_mps', 'peak_speed_mps', 'reaction_s',
                      'braking_distance_m', 'landing_duration_s'):
            value = trial.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0:
                raise ValueError('invalid stopping measurement: ' + field)
            values[field] = value
        if not 0 < values['initial_speed_mps'] <= values['peak_speed_mps'] <= .50:
            raise ValueError('stopping trial speed outside limits')
        if not 0 < values['landing_duration_s'] <= 30:
            raise ValueError('stopping trial landing outside limits')
        if values['reaction_s'] > envelope.reaction_s or values['braking_distance_m'] > envelope.braking_distance_m:
            raise ValueError('envelope does not cover measured stop')
        covered_speed = max(covered_speed, values['initial_speed_mps'])
    if covered_speed < envelope.max_speed_mps:
        raise ValueError('requested speed exceeds measured coverage')
    for field in ('horizontal_radius_m', 'half_height_m'):
        value = report.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError('measured vehicle dimensions required')
        if value > getattr(envelope, field):
            raise ValueError('envelope excludes measured vehicle body')
