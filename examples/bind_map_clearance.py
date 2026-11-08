"""Deployment binding for an explicitly permitted participant prior.

A profile loads geometry but cannot mint an exclusion lease or flight approval.
The deployment must independently install a live lease before using this mode.
"""
from hashlib import sha256
from dataclasses import asdict
import json
from pathlib import Path


def bind(runtime, settings, root):
    from refnav.map_assisted_clearance import ClearanceCertificate
    if runtime.adapter is None or not runtime.adapter.simulated_estimator:
        raise ValueError('map-assisted mode is restricted to isolated simulated-estimator deployment')
    allowed = {'certificate_file', 'certificate_sha256', 'participant_map_file', 'participant_map_sha256'}
    if set(settings) != allowed:
        raise ValueError('explicit certificate and participant map identities required')
    paths = [Path(root) / settings[key] for key in ('certificate_file', 'participant_map_file')]
    expected = [settings[key] for key in ('certificate_sha256', 'participant_map_sha256')]
    raw = [path.read_bytes() for path in paths]
    if [sha256(value).hexdigest() for value in raw] != expected:
        raise ValueError('map-assisted source identity mismatch')
    certificate = ClearanceCertificate.from_json(raw[0], expected_sha256=expected[0])
    survey = json.loads(raw[1])
    if (survey.get('schema') != 'dronehacks.participant_clearance_map.v1'
            or survey.get('permitted_use') != 'controlled_sitl'
            or not isinstance(survey.get('review_id'), str) or not survey['review_id'].strip()
            or certificate.participant_map_sha256 != expected[1]):
        raise ValueError('independently permitted participant survey required')
    box = {'lower_m': list(certificate.bounds_min_m), 'upper_m': list(certificate.bounds_max_m)}
    if box not in survey.get('surveyed_empty_boxes', []):
        raise ValueError('certificate volume lacks positive participant survey coverage')
    if (survey.get('reference') != asdict(certificate.reference)
            or certificate.map_from_prior_yaw_rad != 0.
            or any(value != 0. for value in certificate.prior_origin_in_map_m)):
        raise ValueError('initial participant adapter requires exact same-frame survey geometry')
    if certificate.reference != runtime.loop.replay.profile.reference:
        raise ValueError('participant prior reference mismatch')

    def source_valid():
        try:
            return [sha256(path.read_bytes()).hexdigest() for path in paths] == expected
        except OSError:
            return False

    runtime.map_certificate = certificate
    runtime.map_source_valid = source_valid
    return certificate
