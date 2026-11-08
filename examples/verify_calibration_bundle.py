"""Verify exact calibration deployment bytes before starting a container."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import tarfile


def verify(read):
    result = {}
    for profile, contract in (('c0_hover_land', 'c0_recorder'), ('c1_stop_020', 'c1_recorder'),
                              ('r1_stationary_expiry', 'r1_recorder')):
        path = f'scenarios/calibration/{profile}.json'
        raw = read(path)
        data = json.loads(raw)
        recorder = json.loads(read(f'scenarios/calibration/{contract}.json'))
        if (sha256(raw).hexdigest() != recorder['profile_sha256']
                or data['plan']['trial_kind'] != recorder['trial_kind']
                or data['clock_mapping'] != recorder['clock_mapping']):
            raise ValueError(f'Calibration profile/recorder mismatch: {path}')
        geometry = read(data['geometry_evidence_file'])
        if sha256(geometry).hexdigest() != data['geometry_evidence_sha256']:
            raise ValueError('Calibration geometry bytes differ')
        for name, expected in json.loads(geometry)['source_sha256'].items():
            if name == 'externally_supplied_generated_drone_model.sdf':
                continue  # Generated inside the simulator; checked by the native owner.
            if sha256(read(name)).hexdigest() != expected:
                raise ValueError(f'Calibration source bytes differ: {name}')
        result[profile] = sha256(raw).hexdigest()
    return result


def verify_archive(path):
    with tarfile.open(path) as archive:
        return verify(lambda name: archive.extractfile(name).read())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path)
    args = parser.parse_args()
    print(json.dumps(verify_archive(args.archive) if args.archive else verify(lambda name: Path(name).read_bytes()), indent=2))
