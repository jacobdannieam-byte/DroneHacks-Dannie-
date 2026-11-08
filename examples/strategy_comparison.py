"""External comparison of recorded systems; never launches or instruments Classical.

Missing metrics stay null. A successful run alone does not supply matched seeds,
sensor settings or scoring provenance, so incomplete identities are refused.
"""
import argparse
from hashlib import sha256
import json
import math
from pathlib import Path

CONDITIONS = ('course', 'permitted_map', 'initial_pose', 'vehicle', 'sensors',
              'seed', 'mission', 'scoring', 'termination')
METRICS = ('completion', 'checkpoints', 'touches', 'minimum_clearance_m',
           'unknown_space_violations', 'elapsed_seconds', 'path_length_m',
           'recoveries', 'useful_evidence', 'appropriate_abstentions',
           'unnecessary_abstentions', 'goal_switches', 'invalid_proposals',
           'validator_interventions', 'fallbacks', 'real_time_factor')


def digest_file(path):
    digest = sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def compare(runs):
    if len(runs) < 2:
        raise ValueError('Supply at least two external run records')
    errors, normalized, keys = [], [], set()
    for run in runs:
        label = run.get('run_id', 'missing')
        conditions = run.get('conditions', {})
        missing = [key for key in CONDITIONS if conditions.get(key) is None]
        if missing:
            errors.append({'run_id': label, 'missing_conditions': missing})
        if not run.get('revision') or not run.get('configuration_sha256'):
            errors.append({'run_id': label, 'reason': 'missing_source_or_configuration_identity'})
        key = (run.get('strategy'), label)
        if key in keys:
            errors.append({'run_id': label, 'reason': 'duplicate_run'})
        keys.add(key)
        metrics = run.get('metrics', {})
        for name, value in metrics.items():
            if value is not None and (not isinstance(value, (int, float, bool)) or not math.isfinite(value)):
                raise ValueError('Metrics must be finite numbers, booleans or null')
        normalized.append({'strategy': run.get('strategy'), 'run_id': label,
                           'metrics': {name: metrics.get(name) for name in METRICS}})
    reference = runs[0].get('conditions', {})
    for run in runs[1:]:
        mismatch = [key for key in CONDITIONS if run.get('conditions', {}).get(key) != reference.get(key)]
        if mismatch:
            errors.append({'run_id': run.get('run_id'), 'mismatched_conditions': mismatch})
    return {'schema_version': 1, 'comparable': not errors, 'issues': errors, 'runs': normalized,
            'interpretation': 'Matched descriptive runs only; repeated paired seeds are required for superiority claims.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = compare([json.loads(path.read_text(encoding='utf-8')) for path in args.runs])
    result['input_sha256'] = {str(path): digest_file(path) for path in args.runs}
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    return 0 if result['comparable'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
