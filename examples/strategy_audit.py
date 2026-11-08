"""Read-only provenance collection. Outputs must be outside protected worktrees."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess

from examples.strategy_comparison import digest_file


def git(root, *args):
    return subprocess.check_output(['git', '--no-optional-locks', '-C', str(root), *args], text=True).strip()


def protected_state(workspace):
    result = {}
    for name in ('classical-audit-source', 'classical-gazebo-integration', 'classical-smoke-ad31ae5'):
        root = workspace / name
        tracked = git(root, 'ls-files', '-z').split('\0')
        files = {name: digest_file(root / name) if (root / name).is_file() else None for name in tracked if name}
        result[name] = {'head': git(root, 'rev-parse', 'HEAD'),
                        'status': git(root, 'status', '--porcelain=v1', '--untracked-files=no'),
                        'tracked_file_hashes': files}
    return result


def baseline_record(workspace):
    folder = workspace / 'classical-smoke-ad31ae5/simulator/gazebo_trial/generated/hybrid_runs/run-AdMalPxA'
    summary = json.loads((folder / 'classical-course-summary.json').read_text())
    approval = json.loads((folder / 'trial_approval.json').read_text())
    names = ('CLASSICAL_COURSE_RESULTS.md', 'classical-course-summary.json', 'trial_approval.json',
             'run.txt', 'revision.txt', 'versions.txt', 'observations/metadata.yaml', 'source.tar')
    hashes = {name: digest_file(folder / name) for name in names}
    referee = summary['referee']
    return {'schema_version': 1, 'strategy': 'S1', 'run_id': 'run-AdMalPxA',
            'revision': approval['revision'], 'configuration_sha256': None,
            'identity_confirmation': 'User identified this successful run; saved report and approval agree on revision.',
            'artifact_root': str(folder), 'artifact_sha256': hashes,
            'conditions': {'course': approval['world_sha256'], 'permitted_map': None,
                           'initial_pose': None, 'vehicle': None, 'sensors': None, 'seed': None,
                           'mission': 'side-gap-first', 'scoring': 'saved full-course referee', 'termination': None},
            'metrics': {'completion': referee['landed_on_pad'] and summary['disarmed_after_arm'],
                        'checkpoints': len(referee['checkpoint_seconds']), 'touches': referee['touches'],
                        'minimum_clearance_m': summary['minimum_referee_obstacle_distance_airborne_m'],
                        'elapsed_seconds': referee['motors_off_seconds'],
                        'path_length_m': summary['horizontal_distance_m'], 'real_time_factor': .999},
            'limitations': ['One passing run, not repeated paired evidence.',
                           'Minimum clearance is sampled referee propeller-point distance, not D3 swept-body clearance.',
                           'Complete matched configuration and initial-condition identities still need extraction/review.']}


if __name__ == '__main__':
    root = Path.cwd()
    output = root / 'artifacts/strategy_comparison'
    for name, value in (('protected_before.json', protected_state(root.parent)),
                        ('classical_baseline.json', baseline_record(root.parent))):
        with (output / name).open('x', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2)
            stream.write('\n')
