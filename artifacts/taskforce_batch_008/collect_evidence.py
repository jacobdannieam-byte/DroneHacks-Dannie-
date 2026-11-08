import hashlib
import json
from pathlib import Path
import shutil

root = Path(__file__).resolve().parent
out = root.parent / 'policy-gates-integration/artifacts/taskforce_batch_008'
manifest = json.loads((root / 'candidate.manifest.json').read_text())
shutil.copy2(root / 'candidate.manifest.json', out / 'source-archive-manifest.json')
names = ('calibration_disarmed_check.json', 'calibration_profile.json', 'calibration_recorder_ready.json',
         'calibration_recording.json', 'native_recovery_parameters.txt', 'recorder_contract.json',
         'recording_final.json', 'recording_ready.json', 'versions.txt', 'ogre2.log', 'disarmed-1.json')
summary = {'revision': manifest['revision'], 'archive_sha256': manifest['archive_sha256'], 'runs': {}}
for run in ('c1-check', 'r1-check'):
    src, dest = root / run, out / run
    dest.mkdir(exist_ok=True)
    for name in names:
        shutil.copy2(src / name, dest / name)
    if (src / 'operator-dry-run').exists():
        shutil.copytree(src / 'operator-dry-run', dest / 'operator-dry-run', dirs_exist_ok=True)
        shutil.copy2(src / 'operator-dry-run.txt', dest / 'operator-dry-run.txt')
    record = json.loads((src / 'calibration_recording.json').read_text())
    final = json.loads((src / 'recording_final.json').read_text())
    check = json.loads((src / 'calibration_disarmed_check.json').read_text())
    assert final['passed'] and check['disarmed_check'] and check['revision'] == manifest['revision']
    assert not record['recording_faults'] and record['pending_clock_samples'] == 0 and not record['events']
    assert 'GL_RENDERER = D3D12 (NVIDIA GeForce RTX 5070 Laptop GPU)' in (src / 'ogre2.log').read_text()
    summary['runs'][run] = dict(duration_ns=final['clock_duration_ns'], strict_samples=len(record['samples']),
        strict_statuses=len(record['status_reports']), faults=record['recording_faults'],
        pending=record['pending_clock_samples'], events=len(record['events']), recording_complete=record['recording_complete'],
        renderer='D3D12 (NVIDIA GeForce RTX 5070 Laptop GPU)', evidence_sha256={
            p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in dest.iterdir() if p.is_file()},
        raw_directory=str(src / 'observations'))
failure = out / 'r1-preflight-failed'
failure.mkdir(exist_ok=True)
shutil.copy2(root / 'r1-preflight-failed/operator-dry-run.txt', failure / 'operator-dry-run.txt')
(failure / 'explanation.txt').write_text('Runner failed before ROS/Gazebo startup because the synthetic helper output directory did not exist. Added mkdir in the external runner; retained failure and used a fresh directory. Frozen runtime/archive unchanged.\n')
(out / 'native-summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps({k: {a:b for a,b in v.items() if a != 'evidence_sha256'} for k,v in summary['runs'].items()}))
