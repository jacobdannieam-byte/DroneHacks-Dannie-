import hashlib, io, json, subprocess, tarfile, sys
from pathlib import Path
root=Path(__file__).resolve().parent
repo=root.parents[1]/'policy-gates-integration'
paths=set(subprocess.check_output(['git','-C',str(repo),'ls-files'],text=True).splitlines())
paths.update(subprocess.check_output(['git','-C',str(repo),'ls-files','--others','--exclude-standard'],text=True).splitlines())
output=root/(sys.argv[1] if len(sys.argv)>1 else 'inputs')
output.mkdir(exist_ok=True)
paths.update(str(p.relative_to(repo)).replace('\\','/') for p in (repo/'tests').glob('test_policy_gate*.py'))
hashes={}
with tarfile.open(output/'integrated.tar','w') as archive:
    for name in sorted(paths):
        if not name.startswith(('simulator/','drone_code/','examples/','refnav/','onboard/','scenarios/','tests/','artifacts/goal_comparison/models/','artifacts/goal_comparison/predictors/')): continue
        path=repo/name
        if not path.is_file(): continue
        data=path.read_bytes()
        if name.endswith('.sh'): data=data.replace(b'\r\n',b'\n')
        hashes[name]=hashlib.sha256(data).hexdigest()
        info=tarfile.TarInfo(name); info.size=len(data); info.mode=0o755 if name.endswith('.sh') else 0o644
        archive.addfile(info,io.BytesIO(data))
(output/'source-manifest.json').write_text(json.dumps({'parents':['43712d446b6b81d99a7d7da4ac2347e008ddf6c4','26724362b685b63287a937c25e96143a3f44ddcf'],'scope':'uncommitted integration snapshot; individual gate evidence determines qualification','files':hashes,'archive_sha256':hashlib.sha256((output/'integrated.tar').read_bytes()).hexdigest()},indent=2))
print('Exported pinned integration snapshot',len(hashes),'files')
