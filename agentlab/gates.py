import hashlib, json, re
from pathlib import PurePosixPath
FLAGS = ('agents_enabled','moving_sim_enabled','notion_write_enabled','continuous_loop_enabled','allow_merge_or_push','allow_physical_flight')
def staged(config):
    if config.get('mode') != 'staged_only' or any(config.get(k) is not False for k in FLAGS):
        raise ValueError('Execution must remain disabled')
    if config.get('max_budget_usd_per_batch') != 0 or config.get('protected_strategy') != 'S1':
        raise ValueError('Invalid staging boundary')
def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,allow_nan=False).encode()).hexdigest()
def policy_path(value):
    if not isinstance(value,str) or '\\' in value or ':' in value:
        raise ValueError('Invalid policy path')
    p=PurePosixPath(value)
    if p.is_absolute() or '..' in p.parts or str(p)!=value or not value.startswith('refnav/experiments/'):
        raise ValueError('Only isolated experimental policy paths are eligible')
    return value

def manifest(m):
    for key,n in [('source_sha',40),('world_sha256',64)]:
        if not re.fullmatch('[0-9a-f]{'+str(n)+'}',m.get(key,'')):
            raise ValueError('Invalid '+key)
    for key in ('experiment_id','hypothesis','scenario','baseline','rollback'):
        if not isinstance(m.get(key),str) or not m[key].strip(): raise ValueError('Missing '+key)
    if type(m.get('seed')) is not int: raise ValueError('Integer seed required')
    if m.get('oracle_in_policy') is not False or m.get('command_owner_count') != 1:
        raise ValueError('Invalid policy boundary')
    return digest(m)
def repair(m):
    manifest(m)
    if not m.get('reproduction') or len(m.get('acceptance_tests',[]))<2:
        raise ValueError('Reproduction and two acceptance tests required')
    if not m.get('allowed_paths'): raise ValueError('No allowed paths')
    for p in m['allowed_paths']: policy_path(p)
    if any(m.get(k) for k in ('run_gazebo','auto_merge','alter_safety','alter_scoring')):
        raise ValueError('Forbidden repair authority')
    return digest(m)

