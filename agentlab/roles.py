import tomllib
from pathlib import Path
NAMES = ['analyst', 'documenter', 'experiment_designer', 'implementer', 'performance_auditor', 'policy_diagnostician', 'policy_repairer', 'regression_guard', 'researcher', 'reviewer', 'simulator_triage']
def audit(root):
    found=[]
    for name in NAMES:
        data=tomllib.loads((Path(root)/'.codex/agents'/f'{name}.toml').read_text())
        if data.get('name')!=name or data.get('sandbox_mode')!='read-only' or 'Staged only' not in data.get('developer_instructions',''):
            raise ValueError('Invalid role '+name)
        found.append(name)
    return found
