"""Pinned role assignments. No default-model fallback."""
import json
from pathlib import Path
PROFILES=json.loads(Path(__file__).with_name('model_profiles.json').read_text())
def profile(role):
    value=PROFILES[role]
    if value['model']=='gpt-6-astra':
        if value['reasoning_effort']!='low': raise ValueError('Astra capped at low')
    elif value['model']=='gpt-6.1-sol':
        if value['reasoning_effort'] not in ('high','xhigh'): raise ValueError('Unexpected Sol effort')
    else: raise ValueError('Model outside approved portfolio')
    return dict(value)
