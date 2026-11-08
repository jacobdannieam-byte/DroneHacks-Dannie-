import argparse,json
from pathlib import Path
from .gates import staged
from .roles import audit
from .scoring import score,performance
from .triage import classify
from .queue import Queue
ROOT=Path(__file__).resolve().parent.parent
def main():
    p=argparse.ArgumentParser(description='Offline staging only; no execution commands')
    p.add_argument('command',choices=['doctor','roles','score','perf','triage','queue-add','queue-list','queue-reject','repair-add','repair-list','repair-reject'])
    p.add_argument('--input',type=Path); p.add_argument('--db',default='agentlab-local.sqlite'); p.add_argument('--id')
    a=p.parse_args(); staged(json.loads((ROOT/'agentlab/config.json').read_text()))
    if a.command=='doctor': result=dict(status='STAGING_READY_OFFLINE_ONLY',roles=len(audit(ROOT)),execution_available=False)
    elif a.command=='roles': result=audit(ROOT)
    elif a.command in ('score','perf','triage'):
        if not a.input: p.error('--input required')
        result={'score':score,'perf':performance,'triage':classify}[a.command](json.loads(a.input.read_text()))
    else:
        q=Queue(a.db)
        try:
            if a.command.endswith('-add'):
                if not a.input: p.error('--input required')
                result=q.add(json.loads(a.input.read_text()),'repair' if a.command.startswith('repair') else 'experiment')
            elif a.command.endswith('-list'): result=[r for r in q.list() if r['kind']==('repair' if a.command.startswith('repair') else 'experiment')]
            else: q.reject(a.id); result='rejected'
        finally: q.close()
    print(json.dumps(result,indent=2))
