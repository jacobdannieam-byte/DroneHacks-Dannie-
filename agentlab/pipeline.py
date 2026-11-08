"""One-shot reviewed worker stage; no background loop or implicit simulation."""
import json
import time
from pathlib import Path

from .gates import digest
from .strategy_workflow import validate_proposal


class Pipeline:
    def __init__(self,jobs,worktrees,worker,artifact_root):
        self.jobs=jobs; self.worktrees=worktrees; self.worker=worker; self.artifact_root=artifact_root
    def run_worker(self,key,plan):
        # Check authorization before even creating a worktree or claiming a job.
        if self.worker.activation is None: raise PermissionError('Workers disabled')
        plan=self.worker.validate_plan(plan)
        approval=self.worker.activation.require('agents_enabled',plan)
        payload,author,reviewer=self.jobs.reviewed_manifest(key)
        unbound_plan={field: value for field,value in plan.items() if field!='job_id'}
        if (plan.get('job_id')!=key or payload['source_sha']!=plan['source_sha']
            or payload.get('worker_plan_sha256')!=digest(unbound_plan)):
            raise ValueError('Worker plan differs from the reviewed job')
        if (approval['author'],approval['reviewer'])!=(author,reviewer):
            raise PermissionError('Approval identity differs from the reviewed job')
        validate_proposal(payload, payload.get('strategy_reviews', {}), payload.get('notion_context', {}))
        token=self.jobs.claim_offline(key,seconds=180)

        def require_active():
            if time.time()>=approval['expires']:
                raise PermissionError('Worker approval expired')
            if not self.jobs.lease_active(key,token):
                raise ValueError('Expired or cancelled lease')

        try:
            require_active()
            attempt=digest(token)
            tree=self.worktrees.create(plan['source_sha'],key[:32]+'-'+attempt[:16],check_active=require_active)
            result=self.worker.execute(plan,tree,timeout=150,check_active=require_active)
            require_active()
            record=dict(job_id=key,worker_plan_sha256=digest(plan),attempt_id=attempt,result=result)
            raw=json.dumps(record,sort_keys=True,allow_nan=False).encode()
            evidence=digest(record)
            root=Path(self.artifact_root)
            root.mkdir(parents=True,exist_ok=True)
            with (root/(evidence+'.json')).open('xb') as artifact:
                artifact.write(raw)
            require_active()
            self.jobs.finish(key,token,evidence)
            return result
        except BaseException:
            self.jobs.fail(key,token)
            raise
