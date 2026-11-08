"""One-shot reviewed worker stage; no background loop or implicit simulation."""
from .gates import digest
class Pipeline:
    def __init__(self,jobs,worktrees,worker,artifact_root):
        self.jobs=jobs; self.worktrees=worktrees; self.worker=worker; self.artifact_root=artifact_root
    def run_worker(self,key,plan,now=None):
        # Check authorization before even creating a worktree or claiming a job.
        if self.worker.activation is None: raise PermissionError('Workers disabled')
        self.worker.activation.require('agents_enabled',plan)
        row=self.jobs.db.execute('SELECT payload FROM jobs WHERE id=?',(key,)).fetchone()
        import json
        if row is None or json.loads(row[0])['source_sha']!=plan['source_sha']: raise ValueError('Job/source mismatch')
        from .strategy_workflow import validate_proposal
        payload=json.loads(row[0])
        validate_proposal(payload, payload.get('strategy_reviews', {}), payload.get('notion_context', {}))
        token=self.jobs.claim_offline(key,now=now,seconds=180)
        try:
            tree=self.worktrees.create(plan['source_sha'],key[:32]+'-'+token[:8])
            result=self.worker.execute(plan,tree,timeout=150)
            from pathlib import Path
            raw=json.dumps(result,sort_keys=True,allow_nan=False).encode()
            evidence=digest(result); root=Path(self.artifact_root); root.mkdir(parents=True,exist_ok=True)
            path=root/(evidence+'.json')
            with path.open('xb') as f: f.write(raw)
            self.jobs.finish(key,token,evidence)
            return result
        except BaseException:
            self.jobs.cancel(key)
            raise
