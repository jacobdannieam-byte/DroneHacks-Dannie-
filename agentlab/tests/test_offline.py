import unittest,tempfile,json
from pathlib import Path
from agentlab.gates import staged,manifest,repair,policy_path,FLAGS
from agentlab.queue import Queue
from agentlab.scoring import score,performance
from agentlab.triage import classify
from agentlab.adapters import CodexWorker,GazeboRunner,NotionPublisher,PhysicalFlight
from agentlab.roles import audit
ROOT=Path(__file__).resolve().parents[2]
class Tests(unittest.TestCase):
    def setUp(self):
        self.m=dict(experiment_id='e1',source_sha='a'*40,world_sha256='b'*64,hypothesis='h',scenario='s',baseline='S1',rollback='revert',seed=1,oracle_in_policy=False,command_owner_count=1)
    def test_manifest(self):
        self.assertEqual(len(manifest(self.m)),64)
        for field,value in [('source_sha','bad'),('oracle_in_policy',True),('command_owner_count',2),('seed',True)]:
            with self.subTest(field=field),self.assertRaises(ValueError): manifest(dict(self.m,**{field:value}))
    def test_flags(self):
        c=json.loads((ROOT/'agentlab/config.json').read_text()); staged(c)
        for f in FLAGS:
            with self.subTest(flag=f),self.assertRaises(ValueError): staged(dict(c,**{f:True}))
    def test_paths(self):
        policy_path('refnav/experiments/example.py')
        for p in ['../a','refnav/experiments/../node.py','drone_code/a.py','/refnav/experiments/a','C:/a','refnav/experiments//a']:
            with self.subTest(path=p),self.assertRaises(ValueError): policy_path(p)
    def test_queue_persistence_dedup(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'q.db'; q=Queue(path); key=q.add(self.m); self.assertEqual(key,q.add(self.m)); q.close()
            q=Queue(path); self.assertEqual(len(q.list()),1); q.reject(key)
            with self.assertRaises(ValueError): q.reject(key)
            self.assertEqual(q.list()[0]['status'],'rejected'); q.close()
    def test_repair(self):
        with self.assertRaises(ValueError): repair(self.m)
        r=dict(self.m,reproduction='fixture',acceptance_tests=['baseline','fault'],allowed_paths=['refnav/experiments/a.py'])
        repair(r)
        with self.assertRaises(ValueError): repair(dict(r,alter_scoring=True))
    def test_evidence(self):
        self.assertEqual(score({})['status'],'UNSCORED')
        e=dict(independent_referee=True,telemetry_verified=True,landed=True,disarmed=True,landed_status_fresh=True,seconds=84.3,touches=0,checkpoints=4)
        self.assertIsNone(score(e)['official_score'])
        self.assertEqual(score(dict(e,seconds=float('nan')))['status'],'UNSCORED')
    def test_perf(self):
        self.assertEqual(performance([1,2,3])['p50'],2)
        for x in ([],[-1],[float('inf')],[True]):
            with self.assertRaises(ValueError): performance(x)
    def test_triage(self):
        e=dict(independent_evidence=True,infrastructure_healthy=True,estimator_healthy=True,safety_stop=False,deadline_missed=False,reproduced_policy_defect=True)
        self.assertEqual(classify(e),'POLICY_CANDIDATE')
        self.assertEqual(classify(dict(e,infrastructure_healthy=False)),'SIMULATOR_ENVIRONMENT')
        self.assertEqual(classify({}),'INSUFFICIENT_EVIDENCE')
    def test_roles(self): self.assertEqual(len(audit(ROOT)),11)
    def test_disabled(self):
        for adapter in (CodexWorker,GazeboRunner,NotionPublisher,PhysicalFlight):
            with self.assertRaises(PermissionError): adapter().execute()
if __name__=='__main__': unittest.main()
