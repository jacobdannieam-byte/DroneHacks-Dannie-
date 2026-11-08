import tempfile,unittest
from pathlib import Path
from agentlab.orchestrator import Jobs
class JobTests(unittest.TestCase):
 def test_independent_review_expiry_cancel(self):
  with tempfile.TemporaryDirectory() as d:
   j=Jobs(Path(d)/'jobs.db')
   m=dict(experiment_id='e',source_sha='a'*40,world_sha256='b'*64,hypothesis='h',scenario='s',baseline='S1',rollback='r',seed=1,oracle_in_policy=False,command_owner_count=1)
   key=j.submit(m,'implementer')
   with self.assertRaises(ValueError): j.approve(key,'implementer')
   j.approve(key,'reviewer'); first=j.claim_offline(key,now=0,seconds=1)
   with self.assertRaises(ValueError): j.claim_offline(key,now=0)
   j.recover(now=2); second=j.claim_offline(key,now=2)
   j.fail(key,first,now=3)
   self.assertTrue(j.lease_active(key,second,now=3))
   with self.assertRaises(ValueError): j.finish(key,first,'c'*64,now=3)
   j.cancel(key)
   with self.assertRaises(ValueError): j.finish(key,second,'c'*64,now=3)
   j.close()
 def test_timeout_budget(self):
  with tempfile.TemporaryDirectory() as d:
   j=Jobs(Path(d)/'jobs.db'); m=dict(experiment_id='e',source_sha='a'*40,world_sha256='b'*64,hypothesis='h',scenario='s',baseline='S1',rollback='r',seed=1,oracle_in_policy=False,command_owner_count=1)
   k=j.submit(m,'a'); j.approve(k,'b')
   for now in (0,2): j.claim_offline(k,now=now,seconds=1); j.recover(now=now+2)
   with self.assertRaises(ValueError): j.claim_offline(k,now=4)
   j.close()
