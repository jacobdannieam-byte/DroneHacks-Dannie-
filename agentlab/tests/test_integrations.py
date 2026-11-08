import hashlib,json,os,sqlite3,tempfile,time,unittest
from pathlib import Path
from unittest.mock import patch
from agentlab.integrations import Activation,CodexWorker,GazeboRunner,NotionPublisher,Worktrees,ingest_bag,ingest_referee
from agentlab.gates import digest
class FakeRunner:
 def __init__(self): self.calls=[]
 def run(self,args,cwd,*extra):
  self.calls.append((args,str(cwd),extra))
  if args[:2]==['git','rev-parse']: return 'a'*40
  if args[:2]==['git','worktree']: return ''
  return '{"type":"thread.started","thread_id":"test"}\n{"type":"turn.completed"}'
def permit(cap,p): return Activation({'mode':'reviewed_execution',cap:True},{'capability':cap,'payload_sha256':digest(p),'author':'a','reviewer':'b','expires':time.time()+60})
class Integrations(unittest.TestCase):
 def test_disabled_no_transport(self):
  r=FakeRunner()
  for adapter in (CodexWorker(runner=r),GazeboRunner(runner=r),NotionPublisher()):
   with self.assertRaises(PermissionError): adapter.execute()
  self.assertEqual(r.calls,[])
 def test_bound_approval(self):
  p={'source':'a'}; a=permit('agents_enabled',p); a.require('agents_enabled',p)
  with self.assertRaises(PermissionError): a.require('agents_enabled',{'source':'b'})
  a.approval['expires']=0
  with self.assertRaises(PermissionError): a.require('agents_enabled',p)
 def test_worker_transport_fake_only(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d); (root/'.codex/agents').mkdir(parents=True)
   (root/'.codex/agents/reviewer.toml').write_text('developer_instructions="Review only"')
   r=FakeRunner(); w=CodexWorker(runner=r); p=w.plan('reviewer','a'*40,'review')
   w.activation=permit('agents_enabled',p); result=w.execute(p,root)
   self.assertEqual(result['role'],'reviewer'); self.assertIn('read-only',r.calls[-1][0])
 def test_worktree_command(self):
  with tempfile.TemporaryDirectory() as d:
   r=FakeRunner(); w=Worktrees(d,Path(d)/'trees',r); p=w.create('a'*40,'job')
   self.assertEqual(p.name,'job'); self.assertIn('--detach',r.calls[0][0])
   with self.assertRaises(ValueError): w.create('a'*40,'../escape')
 def test_referee_and_bag(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d)/'summary.json'; s=dict(touches=0,checkpoint_seconds=dict(a=1,b=2,c=3,d=4),landed_on_pad=True,motors_off_seconds=5,height_estimate_went_wrong=False,stayed_above_a_boulder_too_long=False,went_above_walls=False,left_the_course=False)
   p.write_text(json.dumps(s)); h=hashlib.sha256(p.read_bytes()).hexdigest()
   self.assertTrue(ingest_referee(p,h)['full_course'])
   with self.assertRaises(ValueError): ingest_referee(p,'0'*64)
   db=Path(d)/'bag.db3'; c=sqlite3.connect(db); c.executescript('CREATE TABLE topics(id INTEGER,name TEXT,type TEXT); CREATE TABLE messages(id INTEGER,topic_id INTEGER,timestamp INTEGER,data BLOB); INSERT INTO topics VALUES(1,"/test","test/Message"); INSERT INTO messages VALUES(1,1,100,NULL);'); c.commit(); c.close()
   result=ingest_bag(db); self.assertFalse(result['telemetry_verified']); self.assertEqual(result['topics'][0]['count'],1)
 def test_notion_request_fake_only(self):
  import io
  seen=[]
  def fake(request,timeout): seen.append(request); return io.StringIO('{"results":[]}')
  n=NotionPublisher(transport=fake); p=n.plan('40cf393a-a768-8157-8e27-ce0bca9b6f4d',{'source_sha':'a'*40,'status':'offline'})
  n.activation=permit('notion_write_enabled',p)
  with patch.dict(os.environ,{'AGENTLAB_NOTION_TOKEN':'fake-test-token'}): n.execute(p)
  self.assertEqual(seen[0].method,'PATCH'); self.assertTrue(seen[0].full_url.endswith('/children'))
