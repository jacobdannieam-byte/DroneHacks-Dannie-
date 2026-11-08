import hashlib,json,os,sqlite3,subprocess,tempfile,time,unittest
from pathlib import Path
from unittest.mock import MagicMock,patch
from agentlab.integrations import Activation,CodexWorker,GazeboRunner,NotionPublisher,ProcessRunner,Worktrees,ingest_bag,ingest_referee
from agentlab.gates import digest

def worker_transcript(report=None):
    if report is None:
        report=dict(role='reviewer',source_sha='a'*40,evidence=['fixture'],findings=[],
                    limitations=['offline'],proposed_next_step='Independent review')
    events=[dict(type='thread.started',thread_id='test'),dict(type='turn.started'),
            dict(type='item.completed',item=dict(type='agent_message',text=json.dumps(report))),
            dict(type='turn.completed')]
    return '\n'.join(map(json.dumps,events))

class FakeRunner:
 def __init__(self,transcript=None): self.calls=[]; self.transcript=transcript
 def run(self,args,cwd,*extra,**options):
  self.calls.append((args,str(cwd),extra,options))
  if options.get('check_active') is not None: options['check_active']()
  if args[:2]==['git','rev-parse']: return 'a'*40
  if args[:2] in (['git','worktree'],['git','status']): return ''
  return worker_transcript() if self.transcript is None else self.transcript

def permit(cap,p,**approval_fields):
    approval=dict(capability=cap,payload_sha256=digest(p),author='a',reviewer='b',expires=time.time()+60)
    approval.update(approval_fields)
    return Activation({'mode':'reviewed_execution',cap:True},approval,
                      verify_approval=lambda record: record==approval)

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
 def test_approval_requires_verified_identities_and_finite_expiry(self):
  p={'source':'a'}
  for fields in ({'author':None},{'author':'b'},{'author':' a'},{'expires':float('nan')},
                 {'expires':float('inf')}):
   with self.subTest(fields=fields), self.assertRaises(PermissionError):
    permit('agents_enabled',p,**fields).require('agents_enabled',p)
  allowed=permit('agents_enabled',p)
  with self.assertRaises(PermissionError): Activation(allowed.config,allowed.approval).require('agents_enabled',p)
  allowed.approval['reviewer']='claimed-reviewer'
  with self.assertRaises(PermissionError): allowed.require('agents_enabled',p)
 def test_worker_transport_fake_only(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d); (root/'.codex/agents').mkdir(parents=True)
   (root/'.codex/agents/reviewer.toml').write_text('developer_instructions="Review only"')
   r=FakeRunner(); w=CodexWorker(runner=r); p=w.plan('reviewer','a'*40,'review')
   w.activation=permit('agents_enabled',p)
   with patch.dict(os.environ,{'AGENTLAB_NOTION_TOKEN':'fake-test-token','OPENAI_API_KEY':'fake-key',
                              'HOME':'controller-home','PATH':'controller-path'}):
    result=w.execute(p,root)
   self.assertEqual(result['role'],'reviewer'); self.assertIn('read-only',r.calls[-1][0])
   self.assertFalse(result['evidence_verified'])
   self.assertEqual(result['report']['evidence'],['fixture'])
   for _,_,_,options in r.calls:
    env=options['env']
    self.assertNotIn('AGENTLAB_NOTION_TOKEN',env)
    self.assertNotIn('OPENAI_API_KEY',env)
    self.assertNotEqual(env['HOME'],'controller-home')
    self.assertNotEqual(env['PATH'],'controller-path')
 def test_worker_rejects_missing_or_wrong_source_report(self):
  with tempfile.TemporaryDirectory() as d:
   root=Path(d); (root/'.codex/agents').mkdir(parents=True)
   (root/'.codex/agents/reviewer.toml').write_text('developer_instructions="Review only"')
   events=worker_transcript().splitlines()
   report=json.loads(json.loads(events[2])['item']['text'])
   transcripts=['\n'.join(events[:2]+events[-1:]), '\n'.join(events+[events[-1]]),
                worker_transcript(dict(report,source_sha='b'*40)),
                worker_transcript(dict(report,evidence=[{'unverified':'claim'}]))]
   for transcript in transcripts:
    with self.subTest(transcript=transcript), self.assertRaises(ValueError):
     worker=CodexWorker(runner=FakeRunner(transcript)); plan=worker.plan('reviewer','a'*40,'review')
     worker.activation=permit('agents_enabled',plan)
     worker.execute(plan,root)
 def test_process_cancellation_kills_running_group(self):
  runner=ProcessRunner(); process=MagicMock(pid=321)
  process.communicate.side_effect=subprocess.TimeoutExpired(['codex'],0.25)
  check_active=MagicMock(side_effect=[None,None,ValueError('Cancelled lease')])
  with patch('agentlab.integrations.os.name','posix'), \
       patch('agentlab.integrations.os.killpg',create=True) as kill_group, \
       patch('agentlab.integrations.signal.SIGKILL',9,create=True), \
       patch('agentlab.integrations.subprocess.Popen',return_value=process) as launch:
   with self.assertRaisesRegex(ValueError,'Cancelled lease'):
    runner.run(['codex'],'.',input_text='prompt',env={'PATH':'/usr/bin'},check_active=check_active)
  kill_group.assert_called_once_with(321,9)
  process.wait.assert_called_once_with(timeout=1)
  self.assertTrue(launch.call_args.kwargs['start_new_session'])
  self.assertEqual(launch.call_args.kwargs['env'],{'PATH':'/usr/bin'})
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
