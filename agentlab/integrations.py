"""Explicit one-shot adapters. Construction and plan generation have no side effects."""
import hashlib,json,os,re,signal,sqlite3,subprocess,time,urllib.request,uuid
from pathlib import Path
from .gates import digest,manifest
from .roles import NAMES

class Activation:
    def __init__(self,config,approval=None): self.config=config; self.approval=approval or {}
    def require(self,capability,payload):
        a=self.approval
        if (self.config.get('mode')!='reviewed_execution' or self.config.get(capability) is not True
            or a.get('capability')!=capability or a.get('payload_sha256')!=digest(payload)
            or not a.get('reviewer') or a.get('reviewer')==a.get('author')
            or type(a.get('expires')) not in (int,float) or a['expires']<=time.time()):
            raise PermissionError('Separate current approval and execution configuration required')

class ProcessRunner:
    def run(self,argv,cwd,timeout=180,input_text=None):
        if not 0<timeout<=180: raise ValueError('Bounded timeout required')
        # POSIX process groups terminate child workers as well as the immediate process.
        if os.name=='nt': raise RuntimeError('Live execution requires qualified Linux process isolation')
        process=subprocess.Popen(argv,cwd=cwd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,start_new_session=True)
        try: out,err=process.communicate(input_text,timeout=timeout)
        except BaseException:
            os.killpg(process.pid,signal.SIGKILL); process.communicate(); raise
        if process.returncode: raise RuntimeError(f'Process failed ({process.returncode}): {err[-2000:]}')
        return out

class Worktrees:
    def __init__(self,repo,root,runner=None):
        self.repo=Path(repo).resolve(); self.root=Path(root).resolve(); self.runner=runner or ProcessRunner()
    def create(self,sha,job_id):
        if not re.fullmatch('[0-9a-f]{40}',sha) or not re.fullmatch('[a-zA-Z0-9_-]{1,64}',job_id): raise ValueError('Invalid immutable worktree identity')
        target=self.root/job_id
        if target.exists(): raise ValueError('Worktree already exists; preserve it')
        self.root.mkdir(parents=True,exist_ok=True)
        self.runner.run(['git','worktree','add','--detach',str(target),sha],self.repo)
        actual=self.runner.run(['git','rev-parse','HEAD'],target).strip()
        if actual!=sha: raise ValueError('Source mismatch')
        return target

class CodexWorker:
    def __init__(self,activation=None,runner=None): self.activation=activation; self.runner=runner or ProcessRunner()
    def plan(self,role,source_sha,prompt):
        if role not in NAMES or not re.fullmatch('[0-9a-f]{40}',source_sha): raise ValueError('Invalid worker identity')
        from .models import profile
        return dict(role=role,source_sha=source_sha,prompt=prompt,**profile(role))
    def execute(self,plan=None,worktree=None,timeout=180):
        if self.activation is None: raise PermissionError('Workers disabled')
        self.activation.require('agents_enabled',plan)
        role=plan['role']
        expected=self.plan(role,plan['source_sha'],plan['prompt'])
        if plan!=expected: raise ValueError('Worker plan differs from pinned role profile')
        cwd=Path(worktree).resolve()
        if self.runner.run(['git','rev-parse','HEAD'],cwd).strip()!=plan['source_sha']: raise ValueError('Worker source mismatch')
        import tomllib
        contract=tomllib.loads((cwd/'.codex/agents'/f'{role}.toml').read_text())['developer_instructions']
        # Workers return proposed patches as data; none receives write authority.
        prompt=contract+'\nReturn proposed changes as a unified diff in your report, never apply them.\n'+plan['prompt']
        out=self.runner.run(['codex','exec','--json','--sandbox','read-only','--ignore-user-config','--model',plan['model'],'-c','model_reasoning_effort='+json.dumps(plan['reasoning_effort']),'-'],cwd,timeout,prompt)
        events=[json.loads(line) for line in out.splitlines() if line.strip()]
        if not any(e.get('type')=='turn.completed' for e in events) or any(e.get('type') in ('turn.failed','error') for e in events): raise ValueError('Incomplete Codex turn')
        return dict(source_sha=plan['source_sha'],role=role,model=plan['model'],reasoning_effort=plan['reasoning_effort'],events=events,evidence_sha256=digest(events))

class GazeboRunner:
    def __init__(self,activation=None,runner=None): self.activation=activation; self.runner=runner or ProcessRunner()
    def plan(self,m,profile,approval,world,output):
        manifest(m)
        return dict(manifest=m,profile=str(Path(profile).resolve()),approval=str(Path(approval).resolve()),world=str(Path(world).resolve()),output=str(Path(output).resolve()))
    def execute(self,plan=None,worktree=None):
        if self.activation is None: raise PermissionError('Simulation disabled')
        self.activation.require('moving_sim_enabled',plan)
        m=plan['manifest']; manifest(m)
        if hashlib.sha256(Path(plan['world']).read_bytes()).hexdigest()!=m['world_sha256']: raise ValueError('World mismatch')
        if self.runner.run(['git','rev-parse','HEAD'],worktree).strip()!=m['source_sha']: raise ValueError('Source mismatch')
        return self.runner.run(['python3','examples/gazebo_container.py','fly','--mode','hybrid_sitl','--profile',plan['profile'],'--approval',plan['approval'],'--world',plan['world'],'--revision',m['source_sha'],'--output',plan['output']],worktree,180)

def ingest_referee(path,expected_sha256):
    raw=Path(path).read_bytes()
    if hashlib.sha256(raw).hexdigest()!=expected_sha256: raise ValueError('Referee artifact mismatch')
    s=json.loads(raw)
    required=('touches','checkpoint_seconds','landed_on_pad','motors_off_seconds','height_estimate_went_wrong','stayed_above_a_boulder_too_long','went_above_walls','left_the_course')
    if any(k not in s for k in required): raise ValueError('Incomplete referee summary')
    points=s['checkpoint_seconds']
    import math
    if not isinstance(points,dict) or type(s['touches']) is not int or s['touches']<0: raise ValueError('Invalid referee metrics')
    numbers=list(points.values())+[s['motors_off_seconds']]
    valid=all(type(n) in (int,float) and math.isfinite(n) and n>=0 for n in numbers)
    hazards=any(s[k] is not False for k in required[4:])
    return dict(evidence_sha256=expected_sha256,full_course=valid and len(points)==4 and s['touches']==0 and s['landed_on_pad'] is True and not hazards,referee=s,official_score=None)

def ingest_bag(path):
    p=Path(path).resolve()
    db=sqlite3.connect(p.as_uri()+'?mode=ro',uri=True)
    try:
        rows=db.execute('SELECT topics.name,topics.type,count(messages.id),min(messages.timestamp),max(messages.timestamp) FROM topics LEFT JOIN messages ON messages.topic_id=topics.id GROUP BY topics.id').fetchall()
        if not rows or not sum(r[2] for r in rows): raise ValueError('Empty ROS bag')
        return dict(sha256=hashlib.sha256(p.read_bytes()).hexdigest(),topics=[dict(name=a,type=b,count=c,start_ns=d,end_ns=e) for a,b,c,d,e in rows],telemetry_verified=False)
    finally: db.close()

class NotionPublisher:
    def __init__(self,activation=None,transport=None): self.activation=activation; self.transport=transport or urllib.request.urlopen
    def plan(self,page_id,report):
        page=str(uuid.UUID(page_id))
        text=json.dumps(report,sort_keys=True,allow_nan=False)
        if len(text)>18000: raise ValueError('Report exceeds bounded publication size')
        children=[dict(object='block',type='paragraph',paragraph=dict(rich_text=[dict(type='text',text=dict(content=text[i:i+1800]))])) for i in range(0,len(text),1800)]
        return dict(page_id=page,report_sha256=digest(report),children=children)
    def execute(self,plan=None):
        if self.activation is None: raise PermissionError('Publication disabled')
        self.activation.require('notion_write_enabled',plan)
        page=str(uuid.UUID(plan['page_id']))
        token=os.environ.get('AGENTLAB_NOTION_TOKEN')
        if not token: raise ValueError('Notion token unavailable')
        request=urllib.request.Request('https://api.notion.com/v1/blocks/'+page+'/children',data=json.dumps({'children':plan['children']}).encode(),method='PATCH',headers={'Authorization':'Bearer '+token,'Notion-Version':'2022-06-28','Content-Type':'application/json'})
        # Never automatically retry a timed-out write: reconcile the remote result first.
        with self.transport(request,timeout=30) as response: return json.load(response)
