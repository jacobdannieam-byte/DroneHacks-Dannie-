"""Explicit one-shot adapters. Construction and plan generation have no side effects."""
import hashlib,json,os,re,signal,sqlite3,subprocess,time,urllib.request,uuid
from pathlib import Path
from types import MappingProxyType
from .gates import digest,manifest
from .models import profile
from .roles import NAMES


# Worker subprocesses receive an explicit environment. A deployment may provide
# a dedicated worker home for CLI credentials, separate from the controller.
_WORKER_ENV = {
    'PATH': '/usr/local/bin:/usr/bin:/bin',
    'LANG': 'C.UTF-8',
    'HOME': '/nonexistent',
    'CODEX_HOME': '/nonexistent',
    'GIT_CONFIG_NOSYSTEM': '1',
    'GIT_CONFIG_GLOBAL': os.devnull,
    'GIT_CONFIG_COUNT': '1',
    'GIT_CONFIG_KEY_0': 'core.hooksPath',
    'GIT_CONFIG_VALUE_0': os.devnull,
}


class Activation:
    def __init__(self,config,approval=None,*,verify_approval=None):
        self.config=dict(config)
        self.approval=dict(approval or {})
        # The trusted controller supplies the verifier. It must authenticate the
        # entire record and resolve author/reviewer to canonical identities.
        self.verify_approval=verify_approval

    def require(self,capability,payload):
        approval=MappingProxyType(dict(self.approval))
        author=approval.get('author')
        reviewer=approval.get('reviewer')
        expires=approval.get('expires')
        if (self.config.get('mode')!='reviewed_execution' or self.config.get(capability) is not True
            or approval.get('capability')!=capability or approval.get('payload_sha256')!=digest(payload)
            or not all(isinstance(identity,str) and identity and identity==identity.strip()
                       for identity in (author,reviewer)) or reviewer==author
            or type(expires) not in (int,float) or not time.time()<expires<float('inf')
            or self.verify_approval is None or self.verify_approval(approval) is not True
            or expires<=time.time()):
            raise PermissionError('Separate current approval and execution configuration required')
        return approval


class ProcessRunner:
    def run(self,argv,cwd,timeout=180,input_text=None,*,env=None,check_active=None):
        if not 0<timeout<=180: raise ValueError('Bounded timeout required')
        # POSIX process groups terminate child workers as well as the immediate process.
        if os.name=='nt': raise RuntimeError('Live execution requires qualified Linux process isolation')
        if check_active is not None:
            check_active()
        deadline=time.monotonic()+timeout
        process=subprocess.Popen(argv,cwd=cwd,env=env,stdin=subprocess.PIPE,stdout=subprocess.PIPE,
                                 stderr=subprocess.PIPE,text=True,encoding='utf-8',start_new_session=True)
        try:
            while True:
                if check_active is not None:
                    check_active()
                remaining=deadline-time.monotonic()
                if remaining<=0:
                    raise subprocess.TimeoutExpired(argv,timeout)
                try:
                    interval=min(remaining,0.25) if check_active is not None else remaining
                    out,err=process.communicate(input_text,timeout=interval)
                    break
                except subprocess.TimeoutExpired:
                    input_text=None  # communicate keeps the input buffer between calls.
        except BaseException:
            try:
                os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:
                pass
            # A detached descendant can retain a pipe after the process group is
            # gone. Close the streams and bound reaping the immediate child.
            for stream in (process.stdin,process.stdout,process.stderr):
                stream.close()
            process.wait(timeout=1)
            raise
        if check_active is not None:
            check_active()
        if process.returncode: raise RuntimeError(f'Process failed ({process.returncode}): {err[-2000:]}')
        return out

class Worktrees:
    def __init__(self,repo,root,runner=None):
        self.repo=Path(repo).resolve(); self.root=Path(root).resolve(); self.runner=runner or ProcessRunner()
    def create(self,sha,job_id,*,check_active=None):
        if not re.fullmatch('[0-9a-f]{40}',sha) or not re.fullmatch('[a-zA-Z0-9_-]{1,64}',job_id): raise ValueError('Invalid immutable worktree identity')
        target=self.root/job_id
        if target.exists(): raise ValueError('Worktree already exists; preserve it')
        self.root.mkdir(parents=True,exist_ok=True)
        self.runner.run(['git','worktree','add','--detach',str(target),sha],self.repo,
                        env=_WORKER_ENV,check_active=check_active)
        actual=self.runner.run(['git','rev-parse','HEAD'],target,
                               env=_WORKER_ENV,check_active=check_active).strip()
        if actual!=sha: raise ValueError('Source mismatch')
        return target

class CodexWorker:
    def __init__(self,activation=None,runner=None,*,worker_home=None):
        self.activation=activation
        self.runner=runner or ProcessRunner()
        self.environment=dict(_WORKER_ENV)
        if worker_home is not None:
            home=Path(worker_home).resolve()
            self.environment.update(HOME=str(home),CODEX_HOME=str(home/'.codex'))

    def plan(self,role,source_sha,prompt,*,job_id=None):
        if role not in NAMES or not isinstance(source_sha,str) or not re.fullmatch('[0-9a-f]{40}',source_sha):
            raise ValueError('Invalid worker identity')
        if not isinstance(prompt,str) or not prompt.strip():
            raise ValueError('Worker prompt required')
        plan=dict(role=role,source_sha=source_sha,prompt=prompt,**profile(role))
        if job_id is not None:
            if not isinstance(job_id,str) or not re.fullmatch('[0-9a-f]{64}',job_id):
                raise ValueError('Invalid worker job identity')
            plan['job_id']=job_id
        return plan

    def validate_plan(self,plan):
        if not isinstance(plan,dict):
            raise ValueError('Worker plan required')
        expected=self.plan(plan.get('role'),plan.get('source_sha'),plan.get('prompt'),job_id=plan.get('job_id'))
        if plan!=expected:
            raise ValueError('Worker plan differs from pinned role profile')
        return expected

    def execute(self,plan=None,worktree=None,timeout=180,*,check_active=None):
        if self.activation is None: raise PermissionError('Workers disabled')
        plan=self.validate_plan(plan)
        if 'job_id' in plan and check_active is None:
            raise PermissionError('Job-bound workers require a controller lease check')
        approval=self.activation.require('agents_enabled',plan)

        def require_active():
            # Pipeline supplies the trusted controller's cancellable lease check.
            # Approval is authenticated at admission and remains time-bounded.
            if time.time()>=approval['expires']:
                raise PermissionError('Worker approval expired')
            if check_active is not None:
                check_active()

        role=plan['role']
        cwd=Path(worktree).resolve()
        options=dict(env=self.environment,check_active=require_active)
        if self.runner.run(['git','rev-parse','HEAD'],cwd,**options).strip()!=plan['source_sha']:
            raise ValueError('Worker source mismatch')
        if self.runner.run(['git','status','--porcelain','--untracked-files=all','--ignored'],cwd,**options).strip():
            raise ValueError('Worker source has local changes')
        import tomllib
        contract_path=(cwd/'.codex/agents'/f'{role}.toml').resolve()
        if not contract_path.is_relative_to(cwd):
            raise ValueError('Role contract outside worker source')
        contract=tomllib.loads(contract_path.read_text(encoding='utf-8'))['developer_instructions']
        # Workers return proposed patches as data; none receives write authority.
        prompt=(f'{contract}\n\nTask:\n{plan["prompt"]}\n\n'
                f'Return exactly one JSON object with role {role!r} and source_sha {plan["source_sha"]!r}. '
                'Use arrays of nonempty strings for evidence, findings and limitations, and a nonempty '
                'string for proposed_next_step. Include any proposed unified diff as a proposed_patch '
                'string. Do not apply changes.')
        out=self.runner.run(['codex','exec','--json','--sandbox','read-only','--ignore-user-config','--model',plan['model'],'-c','model_reasoning_effort='+json.dumps(plan['reasoning_effort']),'-'],cwd,timeout,prompt,**options)
        require_active()
        events=[json.loads(line) for line in out.splitlines() if line.strip()]
        report=_worker_report(events,plan)
        return dict(source_sha=plan['source_sha'],role=role,model=plan['model'],
                    reasoning_effort=plan['reasoning_effort'],events=events,
                    evidence_sha256=digest(events),report=report,evidence_verified=False)


def _worker_report(events,plan):
    if any(not isinstance(event,dict) for event in events):
        raise ValueError('Invalid Codex event')
    lifecycle=[event.get('type') for event in events if event.get('type') in (
        'thread.started','turn.started','turn.completed','turn.failed','error')]
    if (lifecycle!=['thread.started','turn.started','turn.completed']
        or events[0].get('type')!='thread.started' or events[1].get('type')!='turn.started'
        or events[-1].get('type')!='turn.completed' or not events[0].get('thread_id')):
        raise ValueError('Incomplete Codex turn')
    messages=[event['item'] for event in events[2:-1] if event.get('type')=='item.completed'
              and isinstance(event.get('item'),dict) and event['item'].get('type')=='agent_message']
    if not messages or not isinstance(messages[-1].get('text'),str):
        raise ValueError('Missing worker report')
    report=json.loads(messages[-1]['text'])
    required={'role','source_sha','evidence','findings','limitations','proposed_next_step'}
    if (not isinstance(report,dict) or not required<=report.keys()
        or report.keys()-required-{'proposed_patch'}
        or report['role']!=plan['role'] or report['source_sha']!=plan['source_sha']):
        raise ValueError('Invalid worker report identity or fields')
    for field in ('evidence','findings','limitations'):
        if not isinstance(report[field],list) or any(not isinstance(value,str) or not value.strip()
                                                    for value in report[field]):
            raise ValueError('Invalid worker report '+field)
    for field in ('proposed_next_step','proposed_patch'):
        if field in report and (not isinstance(report[field],str) or not report[field].strip()):
            raise ValueError('Invalid worker report '+field)
    return report

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
