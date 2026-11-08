import hashlib
import json
import tempfile
import unittest
from pathlib import Path

from agentlab.gates import digest
from agentlab.integrations import CodexWorker, Worktrees
from agentlab.orchestrator import Jobs
from agentlab.pipeline import Pipeline
from agentlab.tests.test_integrations import FakeRunner, permit


class PipelineRunner(FakeRunner):
    def __init__(self):
        super().__init__()
        self.on_worker = None

    def run(self, args, cwd, *extra, **options):
        if args[0] == 'codex' and self.on_worker is not None:
            self.on_worker()
        result = super().run(args, cwd, *extra, **options)
        if args[:2] == ['git', 'worktree']:
            roles = Path(args[-2]) / '.codex/agents'
            roles.mkdir(parents=True)
            (roles / 'reviewer.toml').write_text('developer_instructions="Review only"')
        return result


class PipelineTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        root = Path(temporary.name)
        self.db_path = root / 'jobs.sqlite'
        self.jobs = Jobs(self.db_path)
        self.addCleanup(self.jobs.close)
        self.runner = PipelineRunner()
        self.worker = CodexWorker(runner=self.runner)
        self.artifacts = root / 'artifacts'
        self.pipeline = Pipeline(self.jobs, Worktrees(root, root / 'trees', self.runner),
                                 self.worker, self.artifacts)
        self.sha = 'a' * 40
        self.prompt = 'Review this frozen replay'
        self.payload = dict(experiment_id='e', source_sha=self.sha, world_sha256='b'*64,
                            hypothesis='h', scenario='s', baseline='S1', rollback='r', seed=1,
                            oracle_in_policy=False, command_owner_count=1, strategy_id='S0',
                            integration_target='target', acceptance_tests=['baseline'],
                            rejection_criteria=['regression'], notion_issue_url='issue',
                            notion_context={'sources': [dict(page_id='page', retrieved_at='now',
                                                             content_sha256='a'*64, truncated=False)]},
                            worker_plan_sha256=digest(self.worker.plan('reviewer', self.sha, self.prompt)))
        self.key = self.jobs.submit(self.payload, 'a')
        self.jobs.approve(self.key, 'b')
        self.plan = self.worker.plan('reviewer', self.sha, self.prompt, job_id=self.key)
        self.worker.activation = permit('agents_enabled', self.plan)

    def test_worker_artifact_binds_job_plan_and_result(self):
        result = self.pipeline.run_worker(self.key, self.plan)
        state, evidence = self.jobs.db.execute('SELECT state,evidence FROM jobs WHERE id=?', (self.key,)).fetchone()
        self.assertEqual(state, 'offline_complete')
        raw = (self.artifacts / (evidence + '.json')).read_bytes()
        self.assertEqual(hashlib.sha256(raw).hexdigest(), evidence)
        record = json.loads(raw)
        self.assertEqual(record['job_id'], self.key)
        self.assertEqual(record['worker_plan_sha256'], digest(self.plan))
        self.assertEqual(record['result'], result)
        self.assertFalse(result['evidence_verified'])

    def test_alternate_prompt_job_and_reviewer_rejected_before_worktree(self):
        other_prompt = dict(self.plan, prompt='Other work on the same commit')
        self.worker.activation = permit('agents_enabled', other_prompt)
        with self.assertRaisesRegex(ValueError, 'reviewed job'):
            self.pipeline.run_worker(self.key, other_prompt)
        other_key = self.jobs.submit(dict(self.payload, experiment_id='other'), 'a')
        self.jobs.approve(other_key, 'b')
        self.worker.activation = permit('agents_enabled', self.plan)
        with self.assertRaisesRegex(ValueError, 'reviewed job'):
            self.pipeline.run_worker(other_key, self.plan)
        self.worker.activation = permit('agents_enabled', self.plan, reviewer='other')
        with self.assertRaisesRegex(PermissionError, 'identity'):
            self.pipeline.run_worker(self.key, self.plan)
        self.assertEqual(self.runner.calls, [])

    def test_cancelled_worker_cannot_publish_completion(self):
        cancelling_controller = Jobs(self.db_path)
        self.addCleanup(cancelling_controller.close)
        self.runner.on_worker = lambda: cancelling_controller.cancel(self.key)
        with self.assertRaisesRegex(ValueError, 'cancelled lease'):
            self.pipeline.run_worker(self.key, self.plan)
        self.assertFalse(self.artifacts.exists())
        state, evidence = self.jobs.db.execute('SELECT state,evidence FROM jobs WHERE id=?', (self.key,)).fetchone()
        self.assertEqual((state, evidence), ('cancelled', None))

    def test_job_bound_plan_requires_a_lease_check(self):
        with self.assertRaisesRegex(PermissionError, 'controller lease check'):
            self.worker.execute(self.plan, self.artifacts)
        self.assertEqual(self.runner.calls, [])
