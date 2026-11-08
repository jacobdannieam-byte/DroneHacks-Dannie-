import unittest
from agentlab.models import profile,PROFILES
from agentlab.integrations import CodexWorker
from agentlab.roles import NAMES
class Models(unittest.TestCase):
 def test_all_roles_pinned(self):
  self.assertEqual(set(PROFILES),set(NAMES))
  for role in NAMES:
   p=profile(role); self.assertIn(p['model'],('gpt-6-astra','gpt-6.1-sol'))
   if p['model']=='gpt-6-astra': self.assertEqual(p['reasoning_effort'],'low')
 def test_plan_binds_model(self):
  p=CodexWorker().plan('implementer','a'*40,'task')
  self.assertEqual(p['model'],'gpt-6.1-sol'); self.assertEqual(p['reasoning_effort'],'xhigh')
