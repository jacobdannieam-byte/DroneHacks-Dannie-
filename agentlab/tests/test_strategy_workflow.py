import unittest
from agentlab.strategy_workflow import *
class StrategyTests(unittest.TestCase):
 def test_existing_first(self):
  self.assertEqual(next_existing({}),'S1')
  self.assertEqual(next_existing({'S1':{'disposition':'retain_baseline','evidence':['physical user confirmation']}}),'S0')
 def test_new_requires_existing_and_research(self):
  context={'sources':[dict(page_id='p',retrieved_at='now',content_sha256='a'*64,truncated=False)]}
  p=dict(strategy_id='S5',hypothesis='h',integration_target='x',baseline='S1',acceptance_tests=['a'],rejection_criteria=['b'],notion_issue_url='url',primary_sources=['paper'])
  with self.assertRaises(ValueError): validate_proposal(p,{},context)
  records={s:dict(disposition='evaluate',evidence=['artifact']) for s in EXISTING}
  validate_proposal(p,records,context)
  context['sources'][0]['truncated']=True
  with self.assertRaises(ValueError): validate_proposal(p,records,context)
