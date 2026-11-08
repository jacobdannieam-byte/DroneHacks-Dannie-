"""Offline reproduction of the supplied landing review against candidate code.

Imports only the reviewed point-model harness; never constructs a ROS node or
publisher. Source review artifacts are mounted read-only by the test command.
"""
import importlib.util
import json
import sys

spec = importlib.util.spec_from_file_location('landing_review_model', '/review/review.py')
model = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = model
spec.loader.exec_module(model)
results = []
for case in model.CASES:
    run = model.VerticalModel(case, False).run()
    result = run.summary()
    assert result['first_push'] is None, case.name
    assert all(row['command_down_mps'] <= .35 for row in run.trace), case.name
    if case.authority_loss_s is None:
        assert result['result'] == 'disarmed', case.name
    else:
        assert result['result'] == 'authority_lost', case.name
    results.append({'case': case.name, 'result': result['result'], 'first_push': result['first_push']})
print(json.dumps(results, indent=2))
