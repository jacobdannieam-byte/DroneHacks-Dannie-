from types import SimpleNamespace
import pytest
from refnav.owner_timing import OwnerTiming

def simulated():
    return OwnerTiming.for_runtime(SimpleNamespace(adapter=SimpleNamespace(simulated_estimator=True)))

@pytest.mark.parametrize('key,limit', [('authority',750_000_000),('landed',1_500_000_000),('position',300_000_000)])
def test_exact_deadlines_and_future_refusal(key,limit):
    timing=simulated()
    assert timing.fresh(key,0,limit)
    assert not timing.fresh(key,0,limit+1)
    assert not timing.fresh(key,1,0)
    assert not timing.fresh(key,0,limit,1)

def test_hardware_budget_unchanged():
    assert not OwnerTiming().fresh('authority',0,300_000_001)
    assert not OwnerTiming().fresh('landed',0,300_000_001)

def test_longer_detection_requires_stopping_clearance_budget():
    with pytest.raises(ValueError): simulated().validate_stopping_budget(SimpleNamespace(reaction_s=.3))
    simulated().validate_stopping_budget(SimpleNamespace(reaction_s=.75))
