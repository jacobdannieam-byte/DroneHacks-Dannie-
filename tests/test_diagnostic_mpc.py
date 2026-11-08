"""Optimized schedules stay diagnostic and fail closed on deadlines or unknown space."""
from dataclasses import replace

from examples.diagnostic_mpc import optimize
from examples.unified_planner_ablation import cases
from examples.unified_trajectory_replay import VEHICLE
from refnav.hybrid_gateway import HybridGateway


def run(case, **kwargs):
    return optimize(case.state, case.snapshot, case.alignment, VEHICLE, case.objective,
                    case.now_ns, **kwargs)


def test_optimized_schedule_is_checked_and_cannot_enter_gateway():
    case = next(c for c in cases() if c.name == 'clear')
    result = run(case, monotonic=lambda: 0.)
    assert result.status == 'diagnostic_only'
    assert len(result.candidate.schedule) == 50
    assert result.candidate.schedule[0] != result.candidate.schedule[-1]
    gateway = HybridGateway(enabled=True)
    gateway.offer(result, owner_generation=0)
    assert not gateway.pending


def test_unknown_space_and_deadline_discard_all_output():
    case = next(c for c in cases() if c.name == 'unobserved')
    result = run(case, monotonic=lambda: 0.)
    assert result.status == 'infeasible' and result.candidate is None
    times = iter((0., 1., 1.))
    result = run(case, monotonic=lambda: next(times))
    assert result.status == 'deadline_exceeded' and result.candidate is None
