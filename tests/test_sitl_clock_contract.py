from types import SimpleNamespace as S
import pytest
from examples.sitl_clock_contract import require_integral_microsecond_clock
from examples.qualification_recorder import TrialRecording
from examples.h1_observation import ClockMapping


def record():
    return TrialRecording('ros_sim_time','px4_local_ned',clock_mapping=ClockMapping(
        'px4_sitl_lockstep','ros_sim_time',0,0,0,10**12,'fixture-exact-clock'),
        trial_id='fixture',owner_id='/owner',profile_sha256='a'*64)


def test_equal_source_clock_does_not_wait_for_a_nonexistent_future_tick():
    r=record()
    r.status(S(timestamp=47_100_000,arming_state=1),47_100_000_000,wall_ns=10**9)
    r.clock(47_100_000_000,wall_ns=1_052_000_000)
    assert not r.pending and not r.faults
    assert r.document['status_reports'][0]['ingestion_ns']==47_100_000_000


def test_true_future_delivery_still_has_original_wall_time_fence():
    r=record()
    r.status(S(timestamp=47_104_000,arming_state=1),47_100_000_000,wall_ns=10**9)
    r.clock(47_100_000_000,wall_ns=1_052_000_000)
    assert r.faults==['clock_wait_expired']


def test_fractional_microsecond_clock_invalidates_exact_contract():
    require_integral_microsecond_clock(47,100000000)
    with pytest.raises(ValueError):require_integral_microsecond_clock(47,100000001)
