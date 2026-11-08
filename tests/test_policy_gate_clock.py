from types import SimpleNamespace as NS
import pytest
from examples.sim_clock_fence import SimulationClockFence

def fence(): return SimulationClockFence(NS(offset_ns=0,max_error_ns=0))
def clock(ns): return NS(clock=NS(sec=ns//10**9,nanosec=ns%10**9))
def test_deferral_preserves_source_and_callback_times_until_clock():
    f=fence(); msg=NS(timestamp_sample=104000)
    assert f.accept('/fmu/out/vehicle_local_position',msg,100000000,100000000)==[]
    rows=f.accept('/clock',clock(104000000),104000000,105000000)
    assert rows[1][1] is msg and msg.timestamp_sample==104000
    assert rows[1][2]==104000000
    assert f.audit[0]['callback_ros_ns']==100000000
    assert f.audit[0]['ingestion_wall_ns']==105000000

def test_large_future_and_stalled_clock_never_admitted():
    f=fence()
    with pytest.raises(ValueError): f.accept('/fmu/out/vehicle_odometry',NS(timestamp_sample=200000),100000000,0)
    f=fence(); f.accept('/fmu/out/vehicle_odometry',NS(timestamp_sample=104000),100000000,0)
    with pytest.raises(ValueError): f.accept('/clock',clock(100000000),100000000,50000001)
    assert not f.pending

def test_rewind_and_overload_fence_pending_data():
    f=fence(); f.accept('/clock',clock(100000000),100000000,0)
    with pytest.raises(ValueError): f.accept('/clock',clock(99000000),99000000,1)
    f=fence()
    for _ in range(32): f.accept('/fmu/out/vehicle_odometry',NS(timestamp_sample=104000),100000000,0)
    with pytest.raises(ValueError): f.accept('/fmu/out/vehicle_odometry',NS(timestamp_sample=104000),100000000,0)
    assert not f.pending

def test_newer_sample_cannot_overtake_deferred_sample():
    f=fence(); first=NS(timestamp_sample=104000); second=NS(timestamp_sample=108000)
    f.accept('/fmu/out/vehicle_odometry',first,100000000,0)
    assert f.accept('/fmu/out/vehicle_odometry',second,108000000,8000000)==[]
    rows=f.accept('/clock',clock(108000000),108000000,8000000)
    assert [row[1] for row in rows[1:]]==[first,second]

def test_publication_stamp_also_waits_and_audit_is_explicit():
    f=fence(); msg=NS(timestamp_sample=99000,timestamp=104000)
    assert f.accept('/fmu/out/vehicle_odometry',msg,100000000,0)==[]
    assert len(f.accept('/clock',clock(104000000),104000000,4000000))==2
    assert f.report()['audit_complete'] and f.report()['released']==1


def test_failed_native_future_stamp_is_retained_after_close_without_recovery():
    f = fence()
    f.accept('/clock', clock(100_000_000), 100_000_000, 900_000_000)
    msg = NS(timestamp_sample=149_000, timestamp=151_000)
    with pytest.raises(ValueError, match='sim_clock_future_or_overload'):
        f.accept('/fmu/out/vehicle_odometry', msg, 100_000_000, 901_000_000)
    f.close()
    failure = f.report()['failure']
    assert failure['cause'] == 'source_ahead_of_callback_limit'
    assert failure['source_sample_us'] == 149_000
    assert failure['source_publish_us'] == 151_000
    assert failure['mapped_capture_upper_ns'] == 151_000_000
    assert failure['callback_ros_ns'] == failure['last_delivered_clock_ns'] == 100_000_000
    assert failure['callback_wall_ns'] == 901_000_000
    assert failure['future_delta_ns'] == 51_000_000
    with pytest.raises(ValueError):
        f.accept('/clock', clock(151_000_000), 151_000_000, 902_000_000)
    assert f.report()['failure'] == failure


def test_overload_report_retains_pending_evidence_and_distinguishes_cause():
    f = fence()
    for _ in range(32):
        f.accept('/fmu/out/vehicle_odometry', NS(timestamp_sample=104000), 100_000_000, 0)
    with pytest.raises(ValueError):
        f.accept('/fmu/out/vehicle_odometry', NS(timestamp_sample=104000), 100_000_000, 1)
    failure = f.report()['failure']
    assert failure['cause'] == 'pending_capacity_exceeded'
    assert failure['pending_before_rejection'] == len(failure['pending']) == 32
    assert failure['future_delta_ns'] == 4_000_000
    assert not f.pending
