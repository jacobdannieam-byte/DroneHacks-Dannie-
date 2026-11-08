from queue import Empty, Queue
from threading import Event
from types import SimpleNamespace as NS

import pytest

from examples.hybrid_native_shadow import ClockExecutorService
from examples.sim_clock_fence import SimulationClockFence


class DeliveredClock:
    def __init__(self):
        self.value = 0

    def now(self):
        return NS(nanoseconds=self.value)


class ClockExecutor:
    def __init__(self, clock):
        self.clock = clock
        self.messages = Queue(maxsize=2)

    def spin_once(self, timeout_sec):
        try:
            value, delivered = self.messages.get(timeout=timeout_sec)
        except Empty:
            return
        self.clock.value = value
        delivered.set()


def deliver(executor, value):
    delivered = Event()
    executor.messages.put((value, delivered), timeout=.5)
    assert delivered.wait(.5), 'independent clock callback did not run'


def test_receipt_clock_advances_while_spatial_callback_remains_busy():
    clock = DeliveredClock()
    executor = ClockExecutor(clock)
    service = ClockExecutorService(executor, clock)
    assert service.now_ns() == 0  # No startup time is synthesized.
    service.start()
    try:
        deliver(executor, 100_000_000)
        original_receipt = service.now_ns()
        # The main (spatial) callback does not spin any ROS executor here.
        # The separate TimeSource can still deliver a 68 ms clock advance.
        deliver(executor, 168_000_000)
        next_receipt = service.now_ns()
        assert original_receipt == 100_000_000 and next_receipt == 168_000_000
        fence = SimulationClockFence(NS(offset_ns=0, max_error_ns=0))
        msg = NS(timestamp=168_000, timestamp_sample=164_000)
        assert fence.accept('/fmu/out/vehicle_odometry', msg, next_receipt, 200_000_000)
        assert msg.timestamp == 168_000 and msg.timestamp_sample == 164_000
        with pytest.raises(ValueError):
            fence.accept('/fmu/out/vehicle_odometry', NS(timestamp_sample=219_000), next_receipt, 201_000_000)
        assert fence.report()['failure']['future_delta_ns'] == 51_000_000
    finally:
        service.close()
    assert not service.thread.is_alive()


def test_paused_or_rewound_delivered_clock_is_not_extrapolated_or_repaired():
    clock = DeliveredClock()
    executor = ClockExecutor(clock)
    service = ClockExecutorService(executor, clock)
    service.start()
    try:
        deliver(executor, 100_000_000)
        assert service.now_ns() == service.now_ns() == 100_000_000
        fence = SimulationClockFence(NS(offset_ns=0, max_error_ns=0))
        fence.accept('/clock', NS(clock=NS(sec=0, nanosec=100_000_000)), service.now_ns(), 0)
        fence.accept('/fmu/out/vehicle_odometry', NS(timestamp_sample=104_000), service.now_ns(), 0)
        with pytest.raises(ValueError, match='wait_expired'):
            fence.accept('/clock', NS(clock=NS(sec=0, nanosec=100_000_000)), service.now_ns(), 50_000_001)
        deliver(executor, 90_000_000)
        assert service.now_ns() == 90_000_000
        fence = SimulationClockFence(NS(offset_ns=0, max_error_ns=0))
        fence.accept('/clock', NS(clock=NS(sec=0, nanosec=100_000_000)), 100_000_000, 0)
        with pytest.raises(ValueError, match='rewind'):
            fence.accept('/clock', NS(clock=NS(sec=0, nanosec=90_000_000)), service.now_ns(), 1)
    finally:
        service.close()


def test_clock_thread_exception_fails_closed_and_joins():
    class BrokenExecutor:
        def spin_once(self, timeout_sec):
            raise RuntimeError('delivery failed')
    service = ClockExecutorService(BrokenExecutor(), DeliveredClock())
    service.start()
    service.thread.join(timeout=.5)
    with pytest.raises(ValueError, match='native_clock_service_failed:delivery failed'):
        service.now_ns()
    service.close()
    assert service.failure == {'type': 'RuntimeError', 'reason': 'delivery failed'}
