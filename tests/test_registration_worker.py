"""Behavioral tests for bounded background work, without ROS or commands."""
from threading import Event
from types import SimpleNamespace
from refnav.world_model.registration_worker import RegistrationWorker


def test_latest_pending_capture_and_epoch_fence():
    started, release, finished = Event(), Event(), Event()
    captures = []
    def update(frame):
        captures.append(frame.capture_ns)
        if frame.capture_ns == 1:
            started.set()
            assert release.wait(2)
        else:
            finished.set()
        return SimpleNamespace(vio='epoch1', map_version='map1', capture_ns=frame.capture_ns)
    localizer = SimpleNamespace(vio='epoch1', policy=SimpleNamespace(hold_ns=10), update=update)
    worker = RegistrationWorker(localizer)
    try:
        assert worker.submit(SimpleNamespace(reference='epoch1', capture_ns=1))
        assert started.wait(2)
        assert worker.submit(SimpleNamespace(reference='epoch1', capture_ns=2))
        assert worker.submit(SimpleNamespace(reference='epoch1', capture_ns=3))
        assert not worker.submit(SimpleNamespace(reference='epoch0', capture_ns=4))
        release.set()
        assert finished.wait(2)
        with worker._condition:
            assert worker._condition.wait_for(lambda: worker._result is not None and worker._result.capture_ns == 3, 2)
        assert captures == [1, 3]
        assert worker.result(3, map_version='map1', reference='epoch1') is not None
        assert worker.result(20, map_version='map1', reference='epoch1') is None
        assert worker.result(3, map_version='map2', reference='epoch1') is None
        assert worker.result(3, map_version='map1', reference='epoch2') is None
    finally:
        release.set()
        assert worker.close(2)
    assert worker.result(3, map_version='map1', reference='epoch1') is None


def test_worker_failure_withholds_result_and_can_close():
    def fail(frame):
        raise ValueError('bad capture')
    worker = RegistrationWorker(SimpleNamespace(vio='epoch1', policy=SimpleNamespace(hold_ns=10), update=fail))
    try:
        worker.submit(SimpleNamespace(reference='epoch1', capture_ns=1))
        with worker._condition:
            assert worker._condition.wait_for(lambda: worker.failure is not None, 2)
        assert worker.result(1, map_version='map1', reference='epoch1') is None
    finally:
        assert worker.close(2)
