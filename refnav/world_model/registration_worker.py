"""Bounded background registration for read-only consumers; never publishes commands.

One running capture and one replaceable pending capture bound memory and backlog.
Each worker owns one localizer/epoch. Close it and create a new worker on reset.
"""
from copy import deepcopy
from threading import Condition, Thread


class RegistrationWorker:
    def __init__(self, localizer):
        self.localizer = localizer
        self._condition = Condition()
        self._pending = None
        self._result = None
        self._closed = False
        self._last_capture = -1
        self.failure = None
        self._thread = Thread(target=self._run, daemon=True)
        self._thread.start()

    def submit(self, frame):
        with self._condition:
            if self._closed or frame.reference != self.localizer.vio or frame.capture_ns <= self._last_capture:
                return False
            self._last_capture = frame.capture_ns
            self._pending = deepcopy(frame)
            self._condition.notify()
            return True

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._closed or self._pending is not None)
                if self._closed:
                    return
                frame, self._pending = self._pending, None
            try:
                result = self.localizer.update(frame)
            except Exception as error:
                with self._condition:
                    self.failure = type(error).__name__
                    self._result = None
                    self._condition.notify_all()
                continue
            with self._condition:
                if not self._closed:
                    self._result = result
                    self.failure = None
                    self._condition.notify_all()

    def result(self, now_ns, *, map_version, reference):
        with self._condition:
            result = self._result
            if (self._closed or result is None or result.vio != reference
                    or result.map_version != map_version or result.capture_ns is None
                    or not 0 <= now_ns - result.capture_ns <= self.localizer.policy.hold_ns):
                return None
            return result

    def close(self, timeout_seconds=0.1):
        with self._condition:
            self._closed = True
            self._pending = self._result = None
            self._condition.notify()
        self._thread.join(timeout_seconds)
        return not self._thread.is_alive()
