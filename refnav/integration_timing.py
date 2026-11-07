"""Optional, bounded wall-clock timing for shadow and replay diagnostics."""

from collections import deque
from time import perf_counter_ns

import numpy as np


class Timings:
    def __init__(self):
        self.samples = {}

    def record(self, stage, duration_ns):
        self.samples.setdefault(stage, deque(maxlen=2048)).append(duration_ns / 1e6)

    def summary(self):
        return {
            stage: dict(zip(("p50_ms", "p95_ms", "p99_ms"),
                            map(float, np.percentile(values, (50, 95, 99)))),
                        samples=len(values), max_ms=max(values))
            for stage, values in sorted(self.samples.items())
        }


def measure(timings, stage, fn, /, *args, **kwargs):
    if timings is None:
        return fn(*args, **kwargs)
    start = perf_counter_ns()
    try:
        return fn(*args, **kwargs)
    finally:
        timings.record(stage, perf_counter_ns() - start)
