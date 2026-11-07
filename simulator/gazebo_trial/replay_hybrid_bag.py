"""Replay a recorded Gazebo observation bag through the same H1 and mission runtime."""

import argparse
import json
from pathlib import Path
from time import perf_counter_ns

from examples.hybrid_launch import configured
from examples.spatial_bag_input import read_bag


class Replay:
    def __init__(self, runtime, start, end):
        self.runtime = runtime
        self.topics = runtime.topics
        self.now = None
        self.start, self.end = start, end
        self.static = None

    def feed(self, topic, message, received_ns):
        if received_ns < self.start:
            if topic == "/tf_static":
                self.static = message
            return
        if received_ns > self.end:
            return
        if self.now is None and self.static is not None:
            self.runtime.feed("/tf_static", self.static, received_ns, received_ns)
            self.static = None
        if self.now is not None and received_ns > self.now:
            self.runtime.poll(self.now, self.now)
        self.now = received_ns
        self.runtime.feed(topic, message, received_ns, received_ns, receipt_wall_ns=perf_counter_ns())

    def finish(self):
        if self.now is None:
            raise ValueError("no observation timestamps in bag")
        self.runtime.poll(self.now, self.now)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--start-ns", type=int, required=True, help="reviewed post-bootstrap ROS receipt time")
    parser.add_argument("--end-ns", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    mode, runtime = configured(args.profile)
    if mode != "hybrid_shadow" or not runtime.adapter.simulated_estimator:
        parser.error("Gazebo replay requires a simulated-estimator shadow profile")
    clock = runtime.loop.replay.hybrid.clock
    if not clock.valid_from_ns <= args.start_ns < args.end_ns <= clock.valid_until_ns:
        parser.error("select a post-bootstrap window within the reviewed epoch")
    try:
        read_bag(args.bag, Replay(runtime, args.start_ns, args.end_ns))
        report = {"qualification": "recorded_replay_only", "fault": runtime.fault, "records": list(runtime.records),
                  "timings": runtime.timings.summary(), "total_records": runtime.record_count}
        with args.output.open("x", encoding="utf-8") as output:
            json.dump(report, output, indent=2, allow_nan=False)
            output.write("\n")
        return int(runtime.fault is not None or not any("decision" in row for row in runtime.records))
    finally:
        runtime.close()


if __name__ == "__main__":
    raise SystemExit(main())
