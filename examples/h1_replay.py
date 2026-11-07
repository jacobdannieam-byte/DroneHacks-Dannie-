"""Replay H1 sensor evidence and report read-only H0 diagnostics."""

import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import platform

import numpy as np

from examples.h1_fixture import exercise, profile
from examples.h1_observation import HybridBagProfile, HybridBagReplay
from examples.spatial_bag_input import read_bag
from refnav.world_model.spatial_volume import Cell


def _timing(values):
    if not values:
        return {"samples": 0}
    return {"samples": len(values), "p50_ms": float(np.percentile(values, 50)),
            "p95_ms": float(np.percentile(values, 95)), "max_ms": max(values)}


def report(replay, snapshot, *, evidence, rows):
    final = replay.diagnostic()
    counts = Counter(row["hybrid"]["reason"] for row in rows)
    metric = snapshot.metric
    cells = metric.cells_at(metric.query_ns, metric.reference)
    return {
        "evidence": evidence, "native_qualification": "review_required",
        "host": {"platform": platform.platform(), "machine": platform.machine(),
                 "processor": platform.processor(), "python": platform.python_version(), "numpy": np.__version__},
        "topic_types": replay.topics, "clock_mapping": asdict(replay.hybrid.clock),
        "reference": asdict(metric.reference), "calibration_id": metric.calibration_id,
        "reset_markers": {"ev": replay.hybrid.ev_reset_counter,
                          "odometry": replay.hybrid.alignment.estimator_reset_counter,
                          "local_position": replay.hybrid.local_reset_counters},
        "metric": {"available": metric.available, "capture_ns": metric.capture_ns,
                   "received_ns": metric.received_ns, "query_ns": metric.query_ns,
                   "cells": {kind.name.lower(): int(np.count_nonzero(cells == kind)) for kind in Cell}},
        "semantic_entity_count": len(snapshot.semantic["entities"]), "final": final, "frames": replay.rows,
        "timing": {"depth_process": _timing([row["map_update_ms"] for row in rows]),
                   "h0": _timing([row["hybrid"]["h0_ms"] for row in rows if "h0_ms" in row["hybrid"]]),
                   "observed_reasons": counts},
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    synthetic = commands.add_parser("fixture", help="exercise the adapter using rendered depth and synthetic messages")
    synthetic.add_argument("--repeats", type=int, choices=range(1, 101), default=1, metavar="1..100")
    synthetic.add_argument("--enforce-deadline", action="store_true")
    synthetic.add_argument("--offset-rig", action="store_true", help="use the test rig that sees the starting envelope")
    bag = commands.add_parser("bag", help="replay a reviewed single-epoch Workbench bag")
    bag.add_argument("path", type=Path)
    bag.add_argument("--profile", type=Path, required=True)
    for command in (synthetic, bag):
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output already exists")
    replay = None
    try:
        rows = []
        if args.command == "fixture":
            for _ in range(args.repeats):
                replay = HybridBagReplay(profile(offset_rig=args.offset_rig),
                                         monotonic=None if args.enforce_deadline else lambda: 0.)
                snapshot = exercise(replay)
                rows.extend(replay.rows)
            evidence = "synthetic_messages_host_timing" if args.enforce_deadline else "synthetic_messages_fixed_deadline_clock"
        else:
            replay = HybridBagReplay(HybridBagProfile.from_dict(json.loads(args.profile.read_text(encoding="utf-8"))))
            snapshot = read_bag(args.path, replay)
            rows = replay.rows
            evidence = "rosbag_replay_host_timing"
        data = report(replay, snapshot, evidence=evidence, rows=rows)
        code = int(not snapshot.metric.available or "decision" not in data["final"])
    except (OSError, ImportError, ValueError, TypeError, AttributeError, KeyError, RuntimeError) as error:
        data = {"native_qualification": "incomplete", "error": str(error),
                "frames": replay.rows if replay is not None else []}
        code = 2
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(data, output, indent=2, allow_nan=False)
        output.write("\n")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
