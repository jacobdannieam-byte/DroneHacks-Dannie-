"""Opt-in offline P2 evidence beside the existing, read-only H2 shadow replay."""

import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from time import perf_counter, perf_counter_ns, process_time_ns
from types import SimpleNamespace

import numpy as np

from examples.h1_fixture import MS, profile, setup, stamp, tick
from examples.h1_observation import HybridBagProfile
from examples.hybrid_shadow_loop import ShadowLoop
from examples.hybrid_sitl import ReplayDriver
from examples.p2_registration_benchmark import errors, reference_for
from examples.semantic_world_model_demo import render, scene
from examples.spatial_bag_input import read_bag
from refnav.integration_timing import Timings, measure
from refnav.world_model.map_localization import MapLocalization, MapLocalizer, ReviewedAnchor
from refnav.world_model.map_registration import SE3
from refnav.world_model.reference_map import load_reference
from refnav.world_model.spatial_input import SpatialReference
from refnav.world_model.spatial_pipeline import SpatialSnapshot
from refnav.world_model.spatial_topology import TopologyMemory, TopologyView

IDENTITY = SE3(np.eye(3), (0., 0., 0.))
VARIANTS = {
    "h1_metric": (False, False, False),
    "geometry": (True, False, False),
    "localization": (False, True, False),
    "combined": (True, True, True),
}
RECTANGLES = scene(rear=False) + [
    (0, 5.2, (-2., 2.), (.1, 2.6)),
    (1, -1.23, (.2, 5.2), (.1, 2.6)),
    (1, 1.12, (.2, 5.2), (.1, 2.6)),
    (2, .30, (.2, 5.2), (-1.23, 1.12)),
    (2, 2.45, (.2, 5.2), (-1.23, 1.12)),
]
# VIO input carries a known growing bias relative to the renderer. Only the
# evaluator uses the physical positions and poses.
POSITIONS = {100: (0., 0.), 400: (.25, .035), 700: (.50, .07), 1000: (.75, .11)}


@dataclass(frozen=True)
class P2Snapshot:
    schema: int
    spatial: SpatialSnapshot
    localization: MapLocalization | None
    topology: TopologyView | None
    localization_reason: str


class DiagnosticSidecar:
    def __init__(self, replay, localizer=None, topology=None, timings=None):
        self.replay = replay
        self.localizer = localizer
        self.topology = topology
        self.timings = timings
        self._consumed = None

    def snapshot(self, now_ns):
        """Consume the latest valid H1 capture after H2 completes its tick."""
        replay, localizer = self.replay, self.localizer
        spatial = replay.world.snapshot(now_ns=now_ns, immutable_semantic=True)
        fix = topology = None
        reason = "reference_disabled"
        if localizer is not None:
            if replay.fault is not None or replay.world.last_pose_loss_ns is not None:
                reason = "h1_epoch_fault" if replay.fault is not None else "h1_tracking_loss_requires_reanchor"
                fix = localizer.invalidate(reason)
                if self.topology is not None:
                    self.topology.clear()
            else:
                frame = replay.aligned_depth
                if (frame is None or not replay.world.observations_available
                        or not replay.visual_pose_available(now_ns, frame.pose.bracket_ns)
                        or replay.hybrid.tracking_source != "visual"
                        or now_ns - frame.capture_ns > localizer.policy.hold_ns):
                    reason = "no_fresh_h1_visual_capture"
                else:
                    if frame.capture_ns != self._consumed:
                        fix = measure(self.timings, "map_localization", localizer.update, frame)
                        self._consumed = frame.capture_ns
                        if self.topology is not None:
                            measure(self.timings, "topology_update", self.topology.observe, spatial, fix)
                    fix = localizer.snapshot(now_ns)
                    reason = fix.reason
        if self.topology is not None:
            topology = measure(self.timings, "topology_query", self.topology.view, now_ns=now_ns)
        return P2Snapshot(1, spatial, fix, topology, reason)


def _fixture_reference(directory):
    rng = np.random.default_rng(5)
    surfaces = []
    for axis, value, bounds1, bounds2 in RECTANGLES:
        a, b = (i for i in range(3) if i != axis)
        xyz = np.empty((2200, 3))
        xyz[:, axis] = value
        xyz[:, a] = rng.uniform(*bounds1, len(xyz))
        xyz[:, b] = rng.uniform(*bounds2, len(xyz))
        surfaces.append(xyz)
    return reference_for(directory, np.concatenate(surfaces), name="p2-shadow-fixture")


def _fixture_depth(time_ms, _x, *, cal, blocked):
    physical, _ = POSITIONS[time_ms]
    image = render(RECTANGLES, (physical, .10, 1.3))
    return SimpleNamespace(header=SimpleNamespace(stamp=stamp(time_ms), frame_id=cal.camera_frame),
                           width=cal.width, height=cal.height, data=image.astype("<f4").tobytes(),
                           step=4 * cal.width, is_bigendian=False, encoding="32FC1")

def _fixture_inputs(driver):
    setup(driver)
    for time_ms, (physical, bias) in POSITIONS.items():
        tick(driver, time_ms, physical + bias, depth_factory=_fixture_depth)
        driver.poll((time_ms + 3) * MS)
    driver.finish()


def _query_metric(metric, vehicle, alignment, now_ns):
    # Fixed local diagnostic probe. The fixture evaluator independently
    # checks that both swept probes cross physical surfaces.
    margin = vehicle.tracking_error_m + vehicle.state_position_error_m + alignment.position_error_m
    point = (2.3, .10, 1.3)
    floor, ceiling = metric.vertical_clearance(
        point, below_m=1.2, above_m=1.3, reference=metric.reference, now_ns=now_ns,
        vehicle_radius_m=vehicle.radius_m, margin_m=margin,
    )
    return {
        "cell": metric.cell_at(point, reference=metric.reference, now_ns=now_ns).name,
        "occupied_distance_m": metric.nearest_occupied_m(
            point, max_distance_m=2., reference=metric.reference, now_ns=now_ns),
        "floor": asdict(floor), "ceiling": asdict(ceiling),
    }


def _run(bag_profile, timings, *, reference=None, anchor=None, geometry=False, topology=False,
         bag=None, retain_rows=False, monotonic=perf_counter):
    loop = ShadowLoop(bag_profile, timings=timings, measure_receipt_wall=True,
                      retain_aligned_depth=reference is not None, monotonic=monotonic)
    localizer = None if reference is None else MapLocalizer(reference, bag_profile.spatial.reference, anchor)
    sidecar = DiagnosticSidecar(loop.replay, localizer, TopologyMemory() if topology else None, timings)
    rows = []

    def record(tick_result):
        row = tick_result.report()
        now = tick_result.now_ns
        if geometry or localizer is not None:
            snapshot = measure(timings, "diagnostic_snapshot", sidecar.snapshot, now)
            row["p2_schema"] = snapshot.schema
            row["localization_reason"] = snapshot.localization_reason
            metric = snapshot.spatial.metric
            if metric.capture_ns is not None:
                row["metric_age_ms"] = (now - metric.capture_ns) / 1e6
                row["metric_numeric_bytes"] = metric._cells.nbytes + metric._stamps.nbytes
            if geometry and bag is None:
                row["geometry"] = measure(timings, "metric_queries", _query_metric, metric,
                                          bag_profile.vehicle, bag_profile.alignment, now)
            if snapshot.localization is not None:
                row["localization"] = asdict(snapshot.localization)
                fit = snapshot.localization
                if bag is None and fit.reference_from_body is not None:
                    physical, bias = POSITIONS[fit.capture_ns // MS]
                    truth = SE3(np.eye(3), (physical, .10, 1.3))
                    row["translation_error_m"], row["rotation_error_deg"] = errors(fit.reference_from_body, truth)
                    row["vio_translation_error_m"] = bias
            if snapshot.topology is not None:
                view = asdict(snapshot.topology)
                row["topology_entities"] = len(view["entities"])
                row["topology_serialized_bytes"] = len(json.dumps(view).encode())
                if retain_rows:
                    row["topology"] = view
        rows.append(row)
        if len(rows) > 2048:
            raise ValueError("shadow report exceeds the 2048-tick limit")

    driver = ReplayDriver(loop, record, auto_poll=bag is not None, receipt_clock=perf_counter_ns)
    try:
        if bag is None:
            _fixture_inputs(driver)
        else:
            read_bag(bag, driver)
    finally:
        loop.close()
    return rows, loop.skipped_ticks, loop.replay.dropped_depth


def _summary(rows, timings, skipped, dropped, cpu_seconds, wall_seconds):
    errors_m = [row["translation_error_m"] for row in rows if "translation_error_m" in row]
    vio_errors = [row["vio_translation_error_m"] for row in rows if "vio_translation_error_m" in row]
    rotation_errors = [row["rotation_error_deg"] for row in rows if "rotation_error_deg" in row]
    probes = Counter(row["geometry"][side]["status"] for row in rows if "geometry" in row
                     for side in ("floor", "ceiling"))
    return {
        "ticks": len(rows), "h2_reasons": dict(Counter(row["reason"] for row in rows)),
        "h2_decisions": dict(Counter(row.get("decision", "no_assessment") for row in rows)),
        "skipped_ticks": skipped, "dropped_depth": dropped,
        "localization_outcomes": dict(Counter(row["localization_reason"] for row in rows if "localization_reason" in row)),
        "map_fixes": sum(row.get("localization", {}).get("reference_from_body") is not None for row in rows),
        "map_translation_rmse_m": float(np.sqrt(np.mean(np.square(errors_m)))) if errors_m else None,
        "matched_vio_translation_rmse_m": float(np.sqrt(np.mean(np.square(vio_errors)))) if vio_errors else None,
        "max_rotation_error_deg": max(rotation_errors, default=None),
        "floor_ceiling_probes": dict(probes), "false_free_vertical_probes": probes["verified_free"],
        "max_metric_age_ms": max((row["metric_age_ms"] for row in rows if "metric_age_ms" in row), default=None),
        "metric_numeric_bytes": max((row.get("metric_numeric_bytes", 0) for row in rows), default=0),
        "max_topology_entities": max((row.get("topology_entities", 0) for row in rows), default=0),
        "max_topology_serialized_bytes": max((row.get("topology_serialized_bytes", 0) for row in rows), default=0),
        "wall_seconds": wall_seconds, "process_cpu_seconds": cpu_seconds,
        "timing": timings.summary(),
    }


def compare(directory, repeats, *, monotonic=perf_counter):
    reference = _fixture_reference(directory)
    anchor = ReviewedAnchor(reference.version, 1, profile().spatial.reference, 100 * MS,
                            IDENTITY, "synthetic-room-coordinate-evidence")
    results = {}
    for name, (geometry, use_reference, topology) in VARIANTS.items():
        timings = Timings()
        rows, skipped, dropped = [], 0, 0
        start, cpu_start = perf_counter_ns(), process_time_ns()
        for _ in range(repeats):
            run, missed, losses = _run(profile(), timings, geometry=geometry, topology=topology,
                                       reference=reference if use_reference else None, anchor=anchor, monotonic=monotonic)
            rows.extend(run)
            skipped += missed
            dropped += losses
        results[name] = _summary(rows, timings, skipped, dropped, (process_time_ns() - cpu_start) / 1e9,
                                 (perf_counter_ns() - start) / 1e9)
        results[name]["first_history"] = rows[:len(POSITIONS)]
    return {"schema": 1, "evidence": "synthetic_h1_message_replay", "h3_enabled": False,
            "reference_version": reference.version, "reference_points": len(reference.points),
            "reference_array_bytes": reference.points.nbytes, "variants": results}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    fixture = commands.add_parser("fixture", help="compare four arms on matched H1 message histories")
    fixture.add_argument("--repeats", type=int, choices=range(1, 31), default=10, metavar="1..30")
    bag = commands.add_parser("bag", help="replay a recorded single-epoch H1 bag without ROS publishers")
    bag.add_argument("path", type=Path)
    bag.add_argument("--profile", type=Path, required=True)
    bag.add_argument("--reference", type=Path)
    bag.add_argument("--anchor", type=Path)
    for command in (fixture, bag):
        command.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output must be a new file")
    if args.command == "fixture":
        with TemporaryDirectory() as directory:
            report = compare(directory, args.repeats)
    else:
        bag_profile = HybridBagProfile.from_dict(json.loads(args.profile.read_text(encoding="utf-8")))
        reference = anchor = None
        reference_error = None
        if args.reference is not None or args.anchor is not None:
            try:
                if args.reference is None or args.anchor is None:
                    raise ValueError("reference and reviewed anchor must be supplied together")
                reference = load_reference(args.reference)
                data = json.loads(args.anchor.read_text(encoding="utf-8"))
                data["reference_from_vio"] = SE3(**data["reference_from_vio"])
                data["vio"] = SpatialReference(**data["vio"])
                anchor = ReviewedAnchor(**data)
            except (OSError, TypeError, ValueError, KeyError) as error:
                reference_error = str(error)
                reference = None
        timings = Timings()
        start, cpu_start = perf_counter_ns(), process_time_ns()
        rows, skipped, dropped = _run(bag_profile, timings, reference=reference, anchor=anchor,
                                     topology=reference is not None, bag=args.path, retain_rows=True)
        report = {"schema": 1, "evidence": "h1_rosbag_replay", "h3_enabled": False,
                  "reference_error": reference_error, "rows": rows,
                  "summary": _summary(rows, timings, skipped, dropped, (process_time_ns() - cpu_start) / 1e9,
                                      (perf_counter_ns() - start) / 1e9)}
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write("\n")


if __name__ == "__main__":
    main()

class AsyncDiagnosticSidecar:
    """Optional bounded registration beside H2; it does not alter H2 decisions.

    Close and replace this sidecar after any tracking loss or reference reset.
    The worker exclusively owns the localizer until close finishes.
    """
    def __init__(self, replay, localizer):
        from refnav.world_model.registration_worker import RegistrationWorker
        self.replay = replay
        self.worker = RegistrationWorker(localizer)
        self.map_version = localizer.reference.version
        self.reference = localizer.vio
        self.hold_ns = localizer.policy.hold_ns
        self.closed = False

    def snapshot(self, now_ns):
        replay = self.replay
        spatial = replay.world.snapshot(now_ns=now_ns, immutable_semantic=True)
        if replay.fault is not None or replay.world.last_pose_loss_ns is not None:
            self.close()
        frame = replay.aligned_depth
        valid = (not self.closed and frame is not None and replay.world.observations_available
                 and replay.hybrid.tracking_source == 'visual'
                 and replay.visual_pose_available(now_ns, frame.pose.bracket_ns)
                 and 0 <= now_ns - frame.capture_ns <= self.hold_ns)
        if not valid:
            return P2Snapshot(1, spatial, None, None, 'no_fresh_h1_visual_capture')
        self.worker.submit(frame)
        fix = self.worker.result(now_ns, map_version=self.map_version, reference=self.reference)
        return P2Snapshot(1, spatial, fix, None, 'worker_pending' if fix is None else fix.reason)

    def close(self):
        self.closed = True
        return self.worker.close()
