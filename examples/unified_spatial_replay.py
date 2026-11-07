"""Generate Phase 2 diagnostic maps from deterministic fixtures or a recorded ROS 2 bag."""

import argparse
from dataclasses import asdict
from hashlib import sha256
import json
from pathlib import Path
import platform
from time import perf_counter

import numpy as np

from examples.semantic_world_model_demo import H, K, MOUNT, W, percentiles, render
from examples.spatial_bag_input import BagProfile, RecordedSpatialReplay, read_bag
from onboard.adapters.geometry import R_RIG_OPTICAL
from refnav.world_model.spatial_input import DepthCalibration, DepthCapture, PoseSample, SpatialReference
from refnav.world_model.spatial_pipeline import SpatialWorld
from refnav.world_model.spatial_volume import Cell

REFERENCE = SpatialReference("odom", "synthetic", 0)
BODY = (0., 0., 1.2)
REAR = [(0, 3.8, (-3., 3.), (-1., 3.))]
PROBE = (2.1, .10, 1.30)


def calibration(scale=1):
    fx, fy, cx, cy = K
    intrinsics = (fx * scale, fy * scale, (cx + .5) * scale - .5, (cy + .5) * scale - .5)
    return DepthCalibration("depth_optical", "base_link", W * scale, H * scale,
                            intrinsics, R_RIG_OPTICAL, MOUNT, .015, f"synthetic-rig-x{scale}",
                            validated_pixel_footprint=True,
                            extrinsic_position_error_m=0., extrinsic_angle_error_rad=0.)


def observe(world, image, stamp_ns):
    world.push_pose(PoseSample(REFERENCE, stamp_ns, stamp_ns, True, BODY, (1., 0., 0., 0.), .006, .003))
    return world.process(DepthCapture(REFERENCE, stamp_ns, stamp_ns, "depth_optical",
                                      f"frame-{stamp_ns}", image), now_ns=stamp_ns)


def inspect(snapshot, points, radius, margin=0.):
    return asdict(snapshot.metric.check_sweep(points, reference=snapshot.metric.reference,
                                             now_ns=snapshot.metric.query_ns, vehicle_radius_m=radius,
                                             margin_m=margin))


def describe(snapshot):
    grid = snapshot.metric
    cells = grid.cells_at(grid.query_ns, grid.reference)
    return {"reference": asdict(grid.reference), "query_ns": grid.query_ns, "capture_ns": grid.capture_ns,
            "observation_id": grid.observation_id, "calibration_id": grid.calibration_id,
            "available": grid.available, "origin_index": grid.origin_index, "resolution_m": grid.policy.resolution_m,
            "cell_counts": {value.name.lower(): int(np.count_nonzero(cells == value)) for value in Cell},
            "semantic_time_origin_ns": snapshot.semantic_time_origin_ns, "semantic": snapshot.semantic}


def synthetic():
    wall = [(0, 2.6, (-3., 3.), (-1., 3.))]
    ceiling = [(2, 1.6, (1.8, 2.7), (-.45, .45))]
    floor = [(2, 1.0, (1.8, 2.7), (-.45, .45))]
    narrow = [(0, 2.6, (-3., -.18), (-1., 3.)), (0, 2.6, (.18, 3.), (-1., 3.))]
    cases = (
        ("observed_corridor", REAR, (PROBE, (2.7, .10, 1.30)), .045, 0., "verified_free"),
        ("ceiling_intersection", ceiling + REAR, (PROBE,), .25, .08, "occupied"),
        ("floor_intersection", floor + REAR, (PROBE,), .25, .08, "occupied"),
        ("narrow_gap", narrow + REAR, ((2.3, 0., 1.3), (3.0, 0., 1.3)), .22, .05, "occupied"),
        ("behind_wall", wall + REAR, ((3.4, .10, 1.30),), .045, 0., "unknown"),
        ("camera_blind_near_body", REAR, ((.02, 0., 1.2),), .045, 0., "unknown"),
    )
    rows = {}
    rendered = None
    for label, rectangles, points, radius, margin, expected in cases:
        world = SpatialWorld(REFERENCE, calibration())
        updated = observe(world, render(rectangles), 100_000_000)
        if not updated.accepted:
            raise RuntimeError(f"{label}: {updated.reason}")
        snapshot = world.snapshot(now_ns=100_000_000)
        diagnostic = inspect(snapshot, points, radius, margin)
        if diagnostic["status"] != expected:
            raise RuntimeError(f"{label}: expected {expected}, received {diagnostic}")
        rows[label] = {"points_m": points, "vehicle_radius_m": radius, "margin_m": margin,
                       "diagnostic": diagnostic, **describe(snapshot)}
        if label == "ceiling_intersection":
            rendered = snapshot
    return {"evidence": "synthetic_depth_and_pose", "scenarios": rows}, rendered


def render_slices(snapshot, path):
    metric = snapshot.metric
    cells = metric.cells_at(metric.query_ns, metric.reference)
    colors = ("#243044", "#4dbca4", "#f18472")
    panels = (("XY slice", cells[:, :, cells.shape[2] // 2].T),
              ("XZ slice", cells[:, cells.shape[1] // 2, :].T))
    lines = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 740 420">',
             '<rect width="740" height="420" fill="#111826"/>',
             '<g fill="#eff3f8" font-family="sans-serif" font-size="15">',
             '<text x="24" y="30" font-size="20">Phase 2 local voxel evidence</text>']
    for i, (title, image) in enumerate(panels):
        left, top, size = 25 + 365 * i, 78, 7
        lines.append(f'<text x="{left}" y="60">{title} · {metric.policy.resolution_m:g} m per cell</text>')
        for row, values in enumerate(image[::-1]):
            starts = np.r_[0, np.flatnonzero(values[1:] != values[:-1]) + 1, len(values)]
            for a, b in zip(starts, starts[1:]):
                lines.append(f'<rect x="{left + a * size}" y="{top + row * size}" width="{(b - a) * size}" '
                             f'height="{size}" fill="{colors[values[a]]}"/>')
    for i, label in enumerate(("Unknown", "Observed free", "Occupied")):
        x = 25 + 175 * i
        lines.append(f'<rect x="{x}" y="378" width="14" height="14" fill="{colors[i]}"/>')
        lines.append(f'<text x="{x + 22}" y="390">{label}</text>')
    lines.extend(("</g>", "</svg>"))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def benchmark(repeats):
    stats = {}
    source = render(REAR)
    for scale in (1, 4):
        cam = calibration(scale)
        world = SpatialWorld(REFERENCE, cam)
        depth = np.repeat(np.repeat(source, scale, axis=0), scale, axis=1)
        elapsed = []
        for i in range(repeats + 3):
            start = perf_counter()
            result = observe(world, depth, (i + 1) * 66_666_667)
            if not result.accepted:
                raise RuntimeError(result.reason)
            if i >= 3:
                elapsed.append((perf_counter() - start) * 1000)
        stats[f"{cam.width}x{cam.height}"] = {"frames": repeats, **percentiles(elapsed),
                                             "over_15hz_period": int(np.count_nonzero(np.asarray(elapsed) > 1000 / 15))}
    return {"hardware": {"platform": platform.platform(), "processor": platform.processor(),
                         "python": platform.python_version(), "numpy": np.__version__},
            "measurement": "pose ingestion plus metric and semantic frame processing, synthetic offline host",
            "results": stats}


def _json(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("synthetic")
    demo.add_argument("--repeats", type=int, default=40, choices=range(5, 301), metavar="5..300")
    demo.add_argument("--skip-timing", action="store_true")
    bag = sub.add_parser("bag")
    bag.add_argument("bag", type=Path)
    bag.add_argument("--profile", type=Path, required=True)
    bag.add_argument("--sweep", type=Path, help="optional map-frame points and explicit envelope bounds")
    for command in (demo, bag):
        command.add_argument("--output", type=Path, required=True, help="new evidence directory")
    args = parser.parse_args(argv)
    try:
        if args.output.exists():
            raise ValueError("output directory already exists")
        timing = None
        if args.command == "synthetic":
            report, snapshot = synthetic()
            if not args.skip_timing:
                timing = benchmark(args.repeats)
        else:
            raw = args.profile.read_bytes()
            replay = RecordedSpatialReplay(BagProfile.from_dict(json.loads(raw)))
            snapshot = read_bag(args.bag, replay)
            report = {"evidence": "recorded_rosbag", "profile_sha256": sha256(raw).hexdigest(),
                      "accepted_frames": sum(row["accepted"] for row in replay.rows),
                      "frames": replay.rows, "snapshot": describe(snapshot)}
            if args.sweep:
                sweep = json.loads(args.sweep.read_text(encoding="utf-8"))
                report["sweep"] = inspect(snapshot, sweep["points_m"], sweep["vehicle_radius_m"], sweep["margin_m"])
        args.output.mkdir(parents=True)
        _json(args.output / "replay.json", report)
        render_slices(snapshot, args.output / "map.svg")
        if timing:
            _json(args.output / "benchmark.json", timing)
    except ImportError as error:
        parser.exit(2, f"ROS 2 Jazzy bag replay dependencies unavailable: {error}\n")
    except (OSError, ValueError, TypeError, KeyError, RuntimeError) as error:
        parser.exit(2, f"spatial replay failed: {error}\n")
    print(f"Wrote Phase 2 diagnostics to {args.output}")
    return 0 if args.command == "synthetic" or snapshot.metric.available else 1


if __name__ == "__main__":
    raise SystemExit(main())
