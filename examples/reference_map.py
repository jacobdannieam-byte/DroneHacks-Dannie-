"""Preprocess a supplied point cloud and write an offline diagnostic bundle."""

import argparse
from html import escape
import json
from pathlib import Path
from time import perf_counter
import tracemalloc

import numpy as np

from refnav.world_model.reference_map import PreprocessPolicy, ReferenceSpec, preprocess, save_reference


def diagnostics(reference):
    """Coarse projected sample coverage; empty bins have no free-space meaning."""
    low, high = np.array(reference.manifest["bounds_m"])
    extents = high - low
    unit = (reference.points - low) / np.where(extents > 0, extents, 1)
    bins = np.minimum((unit * 32).astype(np.int32), 31)
    projections = {}
    for name, a, b in (("xy", 0, 1), ("xz", 0, 2), ("yz", 1, 2)):
        counts = np.bincount(bins[:, a] * 32 + bins[:, b], minlength=1024)
        area = float(extents[a] * extents[b])
        projections[name] = {
            "sampled_bin_fraction": float(np.count_nonzero(counts) / len(counts)),
            "points_per_projected_bbox_m2": len(bins) / area if area else None,
            "singleton_bins": int(np.count_nonzero(counts == 1)),
        }
    return {
        "version": reference.version,
        "extent_m": extents.tolist(),
        "array_bytes": reference.points.nbytes + (0 if reference.rgb is None else reference.rgb.nbytes),
        "projected_coverage_grid": [32, 32],
        "projections": projections,
        "inspection": "Review sparse regions, isolated returns and abrupt edges in the SVG. "
                      "Sparse or empty bins can reflect occlusion or scan coverage. "
                      "Defects and free space require independent evidence.",
    }


def write_svg(reference, path):
    low, high = np.array(reference.manifest["bounds_m"])
    stride = max(1, (len(reference.points) + 3999) // 4000)
    points = reference.points[::stride]
    colors = reference.rgb
    extent = high - low
    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1010" height="375" viewBox="0 0 1010 375">',
        '<rect width="1010" height="375" fill="#101820"/>',
        '<g fill="#e9f0f5" font-family="sans-serif">',
        f'<text x="20" y="26" font-size="18">{escape(reference.spec.map_id)} — reference surface samples</text>',
        f'<text x="20" y="49" font-size="13">{escape(reference.spec.frame_id)} · meters · '
        f'{len(points):,} of {len(reference.points):,} retained points · empty areas unverified</text>',
    ]
    for i, (label, a, b) in enumerate((("XY", 0, 1), ("XZ", 0, 2), ("YZ", 1, 2))):
        left = 20 + i * 330
        scale = 246 / max(extent[a], extent[b], .001)
        lines.append(f'<rect x="{left}" y="65" width="306" height="268" fill="#162632" stroke="#486274"/>')
        lines.append(f'<text x="{left}" y="355" font-size="13">{label} · {extent[a]:.2f} × {extent[b]:.2f} m</text>')
        for j, point in enumerate(points):
            x = left + 12 + (point[a] - low[a]) * scale
            y = 322 - (point[b] - low[b]) * scale
            color = "#75c9ed" if colors is None else "#%02x%02x%02x" % tuple(colors[j * stride])
            lines.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="1.15" fill="{color}"/>')
    lines.append("</g></svg>")
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ply", type=Path)
    parser.add_argument("output", type=Path, help="new directory for arrays, manifest, report and SVG")
    for name in ("map-id", "frame-id", "axes", "source-unit"):
        parser.add_argument("--" + name, required=True)
    parser.add_argument("--meters-per-unit", type=float, required=True)
    parser.add_argument("--scale-evidence", default="")
    parser.add_argument("--expected-span-m", type=float, nargs=2)
    parser.add_argument("--voxel-m", type=float, default=.05)
    parser.add_argument("--max-points", type=int, default=1_000_000)
    parser.add_argument("--outlier", choices=("none", "radius", "statistical"), default="none")
    parser.add_argument("--neighbors", type=int, default=8)
    parser.add_argument("--radius-m", type=float, default=.20)
    parser.add_argument("--stddev-multiplier", type=float, default=2.5)
    parser.add_argument("--trace-memory", action="store_true", help="instrument allocations; timings include tracing overhead")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output must be a new directory")
    try:
        spec = ReferenceSpec(args.map_id, args.frame_id, args.axes, args.source_unit, args.meters_per_unit,
                             args.scale_evidence, args.expected_span_m)
        policy = PreprocessPolicy(args.voxel_m, args.max_points, args.outlier, args.neighbors,
                                  args.radius_m, args.stddev_multiplier)
        if args.trace_memory:
            tracemalloc.start()
        start = perf_counter()
        reference = preprocess(args.ply, spec, policy)
        elapsed = perf_counter() - start
        traced_peak = tracemalloc.get_traced_memory()[1] if args.trace_memory else None
        if args.trace_memory:
            tracemalloc.stop()
        report = {"manifest": reference.manifest, **diagnostics(reference),
                  "preprocess_seconds": elapsed, "traced_peak_bytes": traced_peak}
        save_reference(reference, args.output)
        write_svg(reference, args.output / "projections.svg")
        (args.output / "diagnostics.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    except (OSError, ValueError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    main()
