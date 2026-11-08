"""Measure bounded ICP using fixed surface geometry and independent evaluation poses."""

import argparse
from collections import Counter
import json
from pathlib import Path
import platform
from tempfile import TemporaryDirectory

import numpy as np
import scipy
from scipy.spatial.transform import Rotation

from refnav.integration_timing import Timings, measure
from refnav.world_model.map_registration import SE3, register
from refnav.world_model.reference_map import PreprocessPolicy, ReferenceSpec, preprocess


def reference_for(directory, points, *, reviewed=True, name="scan"):
    points = np.asarray(points, dtype="<f4")
    path = Path(directory) / (name + ".ply")
    header = (f"ply\nformat binary_little_endian 1.0\nelement vertex {len(points)}\n"
              "property float x\nproperty float y\nproperty float z\nend_header\n")
    path.write_bytes(header.encode() + points.tobytes())
    spec = ReferenceSpec(name, "reference_scan", "right-handed XYZ, Z up", "m", 1.,
                         "fixture metric measurement" if reviewed else "", (.02, 50) if reviewed else None)
    return preprocess(path, spec, PreprocessPolicy(voxel_m=0))


def shape():
    """Three differently sized orthogonal faces with irregular metric sampling."""
    rng = np.random.default_rng(12)
    a = rng.uniform([0., -.7, -.45], [.6, .55, .60], size=(600, 3))
    b = rng.uniform([.02, 0., -.5], [.7, .55, .65], size=(600, 3))
    c = rng.uniform([.01, -.72, 0.], [.8, .54, .51], size=(600, 3))
    a[:, 0], b[:, 1], c[:, 2] = 1.1, -.63, -.4
    b[:, 0] += .4
    c[:, 0] += .35
    return np.concatenate((a, b, c))


def errors(actual, truth):
    translation = float(np.linalg.norm(np.subtract(actual.translation_m, truth.translation_m)))
    rotation = np.asarray(actual.rotation) @ np.asarray(truth.rotation).T
    return translation, float(np.degrees(Rotation.from_matrix(rotation).magnitude()))


def evaluate(directory, repeats):
    rng = np.random.default_rng(41)
    timings = Timings()
    points = shape()
    ref = reference_for(directory, points)
    wrong = reference_for(directory, points + (0., .48, -.46), name="wrong")
    plane = reference_for(directory, points[:600], name="plane")
    outcomes = Counter()
    fit_errors = []
    false_accepts = 0
    for i in range(repeats):
        # The evaluator generates a hidden truth, independently perturbs the
        # starting pose, and passes only the perturbed prior and measured points.
        r = rng.uniform(-.15, .15, 3)
        t = rng.uniform(-.3, .3, 3)
        truth = SE3(Rotation.from_rotvec(r).as_matrix(), t)
        initial = SE3(Rotation.from_rotvec(r + rng.uniform(-.04, .04, 3)).as_matrix(),
                      t + rng.uniform(-.05, .05, 3))
        source = truth.inverse().apply(points[::2]) + rng.normal(0., .003, (900, 3))
        kind = ("full", "partial", "wrong_map", "plane")[i % 4]
        if kind == "partial":
            source = source.copy()
            source[::6] = rng.uniform([.3, -.1, .9], [.5, .1, 1.1], size=(150, 3))
        if kind == "plane":
            source = source[:300]
        target = {"wrong_map": wrong, "plane": plane}.get(kind, ref)
        result = measure(timings, "local_registration", register, target, source, initial)
        outcomes[(kind, result.quality.reason)] += 1
        if result.reference_from_body is not None:
            error = errors(result.reference_from_body, truth)
            if kind in ("wrong_map", "plane") or error[0] > .05 or error[1] > 3.:
                false_accepts += 1
            else:
                fit_errors.append(error)
    fits = np.array(fit_errors)
    return {
        "host": {"platform": platform.platform(), "processor": platform.processor(),
                 "python": platform.python_version(), "numpy": np.__version__, "scipy": scipy.__version__},
        "fixture": "seeded analytic three-plane scan, partial dynamic returns, mismatched scan and single plane",
        "outcomes": {f"{kind}/{reason}": count for (kind, reason), count in sorted(outcomes.items())},
        "false_accepts": false_accepts, "accepted_in_tolerance": len(fits),
        "max_translation_error_m": float(fits[:, 0].max()) if len(fits) else None,
        "max_rotation_error_deg": float(fits[:, 1].max()) if len(fits) else None,
        "timing": timings.summary(),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeats", type=int, choices=range(4, 201), default=40, metavar="4..200")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists")
    with TemporaryDirectory() as directory:
        result = evaluate(directory, args.repeats)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return int(bool(result["false_accepts"]))


if __name__ == "__main__":
    raise SystemExit(main())
