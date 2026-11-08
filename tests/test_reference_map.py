import hashlib
import struct
import tracemalloc

import numpy as np
import pytest

from examples.reference_map import diagnostics, write_svg
from refnav.world_model.reference_map import (
    PreprocessPolicy, ReferenceSpec, load_reference, preprocess, save_reference,
)
from refnav.world_model.reference_ply import read_ply


def _header(fmt, n, rgb=False):
    color = "property uchar red\nproperty uchar green\nproperty uchar blue\n" if rgb else ""
    return (f"ply\nformat {fmt} 1.0\ncomment surveyed source\nelement vertex {n}\n"
            "property float x\nproperty float y\nproperty float z\n"
            + color + "element face 0\nproperty list uchar int vertex_indices\nend_header\n").encode()


def _ascii(path, rows, rgb=False):
    path.write_bytes(_header("ascii", len(rows), rgb) +
                     "".join(" ".join(map(str, row)) + "\n" for row in rows).encode())
    return path


def _spec(**kwargs):
    data = dict(map_id="course", frame_id="course_frame", axes="right-handed XYZ, Z up",
                source_unit="m", meters_per_unit=1.)
    return ReferenceSpec(**(data | kwargs))


@pytest.mark.parametrize("fmt,order", [
    ("ascii", None), ("binary_little_endian", "<"), ("binary_big_endian", ">"),
])
def test_xyz_and_rgb_parsing_and_hash(tmp_path, fmt, order):
    rows = [(1, 2, 3, 100, 20, 30), (-2, 0, 4, 0, 255, 60)]
    path = tmp_path / "scan.ply"
    if order:
        path.write_bytes(_header(fmt, 2, True) + b"".join(struct.pack(order + "fffBBB", *r) for r in rows))
    else:
        _ascii(path, rows, True)
    xyz, rgb, encoding, comments, sha = read_ply(path)
    np.testing.assert_equal(xyz, np.array(rows)[:, :3])
    np.testing.assert_equal(rgb, np.array(rows)[:, 3:])
    assert encoding == fmt and comments == ("comment surveyed source",)
    assert sha == hashlib.sha256(path.read_bytes()).hexdigest()


@pytest.mark.parametrize("payload", [
    b"", b"not ply\n", _header("ascii", 0),
    _header("ascii", 1) + b"1 2\n", _header("ascii", 1) + b"1 2 x\n",
    _header("ascii", 1, True) + b"1 2 3 256 0 1\n",
    _header("ascii", 1) + b"1 2 3\n4 5 6\n",
    _header("ascii", 1).replace(b"property float x", b"property float w") + b"1 2 3\n",
    _header("binary_little_endian", 1) + b"\x00" * 11,
])
def test_malformed_ply_fails(tmp_path, payload):
    path = tmp_path / "bad.ply"
    path.write_bytes(payload)
    with pytest.raises(ValueError):
        read_ply(path)


def test_bounds_and_reviewed_scale(tmp_path):
    path = _ascii(tmp_path / "scan.ply", [(0, 0, 0), (2000, 0, 0)])
    with pytest.raises(ValueError, match="reviewed metric"):
        preprocess(path, _spec(scale_evidence="survey", expected_span_m=(1.9, 2.1)))
    metric = preprocess(path, _spec(source_unit="mm", meters_per_unit=.001,
                                   scale_evidence="survey", expected_span_m=(1.9, 2.1)))
    assert metric.spec.reviewed_scale
    assert metric.manifest["bounds_m"][1] == [2., 0., 0.]
    with pytest.raises(ValueError, match="vertex"):
        read_ply(path, max_points=1)


def test_deterministic_voxelization_bundle_and_bounded_crop(tmp_path):
    path = _ascii(tmp_path / "scan.ply", [(float("nan"), 0, 0, 4, 5, 6),
                                         (0, 0, 0, 1, 2, 3), (0, 0, 0, 7, 8, 9),
                                         (.03125, 0, 0, 4, 5, 6), (1, 0, 1, 7, 8, 9)], True)
    reference = preprocess(path, _spec())
    assert reference.manifest["counts"] == {
        "original": 5, "invalid_xyz": 1, "duplicate_xyz": 1,
        "after_voxel": 2, "outliers_removed": 0, "retained": 2,
    }
    assert reference.rgb.tolist() == [[1, 2, 3], [7, 8, 9]]
    assert not reference.points.flags.writeable
    assert reference.local_points((0, 0, 0), 3, 1) is None
    np.testing.assert_equal(reference.local_points((0, 0, 0), .1, 1), [[0, 0, 0]])
    assert diagnostics(reference)["projections"]["xz"]["sampled_bin_fraction"] == 2 / 1024
    for name in ("a", "b"):
        out = tmp_path / name
        save_reference(preprocess(path, _spec()), out)
        write_svg(reference, out / "projections.svg")
        assert load_reference(out).version == reference.version
    for first in (tmp_path / "a").iterdir():
        assert first.read_bytes() == (tmp_path / "b" / first.name).read_bytes()
    xyz = tmp_path / "a" / "xyz.npy"
    xyz.write_bytes(xyz.read_bytes()[:-1] + b"\x01")
    with pytest.raises(ValueError, match="digest"):
        load_reference(xyz.parent)


@pytest.mark.parametrize("method", ["radius", "statistical"])
def test_outlier_filters_preserve_rgb_correspondence(tmp_path, method):
    rows = [(i / 100., j / 100., 0, 0, 1, 2) for i in range(4) for j in range(4)]
    path = _ascii(tmp_path / "scan.ply", rows + [(5, 5, 5, 255, 0, 0)], True)
    ref = preprocess(path, _spec(), PreprocessPolicy(voxel_m=0, outlier_method=method, neighbors=3))
    assert ref.manifest["counts"]["outliers_removed"] == 1
    assert np.all(ref.rgb == [0, 1, 2])


def test_no_finite_points(tmp_path):
    path = _ascii(tmp_path / "scan.ply", [(float("inf"), 0, 0)])
    with pytest.raises(ValueError, match="finite"):
        preprocess(path, _spec())


def test_binary_preprocessing_allocation_budget(tmp_path):
    grid = np.indices((80, 80, 8)).reshape(3, -1).T.astype("<f4") * np.float32(.025)
    path = tmp_path / "scan.ply"
    path.write_bytes(_header("binary_little_endian", len(grid)) + grid.tobytes())
    tracemalloc.start()
    try:
        reference = preprocess(path, _spec(), PreprocessPolicy(voxel_m=.1, max_points=len(grid)))
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert reference.manifest["counts"]["original"] == len(grid)
    assert 0 < len(reference.points) < len(grid) // 4
    assert peak < 160 * len(grid)  # Python/NumPy-tracked allocations on this bounded fixture.
