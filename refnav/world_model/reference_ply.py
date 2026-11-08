"""Bounded PLY 1.0 point-cloud vertex reader, independent of ROS and the GPU."""

import hashlib
from pathlib import Path

import numpy as np

MAX_POINTS = 2_000_000
MAX_FILE_BYTES = 256 * 1024**2
_TYPES = dict(zip(
    "char uchar short ushort int uint float double".split(),
    "i1 u1 i2 u2 i4 u4 f4 f8".split(),
))
_TYPES.update(zip(
    "int8 uint8 int16 uint16 int32 uint32 float32 float64".split(),
    "i1 u1 i2 u2 i4 u4 f4 f8".split(),
))
_XYZ = ("x", "y", "z")
_RGB = ("red", "green", "blue")


def _line(stream, limit, digest):
    line = stream.readline(limit + 1)
    if not line or len(line) > limit or not line.endswith(b"\n"):
        raise ValueError("missing or oversized PLY line")
    digest.update(line)
    return line.decode("ascii").split()


def _header(stream, digest, max_points):
    if _line(stream, 32, digest) != ["ply"]:
        raise ValueError("expected PLY magic")
    fmt = _line(stream, 80, digest)
    orders = {"ascii": "<", "binary_little_endian": "<", "binary_big_endian": ">"}
    if len(fmt) != 3 or fmt[0] != "format" or fmt[1] not in orders or fmt[2] != "1.0":
        raise ValueError("unsupported PLY format")
    fields, names, comments = [], set(), []
    count, element = None, None
    for _ in range(128):
        row = _line(stream, 1024, digest)
        if row == ["end_header"]:
            break
        if row and row[0] in ("comment", "obj_info"):
            comments.append(" ".join(row))
        elif len(row) == 3 and row[0] == "element":
            element, size = row[1], int(row[2])
            if element == "vertex" and count is None and 0 < size <= max_points:
                count = size
            elif count is None or element == "vertex" or size != 0:
                raise ValueError("expected a bounded point cloud with only nonempty vertex data")
        elif len(row) >= 3 and row[0] == "property":
            if element is None:
                raise ValueError("PLY property has no element")
            if element != "vertex":
                continue  # Empty trailing element, e.g. a face declaration.
            if len(row) != 3 or row[1] not in _TYPES or row[2] in names or len(fields) == 32:
                raise ValueError("invalid PLY vertex property")
            fields.append((row[2], orders[fmt[1]] + _TYPES[row[1]]))
            names.add(row[2])
        else:
            raise ValueError("invalid PLY header")
    else:
        raise ValueError("oversized PLY header")
    if count is None or not set(_XYZ) <= names or (names.intersection(_RGB) and not set(_RGB) <= names):
        raise ValueError("PLY requires XYZ and either all or no RGB channels")
    dtype = np.dtype(fields)
    if set(_RGB) <= names and any(dtype[name].kind not in "iu" for name in _RGB):
        raise ValueError("PLY RGB must be integer channels in [0, 255]")
    return fmt[1], count, dtype, tuple(comments)


def _ascii_vertices(stream, count, dtype, digest):
    vertices = np.empty(count, dtype=dtype)
    columns = [(name, dtype[name], np.iinfo(dtype[name]) if dtype[name].kind in "iu" else None)
               for name in dtype.names]
    for i in range(count):
        tokens = _line(stream, 2048, digest)
        if len(tokens) != len(columns):
            raise ValueError("wrong PLY vertex column count")
        values = []
        for token, (_, kind, bounds) in zip(tokens, columns):
            if bounds is None:
                value = float(token)
                if np.isfinite(value) and abs(value) > np.finfo(kind).max:
                    raise ValueError("PLY coordinate exceeds declared scalar range")
            else:
                value = int(token)
                if not bounds.min <= value <= bounds.max:
                    raise ValueError("PLY integer exceeds declared scalar range")
            values.append(value)
        vertices[i] = tuple(values)
    return vertices


def read_ply(path, *, max_points=1_000_000):
    """Return raw XYZ, optional uint8 RGB, format, header comments and source hash."""
    if type(max_points) is not int or not 1 <= max_points <= MAX_POINTS:
        raise ValueError("invalid PLY vertex budget")
    path = Path(path)
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        size = path.stat().st_size
        if not 0 < size <= MAX_FILE_BYTES:
            raise ValueError("PLY exceeds file size budget or is empty")
        fmt, count, dtype, comments = _header(stream, digest, max_points)
        if fmt == "ascii":
            vertices = _ascii_vertices(stream, count, dtype, digest)
            tail = stream.read(4097)
            if len(tail) > 4096 or tail.strip():
                raise ValueError("unexpected data after PLY vertices")
            digest.update(tail)
        else:
            body_size = count * dtype.itemsize
            if size - stream.tell() != body_size:
                raise ValueError("wrong binary PLY body length")
            raw = stream.read(body_size)
            if len(raw) != body_size or stream.read(1):
                raise ValueError("binary PLY changed during read")
            digest.update(raw)
            vertices = np.frombuffer(raw, dtype=dtype)
    xyz = np.column_stack([vertices[name].astype(np.float64) for name in _XYZ])
    rgb = None
    if _RGB[0] in dtype.names:
        channels = [vertices[name] for name in _RGB]
        if any(ch.min() < 0 or ch.max() > 255 for ch in channels):
            raise ValueError("PLY RGB outside [0, 255]")
        rgb = np.column_stack([ch.astype(np.uint8) for ch in channels])
    return xyz, rgb, fmt, comments, digest.hexdigest()
