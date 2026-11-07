"""Reproducible offline sensor fixture, replay, evaluation and host-only benchmark.

Run from the repo root: python -m examples.semantic_world_model_demo --output artifacts/world_model
Synthetic scene truth is used ONLY by this renderer and evaluation; WorldState sees depth/pose.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import copy
import ctypes
import hashlib
import json
from pathlib import Path
import platform
import sys
from time import perf_counter
import tracemalloc

import numpy as np

from onboard.adapters.geometry import R_RIG_OPTICAL, rot_z
from refnav.world_model import Config, Observation, WorldState, openings
from refnav.world_model.geometry import extract

W, H, FX = 160, 100, 106.
K = (FX, FX, W / 2, H / 2)
MOUNT = np.array([0.16, 0., 0.])


def scene(blocked=False, width=1.2, rear=True):
    """Rectangles: (normal axis, coordinate, other-axis bounds in increasing axis order)."""
    walls = [(0, 4., (-3., 3.), (-1., 3.))] if blocked else [
        (0, 4., (-3., -width / 2), (-1., 3.)), (0, 4., (width / 2, 3.), (-1., 3.))]
    return walls + ([(0, 7., (-5., 5.), (-1., 4.))] if rear else [])


def render(rectangles, position=(0., 0., 1.2), rotation=None):
    """Pinhole optical-z depth with finite rectangle intersections; missing returns are NaN."""
    rotation = np.eye(3) if rotation is None else rotation
    v, u = np.mgrid[0:H, 0:W]
    optical = np.stack(((u - K[2]) / K[0], (v - K[3]) / K[1], np.ones_like(u)), axis=-1)
    directions = optical @ (rotation @ R_RIG_OPTICAL).T
    origin = np.asarray(position) + rotation @ MOUNT
    best = np.full((H, W), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        for axis, coordinate, bounds1, bounds2 in rectangles:
            t = (coordinate - origin[axis]) / directions[..., axis]
            point = origin + t[..., None] * directions
            other = [i for i in range(3) if i != axis]
            hit = ((t > 0) & (point[..., other[0]] >= bounds1[0]) & (point[..., other[0]] <= bounds1[1])
                   & (point[..., other[1]] >= bounds2[0]) & (point[..., other[1]] <= bounds2[1]))
            best = np.where(hit, np.minimum(t, best), best)
    best[~np.isfinite(best)] = np.nan
    return best.astype(np.float32)


def observation(stamp=0., position=(0., 0., 1.2), yaw=0., rectangles=None, depth=None, **kwargs):
    rotation = rot_z(yaw)
    if depth is None:
        depth = render(scene() if rectangles is None else rectangles, position, rotation)
    return Observation(depth, K, R_RIG_OPTICAL.copy(), MOUNT.copy(), rotation, np.array(position),
                       stamp, stamp, f'frame_{stamp:.6f}', source='synthetic_depth', **kwargs)


def sequence():
    partial = render(scene())
    partial[:, W // 2:] = np.nan
    return [('partial_wall', observation(0., depth=partial)),
            ('opening_revealed', observation(0.2)),
            ('translated_and_turned', observation(0.4, position=(0.25, 0.15, 1.2), yaw=0.12)),
            ('out_of_view', observation(0.6, yaw=np.pi)),
            ('reobserved_after_staleness', observation(7.2, position=(0.2, 0.1, 1.2), yaw=0.08)),
            ('passage_blocked', observation(7.4, rectangles=scene(blocked=True)))]


def replay():
    state = WorldState()
    frames = []
    for label, obs in sequence():
        if label == 'reobserved_after_staleness':
            state.tick(6.5)
            frames.append({'label': 'stale_memory', 'snapshot': state.snapshot(), 'openings': openings(state)})
        report = state.process(obs, obs.stamp)
        # Timings belong in benchmark output, not deterministic semantic replay.
        frames.append({'label': label, 'accepted': report['accepted'], 'rejection': report['rejection'],
                       'events': report['events'], 'detections': report['detections'],
                       'diagnostics': report.get('diagnostics'), 'snapshot': state.snapshot(),
                       'openings': openings(state)})
    return state, frames


def percentiles(values):
    return dict(zip(('median_ms', 'p95_ms', 'worst_ms'), map(float, np.percentile(values, [50, 95, 100]))))


def deep_size(value, seen=None):
    """Recursive Python object estimate; shared objects counted once, excludes allocator overhead."""
    seen = set() if seen is None else seen
    if id(value) in seen:
        return 0
    seen.add(id(value))
    size = sys.getsizeof(value)
    if isinstance(value, dict):
        size += sum(deep_size(k, seen) + deep_size(v, seen) for k,v in value.items())
    elif isinstance(value, (list,tuple)):
        size += sum(deep_size(v, seen) for v in value)
    elif hasattr(value, '__dict__'):
        size += deep_size(vars(value), seen)
    return size


def peak_resident_bytes():
    if platform.system() == 'Windows':
        from ctypes import wintypes

        class Counters(ctypes.Structure):
            _fields_ = [('cb', wintypes.DWORD), ('PageFaultCount', wintypes.DWORD)] + [
                (name, ctypes.c_size_t) for name in ('PeakWorkingSetSize', 'WorkingSetSize',
                    'QuotaPeakPagedPoolUsage', 'QuotaPagedPoolUsage', 'QuotaPeakNonPagedPoolUsage',
                    'QuotaNonPagedPoolUsage', 'PagefileUsage', 'PeakPagefileUsage')]
        counters = Counters()
        counters.cb = ctypes.sizeof(counters)
        kernel = ctypes.windll.kernel32
        kernel.GetCurrentProcess.restype = wintypes.HANDLE
        psapi = ctypes.windll.psapi
        psapi.GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.POINTER(Counters), wintypes.DWORD]
        if psapi.GetProcessMemoryInfo(kernel.GetCurrentProcess(), ctypes.byref(counters), counters.cb):
            return counters.PeakWorkingSetSize
        return None
    import resource
    value = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(value if platform.system() == 'Darwin' else value * 1024)


def benchmark(repeats=40):
    state = WorldState()
    obs = observation()
    # Warm numerical libraries, then profile steady processing separately from traced allocation.
    extract(obs, state.config)
    geometry, update, query = [], [], []
    for i in range(repeats):
        stamp = i * 0.1
        current = replace(obs, stamp=stamp, pose_stamp=stamp, observation_id=f'benchmark_{i}')
        report = state.process(current, stamp)
        assert report['accepted']
        geometry.append(report['geometry_ms'])
        update.append(report['update_ms'])
        start = perf_counter()
        openings(state)
        query.append((perf_counter() - start) * 1000)
    footprint = len(state.to_json().encode())
    beam_bytes = sum(r.origin.nbytes + r.directions.nbytes + r.depths.nbytes + r.valid.nbytes for _, r in state._beams)
    tracemalloc.start()
    allocation_state, _ = replay()
    _, peak = tracemalloc.get_traced_memory()
    tracemalloc.stop()
    scaling = []
    template = next(e for e in state._entities.values() if e.semantic_type == 'opening')
    for count in (1, 32, 128):
        owner = WorldState()
        # Instrument a bounded, already populated store; synthetic entity placement is benchmark setup only.
        for i in range(count):
            entity = copy.deepcopy(template)
            entity.entity_id = f'opening_{1000 + i:04d}'
            entity.geometry['center'][1] += i * 5
            entity.last_observed = 0.
            owner._entities[entity.entity_id] = entity
        owner._next_id = 2000
        times, updates = [], []
        for i in range(8):
            stamp = i * 0.1
            current = replace(obs, stamp=stamp, pose_stamp=stamp, observation_id=f'scale_{i}')
            report = owner.process(current, stamp)
            updates.append(report['update_ms'])
            start = perf_counter()
            openings(owner)
            times.append((perf_counter() - start) * 1000)
        scaling.append({'initial_entities': count, 'final_entities': len(owner._entities),
                        'query': percentiles(times), 'update_including_association': percentiles(updates)})
    root = Path(__file__).resolve().parents[1]
    sources = [Path(__file__).resolve(), *sorted((root / 'refnav' / 'world_model').glob('*.py'))]
    hashes = {path.relative_to(root).as_posix(): hashlib.sha256(path.read_bytes()).hexdigest() for path in sources}
    return {'provenance': {'os': platform.platform(), 'machine': platform.machine(),
                          'processor': platform.processor(), 'python': platform.python_version(),
                          'numpy': np.__version__, 'hardware_class': 'available Windows host; not Jetson',
                          'sensor': '160x100 ideal synthetic optical-z; stride >=2',
                          'repeats': repeats, 'clock': 'perf_counter wall latency; replay seconds for data',
                          'source_sha256': hashes},
            'geometry': percentiles(geometry), 'semantic_update': percentiles(update), 'query': percentiles(query),
            'checkpoint_utf8_bytes': footprint, 'beam_array_bytes': beam_bytes,
            'estimated_entity_store_python_bytes': deep_size(state._entities),
            'replay_peak_traced_python_bytes': peak, 'process_peak_resident_bytes': peak_resident_bytes(),
            'scaling': scaling, 'evaluation': evaluate()}


def evaluate():
    state, frames = replay()
    opening_frames = [f for f in frames if f['label'] in ('opening_revealed', 'translated_and_turned',
                                                         'reobserved_after_staleness')]
    candidates = [f['openings'][0] for f in opening_frames]
    false_count = 0
    cases = [observation(rectangles=scene(blocked=True)), observation(rectangles=scene(rear=False)),
             observation(depth=np.full((H, W), np.nan)), observation(depth=np.full((H, W), np.inf))]
    for obs in cases:
        world = WorldState()
        world.process(obs, obs.stamp)
        false_count += len(openings(world))
    return {'fixture_truth': 'opening center horizontal (4,0), width 1.2m; height intentionally not certified',
            'opening_location_xy_error_m': [float(np.linalg.norm(np.array(c['geometry']['center'][:2]) - (4., 0.)))
                                             for c in candidates],
            'width_absolute_error_m': [abs(c['width_m'] - 1.2) for c in candidates],
            'height_error_m': None, 'false_opening_count_four_negative_cases': false_count,
            'duplicate_openings_on_reobservation': len(set(c['entity_id'] for c in candidates)) - 1,
            'false_confirmed_passages': sum(c['status'] == 'confirmed' for f in frames for c in f['openings']),
            'occlusion_retains_id': frames[3]['openings'][0]['entity_id'] == candidates[0]['entity_id'],
            'stale_status': frames[4]['openings'][0]['status'],
            'final_opening_status': openings(state)[0]['status'],
            'final_traversability': openings(state)[0]['traversability'],
            'semantic_types_detected': sorted(set(e['semantic_type'] for e in state.snapshot()['entities'])),
            'evidence_scope': 'ideal fixture cases only; no real sensor accuracy or navigation improvement measured'}


def svg(frames):
    elements = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="340" viewBox="0 0 1000 340">',
                '<rect width="1000" height="340" fill="#101827"/>',
                '<text x="20" y="28" fill="white" font-family="sans-serif" font-size="18">Offline world memory: top-down x/y, metres (opening height unknown)</text>']
    for i, frame in enumerate(frames):
        x0 = 20 + i * 140
        elements.append(f'<text x="{x0}" y="60" fill="#cbd5e1" font-size="11">{i + 1}: {frame["label"][:19]}</text>')
        for entity in frame['snapshot']['entities']:
            g = entity['geometry']
            normal = np.array(g['normal'])
            tangent = np.array([-normal[1], normal[0], 0.])
            a, b = np.array(g['center']) - tangent * g['width_m'] / 2, np.array(g['center']) + tangent * g['width_m'] / 2
            color = '#94a3b8' if entity['semantic_type'] == 'wall' else '#38bdf8'
            if entity['status'] == 'invalidated':
                color = '#f87171'
            elements.append(f'<line x1="{x0 + 65 + a[1] * 17}" y1="{285 - a[0] * 25}" '
                            f'x2="{x0 + 65 + b[1] * 17}" y2="{285 - b[0] * 25}" stroke="{color}" stroke-width="4"/>')
        for j, candidate in enumerate(frame['openings']):
            elements.append(f'<text x="{x0}" y="{310 + j * 12}" fill="#e2e8f0" font-size="10">'
                            f'{candidate["entity_id"]}: {candidate["status"]}</text>')
    return ''.join(elements) + '</svg>'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('artifacts/world_model'))
    parser.add_argument('--repeats', type=int, default=40)
    args = parser.parse_args()
    if args.repeats < 2:
        parser.error('--repeats must be >=2')
    args.output.mkdir(parents=True, exist_ok=True)
    state, frames = replay()
    for frame in frames:
        print(json.dumps({k: frame[k] for k in ('label', 'events', 'openings') if k in frame}))
    for name, doc in [('replay.json', frames), ('snapshot.json', state.snapshot()), ('benchmark.json', benchmark(args.repeats))]:
        (args.output / name).write_text(json.dumps(doc, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    (args.output / 'checkpoint.json').write_text(state.to_json() + '\n', encoding='utf-8')
    (args.output / 'top_down.svg').write_text(svg(frames), encoding='utf-8')
    print(f'Wrote replay, snapshot, restart checkpoint, benchmark and top_down.svg to {args.output}')


if __name__ == '__main__':
    main()
