"""Offline evaluation only: synthetic fixtures, policy comparison and host timings.

Fixture generation and benchmark population are outside measured pipeline regions.
Runtime modules do not import this module or its sensor generator.
"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import platform
import sys
from time import perf_counter
import tracemalloc

from .baseline import DeterministicReasoner
from .context import ContextConfig, build_context
from .schema import MissionGoal, WorldReference, make_proposal
from .validation import validate_proposal


def evaluate_replay(frames=None):
    if frames is None:
        from examples.semantic_world_model_demo import replay
        frames = replay()[1]
    if len(frames) != 7:
        raise ValueError('evaluation requires all seven original replay snapshots')
    expected = ('insufficient_information', 'observe_more', 'observe_more', 'observe_more',
                'observe_more', 'observe_more', 'reject_blocked_candidate')
    rows = []
    reference, reasoner = WorldReference(), DeterministicReasoner()
    for frame, intent in zip(frames, expected):
        now = frame['snapshot']['timestamp']
        context = build_context(frame['snapshot'], now=now, reference=reference)
        proposal = reasoner.propose(context)
        validation = validate_proposal(proposal, context, now=now, reference=reference)
        candidates = [e for e in context.entities if e.semantic_type == 'opening']
        # Deliberately weak comparison: choose by width and omit all unknowns/status.
        # It emits only a non-executable proposal. It is NOT a navigation baseline.
        if candidates:
            chosen = max(candidates, key=lambda e: (e.geometry.width_m, e.entity_id))
            guess = make_proposal(context, 'investigate_candidate', chosen.entity_id,
                                  reasons=('candidate_passage_exists','candidate_requires_investigation'),
                                  sources=(chosen.entity_id,))
        else:
            guess = make_proposal(context, 'insufficient_information', reasons=('no_relevant_entities',),
                                  missing=('candidate_observation',))
        guess_validation = validate_proposal(guess, context, now=now, reference=reference)
        rows.append({'label': frame['label'], 'expected_intent': intent,
                     'selected_intent': proposal.intent, 'target_entity_id': proposal.target_entity_id,
                     'matches_expected': proposal.intent == intent, 'validated': validation.valid,
                     'width_only_intent': guess.intent, 'width_only_validated': guess_validation.valid,
                     'width_only_rejections': list(guess_validation.reason_codes) if not guess_validation.valid else []})
    return {'scenario_count': len(rows), 'expected_decisions_matched': sum(r['matches_expected'] for r in rows),
            'grounded_proposals_validated': sum(r['validated'] for r in rows),
            'width_only_proposals_rejected': sum(not r['width_only_validated'] for r in rows),
            'scope': 'Hand-written expected replay decisions and deliberately weak width-only comparison; '
                     'no navigation, learned model or physical flight benefit measured.', 'scenarios': rows}


def populated_world(count):
    """Public checkpoint fixture; fill a store with 1/32/128 synthetic semantic records.

    These records are a load fixture, not detections measured from 128 physical openings.
    Histories have eight sightings, matching the original P1 scaling workload.
    """
    from examples.semantic_world_model_demo import observation
    from refnav.world_model import WorldState
    world = WorldState()
    for i in range(8):
        world.process(observation(i * .1), i * .1)
    checkpoint = json.loads(world.to_json())
    opening = next(e for e in checkpoint['entities'] if e['semantic_type'] == 'opening')
    entities = []
    for i in range(count):
        entity = copy.deepcopy(opening)
        entity['entity_id'] = f'opening_{1000+i:04d}'
        entity['geometry']['center'][1] += i * 5.
        entity['relations'] = []
        entities.append(entity)
    checkpoint['entities'] = entities
    checkpoint['next_id'] = 1000 + count
    return WorldState.from_json(json.dumps(checkpoint))


def _stats(values):
    ordered = sorted(values)
    def quantile(q):
        location = (len(ordered) - 1) * q
        lo = int(location)
        hi = min(lo + 1, len(ordered) - 1)
        return ordered[lo] + (ordered[hi] - ordered[lo]) * (location - lo)
    return {'median_ms': quantile(.5), 'p95_ms': quantile(.95), 'worst_ms': max(ordered)}


def _measure(call, repeats):
    values = []
    call()  # warmup, excluded
    for _ in range(repeats):
        start = perf_counter()
        call()
        values.append((perf_counter() - start) * 1000.)
    return _stats(values)


def _size(value, seen=None):
    """Approximate recursive Python object bytes; excludes allocator overhead."""
    from dataclasses import fields, is_dataclass
    seen = set() if seen is None else seen
    if id(value) in seen:
        return 0
    seen.add(id(value))
    result = sys.getsizeof(value)
    if is_dataclass(value):
        result += sum(_size(getattr(value,f.name),seen) for f in fields(value))
    elif isinstance(value, (tuple,list)):
        result += sum(_size(v,seen) for v in value)
    elif isinstance(value, dict):
        result += sum(_size(k,seen)+_size(v,seen) for k,v in value.items())
    return result


def benchmark(repeats=40):
    from examples.semantic_world_model_demo import peak_resident_bytes
    from refnav.world_model import openings
    reference, reasoner, config = WorldReference(), DeterministicReasoner(), ContextConfig()
    rows = []
    for count in (1,32,128):
        world = populated_world(count)
        now = world.snapshot()['timestamp']
        compact = world.semantic_snapshot(now, max_entities=config.max_entities)
        context = build_context(world,now=now,reference=reference,config=config)
        proposal = reasoner.propose(context)
        assert context.context_valid and validate_proposal(proposal,context,now=now,reference=reference).valid
        def pipeline():
            current = build_context(world,now=now,reference=reference,config=config)
            return validate_proposal(reasoner.propose(current),current,now=now,reference=reference)
        measurements = {
            'world_snapshot': _measure(lambda: world.snapshot(now), repeats),
            'legacy_opening_query': _measure(lambda: openings(world,now), repeats),
            'bounded_semantic_read': _measure(lambda: world.semantic_snapshot(now,max_entities=config.max_entities), repeats),
            'context_from_preacquired_semantic_read': _measure(lambda: build_context(compact,now=now,reference=reference,config=config), repeats),
            'context_from_world_including_read': _measure(lambda: build_context(world,now=now,reference=reference,config=config), repeats),
            'reasoner': _measure(lambda: reasoner.propose(context), repeats),
            'independent_validation': _measure(lambda: validate_proposal(proposal,context,now=now,reference=reference), repeats),
            'total_pipeline': _measure(pipeline, repeats),
        }
        tracemalloc.start()
        pipeline()
        _, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        rows.append({'world_entities': count, 'semantic_read_entities': len(compact['entities']),
                     'context_entities': len(context.entities), 'omitted_entities': context.omitted_entities,
                     'serialized_context_bytes': len(context.to_json().encode()),
                     'estimated_context_python_bytes': _size(context), 'one_pipeline_peak_traced_bytes': peak,
                     'latencies': measurements,
                     'pipeline_to_legacy_query_median_ratio': measurements['total_pipeline']['median_ms'] /
                                                            measurements['legacy_opening_query']['median_ms']})
    root = Path(__file__).resolve().parents[2]
    sources = [*sorted((root / 'refnav' / 'reasoning').glob('*.py')),
               root / 'refnav' / 'world_model' / 'state.py', root / 'examples' / 'semantic_reasoning_demo.py']
    hashes = {p.relative_to(root).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    original = root / 'artifacts' / 'world_model' / 'benchmark.json'
    return {'provenance': {'os': platform.platform(), 'processor': platform.processor(), 'machine': platform.machine(),
                          'python': platform.python_version(), 'hardware_class': 'available host; not Jetson',
                          'clock': 'perf_counter wall latency; replay seconds in contracts',
                          'repeats': repeats, 'source_sha256': hashes,
                          'original_p1_benchmark_sha256': hashlib.sha256(original.read_bytes()).hexdigest()},
            'configuration': {'max_entities':config.max_entities,'max_context_bytes':config.max_context_bytes,
                              'evidence_per_entity':config.evidence_limit},
            'scope': 'Independent timings; fixture generation excluded. Eight-entry histories; '
                     'new pipeline bounds entities/bytes and keeps two evidence entries. '
                     'It is not a full-graph replacement; included and omitted counts are reported.',
            'counts': rows, 'process_peak_resident_bytes': peak_resident_bytes()}
