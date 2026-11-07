"""Single-owner bounded world memory. No ROS, vehicle commands, or wall-clock reads."""
from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass, field
import json
import math
from threading import RLock
from time import perf_counter

import numpy as np

from .geometry import extract, window_evidence
from .observations import Config, Rays, validate


@dataclass
class Entity:
    entity_id: str
    semantic_type: str
    geometry: dict
    frame_id: str
    epoch: int
    source_timestamp: float
    last_observed: float
    geometric_confidence: float
    semantic_confidence: float
    status: str = 'provisional'
    observations: int = 1
    contradictions: int = 0
    evidence: list = field(default_factory=list)
    missing_evidence: list = field(default_factory=list)
    relations: list = field(default_factory=list)
    connected_regions: list = field(default_factory=list)


class WorldState:
    """Serialized writes and detached snapshots; retain semantic memory beyond local beams.

    One calibrated depth stream per instance. Call reset_reference explicitly after
    localization/clock resets. Unannounced frame/epoch changes latch a reset requirement.
    """

    def __init__(self, config=None, frame_id='odom', epoch=0, clock_id='replay'):
        self.config = config or Config()
        self.frame_id, self.epoch, self.clock_id = frame_id, epoch, clock_id
        self._entities = {}
        self._beams = deque(maxlen=self.config.beam_frames)
        self._next_id = 1
        self._now = None
        self._last_capture = None
        self._last_position = None
        self._last_observation_id = None
        self._reset_required = False
        self._lock = RLock()

    def reset_reference(self, frame_id, epoch, clock_id=None):
        """Discard incompatible geometry. Export a snapshot first if an archive is wanted."""
        if not frame_id or epoch <= self.epoch:
            raise ValueError('reset needs a named reference and a strictly newer epoch')
        with self._lock:
            self.frame_id, self.epoch = frame_id, epoch
            self.clock_id = clock_id or self.clock_id
            self._entities.clear()
            self._beams.clear()
            self._now = self._last_capture = self._last_position = None
            self._last_observation_id = None
            self._reset_required = False
            # Never recycle IDs within this owner, even across resets.

    def _advance(self, now):
        if not math.isfinite(now) or (self._now is not None and now < self._now):
            raise ValueError('nonfinite/backward clock: reset the reference explicitly')
        self._now = float(now)
        events = []
        for key, entity in list(self._entities.items()):
            if now - entity.last_observed > self.config.forget_s:
                del self._entities[key]
                events.append({'event': 'forgotten', 'entity_id': key})
        while self._beams and now - self._beams[0][0] > self.config.beam_age_s:
            self._beams.popleft()
        self._clean_relations()
        return events

    def tick(self, now):
        """Age memory even when no frame arrives; absence never contradicts an entity."""
        with self._lock:
            return self._advance(now)

    def _clean_relations(self):
        for entity in self._entities.values():
            entity.relations = [r for r in entity.relations if r['target'] in self._entities]

    def process(self, obs, now):
        with self._lock:
            result = {'accepted': False, 'rejection': None, 'events': [], 'detections': [],
                      'geometry_ms': 0., 'update_ms': 0.}
            reason = validate(obs, now, self.config)
            if reason:
                # Arrival time still ages old beliefs when a current-domain sensor packet is bad.
                if (math.isfinite(now) and (self._now is None or now >= self._now)
                        and (obs.frame_id, obs.epoch, obs.clock_id) == (self.frame_id, self.epoch, self.clock_id)):
                    result['events'] = self._advance(now)
                result['rejection'] = reason
                return result
            if self._now is not None and now < self._now:
                self._reset_required = True
            if (obs.frame_id, obs.epoch, obs.clock_id) != (self.frame_id, self.epoch, self.clock_id):
                self._reset_required = True
            if self._reset_required:
                result['rejection'] = 'reference_reset_required'
                return result
            if self._last_capture is not None and (obs.stamp <= self._last_capture
                                                   or obs.observation_id == self._last_observation_id):
                result['events'] = self._advance(now)
                result['rejection'] = 'duplicate_or_out_of_order'
                return result
            if self._last_position is not None:
                limit = self.config.jump_m + self.config.max_speed_mps * (obs.stamp - self._last_capture)
                if np.linalg.norm(np.asarray(obs.t_world_body) - self._last_position) > limit:
                    self._reset_required = True
                    result['rejection'] = 'pose_jump_reset_required'
                    return result
            start = perf_counter()
            geometry = extract(obs, self.config)
            result['geometry_ms'] = (perf_counter() - start) * 1000
            start = perf_counter()
            result['events'] = self._advance(now)
            self._last_capture, self._last_position = obs.stamp, np.array(obs.t_world_body, copy=True)
            self._last_observation_id = obs.observation_id
            self._beams.append((obs.stamp, geometry.rays))
            detections = geometry.detections
            # Invalidate a remembered opening only from current, observed blocking rays.
            # A turn-away, invalid return, or occluder outside its projection is no evidence.
            for entity in self._entities.values():
                if entity.semantic_type != 'opening':
                    continue
                g = entity.geometry
                evidence = window_evidence(geometry.rays, g['center'], g['normal'],
                                           max(0.05, g['width_m'] - 2 * self.config.bin_m), self.config.band_m)
                if evidence['rays'] >= 12 and evidence['blocked_fraction'] >= 0.6:
                    entity.status = 'invalidated'
                    entity.contradictions += 1
                    entity.geometric_confidence *= 0.4
                    entity.semantic_confidence *= 0.4
                    entity.last_observed = obs.stamp
                    self._record(entity, obs, 'blocking_evidence', evidence)
                    result['events'].append({'event': 'invalidated', 'entity_id': entity.entity_id})
            # Global nearest pairs, one entity per detection and one detection per entity.
            pairs = []
            for i, detection in enumerate(detections):
                for key, entity in self._entities.items():
                    if entity.semantic_type != detection.semantic_type:
                        continue
                    g = entity.geometry
                    delta = np.asarray(g['center']) - detection.center
                    distance = float(np.linalg.norm(delta))
                    aligned = abs(np.dot(g['normal'], detection.normal)) > 0.96
                    width_ok = abs(g['width_m'] - detection.width_m) < 0.6
                    # Do not associate disjoint horizontal opening intervals.
                    tangent = np.array([-detection.normal[1], detection.normal[0], 0.])
                    overlap = abs(delta @ tangent) < 0.5 * min(g['width_m'], detection.width_m)
                    if distance < self.config.association_m and aligned and (detection.semantic_type != 'opening'
                                                                                  or (width_ok and overlap)):
                        pairs.append((distance, i, key))
            matches, used = {}, set()
            for _, i, key in sorted(pairs):
                if i not in matches and key not in used:
                    matches[i] = key
                    used.add(key)
            protected = set(matches.values())
            for i, detection in enumerate(detections):
                # Use latest supported geometry, rather than accumulating overconfident averages
                # of correlated stereo frames. Preserve the original first source timestamp.
                g = {name: getattr(detection, name) for name in
                     ('center', 'normal', 'width_m', 'height_m', 'uncertainty_m')}
                if i in matches:
                    entity = self._entities[matches[i]]
                    entity.observations += 1
                    event = 'updated'
                else:
                    if len(self._entities) >= self.config.max_entities:
                        eligible = [e for e in self._entities.values() if e.entity_id not in protected]
                        if not eligible:
                            result['events'].append({'event': 'capacity_rejected', 'semantic_type': detection.semantic_type})
                            continue
                        victim = min(eligible, key=lambda e: (e.last_observed, e.entity_id))
                        del self._entities[victim.entity_id]
                        result['events'].append({'event': 'evicted', 'entity_id': victim.entity_id})
                    key = f'{detection.semantic_type}_{self._next_id:04d}'
                    self._next_id += 1
                    entity = Entity(key, detection.semantic_type, g, self.frame_id, self.epoch,
                                    obs.stamp, obs.stamp, 0., 0.)
                    self._entities[key] = entity
                    event = 'created'
                protected.add(entity.entity_id)
                entity.geometry = g
                entity.last_observed = obs.stamp
                penalty = max(0.2, 1 - detection.uncertainty_m / self.config.max_pose_error_m)
                entity.geometric_confidence = detection.geometric_confidence * penalty
                entity.semantic_confidence = detection.semantic_confidence * penalty
                entity.missing_evidence = list(detection.missing_evidence)
                entity.status = ('confirmed' if entity.semantic_type == 'wall' and entity.observations >= 3
                                 and detection.uncertainty_m < 0.15 else 'provisional')
                self._record(entity, obs, 'support', detection.support)
                result['events'].append({'event': event, 'entity_id': entity.entity_id})
            self._clean_relations()
            for entity in self._entities.values():
                if entity.semantic_type != 'opening' or entity.last_observed != obs.stamp or entity.status == 'invalidated':
                    continue
                g = entity.geometry
                normal = np.array(g['normal'])
                tangent = np.array([-normal[1], normal[0], 0.])
                sides = {}
                for wall in self._entities.values():
                    if wall.semantic_type != 'wall' or wall.last_observed != obs.stamp:
                        continue
                    delta = np.array(wall.geometry['center']) - g['center']
                    across = float(delta @ tangent)
                    edge_error = abs(abs(across) - wall.geometry['width_m']/2 - g['width_m']/2)
                    if abs(delta @ normal) < self.config.plane_tol_m * 2 and edge_error < self.config.bin_m * 2:
                        side = 'left' if across < 0 else 'right'
                        if side not in sides or edge_error < sides[side][0]:
                            sides[side] = (edge_error, wall.entity_id)
                entity.relations = [{'type': 'bounded_by', 'target': key, 'status': 'observed_geometry'}
                                    for _, key in sides.values()] if len(sides) == 2 else []
            result.update(accepted=True, detections=[asdict(d) for d in detections],
                          diagnostics=geometry.diagnostics, update_ms=(perf_counter() - start) * 1000)
            return result

    def _record(self, entity, obs, kind, support):
        entity.evidence.append({'observation_id': obs.observation_id, 'source': obs.source,
                                'stamp': obs.stamp, 'pose_stamp': obs.pose_stamp,
                                'position_sigma_m': obs.position_sigma_m,
                                'angle_sigma_rad': obs.angle_sigma_rad, 'kind': kind, 'support': support})
        entity.evidence = entity.evidence[-self.config.max_evidence:]

    def space_at(self, point, now):
        """Diagnostic sampled-ray evidence, NOT a collision map or volume certificate.

        An endpoint supports occupied; the line segment strictly before it supports
        observed_free. No return / out of range / off sampled rays is unknown.
        Old occupied evidence takes precedence until it expires.
        """
        point = np.asarray(point, dtype=float)
        if point.shape != (3,) or not np.isfinite(point).all():
            raise ValueError('point must be finite xyz')
        with self._lock:
            if not math.isfinite(now) or (self._now is not None and now < self._now):
                raise ValueError('invalid query time')
            if self._reset_required:
                return 'unknown'
            free = False
            for stamp, rays in self._beams:
                if now - stamp > self.config.beam_age_s:
                    continue
                endpoints = rays.points()
                if len(endpoints) and np.any(np.linalg.norm(endpoints - point, axis=1) < 0.07):
                    return 'occupied'
                dirs = rays.directions[rays.valid]
                depth = rays.depths[rays.valid]
                lengths = np.linalg.norm(dirs, axis=1)
                unit = dirs / lengths[:, None]
                delta = point - rays.origin
                along = unit @ delta
                perpendicular = np.linalg.norm(delta - unit * along[:, None], axis=1)
                free |= bool(np.any((along > 0) & (along < depth * lengths - 0.1) & (perpendicular < 0.02)))
            return 'observed_free' if free else 'unknown'

    def snapshot(self, now=None):
        with self._lock:
            now = self._now if now is None else now
            if now is not None and (not math.isfinite(now) or (self._now is not None and now < self._now)):
                raise ValueError('invalid snapshot time')
            entities = []
            for entity in sorted(self._entities.values(), key=lambda e: e.entity_id):
                age = 0. if now is None else max(0., now - entity.last_observed)
                if age > self.config.forget_s:
                    continue
                doc = asdict(entity)
                doc['age_s'] = age
                decay = math.exp(-age / self.config.stale_s)
                doc['geometric_confidence'] *= decay
                doc['semantic_confidence'] *= decay
                if self._reset_required:
                    doc['status'] = 'uncertain'
                elif age > self.config.stale_s and doc['status'] != 'invalidated':
                    doc['status'] = 'stale'
                entities.append(doc)
            present = {doc['entity_id'] for doc in entities}
            for doc in entities:
                doc['relations'] = [r for r in doc['relations'] if r['target'] in present]
            # Round trip makes the snapshot detached and enforces JSON-compatible finite data.
            return json.loads(json.dumps({'schema_version': 1, 'frame_id': self.frame_id, 'epoch': self.epoch,
                                         'clock_id': self.clock_id, 'timestamp': now,
                                         'reset_required': self._reset_required, 'entities': entities}, allow_nan=False))

    def semantic_snapshot(self, now=None, *, max_entities=32, evidence_limit=2, preferred_entity_id=None):
        """Bounded detached semantic read, without raw rays or copying full evidence history.

        Unlike snapshot(now), timestamp is the owner's last update, and query_timestamp
        is the requested evaluation time. This lets readers detect an obsolete owner.
        Opening hypotheses are selected first; omitted relations are counted explicitly.
        Existing snapshot/query behavior is unchanged. No world state is mutated.
        """
        if type(max_entities) is not int or not 1 <= max_entities <= self.config.max_entities:
            raise ValueError('max_entities exceeds the configured entity budget')
        if type(evidence_limit) is not int or not 1 <= evidence_limit <= self.config.max_evidence:
            raise ValueError('evidence_limit exceeds the configured evidence budget')
        with self._lock:
            now = self._now if now is None else now
            if now is not None and (not math.isfinite(now) or (self._now is not None and now < self._now)):
                raise ValueError('invalid semantic query time')
            active = [e for e in self._entities.values()
                      if now is None or now - e.last_observed <= self.config.forget_s]
            selected = sorted(active, key=lambda e: (e.entity_id != preferred_entity_id,
                              e.semantic_type != 'opening', -e.last_observed, e.entity_id))[:max_entities]
            present = {e.entity_id for e in selected}
            summaries = []
            for entity in selected:
                age = 0. if now is None else max(0., now - entity.last_observed)
                status = entity.status
                if self._reset_required:
                    status = 'uncertain'
                elif age > self.config.stale_s and status != 'invalidated':
                    status = 'stale'
                relations = [dict(r) for r in entity.relations if r['target'] in present]
                evidence = []
                for item in entity.evidence[-evidence_limit:]:
                    evidence.append({key: item[key] if key in ('observation_id', 'source', 'kind') else float(item[key])
                                     for key in ('observation_id', 'source', 'stamp', 'pose_stamp',
                                                 'position_sigma_m', 'angle_sigma_rad', 'kind')})
                    evidence[-1]['support'] = {key: item['support'][key] for key in
                                               ('rays', 'observed_fraction', 'behind_fraction', 'blocked_fraction')
                                               if key in item['support']}
                geometry = {key: [float(v) for v in value] if isinstance(value, list)
                            else None if value is None else float(value)
                            for key, value in entity.geometry.items()}
                decay = math.exp(-age / self.config.stale_s)
                summaries.append({'entity_id': entity.entity_id, 'semantic_type': entity.semantic_type,
                                  'geometry': geometry, 'frame_id': entity.frame_id, 'epoch': entity.epoch,
                                  'source_timestamp': float(entity.source_timestamp), 'last_observed': float(entity.last_observed),
                                  'age_s': age, 'status': status, 'observations': entity.observations,
                                  'contradictions': entity.contradictions,
                                  'geometric_confidence': float(entity.geometric_confidence * decay),
                                  'semantic_confidence': float(entity.semantic_confidence * decay),
                                  'evidence': evidence, 'missing_evidence': list(entity.missing_evidence),
                                  'relations': relations, 'omitted_relations': len(entity.relations) - len(relations),
                                  'connected_regions': list(entity.connected_regions)})
            return {'schema_version': 1, 'frame_id': self.frame_id, 'epoch': self.epoch, 'clock_id': self.clock_id,
                    'timestamp': self._now, 'query_timestamp': now, 'reset_required': self._reset_required,
                    'stale_after_s': self.config.stale_s, 'total_entities': len(active), 'entities': summaries}

    def to_json(self):
        """Diagnostic AND restart checkpoint, including bounded metric evidence and association state."""
        with self._lock:
            beams = []
            for stamp, rays in self._beams:
                beams.append({'stamp': stamp, 'origin': rays.origin.tolist(),
                              'directions': rays.directions.tolist(), 'valid': rays.valid.tolist(),
                              'depths': [float(z) if v else None for z, v in zip(rays.depths, rays.valid)]})
            return json.dumps({'schema_version': 1, 'config': asdict(self.config), 'snapshot': self.snapshot(),
                               'entities': [asdict(e) for e in self._entities.values()], 'next_id': self._next_id,
                               'last_capture': self._last_capture, 'last_position': None if self._last_position is None
                               else self._last_position.tolist(), 'last_observation_id': self._last_observation_id,
                               'beams': beams}, indent=2, allow_nan=False)

    @classmethod
    def from_json(cls, data):
        """Restore trusted diagnostic checkpoints (not an untrusted network format)."""
        doc = json.loads(data, parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)))
        if doc['schema_version'] != 1:
            raise ValueError('unsupported schema version')
        config, snap = Config(**doc['config']), doc['snapshot']
        state = cls(config, snap['frame_id'], snap['epoch'], snap['clock_id'])
        if len(doc['entities']) > config.max_entities or len(doc['beams']) > config.beam_frames:
            raise ValueError('checkpoint exceeds memory budget')
        for item in doc['entities']:
            if len(item['evidence']) > config.max_evidence or item['frame_id'] != state.frame_id or item['epoch'] != state.epoch:
                raise ValueError('inconsistent entity checkpoint')
            entity = Entity(**item)
            if entity.entity_id in state._entities:
                raise ValueError('duplicate entity ID')
            state._entities[entity.entity_id] = entity
        for beam in doc['beams']:
            if len(beam['depths']) > config.max_rays:
                raise ValueError('checkpoint exceeds ray budget')
            depths = np.array([np.nan if z is None else z for z in beam['depths']])
            state._beams.append((beam['stamp'], Rays(np.array(beam['origin']), np.array(beam['directions']),
                                                     depths, np.array(beam['valid'], dtype=bool))))
        state._now, state._next_id = snap['timestamp'], doc['next_id']
        state._last_capture = doc['last_capture']
        state._last_position = None if doc['last_position'] is None else np.array(doc['last_position'])
        state._last_observation_id = doc['last_observation_id']
        state._reset_required = snap['reset_required']
        state.snapshot()  # Check finite JSON output as well as the schema version.
        return state
