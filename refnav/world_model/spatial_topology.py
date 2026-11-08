"""Bounded map-relative semantic anchors over existing WorldState evidence."""

from dataclasses import dataclass, replace
import math

import numpy as np

from .spatial_input import NS, stamp_ok


@dataclass(frozen=True)
class GeometryLink:
    relation: str
    target_id: str
    evidence_status: str


@dataclass(frozen=True)
class AnchoredEntity:
    entity_id: str
    kind: str
    capture_ns: int
    observation_id: str
    evidence_kind: str
    map_position_m: tuple
    map_normal: tuple
    metric_region_index: tuple
    width_m: float
    observed_width_segment_m: tuple
    observed_height_m: float | None
    uncertainty_m: float
    evidence_state: str
    connectivity: str
    relations: tuple[GeometryLink, ...]
    missing_evidence: tuple


@dataclass(frozen=True)
class TopologyView:
    schema: int
    reference_key: tuple | None
    region_size_m: float
    entities: tuple[AnchoredEntity, ...]
    omitted: int


class TopologyMemory:
    """Single-owner semantic history, scoped to one scan and VIO epoch."""

    def __init__(self, *, max_entities=128, fresh_ns=2 * NS, region_size_m=.5):
        if (type(max_entities) is not int or not 1 <= max_entities <= 128
                or not stamp_ok(fresh_ns) or not 0 < fresh_ns <= 5 * NS or not .1 <= region_size_m <= 5):
            raise ValueError("invalid semantic topology budget")
        self.max_entities = max_entities
        self.fresh_ns = fresh_ns
        self.region_size_m = region_size_m
        self.clear()

    def clear(self):
        self._key = self._last_capture = None
        self._entities = {}

    def observe(self, spatial, localization):
        """Anchor only current WorldState evidence using an accepted capture-matched fit."""
        metric = spatial.metric
        semantic = spatial.semantic
        if self._key is not None and metric.reference != self._key[-1]:
            if metric.reference.epoch >= self._key[-1].epoch:
                self.clear()
            else:
                return 0
        if (localization is None or not metric.available or semantic["reset_required"]
                or localization.status != "accepted" or localization.reference_from_vio is None
                or (localization.vio, localization.capture_ns, localization.observation_id, localization.calibration_id)
                != (metric.reference, metric.capture_ns, metric.observation_id, metric.calibration_id)):
            return 0
        key = (localization.map_id, localization.map_version, localization.reference_frame,
               localization.reference_epoch, localization.vio)
        if self._key != key:
            self.clear()
        if self._last_capture is not None and metric.capture_ns <= self._last_capture:
            return 0
        self._key, self._last_capture = key, metric.capture_ns
        transform = localization.reference_from_vio
        body_position = localization.reference_from_body.translation_m
        updated = 0
        for entity in semantic["entities"]:
            if not entity["evidence"] or entity["status"] in ("stale", "uncertain"):
                continue
            evidence = entity["evidence"][-1]
            if evidence["observation_id"] != metric.observation_id:
                continue
            geo = entity["geometry"]
            center = tuple(map(float, transform.apply(geo["center"])))
            normal = tuple(map(float, np.asarray(transform.rotation) @ geo["normal"]))
            tangent = np.array((-geo["normal"][1], geo["normal"][0], 0.))
            half_width = tangent * (geo["width_m"] / 2)
            width_segment = tuple(tuple(map(float, transform.apply(np.asarray(geo["center"]) + sign * half_width)))
                                  for sign in (-1, 1))
            region = tuple(math.floor(p / self.region_size_m) for p in center)
            if entity["status"] == "invalidated":
                state = "blocked"
            elif entity["semantic_type"] == "wall" and entity["status"] == "confirmed":
                state = "observed_surface"
            else:
                state = "partially_observed"
            relations = tuple(GeometryLink(r["type"], r["target"], r["status"])
                              for r in entity["relations"][:8] if r["type"] == "bounded_by")
            error = (geo["uncertainty_m"] + localization.translation_uncertainty_m
                     + math.dist(center, body_position) * localization.rotation_uncertainty_rad)
            identifier = entity["entity_id"]
            if identifier not in self._entities and len(self._entities) >= self.max_entities:
                victim = min(self._entities, key=lambda k: (self._entities[k].capture_ns, k))
                del self._entities[victim]
            self._entities[identifier] = AnchoredEntity(
                identifier, entity["semantic_type"], metric.capture_ns, metric.observation_id,
                evidence["kind"], center, normal, region, geo["width_m"], width_segment, geo["height_m"], error,
                state, "blocked" if state == "blocked" else "unknown", relations, tuple(entity["missing_evidence"]),
            )
            updated += 1
        return updated

    def view(self, *, now_ns, limit=16, entity_id=None):
        """Return bounded immutable entity/neighbor evidence for a diagnostic reader."""
        if (type(limit) is not int or not 1 <= limit <= 32 or not stamp_ok(now_ns)
                or (self._last_capture is not None and now_ns < self._last_capture)):
            raise ValueError("invalid topology query bound or time")
        entities = self._entities
        if entity_id is not None:
            item = entities.get(entity_id)
            ids = () if item is None else (entity_id, *(r.target_id for r in item.relations if r.target_id in entities))
        else:
            ids = sorted(entities, key=lambda k: (-entities[k].capture_ns, k))
        rows = []
        for identifier in ids[:limit]:
            item = entities[identifier]
            if now_ns - item.capture_ns > self.fresh_ns:
                item = replace(item, evidence_state="stale", connectivity="unknown", relations=())
            else:
                links = tuple(r for r in item.relations if r.target_id in entities
                              and now_ns - entities[r.target_id].capture_ns <= self.fresh_ns)
                item = replace(item, relations=links)
            rows.append(item)
        return TopologyView(1, self._key, self.region_size_m, tuple(rows), max(0, len(ids) - len(rows)))
