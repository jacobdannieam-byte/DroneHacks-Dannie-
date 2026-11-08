from dataclasses import replace

import numpy as np
import pytest

from refnav.world_model.map_localization import MapLocalization
from refnav.world_model.map_registration import SE3
from refnav.world_model.spatial_input import SpatialReference
from refnav.world_model.spatial_pipeline import SpatialSnapshot
from refnav.world_model.spatial_topology import TopologyMemory
from refnav.world_model.spatial_volume import Cell, MapPolicy, RollingVolume

MS = 1_000_000
REF = SpatialReference("vio", "test", 2, "body")
T = 100 * MS


def volume_snapshot(*, reference=REF, time=T, observation="depth-1"):
    volume = RollingVolume(MapPolicy(resolution_m=.25, shape=(16, 16, 16)))
    volume.origin[:] = -8
    volume.cells.fill(Cell.OBSERVED_FREE)
    volume.cells[:, :, 4] = Cell.OCCUPIED       # Floor, z in [-1, -.75)
    volume.cells[:, :, 12] = Cell.OCCUPIED      # Ceiling, z in [1, 1.25)
    volume.cells[8, 10, 8] = Cell.OCCUPIED     # Thin jamb beside a narrow opening
    volume.cells[11, 8, 8] = Cell.UNKNOWN
    volume.stamps.fill(time)
    return volume.snapshot(reference, time, time, observation, "rig", True)


def test_point_proximity_and_vertical_clearance_age_out():
    metric = volume_snapshot()
    query = {"reference": REF, "now_ns": T}
    point = (.125, .125, .125)
    assert metric.cell_at(point, **query) == Cell.OBSERVED_FREE
    assert metric.cell_at((.875, .125, .125), **query) == Cell.UNKNOWN
    assert metric.cell_at((2., 0., 0.), **query) == Cell.UNKNOWN
    assert metric.cell_at((0., 0., -1.), **query) == Cell.OCCUPIED
    assert metric.nearest_occupied_m(point, max_distance_m=.7, **query) == pytest.approx(.375)
    assert metric.nearest_occupied_m(point, max_distance_m=.25, **query) is None
    floor, ceiling = metric.vertical_clearance(point, below_m=.8, above_m=.8, vehicle_radius_m=.1,
                                               margin_m=.08, **query)
    assert floor.status == ceiling.status == "occupied"
    assert floor.in_volume_fractions[0] > 0
    assert metric.cell_at(point, reference=REF, now_ns=T + metric.policy.free_age_ns + 1) == Cell.UNKNOWN
    assert metric.nearest_occupied_m(point, max_distance_m=.7, reference=REF,
                                     now_ns=T + metric.policy.occupied_age_ns + 1) is None
    assert metric.cell_at(point, reference=replace(REF, epoch=3), now_ns=T) == Cell.UNKNOWN


def test_inflated_narrow_opening_unknown_segment_and_map_boundary():
    metric = volume_snapshot()
    query = {"reference": REF, "now_ns": T, "vehicle_radius_m": .16}
    start, end = (-.375, .125, .125), (.375, .125, .125)
    small = metric.check_sweep((start, end), margin_m=.02, **query)
    assert small.status == "verified_free" and small.in_volume_fractions == (1., 0.)
    large = metric.check_sweep((start, end), margin_m=.23, **query)
    assert large.status == "occupied" and large.occupied_cells
    unknown = metric.check_sweep((start, (1.125, .125, .125)), margin_m=.02, **query)
    assert unknown.status == "unknown" and unknown.in_volume_fractions[1] > 0
    outside = metric.check_sweep(((2., 0., 0.), (2.5, 0., 0.)), margin_m=.02, **query)
    assert outside.status == "unknown" and outside.outside_map


def entity(identifier, kind, observation, *, status="provisional", relations=()):
    return {"entity_id": identifier, "semantic_type": kind, "status": status,
            "geometry": {"center": (.125, .125, .125), "normal": (1., 0., 0.),
                         "width_m": .9, "height_m": None, "uncertainty_m": .07},
            "evidence": ({"observation_id": observation, "kind": "support"},),
            "relations": relations, "missing_evidence": ("upper_boundary", "region_connectivity")}


def observation(time=T, identity="depth-1", *, state="provisional", reference=REF):
    transform = SE3(np.eye(3), (5, 0, 0))
    localization = MapLocalization(1, "scan", "a" * 64, "scan_frame", 3, reference, time,
                                   (time, time), identity, "rig", "accepted", "registration_accepted",
                                   transform, transform, .05, .03, last_success_ns=time)
    wall_link = {"type": "bounded_by", "target": "wall_1", "status": "observed_geometry"}
    entities = (entity("opening_1", "opening", identity, status=state, relations=(wall_link,)),
                entity("wall_1", "wall", identity, status="confirmed"))
    spatial = SpatialSnapshot(volume_snapshot(time=time, observation=identity, reference=reference),
                              {"entities": entities, "reset_required": False}, 0)
    return spatial, localization


def test_topology_uses_only_current_bound_evidence_and_stable_ids():
    memory = TopologyMemory()
    spatial, localization = observation()
    wrong = replace(localization, capture_ns=T - 1)
    assert memory.observe(spatial, wrong) == 0
    assert memory.observe(spatial, replace(localization, status="propagated")) == 0
    assert memory.observe(spatial, localization) == 2
    initial = memory.view(now_ns=T, entity_id="opening_1")
    opening, wall = initial.entities
    assert opening.map_position_m == (5.125, .125, .125)
    assert opening.metric_region_index == (10, 0, 0)
    assert opening.observed_height_m is None
    assert opening.evidence_state == "partially_observed" and opening.connectivity == "unknown"
    assert opening.relations[0].target_id == wall.entity_id
    assert opening.uncertainty_m > .12
    assert memory.view(now_ns=T, limit=1).omitted == 1
    assert memory.observe(spatial, localization) == 0
    second = observation(T + 1, "depth-2", state="invalidated")
    assert memory.observe(*second) == 2
    blocked = memory.view(now_ns=T + 1, entity_id=opening.entity_id).entities[0]
    assert blocked.evidence_state == "blocked" and blocked.observation_id == "depth-2"
    aged = memory.view(now_ns=T + 2 + memory.fresh_ns, entity_id=opening.entity_id).entities[0]
    assert aged.evidence_state == "stale" and aged.connectivity == "unknown" and not aged.relations


def test_topology_clears_on_new_epoch_without_new_fix():
    memory = TopologyMemory(max_entities=1)
    assert memory.observe(*observation()) == 2
    assert len(memory.view(now_ns=T).entities) == 1
    new_spatial, _ = observation(T + 1, reference=replace(REF, epoch=3))
    assert memory.observe(new_spatial, None) == 0
    assert memory.view(now_ns=T + 1).entities == ()
