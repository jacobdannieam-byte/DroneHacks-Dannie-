from examples.h1_fixture import MS, TRACKING, clock, profile, setup, tick, tracking
from examples.h1_observation import HybridBagReplay
from examples.p2_shadow_replay import (
    DiagnosticSidecar, IDENTITY, _fixture_depth, _fixture_reference, _run, compare,
)
from refnav.integration_timing import Timings
from refnav.world_model.map_localization import MapLocalizer, ReviewedAnchor
from refnav.world_model.spatial_topology import TopologyMemory


def test_matched_h1_h2_replay_keeps_safety_decisions_and_improves_map_pose(tmp_path):
    data = compare(tmp_path, 1, monotonic=lambda: 0.)
    assert data["h3_enabled"] is False
    arms = data["variants"]
    baseline = arms["h1_metric"]
    assert baseline["h2_decisions"] == {"safe_stop_requested": 4}
    for arm in arms.values():
        assert arm["h2_decisions"] == baseline["h2_decisions"]
        assert arm["h2_reasons"] == baseline["h2_reasons"]
        for recorded, original in zip(arm["first_history"], baseline["first_history"]):
            assert recorded["ranked_ids"] == original["ranked_ids"]
            assert recorded["candidates"] == original["candidates"]
        assert not arm["dropped_depth"] and not arm["skipped_ticks"]
    for name in ("localization", "combined"):
        arm = arms[name]
        assert arm["localization_outcomes"] == {"registration_accepted": 4}
        assert arm["map_translation_rmse_m"] < arm["matched_vio_translation_rmse_m"] / 2
        assert arm["max_rotation_error_deg"] < 2.
    assert arms["combined"]["max_topology_entities"] > 0
    assert arms["geometry"]["floor_ceiling_probes"] == {"occupied": 8}
    assert arms["combined"]["false_free_vertical_probes"] == 0


def test_reference_without_reviewed_anchor_preserves_h2_fallback(tmp_path):
    baseline, _, _ = _run(profile(), Timings(), monotonic=lambda: 0.)
    ref = _fixture_reference(tmp_path)
    with_map, _, _ = _run(profile(), Timings(), reference=ref, anchor=None, monotonic=lambda: 0.)
    assert len(with_map) == len(baseline)
    for original, diagnostic in zip(baseline, with_map):
        assert diagnostic["localization_reason"] == "missing_reviewed_anchor"
        assert diagnostic["localization"]["reference_from_body"] is None
        assert (diagnostic["decision"], diagnostic["candidates"]) == (original["decision"], original["candidates"])


def test_sidecar_uses_h1_tracking_loss_and_fences_without_another_depth(tmp_path):
    ref = _fixture_reference(tmp_path)
    rig = profile()
    replay = HybridBagReplay(rig, continuous=True, retain_aligned_depth=True)
    localizer = MapLocalizer(ref, rig.spatial.reference,
                            ReviewedAnchor(ref.version, 3, rig.spatial.reference, 100 * MS, IDENTITY, "fixture"))
    sidecar = DiagnosticSidecar(replay, localizer, TopologyMemory())
    setup(replay)
    tick(replay, 100, 0., depth_factory=_fixture_depth)
    accepted = sidecar.snapshot(103 * MS)
    assert accepted.localization.status == "accepted"
    assert accepted.topology.entities
    assert sidecar.snapshot(103 * MS).localization == accepted.localization
    replay.feed("/clock", clock(110), 110 * MS)
    replay.feed(TRACKING, tracking(110, result="invalid"), 112 * MS)
    lost = sidecar.snapshot(112 * MS)
    assert lost.localization.reference_from_body is None
    assert lost.localization_reason == "h1_tracking_loss_requires_reanchor"
    assert not lost.topology.entities
    tick(replay, 400, .285, depth_factory=_fixture_depth)
    assert sidecar.snapshot(403 * MS).localization.reference_from_body is None
