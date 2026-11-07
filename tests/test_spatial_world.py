from dataclasses import replace
import math

import numpy as np
import pytest

from examples.semantic_world_model_demo import H, K, MOUNT, W, render, scene
from onboard.adapters.geometry import R_RIG_OPTICAL, rot_z
from onboard.vio.frames import quat_from_yaw
from refnav.reasoning import DeterministicReasoner, WorldReference, build_context, validate_proposal
from refnav.world_model.spatial_input import (
    DepthCalibration, DepthCapture, PoseSample, SpatialReference, decode_rectified_depth,
)
from refnav.world_model.spatial_pipeline import SpatialWorld
from refnav.world_model.spatial_volume import Cell, MapPolicy, RollingVolume, _rectangle_ranges, _tile_tables

MS = 1_000_000
REF = SpatialReference("odom", "synthetic", 0)
CAL = DepthCalibration("depth_optical", "base_link", W, H, K, R_RIG_OPTICAL, MOUNT, .015, "fixture-rig",
                       validated_pixel_footprint=True, extrinsic_position_error_m=0., extrinsic_angle_error_rad=0.)
BODY = (0., 0., 1.2)
POINT = (2.1, .10, 1.30)


def opening_scene():
    return scene(rear=False) + [(0, 5.4, (-5., 5.), (-1., 4.))]


def pose(t, *, received=None, ref=REF, position=BODY, yaw=0., healthy=True):
    return PoseSample(ref, t * MS, (t if received is None else received) * MS, healthy, position,
                      quat_from_yaw(yaw), .006, .003)


def capture(t, depth, *, ref=REF, received=None):
    return DepthCapture(ref, t * MS, (t if received is None else received) * MS,
                        CAL.camera_frame, f"capture-{t}", depth)


def sweep(snapshot, points=(POINT,), radius=.045, margin=0., now=None, ref=REF):
    return snapshot.metric.check_sweep(points, reference=ref,
                                       now_ns=snapshot.metric.query_ns if now is None else now * MS,
                                       vehicle_radius_m=radius, margin_m=margin)


def test_observed_volume_and_occluded_space_are_distinct():
    world = SpatialWorld(REF, CAL)
    depth = render([(0, 3., (-3., 3.), (-1., 3.))])
    assert world.push_pose(pose(100))
    assert world.process(capture(100, depth), now_ns=100 * MS).accepted
    snapshot = world.snapshot(now_ns=100 * MS)
    assert sweep(snapshot).status == "verified_free"
    assert sweep(snapshot, ((3., .1, 1.3),)).status == "occupied"
    assert sweep(snapshot, ((3.5, .1, 1.3),)).status == "unknown"
    assert sweep(snapshot, ((.03, .1, 1.3),)).status == "unknown"
    assert sweep(snapshot, ((2.1, .1, 1.3), (4., .1, 1.3))).status == "occupied"
    assert sweep(snapshot, ((9., .1, 1.3),)).outside_map
    assert sweep(snapshot, now=601).status == "unknown"
    assert snapshot.metric.observation_id == "capture-100"
    assert snapshot.metric.calibration_id == "fixture-rig"
    assert snapshot.semantic_time_origin_ns == 100 * MS
    with pytest.raises(ValueError):
        snapshot.metric._cells.setflags(write=True)


def test_missing_pixels_and_later_occlusion_revoke_previous_free():
    world = SpatialWorld(REF, CAL)
    far = render([(0, 3.8, (-3., 3.), (-1., 3.))])
    for t, depth in ((100, far), (200, far.copy()), (300, render([(0, 1.5, (-3., 3.), (-1., 3.))]))):
        if t == 200:
            depth[H // 2 - 8:H // 2 + 5, W // 2 - 14:W // 2 + 8] = np.nan
        world.push_pose(pose(t))
        assert world.process(capture(t, depth), now_ns=t * MS).accepted
        assert sweep(world.snapshot(now_ns=t * MS)).status == ("verified_free" if t == 100 else "unknown")


def test_without_validated_pixel_footprints_depth_cannot_clear_volume():
    world = SpatialWorld(REF, replace(CAL, validated_pixel_footprint=False))
    depth = render([(0, 3., (-3., 3.), (-1., 3.))])
    world.push_pose(pose(100))
    assert world.process(capture(100, depth), now_ns=100 * MS).accepted
    snapshot = world.snapshot(now_ns=100 * MS)
    assert sweep(snapshot).status == "unknown"
    assert sweep(snapshot, ((3., .1, 1.3),)).status == "occupied"
    with pytest.raises(ValueError, match="extrinsic error bounds"):
        replace(CAL, extrinsic_position_error_m=None)


def test_occupied_evidence_wins_until_expiry_and_reobservation():
    world = SpatialWorld(REF, CAL)
    near = render([(0, 2.1, (-3., 3.), (-1., 3.))])
    far = render([(0, 3.8, (-3., 3.), (-1., 3.))])
    for t, image, expected in ((100, near, "occupied"), (200, far, "occupied"), (2200, far, "verified_free")):
        world.push_pose(pose(t))
        assert world.process(capture(t, image), now_ns=t * MS).accepted
        assert sweep(world.snapshot(now_ns=t * MS)).status == expected


def test_sweep_includes_ceiling_and_vehicle_margin():
    world = SpatialWorld(REF, CAL)
    overhead = [(2, 1.60, (1.8, 2.7), (-.45, .45))]
    depth = render(overhead + [(0, 3.8, (-3., 3.), (-1., 3.))])
    world.push_pose(pose(100))
    assert world.process(capture(100, depth), now_ns=100 * MS).accepted
    snapshot = world.snapshot(now_ns=100 * MS)
    assert sweep(snapshot, radius=.025).status == "verified_free"
    assert sweep(snapshot, radius=.25, margin=.08).status == "occupied"


def test_odom_interpolation_waits_for_received_bracket():
    world = SpatialWorld(REF, CAL)
    depth = render(scene())
    world.push_pose(pose(95, position=(-.004, 0, 1.2), yaw=-.005))
    world.push_pose(pose(105, received=120, position=(.004, 0, 1.2), yaw=.005))
    image = capture(100, depth)
    assert world.process(image, now_ns=110 * MS).reason == "missing_healthy_capture_pose"
    result = world.process(image, now_ns=120 * MS)
    assert result.accepted and result.pose_bracket_ns == (95 * MS, 105 * MS)
    assert world.snapshot(now_ns=120 * MS).metric.capture_ns == 100 * MS


def test_invalid_capture_revokes_free_until_fresh_observation():
    world = SpatialWorld(REF, CAL)
    depth = render(opening_scene())
    world.push_pose(pose(100))
    assert world.process(capture(100, depth), now_ns=100 * MS).accepted
    assert sweep(world.snapshot(now_ns=100 * MS)).status == "verified_free"
    world.push_pose(pose(200))
    assert world.process(capture(200, np.zeros_like(depth)), now_ns=200 * MS).reason == "no_valid_depth"
    assert sweep(world.snapshot(now_ns=200 * MS)).reason == "no_valid_observation"
    world.push_pose(pose(300))
    assert world.process(capture(300, depth), now_ns=600 * MS).reason == "stale_capture"


def test_epoch_changes_clear_geometry_and_old_packets_do_not_revoke_new_map():
    world = SpatialWorld(REF, CAL)
    depth = render(opening_scene())
    world.push_pose(pose(100))
    world.process(capture(100, depth), now_ns=100 * MS)
    newer = replace(REF, epoch=1)
    world.reset_reference(newer)
    assert not world.snapshot(now_ns=0).metric.available
    world.push_pose(pose(100, ref=newer))
    assert world.process(capture(100, depth, ref=newer), now_ns=100 * MS).accepted
    assert world.process(capture(150, depth), now_ns=150 * MS).reason == "reference_mismatch"
    snapshot = world.snapshot(now_ns=150 * MS)
    assert sweep(snapshot, ref=newer).status == "verified_free"
    assert sweep(snapshot).reason == "reference_or_time_mismatch"


def test_pose_loss_and_implausible_motion_fail_closed():
    world = SpatialWorld(REF, CAL)
    depth = render(scene())
    world.push_pose(pose(100))
    world.process(capture(100, depth), now_ns=100 * MS)
    world.push_pose(pose(120, healthy=False))
    assert not world.snapshot(now_ns=120 * MS).metric.available
    world.push_pose(pose(140))
    assert world.process(capture(130, depth), now_ns=140 * MS).reason == "missing_healthy_capture_pose"
    assert not world.push_pose(pose(150, position=(.9, 0, 1.2)))
    assert world.process(capture(150, depth), now_ns=150 * MS).reason == "reference_reset_required"


def test_rolling_bounds_keep_only_same_global_overlap():
    policy = MapPolicy(shape=(4, 4, 4))
    volume = RollingVolume(policy)
    volume.cells[2, 1, 1], volume.stamps[2, 1, 1] = Cell.FREE, 42
    result = np.zeros(policy.shape, dtype=np.uint8)
    touched = np.zeros(policy.shape, dtype=bool)
    volume.apply((np.array([1, 0, 0]), result, touched), 50)
    assert volume.cells[1, 1, 1] == Cell.FREE and volume.stamps[1, 1, 1] == 42
    assert not np.any(volume.cells[-1])
    volume.apply((np.array([10, 0, 0]), result, touched), 60)
    assert not np.any(volume.cells)


def test_range_lookup_matches_direct_pixel_extrema_including_invalid():
    policy = MapPolicy()
    depth = np.arange(24 * 40, dtype=np.float32).reshape(24, 40) / 1000 + 2.
    depth[0, 0] = np.nan
    tables = _tile_tables(depth, policy)
    bounds = np.array([[0, 0, 4, 2], [1, 0, 3, 1], [4, 2, 4, 2], [0, 0, 0, 0]])
    got = _rectangle_ranges(tables, *bounds.T)
    for (x0, y0, x1, y1), (free, minimum, negmax) in zip(bounds, got):
        pixels = depth[y0 * 8:(y1 + 1) * 8, x0 * 8:(x1 + 1) * 8]
        assert minimum == np.nanmin(pixels) and -negmax == np.nanmax(pixels)
        assert free == (np.nanmin(pixels) if np.isfinite(pixels).all() else -math.inf)


def test_decode_depth_endian_padding_and_no_return():
    source = np.asarray([[1000, 0, 12], [65535, 2500, 34]], dtype=">u2")
    values = decode_rectified_depth(source.tobytes(), encoding="16UC1", width=2, height=2,
                                    step=6, bigendian=True)
    np.testing.assert_allclose(values, [[1., np.nan], [np.nan, 2.5]], equal_nan=True)
    with pytest.raises(ValueError, match="layout"):
        decode_rectified_depth(source.tobytes(), encoding="32FC1", width=2, height=2, step=6)


def test_semantic_opening_retains_identity_and_unknown_height():
    world = SpatialWorld(REF, CAL)
    ids = []
    for t in (100, 200):
        world.push_pose(pose(t))
        assert world.process(capture(t, render(opening_scene())), now_ns=t * MS).accepted
        snapshot = world.snapshot(now_ns=t * MS)
        openings = [e for e in snapshot.semantic["entities"] if e["semantic_type"] == "opening"]
        assert len(openings) == 1
        assert openings[0]["geometry"]["height_m"] is None
        assert openings[0]["connected_regions"] == []
        ids.append(openings[0]["entity_id"])
    assert ids[0] == ids[1]
    reference = WorldReference(REF.frame_id, REF.epoch, REF.clock_id)
    now = snapshot.semantic["query_timestamp"]
    context = build_context(snapshot.semantic, now=now, reference=reference)
    proposal = DeterministicReasoner().propose(context)
    assert proposal.intent == "observe_more" and proposal.target_entity_id == ids[0]
    assert validate_proposal(proposal, context, now=now, reference=reference).valid
