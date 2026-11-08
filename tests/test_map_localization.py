from dataclasses import FrozenInstanceError, replace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from refnav.world_model.map_localization import LocalizationPolicy, MapLocalizer, ReviewedAnchor
from refnav.world_model.map_registration import SE3, RegistrationPolicy, register
from examples.p2_registration_benchmark import reference_for, shape
from refnav.world_model.spatial_input import DepthCalibration, DepthCapture, PoseSample, SpatialReference
from refnav.world_model.spatial_pipeline import SpatialWorld

MS = 1_000_000
VIO = SpatialReference("vio_odom", "test_clock", 4, "body_flu")
IDENTITY = SE3(np.eye(3), (0, 0, 0))
OPTICAL_TO_FLU = ((0, 0, 1), (-1, 0, 0), (0, -1, 0))
CAL = DepthCalibration("camera_optical", VIO.body_frame, 96, 72, (62, 62, 47.5, 35.5), OPTICAL_TO_FLU,
                       (.12, .02, -.01), .003, "reviewed-rig", False, .003, .001)


def assert_pose(actual, truth, translation=.025, degrees=2.):
    assert actual is not None
    assert np.linalg.norm(np.subtract(actual.translation_m, truth.translation_m)) < translation
    delta = np.asarray(actual.rotation) @ np.asarray(truth.rotation).T
    assert np.degrees(Rotation.from_matrix(delta).magnitude()) < degrees


@pytest.mark.parametrize("partial", [False, True])
def test_icp_translation_rotation_and_partial_overlap(tmp_path, partial):
    points = shape()
    ref = reference_for(tmp_path, points)
    truth = SE3(Rotation.from_euler("xyz", (.08, -.05, .12)).as_matrix(), (.21, -.14, .1))
    source = truth.inverse().apply(points[::2])
    if partial:
        source[::5] = np.random.default_rng(4).uniform([.3, -.1, .9], [.5, .1, 1.1], size=(180, 3))
    seed = SE3(Rotation.from_euler("xyz", (.055, -.06, .1)).as_matrix(), (.24, -.11, .08))
    result = register(ref, source, seed)
    assert result.quality.reason == "accepted"
    assert result.quality.overlap == pytest.approx(.8 if partial else 1.)
    assert_pose(result.reference_from_body, truth, .012, 1.)


@pytest.mark.parametrize("case,reason", [
    ("sparse", "insufficient_source"),
    ("wrong_map", "insufficient_local_map"),
    ("near_wrong_map", "high_residual"),
    ("bad_seed", "insufficient_local_map"),
    ("plane", "degenerate_geometry"),
    ("corridor", "degenerate_geometry"),
    ("unreviewed", "unreviewed_reference_scale"),
    ("crop_overflow", "crop_budget_exceeded"),
])
def test_invalid_or_ambiguous_geometry_has_no_pose(tmp_path, case, reason):
    points = shape()
    source = points[::2].copy()
    initial, policy = IDENTITY, RegistrationPolicy()
    if case == "sparse":
        source = source[:10]
    elif case == "wrong_map":
        points = points + (20., 20., 20.)
    elif case == "near_wrong_map":
        # Move two faces to a nearby room with incompatible widths.
        points = points + (0., .48, -.46)
    elif case == "bad_seed":
        initial = SE3(np.eye(3), (10, 0, 0))
    elif case == "plane":
        points, source = points[:600], source[:300]
    elif case == "corridor":
        # Two parallel corridor walls and a floor, with no observed end.
        rng = np.random.default_rng(5)
        points = rng.uniform([-2, -1., 0.], [2, 1., 2.], size=(1800, 3))
        points[:600, 1], points[600:1200, 1], points[1200:, 2] = -1., 1., 0.
        source = points[::2]
    elif case == "crop_overflow":
        policy = replace(policy, max_crop_points=100)
    ref = reference_for(tmp_path, points, reviewed=(case != "unreviewed"))
    result = register(ref, source, initial, policy)
    assert result.reference_from_body is None
    assert result.quality.reason == reason


def test_repeated_distinct_features_trigger_competing_hypotheses(tmp_path):
    # Two short identical trihedra separated along the forward axis.
    rng = np.random.default_rng(7)
    corner = rng.uniform([0., -.15, -.15], [.22, .15, .15], size=(1500, 3))
    corner[:500, 0], corner[500:1000, 1], corner[1000:, 2] = 0., -.15, -.15
    repeated = np.concatenate((corner, corner + (.50, 0, 0)))
    ref = reference_for(tmp_path, repeated)
    result = register(ref, corner[::2], SE3(np.eye(3), (.25, 0, 0)))
    assert result.reference_from_body is None
    assert result.quality.reason == "ambiguous_alignment"
    assert result.quality.competing_solutions > 0


def render_depth(body_x=0., cal=CAL):
    """Analytic orthogonal walls/floor with independent metric scan samples."""
    u, v = np.meshgrid(np.arange(cal.width), np.arange(cal.height))
    fx, fy, cx, cy = cal.K
    rays = np.stack(((u - cx) / fx, (v - cy) / fy, np.ones_like(u)), axis=-1) @ np.asarray(cal.R_body_optical).T
    origin = np.asarray(cal.t_body_optical_m) + (body_x, 0, 0)
    distance = np.full(u.shape, np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for axis, surface in ((0, 2.35), (1, -.74), (2, -.61)):
            z = (surface - origin[axis]) / rays[..., axis]
            distance = np.minimum(distance, np.where(z > 0, z, np.inf))
    return distance.astype(np.float32)


def rendered_reference(directory):
    rng = np.random.default_rng(30)
    points = rng.uniform([.16, -.74, -.61], [2.35, 1.62, 1.42], (18_000, 3))
    points[:6000, 0], points[6000:12000, 1], points[12000:, 2] = 2.35, -.74, -.61
    return reference_for(directory, points)


def aligned(world, t, *, true_x=0., vio_x=0., later=None, ref=VIO, image=None):
    ms = t * MS
    points = [(t, vio_x)] if later is None else [(t - 5, vio_x - later), (t + 5, vio_x + later)]
    for stamp, position in points:
        assert world.push_pose(PoseSample(ref, stamp * MS, stamp * MS + 1, True, (position, 0, 0),
                                          (1, 0, 0, 0), .006, .002))
    depth = render_depth(true_x) if image is None else image
    frame = DepthCapture(ref, ms, ms + 1, CAL.camera_frame, f"capture-{t}", depth)
    update = world.process(frame, now_ns=(points[-1][0] + 1) * MS, retain_aligned_depth=True)
    assert update.accepted, update
    return update.aligned_depth


def test_capture_time_pose_immutable_and_opt_in(tmp_path):
    ref = rendered_reference(tmp_path)
    world = SpatialWorld(VIO, CAL)
    anchor = ReviewedAnchor(ref.version, 2, VIO, 100 * MS, IDENTITY, "survey-evidence")
    localizer = MapLocalizer(ref, VIO, anchor)
    frame = aligned(world, 100, later=.01)
    assert frame.pose.bracket_ns == (95 * MS, 105 * MS)
    assert frame.pose.position_m == pytest.approx((0., 0., 0.))
    for array in (frame.depth_m, frame.pose.rotation):
        with pytest.raises(ValueError):
            array.setflags(write=True)
    snapshot = localizer.update(frame)
    assert snapshot.status == "accepted", snapshot
    assert_pose(snapshot.reference_from_body, IDENTITY)
    assert snapshot.schema == 1 and snapshot.capture_ns == 100 * MS
    assert snapshot.calibration_id == CAL.calibration_id and snapshot.reference_epoch == 2
    with pytest.raises(FrozenInstanceError):
        snapshot.reference_from_body = None


def test_drift_rejection_reacquisition_and_tracking_loss(tmp_path):
    ref = rendered_reference(tmp_path)
    world = SpatialWorld(VIO, CAL)
    anchor = ReviewedAnchor(ref.version, 2, VIO, 100 * MS, IDENTITY, "survey-evidence")
    localizer = MapLocalizer(ref, VIO, anchor)
    first = localizer.update(aligned(world, 100))
    assert first.status == "accepted"
    held = localizer.update(aligned(world, 200, true_x=.08, vio_x=.11))
    assert held.status == "propagated"
    assert held.translation_uncertainty_m > first.translation_uncertainty_m
    drifted = localizer.update(aligned(world, 400, true_x=.16, vio_x=.24))
    assert drifted.status == "accepted", drifted
    assert_pose(drifted.reference_from_body, SE3(np.eye(3), (.16, 0, 0)))
    bad_image = np.full((CAL.height, CAL.width), np.nan, np.float32)
    bad_image[0, 0] = 2.
    rejected = localizer.update(aligned(world, 700, true_x=.2, vio_x=.28, image=bad_image))
    assert rejected.status == "rejected" and rejected.reference_from_body is None
    assert rejected.last_success_ns == 400 * MS
    regained = localizer.update(aligned(world, 1000, true_x=.24, vio_x=.32))
    assert regained.status == "accepted", regained
    assert_pose(regained.reference_from_body, SE3(np.eye(3), (.24, 0, 0)))
    expired = localizer.snapshot(1600 * MS)
    assert expired.reason == "map_fix_expired" and expired.reference_from_body is None
    localizer.invalidate()
    lost = localizer.update(aligned(world, 1100, true_x=.3, vio_x=.39))
    assert lost.reason == "tracking_loss" and lost.reference_from_body is None


def test_reference_epoch_order_and_gap(tmp_path):
    ref = rendered_reference(tmp_path)
    anchor = ReviewedAnchor(ref.version, 2, VIO, 100 * MS, IDENTITY, "survey-evidence")
    localizer = MapLocalizer(ref, VIO, anchor, policy=LocalizationPolicy(max_gap_ns=800 * MS))
    world = SpatialWorld(VIO, CAL)
    first = aligned(world, 100)
    assert localizer.update(first).status == "accepted"
    assert localizer.update(first).reason == "out_of_order_capture"
    old_vio = replace(VIO, epoch=3)
    mismatch = localizer.update(replace(first, reference=old_vio))
    assert mismatch.reason == "vio_reference_mismatch" and mismatch.reference_from_body is None
    gap = localizer.update(aligned(world, 1100))
    assert gap.reason == "capture_gap_requires_reanchor"
    assert MapLocalizer(ref, VIO).update(first).reason == "missing_reviewed_anchor"
    stale_anchor = replace(anchor, reference_version="0" * 64)
    assert MapLocalizer(ref, VIO, stale_anchor).update(first).reason == "anchor_reference_or_time_mismatch"


def test_new_epoch_fences_an_accepted_fix(tmp_path):
    ref = rendered_reference(tmp_path)
    localizer = MapLocalizer(ref, VIO, ReviewedAnchor(ref.version, 2, VIO, 100 * MS, IDENTITY, "survey"))
    first = aligned(SpatialWorld(VIO, CAL), 100)
    assert localizer.update(first).status == "accepted"
    assert localizer.update(replace(first, reference=replace(VIO, epoch=5))).reason == "vio_reference_mismatch"
    assert localizer.snapshot(first.processed_ns).reference_from_body is None


def test_transform_is_rigid_and_explicit():
    with pytest.raises(ValueError, match="right-handed"):
        SE3(np.diag([1, 1, -1]), (0, 0, 0))
    pose = SE3(Rotation.from_euler("z", .27).as_matrix(), (.15, -.2, .1))
    point = (.1, .2, .3)
    np.testing.assert_allclose(pose.inverse().apply(pose.apply(point)), point, atol=1e-12)
    np.testing.assert_allclose(pose.compose(pose).apply(point), pose.apply(pose.apply(point)), atol=1e-12)
