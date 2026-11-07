from types import SimpleNamespace as Msg

import numpy as np
import pytest

from examples.semantic_world_model_demo import H, K, MOUNT, W
from examples.spatial_bag_input import BagProfile, RecordedSpatialReplay
from onboard.adapters.geometry import R_RIG_OPTICAL
from refnav.world_model.spatial_input import DepthCalibration, SpatialReference

MS = 1_000_000
REF = SpatialReference("odom", "bag-sim", 0)
CAL = DepthCalibration("depth_optical", "base_link", W, H, K, R_RIG_OPTICAL, MOUNT, .02, "bag-fixture")


def stamp(ms):
    return Msg(sec=ms // 1000, nanosec=(ms % 1000) * MS)


def initialize():
    replay = RecordedSpatialReplay(BagProfile(REF, CAL, .01, .005))
    replay.feed("/clock", Msg(clock=stamp(90)), 90 * MS)
    replay.feed("/fmu/in/vehicle_visual_odometry", Msg(reset_counter=7), 90 * MS)
    replay.feed("/estimator/healthy", Msg(data=True), 90 * MS)
    fx, fy, cx, cy = K
    replay.feed("/camera/depth/camera_info", Msg(
        header=Msg(frame_id=CAL.camera_frame), width=W, height=H,
        p=[fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]), 90 * MS)
    replay.feed("/clock", Msg(clock=stamp(100)), 100 * MS)
    return replay


def image(t=100):
    data = np.full((H, W), 3., dtype="<f4").tobytes()
    return Msg(header=Msg(stamp=stamp(t), frame_id=CAL.camera_frame), width=W, height=H,
               data=data, step=4 * W, is_bigendian=False, encoding="32FC1")


def odometry(t=100, covariance=None):
    if covariance is None:
        covariance = (np.eye(6) * 1e-6).ravel()
    return Msg(header=Msg(frame_id=REF.frame_id, stamp=stamp(t)), child_frame_id=REF.body_frame,
               pose=Msg(covariance=covariance, pose=Msg(
                   position=Msg(x=0., y=0., z=1.2), orientation=Msg(x=0., y=0., z=0., w=1.))))


def test_recorded_messages_wait_for_capture_pose_after_image_delivery():
    replay = initialize()
    replay.feed("/camera/depth/image_raw", image(), 100 * MS)
    assert not replay.rows
    replay.feed("/visual_slam/tracking/odometry", odometry(), 110 * MS)
    snapshot = replay.finish()
    assert replay.rows[0]["accepted"]
    assert replay.rows[0]["pose_bracket_ns"] == (100 * MS, 100 * MS)
    assert replay.rows[0]["processed_ns"] == 110 * MS
    assert snapshot.metric.capture_ns == 100 * MS


def test_same_tick_health_loss_revokes_snapshot_even_with_duplicate_pose_stamp():
    replay = initialize()
    replay.feed("/visual_slam/tracking/odometry", odometry(), 100 * MS)
    replay.feed("/camera/depth/image_raw", image(), 100 * MS)
    replay.feed("/estimator/healthy", Msg(data=False), 100 * MS)
    snapshot = replay.finish()
    assert replay.rows[0]["accepted"]
    assert not snapshot.metric.available
    assert not np.any(snapshot.metric.cells_at(100 * MS, REF))


def test_recorded_pose_with_missing_covariance_is_not_used():
    replay = initialize()
    replay.feed("/visual_slam/tracking/odometry", odometry(covariance=np.zeros(36)), 100 * MS)
    replay.feed("/camera/depth/image_raw", image(), 100 * MS)
    assert not replay.finish().metric.available
    assert not replay.rows[0]["accepted"]


@pytest.mark.parametrize(("topic", "msg", "time", "pattern"), [
    ("/clock", Msg(clock=stamp(95)), 101, "clock rewind"),
    ("/clock", Msg(clock=stamp(101)), 1000, "recording clock"),
    ("/fmu/in/vehicle_visual_odometry", Msg(reset_counter=8), 101, "EV reset"),
    ("/camera/depth/camera_info", Msg(header=Msg(frame_id="wrong"), width=W, height=H,
                                    p=[1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]),
     101, "CameraInfo"),
])
def test_unreviewed_clock_epoch_and_calibration_are_rejected(topic, msg, time, pattern):
    replay = initialize()
    with pytest.raises(ValueError, match=pattern):
        replay.feed(topic, msg, time * MS)
