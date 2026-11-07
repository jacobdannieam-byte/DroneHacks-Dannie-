"""Synthetic ROS message shapes rendered with the existing Phase 2 corridor."""

from dataclasses import replace
from math import pi
from types import SimpleNamespace as Msg

import numpy as np

from examples.h1_observation import ClockMapping, HybridBagProfile, RESET_FIELDS, TF, TRACKING
from examples.semantic_world_model_demo import H, K, W, render, scene
from examples.spatial_bag_input import BagProfile
from examples.unified_spatial_replay import REFERENCE, calibration
from examples.unified_trajectory_replay import VEHICLE
from onboard.adapters.geometry import mat_to_quat
from refnav.dynamics import LocalNED
from refnav.reasoning.schema import MissionGoal
from refnav.trajectory_evaluation import FrameAlignment, MetricGoal

MS = 1_000_000
PX4_TOPICS = {
    "vehicle_odometry": "/fmu/out/vehicle_odometry",
    "vehicle_local_position": "/fmu/out/vehicle_local_position_v1",
    "estimator_status_flags": "/fmu/out/estimator_status_flags",
}
DEPTH = "/camera/depth/image_raw"
VIO = "/visual_slam/tracking/odometry"
HEALTH = "/estimator/healthy"


def profile(*, offset_rig=False):
    ref = REFERENCE
    clock = ClockMapping("synthetic_px4", ref.clock_id, 10 * MS, MS, 0, 2_000 * MS, "synthetic-offset")
    frame = FrameAlignment(LocalNED("local_ned", ref.epoch, ref.clock_id), ref, 0, 0,
                           -pi / 2, (0., 0., 0.), .005, .002, "synthetic-alignment", 4)
    cal = calibration()
    if offset_rig:
        # Test rig mounted behind the reference point so the camera sees the starting envelope.
        cal = replace(cal, t_body_optical_m=(-1.45, 0., 0.), calibration_id="synthetic-offset-rig")
    return HybridBagProfile(BagProfile(ref, cal, .01, .005), clock, frame, VEHICLE,
                            MetricGoal(ref, (4., .10, 1.3)), PX4_TOPICS, 7, (0,) * len(RESET_FIELDS),
                            None if offset_rig else MissionGoal())


def stamp(ms):
    return Msg(sec=ms // 1000, nanosec=(ms % 1000) * MS)


def xyz(x, y, z):
    return Msg(x=x, y=y, z=z)


def transform(cal):
    x, y, z, w = mat_to_quat(cal.R_body_optical)
    return Msg(transforms=[Msg(header=Msg(frame_id=cal.body_frame), child_frame_id=cal.camera_frame,
                               transform=Msg(translation=xyz(*cal.t_body_optical_m),
                                             rotation=Msg(x=x, y=y, z=z, w=w)))])


def clock(t):
    return Msg(clock=stamp(t))


def tracking(t, result="visual"):
    return Msg(header=Msg(stamp=stamp(t)), status=[Msg(name="visual_slam/frame", message=result, values=[
        Msg(key="capture_ns", value=str(t * MS)), Msg(key="duration_ms", value="1")])])


def pose(t, x, *, covariance=None):
    return Msg(header=Msg(stamp=stamp(t), frame_id=REFERENCE.frame_id), child_frame_id=REFERENCE.body_frame,
               pose=Msg(covariance=(np.eye(6) * 1e-6).ravel() if covariance is None else covariance,
                        pose=Msg(position=xyz(x, .10, 1.3), orientation=Msg(x=0., y=0., z=0., w=1.))))


def odometry(t, x, velocity=0.):
    return Msg(timestamp_sample=(t - 10) * 1000, timestamp=(t - 9) * 1000,
               pose_frame=1, velocity_frame=1, q=(1., 0., 0., 0.),
               position=(x, -.10, -1.3), velocity=(velocity, 0., 0.), angular_velocity=(0., 0., 0.),
               reset_counter=4, position_variance=(1e-6,) * 3, orientation_variance=(1e-6,) * 3,
               velocity_variance=(1e-6,) * 3)


def depth(t, x, *, invalid=False, cal=None):
    cal = cal or calibration()
    offset = cal.t_body_optical_m[0] - calibration().t_body_optical_m[0]
    surface = scene(rear=False) + [(0, 4.3 if offset else 5.4, (-5., 5.), (-1., 4.))]
    image = render(surface, (x + offset, .10, 1.3))
    if invalid:
        image.fill(np.nan)
    return Msg(header=Msg(stamp=stamp(t), frame_id=calibration().camera_frame), width=W, height=H,
               data=image.astype("<f4").tobytes(), step=4 * W, is_bigendian=False, encoding="32FC1")


def setup(replay):
    cal = replay.profile.calibration
    replay.feed(TF, transform(cal), 85 * MS)
    replay.feed("/clock", clock(90), 90 * MS)
    fx, fy, cx, cy = K
    replay.feed("/camera/depth/camera_info", Msg(header=Msg(frame_id=cal.camera_frame), width=W, height=H,
                p=(fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0)), 90 * MS)


def tick(replay, t, x, *, with_depth=True):
    replay.feed("/clock", clock(t), t * MS)
    replay.feed(HEALTH, Msg(data=True), t * MS)
    replay.feed("/fmu/in/vehicle_visual_odometry", Msg(reset_counter=7), t * MS)
    received = (t + 2) * MS
    replay.feed(PX4_TOPICS["estimator_status_flags"],
                Msg(timestamp=(t - 10) * 1000, cs_ev_pos=True, cs_yaw_align=True), received)
    replay.feed(PX4_TOPICS["vehicle_local_position"],
                Msg(timestamp_sample=(t - 10) * 1000, timestamp=(t - 9) * 1000,
                    xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True, heading_good_for_control=True,
                    x=x, y=-.1, z=-1.3, vx=0., vy=0., vz=0., heading=0.,
                    **dict.fromkeys(RESET_FIELDS, 0)), received)
    replay.feed(PX4_TOPICS["vehicle_odometry"], odometry(t, x), received)
    replay.feed(TRACKING, tracking(t), received)
    if with_depth:
        replay.feed(DEPTH, depth(t, x, cal=replay.profile.calibration), received)
    replay.feed(VIO, pose(t, x), (t + 3) * MS)


def exercise(replay):
    setup(replay)
    for t, x in ((100, 0.), (300, .45), (500, .90)):
        tick(replay, t, x)
    return replay.finish()
