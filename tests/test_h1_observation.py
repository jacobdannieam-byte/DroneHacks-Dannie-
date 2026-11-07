from dataclasses import asdict, replace
from types import SimpleNamespace as Msg

import numpy as np
import pytest

from examples.h1_fixture import (
    DEPTH, HEALTH, MS, PX4_TOPICS, VIO, clock, depth, exercise, odometry, pose, profile, setup, tick, tracking,
    transform,
)
from examples.h1_observation import HybridBagProfile, HybridBagReplay, RESET_FIELDS, TF, TRACKING
from refnav.world_model.spatial_volume import Cell


def initialized(**kwargs):
    replay = HybridBagReplay(profile(), monotonic=lambda: 0., **kwargs)
    setup(replay)
    return replay


def test_replay_joins_distinct_sample_and_receive_times_and_preserves_unknown():
    replay = HybridBagReplay(profile(), monotonic=lambda: 0.)
    snapshot = exercise(replay)
    assert all(row["accepted"] for row in replay.rows)
    assert replay.rows[0]["pose_bracket_ns"] == (100 * MS, 100 * MS)
    assert replay.rows[0]["processed_ns"] == 103 * MS
    assert snapshot.metric.received_ns == 502 * MS
    assert any(entity["semantic_type"] == "opening" for entity in snapshot.semantic["entities"])
    result = replay.diagnostic()
    assert result["px4_sample_us"] == 490000
    assert result["state_capture_ns"] == result["depth_capture_ns"] == 500 * MS
    assert result["state_received_ns"] == result["depth_received_ns"] == 502 * MS
    assert result["assessment_ns"] == 503 * MS
    assert result["goal_id"]
    assert result["reason"] == "no_verified_progress"
    assert result["ranked_ids"] == ()
    assert all("unobserved_swept_volume" in row["reasons"] for row in result["candidates"])
    assert not any(topic.startswith("/fmu/in/") and topic != "/fmu/in/vehicle_visual_odometry"
                   for topic in replay.topics)


def test_metric_goal_can_produce_a_d5_proposal_with_full_fixture_coverage():
    replay = HybridBagReplay(profile(offset_rig=True), monotonic=lambda: 0.)
    setup(replay)
    tick(replay, 100, 0.)
    result = replay.diagnostic()
    assert result["reason"] == "eligible_for_offline_review"
    assert result["decision"] == "candidate_proposed"
    assert result["ranked_ids"][0] == "forward"


def test_missing_delayed_and_invalid_depth_fail_closed():
    replay = initialized()
    assert replay.diagnostic()["reason"] == "missing_synchronized_depth"
    tick(replay, 100, 0.)
    replay.feed("/clock", clock(400), 400 * MS)
    replay.feed(DEPTH, depth(110, 0.), 400 * MS)
    assert replay.diagnostic()["reason"] == "awaiting_capture_pose"
    snapshot = replay.finish()
    assert replay.rows[-1]["reason"] == "stale_capture"
    assert not snapshot.metric.available
    assert not np.any(snapshot.metric.cells_at(400 * MS, snapshot.metric.reference) == Cell.FREE)

    replay = initialized()
    tick(replay, 100, 0., with_depth=False)
    replay.feed(DEPTH, depth(100, 0., invalid=True), 104 * MS)
    snapshot = replay.finish()
    assert replay.rows[-1]["reason"] == "no_valid_depth"
    assert not np.any(snapshot.metric.cells_at(104 * MS, snapshot.metric.reference) == Cell.FREE)


def test_tracking_loss_revokes_free_geometry_even_if_bridge_is_healthy():
    replay = initialized()
    tick(replay, 100, 0.)
    replay.feed(TRACKING, tracking(104, "inertial"), 104 * MS)
    assert replay.diagnostic()["reason"] == "geometry_unavailable"
    metric = replay.finish().metric
    assert not np.any(metric.cells_at(104 * MS, metric.reference) == Cell.FREE)


@pytest.mark.parametrize("scenario, reason", [
    ("rewind", "clock rewind"),
    ("wrong_clock", "px4_stamp_outside_reviewed_clock"),
    ("ev_reset", "ev_reset_requires_new_epoch"),
    ("px4_reset", "px4_odometry_reset_requires_new_epoch"),
    ("local_reset", "px4_local_reset_requires_new_epoch"),
    ("jump", "px4_pose_jump_requires_new_epoch"),
    ("vio_jump", "vio_pose_reset_requires_new_epoch"),
    ("depth_frame", "depth optical frame"),
    ("extrinsics", "invalid_static_tf|depth_extrinsics_mismatch"),
    ("overload", "depth_queue_overload"),
])
def test_faults_fence_the_epoch(scenario, reason):
    replay = initialized()
    tick(replay, 100, 0.)
    wrong_depth = depth(109, 0.)
    wrong_depth.header.frame_id = "wrong_camera"
    reset_odom = odometry(109, 0.)
    reset_odom.reset_counter = 5
    bad_transform = transform(replace(replay.profile.calibration, t_body_optical_m=(.2, 0., 0.)))
    if scenario == "overload":
        replay.feed(DEPTH, depth(109, 0.), 110 * MS)
    if scenario == "vio_jump":
        replay.feed(VIO, pose(109, 1.), 110 * MS)
    topic, msg = {
        "rewind": ("/clock", clock(99)),
        "wrong_clock": (PX4_TOPICS["vehicle_odometry"], odometry(180, 0.)),
        "ev_reset": ("/fmu/in/vehicle_visual_odometry", Msg(reset_counter=8)),
        "px4_reset": (PX4_TOPICS["vehicle_odometry"], reset_odom),
        "local_reset": (PX4_TOPICS["vehicle_local_position"],
                        Msg(timestamp_sample=99000, timestamp=100000, **dict.fromkeys(RESET_FIELDS, 1))),
        "jump": (PX4_TOPICS["vehicle_odometry"], odometry(109, 1.)),
        "vio_jump": (DEPTH, depth(109, 1.)),
        "depth_frame": (DEPTH, wrong_depth),
        "extrinsics": (TF, bad_transform),
        "overload": (DEPTH, depth(110, 0.)),
    }[scenario]
    with pytest.raises(ValueError, match=reason):
        replay.feed(topic, msg, 111 * MS)
    assert replay.diagnostic()["reason"] == replay.fault
    with pytest.raises(ValueError, match=reason):
        replay.finish()


def test_missing_static_extrinsics_do_not_reach_the_spatial_reader():
    replay = HybridBagReplay(profile())
    replay.feed("/clock", clock(100), 100 * MS)
    with pytest.raises(ValueError, match="missing_reviewed_depth_extrinsics"):
        replay.feed(DEPTH, depth(100, 0.), 102 * MS)


def test_state_and_geometry_expire_independently():
    replay = initialized()
    tick(replay, 100, 0.)
    replay.feed("/clock", clock(201), 201 * MS)
    assert replay.diagnostic()["reason"] == "stale_state_or_clock_mapping"
    tick(replay, 360, 0., with_depth=False)
    assert replay.diagnostic()["reason"] == "stale_geometry"


def test_invalid_px4_frames_and_covariance_have_no_usable_state():
    replay = initialized()
    tick(replay, 100, 0.)
    bad = odometry(109, 0.)
    bad.velocity_frame = 3
    replay.feed(PX4_TOPICS["vehicle_odometry"], bad, 111 * MS)
    assert replay.diagnostic()["reason"] == "invalid_px4_frame_or_odometry"
    uncertain = odometry(111, 0.)
    uncertain.position_variance = (.1,) * 3
    replay.feed(PX4_TOPICS["vehicle_odometry"], uncertain, 113 * MS)
    assert replay.diagnostic()["reason"] == "invalid_or_excessive_px4_covariance"


def test_h0_deadline_miss_discards_candidate_ids():
    ticks = iter((0., .026))
    replay = HybridBagReplay(profile(offset_rig=True), monotonic=lambda: next(ticks))
    setup(replay)
    tick(replay, 100, 0.)
    result = replay.rows[-1]["hybrid"]
    assert result["reason"] == "deadline_exceeded"
    assert result["ranked_ids"] == ()


def test_profile_rejects_inconsistent_epochs_and_clocks():
    p = profile()
    with pytest.raises(ValueError, match="inconsistent_reviewed_reference"):
        replace(p, alignment=replace(p.alignment, ned=replace(p.alignment.ned, reference_epoch=8)))
    with pytest.raises(ValueError, match="inconsistent_reviewed_reference"):
        replace(p, clock=replace(p.clock, ros_clock_id="not_the_recording_clock"))


def test_native_profile_round_trips_reviewed_provenance():
    p = profile()
    spatial, alignment = asdict(p.spatial), asdict(p.alignment)
    data = {"spatial": {**spatial, "record_clock": "ros_sim", "rectified": True},
            "clock": asdict(p.clock), "ned_frame": p.alignment.ned.frame_id,
            "alignment": {k: v for k, v in alignment.items() if k not in ("ned", "map", "capture_ns")},
            "vehicle": asdict(p.vehicle), "objective_m": list(p.objective.position_m),
            "px4_topics": p.px4_topics, "ev_reset_counter": p.ev_reset_counter,
            "local_reset_counters": list(p.local_reset_counters), "mission_goal": p.mission_goal.to_dict()}
    assert HybridBagProfile.from_dict(data) == p
