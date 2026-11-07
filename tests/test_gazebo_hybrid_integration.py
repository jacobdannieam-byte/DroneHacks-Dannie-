from dataclasses import replace
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
from types import SimpleNamespace as Msg

import numpy as np
import pytest

from examples.gazebo_container import load_approval, sha
from examples.gazebo_hybrid_adapter import GazeboObservations
from examples.gazebo_sensor_contract import (
    OPTICAL_TO_FLU, SIM_BINDINGS, SIM_DEPTH, SIM_ESTIMATOR,
    depth_layout, read_camera_rig, ros_clock_parts, simulated_health,
)
from examples.h1_fixture import HEALTH, MS, profile, setup, tick
from examples.h1_observation import TRACKING, HybridBagReplay
from examples.hybrid_shadow_loop import ShadowLoop
from onboard.adapters.geometry import mat_to_quat, quat_to_mat
from onboard.diagnostics.sitl_graph import inspect


ROOT = Path(__file__).resolve().parents[1]


def test_rig_uses_authored_mount_and_optical_axes():
    rig = read_camera_rig(ROOT / "simulator/gazebo_trial/models")
    assert (rig.width, rig.height, rig.rate, rig.near_m, rig.far_m) == (512, 320, 10, .35, 10.)
    assert rig.translation == pytest.approx((.131, 0., 0.))
    x, y, z, w = mat_to_quat(OPTICAL_TO_FLU)
    assert np.allclose(quat_to_mat(w, x, y, z) @ [0, 0, 1], (1, 0, 0))
    assert np.allclose(quat_to_mat(w, x, y, z) @ [1, 0, 0], (0, -1, 0))
    image = Msg(pixel_format_type=13, width=512, height=320, step=2048, data=b"\0" * (2048 * 320))
    assert depth_layout(image, rig, float32_enum=13)
    image.step -= 4
    assert not depth_layout(image, rig, float32_enum=13)


def test_simulated_clock_and_command_invert_only_in_reviewed_epoch():
    clock = Msg(sim=Msg(sec=1, nsec=200), system=Msg(sec=2, nsec=2))
    assert ros_clock_parts(clock) == (1, 200)
    clock.sim.nsec = 10**9
    with pytest.raises(ValueError, match="invalid Gazebo"):
        ros_clock_parts(clock)
    mapping = profile().clock
    assert mapping.source_stamp_us(100 * MS) == 90_000
    assert mapping.capture(90_000, 102 * MS) == 100 * MS
    with pytest.raises(ValueError, match="epoch"):
        mapping.source_stamp_us(2_001 * MS)


def bindings(h1):
    return {name: {"topic": SIM_BINDINGS.get(name, name), "type": kind, "provenance": "fixture"}
            for name, kind in HybridBagReplay(h1).topics.items()}


def test_simulator_estimator_profile_is_explicit_and_raw_truth_is_rejected():
    with pytest.raises(ValueError, match="invalid observation binding"):
        GazeboObservations(profile(), bindings(profile()), profile_id="sim")
    sim = replace(profile(), tracking_source=SIM_ESTIMATOR)
    topics = bindings(sim)
    good = GazeboObservations(sim, topics, profile_id="sim")
    assert good.canonical(SIM_DEPTH) == "/camera/depth/image_raw"
    topics["/visual_slam/tracking/odometry"]["topic"] = "/fmu/out/vehicle_odometry_groundtruth"
    with pytest.raises(ValueError, match="invalid observation binding"):
        GazeboObservations(sim, topics, profile_id="sim")


def test_simulated_health_requires_fresh_tracking_and_estimator_flags():
    assert simulated_health(now_ns=100 * MS, sample_ns=40 * MS, status=(95 * MS, True),
                            flags=(95 * MS, True))
    assert not simulated_health(now_ns=100 * MS, sample_ns=40 * MS, status=(95 * MS, True),
                                flags=(95 * MS, False))
    assert not simulated_health(now_ns=250 * MS, sample_ns=40 * MS, status=(240 * MS, True),
                                flags=(240 * MS, True))


def test_delayed_simulated_pose_keeps_capture_health_and_h1_rejects_wrong_tracking_source():
    for tracking_label in (SIM_ESTIMATOR, "visual"):
        sim = replace(profile(offset_rig=True), tracking_source=SIM_ESTIMATOR)
        loop = ShadowLoop(sim, monotonic=lambda: 0.)

        class Adapter:
            profile = sim.spatial
            @staticmethod
            def feed(topic, msg, received_ns):
                if topic == TRACKING:
                    msg.status[0].message = tracking_label
                loop.feed(topic, msg, received_ns, received_ns)

        setup(Adapter())
        tick(Adapter(), 100, 0.)
        result = loop.poll(103 * MS, 103 * MS)
        assert (result.assessment is not None) == (tracking_label == SIM_ESTIMATOR)
        if tracking_label == SIM_ESTIMATOR:
            replay = loop.replay
            replay._health_history.clear()
            replay._health_history.extend(((90 * MS, True), (115 * MS, True)))
            assert replay._health_for_pose(100 * MS)
            replay._health_history.append((120 * MS, False))
            assert not replay._health_for_pose(100 * MS)


def test_approval_binds_exact_inputs_and_world(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "37")
    world, h1, course, profile, approval = [tmp_path / name for name in
                                             ("world.json", "h1.json", "course.json", "profile.json", "approval.json")]
    for file in (world, h1, course):
        file.write_text("{}", encoding="utf-8")
    profile.write_text(json.dumps({"h1_profile": h1.name, "mission_profile": course.name}), encoding="utf-8")
    receipt = {"trial_id": "test-only", "reviewed_by": "fixture", "mode": "hybrid_sitl", "revision": "test-sha",
               "ros_domain_id": 37, "world_sha256": sha(world),
               "expires_utc": (datetime.now(timezone.utc) + timedelta(minutes=2)).isoformat(),
               "profiles": {"launcher": sha(profile), "h1": sha(h1), "course": sha(course)},
               "native_shadow_accepted": True, "fallback_reviewed": True,
               "isolation_reviewed": True, "course_prior_permitted": True}
    approval.write_text(json.dumps(receipt), encoding="utf-8")
    assert load_approval(approval, profile, mode="hybrid_sitl", world=world, revision="test-sha") == receipt
    course.write_text('{"changed": true}', encoding="utf-8")
    with pytest.raises(ValueError, match="exact profiles"):
        load_approval(approval, profile, mode="hybrid_sitl", world=world, revision="test-sha")


def test_strict_owner_graph_refuses_pretend_rc_even_without_setpoints():
    rc = "/fmu/in/manual_control_input"
    class Node:
        def get_node_names_and_namespaces(self):
            return [("pretend_remote_control", "/")]
        def get_topic_names_and_types(self):
            return [(rc, ["px4_msgs/msg/ManualControlSetpoint"])]
        def get_publishers_info_by_topic(self, topic):
            return [Msg(node_namespace="/", node_name="pretend_remote_control",
                        topic_type="px4_msgs/msg/ManualControlSetpoint", endpoint_gid=[1])] if topic == rc else []
    assert not inspect(Node())["issues"]
    assert inspect(Node(), strict_motion=True)["issues"] == [f"unexpected_command_publisher:{rc}"]
