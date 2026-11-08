from dataclasses import asdict, replace
import json

import pytest

from examples.gazebo_hybrid_adapter import GazeboObservations, IsolationApproval
from examples.h1_fixture import MS, PX4_TOPICS, clock, odometry, setup, tick
from examples.hybrid_launch import configured, main, run_reviewed_sitl
from examples.hybrid_mission_smoke import FakeSink, exercise, runtime_fixture
from refnav.core import NAV_OFFBOARD
from refnav.hybrid_mission import CourseMission
from refnav.reasoning.schema import MissionGoal
from refnav.trajectory_evaluation import MetricGoal
from refnav.world_model.spatial_volume import Cell


def test_two_checkpoints_end_through_real_landing_lifecycle():
    result = exercise()
    assert result["result"] == "complete"
    assert result["status"]["mission_state"] == "DISARMED"
    records = result["decisions"]
    assert "SELECT_NEXT_GOAL" in [r["mission_state"] for r in records]
    assert [r["stage_id"] for r in records if r.get("reason") == "hybrid_position_selected"] == [
        "start_corridor", "start_corridor", "land", "land"]
    command_row = next(row for row in result["fake_sink"] if row["commands"] == ["land"])
    assert command_row["setpoint"] is None
    assert all(row["setpoint"] is None for row in result["fake_sink"] if row["tick_ns"] >= command_row["tick_ns"])


def test_unobserved_route_does_not_publish_classical_fallback_or_brake():
    result = exercise(unavailable=True)
    assert result["status"]["mission_state"] == "DISARMED"
    assert "unobserved_swept_volume" in result["result"]
    land = next(row for row in result["fake_sink"] if row["commands"] == ["land"])
    assert land["setpoint"] is None
    assert not any(r.get("reason") == "hybrid_position_selected" for r in result["decisions"])


def start_navigation(runtime):
    sink = FakeSink(runtime)
    sink.step(100, 0., armed=False, landed=True, nav_state=0, z=-1.)
    sink.step(150, 0., armed=False, landed=True, nav_state=0, z=-1.)
    sink.step(200, 0., armed=True, landed=True, nav_state=NAV_OFFBOARD, z=-1.)
    for t in (260, 320, 380):
        sink.step(t, 0., armed=True, landed=False, nav_state=NAV_OFFBOARD)
    return sink


def test_held_target_revalidated_on_new_state_and_replaced_on_replan():
    runtime = runtime_fixture()
    sink = start_navigation(runtime)
    first = sink.messages[-1]["setpoint"]
    assert first[0] > .3
    out = sink.step(420, .05, armed=True, landed=False, nav_state=NAV_OFFBOARD)
    assert out.setpoint == first
    assert runtime.records[-1]["reason"] == "held_target_revalidated"
    out = sink.step(470, .10, armed=True, landed=False, nav_state=NAV_OFFBOARD, generation=2)
    assert out.setpoint[:3] == (.10, -.10, -1.3)
    assert runtime.records[-1]["reason"] == "verified_stationary_hold:held_target_expired_or_changed"
    out = sink.step(520, .13, armed=True, landed=False, nav_state=NAV_OFFBOARD, generation=2)
    assert out.setpoint[0] > .3
    assert runtime.records[-1]["reason"] == "hybrid_position_selected"


def test_changed_geometry_revokes_held_target_before_use():
    runtime = runtime_fixture()
    sink = start_navigation(runtime)
    tick(sink, 420, 0.)
    replay = runtime.loop.replay
    snapshot = replay._snapshot
    cells = snapshot.metric._cells.copy()
    cells[cells == Cell.FREE] = Cell.UNKNOWN
    cells.setflags(write=False)
    replay._snapshot = replace(snapshot, metric=replace(snapshot.metric, _cells=cells))
    from refnav.core import Vehicle
    vehicle = Vehicle(pos=(0., -.10, -1.3), heading=0., pos_ok=True, armed=True,
                      landed=False, nav_state=NAV_OFFBOARD)
    out = runtime.owner_step(sink.mission, vehicle, now_ns=423 * MS, wall_ns=423 * MS,
                             generation=1, admission=True, owner_ready=True)
    assert out.commands == ["land"] and not out.stream
    assert runtime.executor.held is None
    assert "unobserved_swept_volume" in runtime.course.land_reason


def test_wall_expiry_is_not_extended_by_unchanged_sim_clock():
    wall = [0.]
    runtime = runtime_fixture(monotonic=lambda: wall[0])
    sink = start_navigation(runtime)
    wall[0] = .26
    out = sink.step(420, .04, armed=True, landed=False, nav_state=NAV_OFFBOARD)
    assert out.setpoint[0] == .04
    assert runtime.executor.held is None


def test_watchdog_expires_held_target_without_a_ros_owner_tick():
    wall = [0.]
    runtime = runtime_fixture(monotonic=lambda: wall[0])
    sink = start_navigation(runtime)
    target = sink.messages[-1]["setpoint"]
    now_ns = sink.messages[-1]["tick_ns"]
    assert runtime.executor.held.target == target
    wall[0] = .249
    assert not runtime.expire_owner_target(now_ns, 249 * MS)
    wall[0] = .25
    assert runtime.expire_owner_target(now_ns, 250 * MS)
    assert runtime.executor.held is None
    assert runtime.records[-1]["event"] == "owner_target_expired"
    assert runtime.records[-1]["wall_expired"] and not runtime.records[-1]["ros_expired"]
    out = sink.step(420, .04, armed=True, landed=False, nav_state=NAV_OFFBOARD)
    assert out.setpoint[:3] == (.04, -.10, -1.3) and out.setpoint != target


def test_publication_fence_expires_target_on_ros_time_jump():
    runtime = runtime_fixture()
    start_navigation(runtime)
    expiry = runtime.executor.held.ros_expires_ns
    assert not runtime.expire_owner_target(expiry - 1, 383 * MS)
    assert runtime.expire_owner_target(expiry, 383 * MS)
    assert runtime.executor.held is None
    assert runtime.records[-1]["ros_expired"] and not runtime.records[-1]["wall_expired"]


def test_terminal_phase_with_pending_command_still_checks_clock():
    runtime = runtime_fixture()
    sink = start_navigation(runtime)
    now_ns = sink.messages[-1]["tick_ns"]
    assert not runtime.watch_owner_clock(sink.mission, now_ns, now_ns)
    sink.mission.phase = "done"
    assert not runtime.watch_owner_clock(sink.mission, now_ns, now_ns + 300 * MS)
    assert runtime.watch_owner_clock(sink.mission, now_ns, now_ns + 300 * MS, control_pending=True)
    assert sink.mission.result == "yielded: owner_ros_clock_stalled"


@pytest.mark.parametrize("clock_delta_ms,wall_delta_ms,reason", [
    (0, 300, "owner_ros_clock_stalled"),
    (5, 300, "owner_ros_clock_stalled"),  # /clock resumed before the overdue callback ran
    (-1, 50, "owner_ros_clock_rewind"),
])
def test_active_clock_watchdog_yields_and_latches_across_recovery(clock_delta_ms, wall_delta_ms, reason):
    runtime = runtime_fixture()
    sink = start_navigation(runtime)
    mission = sink.mission
    now_ns = sink.messages[-1]["tick_ns"]
    assert mission.phase == "go" and runtime.executor.held is not None
    assert not runtime.watch_owner_clock(mission, now_ns, now_ns)
    assert not runtime.watch_owner_clock(mission, now_ns, now_ns + 20 * MS)
    assert runtime.watch_owner_clock(mission, now_ns + clock_delta_ms * MS, now_ns + wall_delta_ms * MS)
    assert runtime.owner_watchdog_fault == reason
    assert runtime.executor.held is None
    assert mission.result == f"yielded: {reason}"
    assert runtime.course.state == "AUTHORITY_LOST"
    assert runtime.records[-1]["event"] == "owner_watchdog"
    assert not runtime.watch_owner_clock(mission, now_ns + 400 * MS, now_ns + 400 * MS)
    out = sink.step(440, .04, armed=True, landed=False, nav_state=NAV_OFFBOARD)
    assert out.phase == "done" and not out.stream and out.setpoint is None and out.commands == []


@pytest.mark.parametrize("loss", ["takeover", "approval", "epoch"])
def test_owner_loss_fences_motion_and_never_replays_target(loss):
    runtime = runtime_fixture()
    sink = start_navigation(runtime)
    if loss == "epoch":
        sink.feed("/clock", clock(410), 410 * MS)
        reset = odometry(410, 0.)
        reset.reset_counter += 1
        sink.feed(PX4_TOPICS["vehicle_odometry"], reset, 412 * MS)
    out = sink.step(420, 0., armed=True, landed=False,
                    nav_state=0 if loss == "takeover" else NAV_OFFBOARD,
                    admission=loss != "approval")
    assert out.setpoint is None and not out.stream and runtime.executor.held is None
    if loss == "epoch":
        assert out.commands == ["land"]
    else:
        assert out.commands == [] and runtime.course.state == "AUTHORITY_LOST"


def test_unapproved_owner_does_not_prime_or_arm():
    runtime = runtime_fixture()
    sink = FakeSink(runtime)
    for t in (100, 240, 380):
        out = sink.step(t, 0., armed=False, landed=True, nav_state=0, admission=False)
        assert out.phase == "wait" and not out.stream and not out.commands


def test_semantic_stage_rejects_missing_or_unverified_landing_evidence():
    runtime = runtime_fixture(strategy="hybrid_shadow")
    sink = FakeSink(runtime)
    tick(sink, 100, 0.)
    _, inputs = runtime.loop.replay.prepare(103 * MS)
    first, last = runtime.course.stages
    region = MissionGoal("reach_region", "unseen-region")
    course = CourseMission((replace(first, semantic=region), last))
    assert course.select(103 * MS, inputs, None) is None
    assert course.reason == "semantic_goal_unavailable"
    course = CourseMission((replace(last, landing_site_review=None),))
    assert course.select(103 * MS, inputs, None) is None
    assert course.reason == "landing_site_unverified"


def test_bounded_detour_uses_only_reviewed_metric_and_ignores_duplicate_failure():
    runtime = runtime_fixture(strategy="hybrid_shadow")
    sink = FakeSink(runtime)
    first, last = runtime.course.stages
    detour = MetricGoal(first.goal.reference, (.5, .25, 1.3))
    course = CourseMission((replace(first, alternatives=(detour,)), last), failure_limit=2)
    for t in (100, 250):
        tick(sink, t, 0.)
        ns = (t + 3) * MS
        _, inputs = runtime.loop.replay.prepare(ns)
        assert course.select(ns, inputs, None) == first.goal
        for _ in range(3):
            course.result(ns, "no_verified_progress")
        if t == 100:
            assert course.goal == first.goal
    assert course.goal == detour
    assert course.land_reason is None


def test_goal_timeout_cannot_be_reset_by_repeated_observations():
    runtime = runtime_fixture()
    first, last = runtime.course.stages
    course = CourseMission((replace(first, timeout_s=1.), last))
    assert course.select(100 * MS, None, None) is None
    assert course.select(1_101 * MS, None, None) is None
    assert course.land_reason == "goal_timeout"


def test_shadow_remaps_sensors_without_rewriting_ros_data():
    runtime = runtime_fixture(strategy="hybrid_shadow")
    old = runtime.adapter
    sources = {name: {"topic": "/rig/depth" if name == "/camera/depth/image_raw" else name,
                      "type": kind, "provenance": "fixture"} for name, kind in old.topics.items()}
    runtime.adapter = GazeboObservations(runtime.loop.replay.hybrid, sources, profile_id="fixture")

    class Remap:
        profile = runtime.loop.replay.profile

        @staticmethod
        def feed(topic, message, received_ns):
            runtime.feed(runtime.adapter.native(topic), message, received_ns, received_ns)

    driver = Remap()
    setup(driver)
    tick(driver, 100, 0.)
    report = runtime.poll(103 * MS, 103 * MS).report()
    assert report["depth_capture_ns"] == 100 * MS and report["depth_received_ns"] == 102 * MS
    assert not runtime.executor.gateway.enabled
    runtime.close()
    assert runtime.poll(250 * MS, 250 * MS).reason == "shadow_closed"
    sources["/camera/depth/image_raw"]["topic"] = "/simulator_only/depth"
    with pytest.raises(ValueError, match="invalid observation binding"):
        GazeboObservations(runtime.loop.replay.hybrid, sources, profile_id="fixture")


def test_reviewed_sitl_config_parses_but_cli_cannot_arm(tmp_path, capsys):
    runtime = runtime_fixture()
    p = runtime.loop.replay.hybrid
    h1 = {"spatial": {**asdict(p.spatial), "record_clock": "ros_sim", "rectified": True},
          "clock": asdict(p.clock), "alignment": asdict(p.alignment), "ned_frame": p.alignment.ned.frame_id,
          "vehicle": asdict(p.vehicle), "objective_m": p.objective.position_m, "px4_topics": p.px4_topics,
          "ev_reset_counter": p.ev_reset_counter, "local_reset_counters": p.local_reset_counters}
    for field in ("ned", "map", "capture_ns"):
        h1["alignment"].pop(field)
    course = {"reference": {k: getattr(p.spatial.reference, k) for k in ("frame_id", "epoch", "clock_id")},
              "route_review": "offline fixture", "stages": [
                  {"stage_id": stage.stage_id, "kind": stage.kind, "position_m": stage.goal.position_m,
                   "landing_site_review": stage.landing_site_review} for stage in runtime.course.stages]}
    launch = {"strategy": "hybrid_sitl", "review_status": "reviewed", "profile_id": "fixture",
              "h1_profile": "h1.json", "mission_profile": "course.json",
              "sources": {name: {"topic": name, "type": kind, "provenance": "fixture"}
                          for name, kind in runtime.topics.items()},
              "owner_topics": dict(runtime.adapter.owner_topics)}
    for name, data in (("h1", h1), ("course", course), ("launch", launch)):
        (tmp_path / (name + ".json")).write_text(json.dumps(data), encoding="utf-8")
    config = tmp_path / "launch.json"
    mode, loaded = configured(config)
    assert mode == "hybrid_sitl" and loaded.course.stage.stage_id == "start_corridor"
    assert main(["--config", str(config)]) == 2
    assert "no Gazebo activation launcher" in capsys.readouterr().err

    course['startup_scan_position_m'] = [.0, .1, 1.3]
    course['stages'][0].update(kind='waypoint', scan_on_arrival=True)
    (tmp_path / 'course.json').write_text(json.dumps(course), encoding='utf-8')
    with pytest.raises(ValueError, match='shadow-only'):
        configured(config)
    launch['strategy'] = 'hybrid_shadow'
    config.write_text(json.dumps(launch), encoding='utf-8')
    mode, loaded = configured(config)
    assert mode == 'hybrid_shadow' and loaded.course.scans_enabled
    assert loaded.course.startup_scan.position_m == (.0, .1, 1.3)
    assert loaded.course.stage.scan_on_arrival


def test_profile_default_and_activation_checks_without_ros(tmp_path, capsys, monkeypatch):
    path = tmp_path / "classical.json"
    path.write_text("{}", encoding="utf-8")
    assert main(["--config", str(path), "--check"]) == 0
    assert json.loads(capsys.readouterr().out)["strategy"] == "classical"
    assert main(["--config", "scenarios/hybrid_gazebo.template.json", "--check"]) == 2
    assert "incomplete" in capsys.readouterr().err
    runtime = runtime_fixture()
    monkeypatch.setenv("ROS_DOMAIN_ID", "23")
    with pytest.raises(ValueError, match="approval refused"):
        run_reviewed_sitl(runtime, None, ros_args=(), output=tmp_path / "never.json")
    permit = IsolationApproval("fixture", 23, "evidence", lambda *args: True)
    assert permit.permits(runtime.adapter, "23")
    assert not permit.permits(runtime.adapter, "0")
    with pytest.raises(ValueError, match="approval refused"):
        run_reviewed_sitl(runtime, replace(permit, verify_isolation=lambda *args: False),
                          ros_args=(), output=tmp_path / "never.json")
