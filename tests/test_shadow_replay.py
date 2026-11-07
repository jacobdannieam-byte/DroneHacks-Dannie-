
import json
from math import pi
import sys
from types import ModuleType, SimpleNamespace

import pytest

from examples.unified_shadow_benchmark import CONTRACT, events
from examples.unified_shadow_replay import TOPIC, consume, listen, main
from refnav.shadow_replay import MAX_AGE_NS, ShadowReplay


def replay():
    return ShadowReplay(CONTRACT, monotonic=lambda: 0.)


def test_clean_replay_matches_only_verified_held_commands():
    engine = replay()
    rows = [engine.feed(event) for event in events()]
    assert any(row.get("history") == "ready" for row in rows)
    scores = [row["comparison"] for row in rows if isinstance(row.get("comparison"), dict)]
    assert len(scores) == 4 and all(set(score) == {"d0", "ridge"} for score in scores)
    assert any(row.get("comparison") == "effective_command_changed" for row in rows)
    forecast = next(row["forecast"] for row in rows if "forecast" in row)
    assert forecast["end_ns"] - forecast["anchor_ns"] == 1_000_000_000
    assert forecast["assumption"].startswith("counterfactual")


def test_issued_or_future_commands_cannot_warm_history():
    rows = list(events(steps=15))
    for update in ({"acceptance": "issued"}, {"effective_until_ns": 10**12}):
        engine = replay()
        for event in rows:
            row = engine.feed({**event, **update} if event["kind"] == "effective" else event)
            assert "forecast" not in row
        assert engine.history.window(engine.before[1]).status == "empty"


def test_enu_normalization_requires_world_twist_and_heading_rate():
    engine = replay()
    sample = next(events())
    row = engine.feed({**sample, "axes": "ENU", "position_m": [1., 2., 3.],
                       "velocity_mps": [4., 5., 6.], "yaw_rad": 0., "yaw_rate_rad_s": .2})
    assert row["status"] == "anchor"
    state = engine.before[1]
    assert state.position_m == (2., 1., -3.)
    assert state.velocity_mps == (5., 4., -6.)
    assert state.yaw_rad == pytest.approx(pi / 2)
    assert state.yaw_rate_rad_s == pytest.approx(-.2)
    assert replay().feed({**sample, "twist_frame": "FRD"})["status"] == "unverified_velocity_or_heading_frame"
    assert replay().feed({**sample, "yaw_rate_kind": "body_z"})["status"] == "unverified_velocity_or_heading_frame"


def test_backpressure_unhealthy_and_clock_rewind_clear_forecasts():
    rows = list(events(steps=8))
    engine = replay()
    for row in rows[:3]:
        engine.feed(row)
    assert engine.pending is not None
    assert engine.feed(rows[3], delivery_ns=rows[3]["received_ns"] + MAX_AGE_NS + 1)[
        "status"] == "delivery_age_exceeded"
    assert engine.pending is None
    assert engine.feed({**rows[4], "healthy": False})["status"] == "stale_or_unhealthy_state"
    assert engine.feed({**rows[5], "received_ns": 0})["status"] == "clock_rewind_requires_epoch"
    assert engine.feed(rows[6])["status"] == "clock_rewind_requires_epoch"
    assert engine.feed({**rows[6], "epoch": 1, "sequence": rows[6]["sequence"] + 1})["status"] == "anchor"
    assert engine.before[1].reference.reference_epoch == 1
    assert engine.feed({**rows[8], "epoch": 0, "sequence": rows[8]["sequence"] + 2})["status"] == "old_epoch"


def test_sequence_gap_drops_history_and_invalid_payload_fails_closed():
    engine = replay()
    rows = list(events(steps=8))
    for event in rows[:11]:
        engine.feed(event)
    assert engine.pending is not None
    assert engine.feed(rows[12])["sequence_gap"]
    assert engine.pending is None
    assert consume(engine, "[1]") == {"status": "invalid_payload"}
    assert engine.before is None


def test_deadline_discards_prediction():
    ticks = iter((0., .03))
    engine = ShadowReplay(CONTRACT, monotonic=lambda: next(ticks))
    rows = [engine.feed(event) for event in events(steps=1)]
    assert rows[-1]["status"] == "deadline_miss" and "forecast" not in rows[-1]
    assert engine.pending is None


def test_cli_disabled_by_default(capsys):
    main([])
    assert json.loads(capsys.readouterr().out) == {"status": "disabled"}
    with pytest.raises(SystemExit):
        main(["--enable-shadow"])


def test_live_node_is_subscription_only(monkeypatch, capsys):
    rclpy = ModuleType("rclpy")
    callbacks = []

    class Node:
        def create_subscription(self, message_type, topic, callback, qos):
            assert message_type is str and topic == TOPIC and qos.depth == 4
            callbacks.append(callback)

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=5_000_000))

        def destroy_node(self):
            pass

    def create_node(name, **kwargs):
        assert name == "dronehacks_shadow_replay"
        assert kwargs["enable_rosout"] is kwargs["start_parameter_services"] is False
        return Node()

    rclpy.create_node = create_node
    rclpy.init = lambda **_: None
    rclpy.try_shutdown = lambda: None
    rclpy.spin = lambda node: callbacks[0](SimpleNamespace(data=json.dumps(next(events()))))
    monkeypatch.setitem(sys.modules, "rclpy", rclpy)
    monkeypatch.setitem(sys.modules, "rclpy.parameter", SimpleNamespace(Parameter=lambda name, **kw: kw))
    monkeypatch.setitem(sys.modules, "rclpy.qos", SimpleNamespace(
        QoSProfile=lambda **kw: SimpleNamespace(**kw), ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=str))
    listen(replay(), use_sim_time=True)
    assert json.loads(capsys.readouterr().out)["status"] == "anchor"
