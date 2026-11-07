
from types import SimpleNamespace

from examples.unified_sitl_rehearsal import exercise
from onboard.diagnostics.sitl_graph import COMMAND_TOPICS, inspect


def test_phase7_rehearsal_accepts_only_fresh_reviewable_evidence():
    report = exercise()
    assert report["native_gate"] == "open"
    assert all(row["matched"] for row in report["checks"])
    assert [row["scenario"] for row in report["checks"] if row["observed"] == "eligible_for_offline_review"
            ] == ["fresh_proposal"]


def test_graph_requires_one_unambiguous_owner_and_every_command_endpoint():
    def endpoint(topic, name="refnav", gid=1):
        return SimpleNamespace(node_name=name, node_namespace="/", topic_type=COMMAND_TOPICS[topic],
                               endpoint_gid=bytes([gid]))

    records = {topic: [endpoint(topic)] for topic in COMMAND_TOPICS}
    node = SimpleNamespace(get_node_names_and_namespaces=lambda: [("refnav", "/")],
                           get_publishers_info_by_topic=lambda topic: records.get(topic, []),
                           get_topic_names_and_types=lambda: [(topic, [rows[0].topic_type])
                                                                for topic, rows in records.items()])
    assert len(inspect(node)["issues"]) == 3
    assert not inspect(node, "/refnav")["issues"]
    topic = "/fmu/in/trajectory_setpoint"
    records[topic].append(endpoint(topic, "second_owner", gid=2))
    assert inspect(node, "/refnav")["issues"] == [f"unexpected_command_ownership:{topic}"]
    records[topic].pop()
    node.get_node_names_and_namespaces = lambda: [("refnav", "/"), ("refnav", "/")]
    assert inspect(node, "/refnav")["issues"] == ["expected_owner_missing_or_ambiguous"]
    node.get_node_names_and_namespaces = lambda: [("refnav", "/")]
    alias = "/fmu/in/trajectory_setpoint_v1"
    records[alias] = [endpoint(topic)]
    assert inspect(node, "/refnav")["issues"] == [f"unexpected_command_publisher:{alias}"]
    records.clear()
    assert not inspect(node)["issues"]
