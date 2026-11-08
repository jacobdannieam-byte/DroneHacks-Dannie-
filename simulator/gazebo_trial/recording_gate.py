"""Bounded startup and recorder checks for isolated Gazebo validation."""

import json
import os
from pathlib import Path
import sqlite3
import sys
from time import monotonic, sleep


def startup_ready(clock, publishers, required):
    return len(clock) == 2 and 0 < clock[0] < clock[1] and set(required) <= publishers


def wait_for_topics(required, seconds):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock

    rclpy.init(args=[])
    node = rclpy.create_node("gazebo_recording_startup")
    clock = []

    def receive(message):
        ns = message.clock.sec * 10**9 + message.clock.nanosec
        if not clock or ns < clock[-1]:
            clock[:] = [ns]
        elif ns > clock[-1]:
            clock[:] = [clock[-1], ns]

    node.create_subscription(Clock, "/clock", receive, qos_profile_sensor_data)
    deadline = monotonic() + seconds
    try:
        while rclpy.ok() and monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            publishers = {name for name in required if node.count_publishers(name)}
            if startup_ready(clock, publishers, required):
                return True
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return False


def bag_snapshot(path, required, *, finalized=False, minimum_duration_ns=0):
    bag = path / f"{path.name}_0.db3"
    if finalized and not (path / "metadata.yaml").is_file():
        return {"passed": False, "issue": "bag metadata missing"}
    try:
        with sqlite3.connect(bag.resolve().as_uri() + "?mode=ro", uri=True, timeout=.3) as db:
            if not finalized and db.execute("pragma journal_mode").fetchone()[0] != "wal":
                return {"passed": False, "issue": "recorder is not using WAL"}
            rows = db.execute("select t.name,count(m.id),min(m.timestamp),max(m.timestamp) "
                              "from topics t join messages m on t.id=m.topic_id group by t.id")
            topics = {name: {"count": count, "first_ns": first, "last_ns": last}
                      for name, count, first, last in rows}
            integrity = db.execute("pragma quick_check").fetchone()[0] if finalized else None
    except sqlite3.Error as error:
        return {"passed": False, "issue": str(error)}
    clock = topics.get("/clock", {})
    duration_ns = clock.get("last_ns", 0) - clock.get("first_ns", 0)
    missing = []
    for name in required:
        topic = topics.get(name)
        if topic is None or topic["first_ns"] <= 0:
            missing.append(name)
        elif name != "/tf_static" and (
                topic["count"] < 2 or topic["first_ns"] == topic["last_ns"]
                or (finalized and clock.get("last_ns", 0) - topic["last_ns"] > 2_000_000_000)):
            missing.append(name)
    return {"passed": not missing and duration_ns > 0 and duration_ns >= minimum_duration_ns
            and (not finalized or integrity == "ok"),
            "missing_or_not_advancing": sorted(missing), "clock_duration_ns": duration_ns,
            "minimum_duration_ns": minimum_duration_ns, "integrity": integrity, "topics": topics}


def wait_for_bag(path, required, seconds, recorder_pid):
    deadline = monotonic() + seconds
    result = {"passed": False, "issue": "recording deadline expired"}
    while monotonic() < deadline:
        try:
            os.kill(recorder_pid, 0)
        except ProcessLookupError:
            return {**result, "passed": False, "issue": "recorder exited"}
        if path.is_dir():
            result = bag_snapshot(path, required)
            if result["passed"]:
                return result
        sleep(.25)
    return result


def main():
    operation, argument, output, *required = sys.argv[1:]
    if operation == "startup":
        result = {"passed": wait_for_topics(required, int(argument))}
    elif operation == "verify":
        result = bag_snapshot(Path(argument), required, finalized=True,
                              minimum_duration_ns=int(os.environ.get("MIN_BAG_DURATION_NS", "0")))
    elif operation == "writes":
        result = wait_for_bag(Path(argument), required, 25, int(os.environ["BAG_PID"]))
    else:
        raise SystemExit(f"Unknown recording check {operation}")
    Path(output).write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
