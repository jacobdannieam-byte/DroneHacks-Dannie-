import sqlite3

from simulator.gazebo_trial.recording_gate import bag_snapshot, startup_ready


def test_delayed_startup_waits_for_both_clock_advance_and_publishers():
    required = {"/clock", "/estimator"}
    assert not startup_ready([], required, required)
    assert not startup_ready([10, 10], required, required)
    assert not startup_ready([10, 11], {"/clock"}, required)
    assert startup_ready([10, 11], required, required)


def test_discovered_but_empty_bag_fails_before_and_after_recording(tmp_path):
    bag = tmp_path / "observations"
    bag.mkdir()
    (bag / "metadata.yaml").touch()
    with sqlite3.connect(bag / "observations_0.db3") as db:
        db.execute("pragma journal_mode = WAL")
        db.execute("create table topics (id integer, name text)")
        db.execute("create table messages (id integer, topic_id integer, timestamp integer)")
        db.executemany("insert into topics values (?,?)", [(1, "/clock"), (2, "/estimator"), (3, "/tf_static")])
        required = ["/clock", "/estimator", "/tf_static"]
        assert not bag_snapshot(bag, required)["passed"]
        assert not bag_snapshot(bag, required, finalized=True)["passed"]
        db.executemany("insert into messages values (?,?,?)", [(1, 1, 10), (2, 1, 20), (3, 2, 10), (4, 3, 10)])
        db.commit()
        assert not bag_snapshot(bag, required)["passed"]
        db.execute("insert into messages values (5, 2, 20)")
        db.commit()
        assert bag_snapshot(bag, required)["passed"]
        assert bag_snapshot(bag, required, finalized=True, minimum_duration_ns=10)["passed"]
        assert not bag_snapshot(bag, required, finalized=True, minimum_duration_ns=11)["passed"]
    db.close()
