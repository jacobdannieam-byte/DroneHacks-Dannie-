
from hashlib import sha256

import pytest

from examples.unified_qualification import matrix, protocol, review


def artifact(root, label):
    content = label.encode()
    (root / label).write_bytes(content)
    return {"path": label, "sha256": sha256(content).hexdigest()}


@pytest.fixture
def bundle(tmp_path):
    """Unit-test placeholders exercise review logic and artifact integrity checks."""
    spec, digest = protocol()
    build = "1" * 40
    timings = {"p95_ms": 15., "p99_ms": 20., "worst_ms": 23., "decision_ticks": 500, "deadline_misses": 0}
    rows = []
    for index, (layout, seed, arm, fault) in enumerate(matrix(spec)):
        row = {"layout": layout, "seed": seed, "arm": arm, "fault": fault, "build_sha": build,
               "result": spec["faults"].get(fault, "completed"), "landed_disarmed": True,
               "collisions": 0, "min_clearance_m": .12, "moving_rtf": .96, "max_state_age_ms": 40,
               "max_geometry_age_ms": 115, "mission_score": .91, "command_owner_violations": 0,
               "unsafe_motion_ticks": 0, "late_result_motion_ticks": 0, "invalid_scene_graph_uses": 0,
               "fault_response_ms": 50., "dynamics_rmse_m": .09,
               "scenario_sha256": sha256(f"{layout}:{seed}:{fault}".encode()).hexdigest(),
               "policy_sha256": sha256(arm.encode()).hexdigest(), **timings,
               **{label: artifact(tmp_path, f"{index}-{label}") for label in ("rosbag", "ulog", "grading_trace")}}
        if fault == "planner_deadline":
            row.update(p95_ms=45., p99_ms=55., worst_ms=70., deadline_misses=40)
        rows.append(row)
    return {
        "evidence_kind": "native_isolated_sitl", "protocol_sha256": digest, "build_sha": build,
        "phase7_review": {"status": "reviewed_pass", "build_sha": build, "record": artifact(tmp_path, "phase7")},
        "orin_profile": {"device": "Jetson Orin Nano 8GB", "build_sha": build,
                         "workloads": ["stereo", "cuvslam", "mapping", "px4_ros"], **timings,
                         "late_result_motion_ticks": 0, "peak_ram_mib": 4700., "mean_power_w": 13.,
                         "cpu_p95_pct": 70., "gpu_p95_pct": 82., "profile_trace": artifact(tmp_path, "profile")},
        "runs": rows,
    }


def test_complete_matrix_is_reviewable_and_missing_or_synthetic_evidence_is_blocked(bundle, tmp_path):
    report = review(bundle, tmp_path)
    assert report["status"] == "thresholds_met_pending_independent_review"
    assert report["recorded_runs"] == report["expected_runs"] == 100
    bundle["evidence_kind"] = "synthetic"
    bundle["runs"].pop()
    report = review(bundle, tmp_path)
    assert report["status"] == "qualification_open"
    assert {"native_isolated_sitl_evidence_missing", "trial_matrix_incomplete"}.issubset(report["blockers"])


def test_corrupt_artifact_unpaired_seed_and_duplicate_run_are_reported(bundle, tmp_path):
    row = bundle["runs"][1]
    row["scenario_sha256"] = "3" * 64
    (tmp_path / row["rosbag"]["path"]).write_text("bad hash", encoding="utf-8")
    bundle["runs"].append(row)
    report = review(bundle, tmp_path)
    issues = {issue for finding in report["findings"] for issue in finding["issues"]}
    assert {"missing_or_corrupt_rosbag", "unmatched_scenario_digest", "duplicate_run"} <= issues


def test_reused_paths_and_scope_escape_are_rejected(bundle, tmp_path):
    first, second = bundle["runs"][:2]
    second["rosbag"] = first["rosbag"]
    second["ulog"]["path"] = "../not-part-of-bundle"
    report = review(bundle, tmp_path)
    assert set(report["findings"][0]["issues"]) == {"reused_rosbag", "missing_or_corrupt_ulog"}


def test_collision_late_motion_and_missing_deadline_injection_fail(bundle, tmp_path):
    collision = next(row for row in bundle["runs"] if row["arm"] == "full_planner" and row["fault"] == "nominal")
    collision.update(collisions=1, min_clearance_m=-.01, late_result_motion_ticks=1)
    fault = next(row for row in bundle["runs"] if row["fault"] == "planner_deadline")
    fault.update(deadline_misses=0, worst_ms=25., p99_ms=24., p95_ms=20.)
    report = review(bundle, tmp_path)
    issues = {issue for finding in report["findings"] for issue in finding["issues"]}
    assert {"collision_or_clearance_failed", "safety_gate_violation", "deadline_fault_not_observed"} <= issues


def test_failure_stays_in_denominator_and_orin_concurrency_is_required(bundle, tmp_path):
    runs = [row for row in bundle["runs"] if row["arm"] == "full_planner" and row["fault"] == "nominal"]
    for row in runs[:2]:
        row["result"] = "failed"
    bundle["orin_profile"]["workloads"].remove("mapping")
    report = review(bundle, tmp_path)
    assert {"nominal_completion_below_threshold", "orin_concurrent_profile_missing_or_out_of_budget"} <= set(
        report["blockers"])
    assert report["summary"]["full_planner"]["results"] == {"failed": 2, "completed": 13}
