
"""Review preregistered Phase 8 evidence. Reads files and emits a review report."""

import argparse
from collections import Counter
from hashlib import file_digest, sha256
import itertools
import json
from pathlib import Path
import re

PROTOCOL_PATH = Path(__file__).resolve().parents[1] / "docs" / "unified_phase8_protocol.json"
HEX_256 = re.compile(r"[a-f0-9]{64}")
GATE_COUNTS = ("command_owner_violations", "unsafe_motion_ticks", "late_result_motion_ticks",
               "invalid_scene_graph_uses")
RUN_RESULTS = ("completed", "safe_stop", "yielded", "failed")
ORIN_WORKLOADS = {"stereo", "cuvslam", "mapping", "px4_ros"}


def protocol():
    raw = PROTOCOL_PATH.read_bytes().replace(b"\r\n", b"\n")
    return json.loads(raw), sha256(raw).hexdigest()


def matrix(spec):
    nominal = itertools.product(spec["layouts"], spec["seeds"], spec["arms"], ("nominal",))
    fault = itertools.product((spec["fault_layout"],), spec["seeds"], ("full_planner",), spec["faults"])
    return tuple(itertools.chain(nominal, fault))


def _hex(value, pattern=HEX_256):
    return isinstance(value, str) and pattern.fullmatch(value) is not None


def _number(value, low=0.):
    return type(value) in (int, float) and low <= value <= 1e15


def _count(value):
    return type(value) is int and 0 <= value <= 10**9


def _artifact(root, reference):
    if not isinstance(reference, dict):
        return False
    name, checksum = reference.get("path"), reference.get("sha256")
    if not isinstance(name, str) or not name or Path(name).is_absolute() or not _hex(checksum):
        return False
    try:
        path = (root / name).resolve()
        if not path.is_relative_to(root) or not path.is_file() or path.stat().st_size == 0:
            return False
        with path.open("rb") as handle:
            return file_digest(handle, "sha256").hexdigest() == checksum
    except (OSError, ValueError, RuntimeError):
        return False


def _timing(row, limits, *, deadline_probe=False):
    keys = ("p95_ms", "p99_ms", "worst_ms")
    values = [row.get(key) for key in keys]
    ticks, misses = row.get("decision_ticks"), row.get("deadline_misses")
    if (not all(_number(value) for value in values) or values != sorted(values)
            or not _count(ticks) or ticks == 0 or not _count(misses) or misses > ticks):
        return False
    if (values[2] > limits["decision_budget_ms"]) != bool(misses):
        return False
    return (deadline_probe or (values[0] <= limits["latency_p95_ms"]
                               and values[1] <= limits["latency_p99_ms"]
                               and misses / ticks <= limits["max_deadline_miss_fraction"]))


def _run_issues(row, key, spec):
    _, _, arm, fault = key
    limits = spec["thresholds"]
    result = row.get("result")
    if result not in RUN_RESULTS or any(not _count(row.get(field)) for field in ("collisions", *GATE_COUNTS)):
        return ["invalid_result_or_safety_counts"]

    issues = []
    if any(row[field] != 0 for field in GATE_COUNTS):
        issues.append("safety_gate_violation")
    if (not _number(row.get("min_clearance_m"), low=-1000.) or not _number(row.get("moving_rtf"))
            or not _number(row.get("max_state_age_ms")) or not _number(row.get("max_geometry_age_ms"))
            or not _number(row.get("mission_score")) or type(row.get("landed_disarmed")) is not bool):
        issues.append("invalid_navigation_metrics")
    if not _timing(row, limits, deadline_probe=fault == "planner_deadline" or arm != "full_planner"):
        issues.append("timing_metrics_or_budget_failed")
    if arm in ("d0_predictive", "ridge_predictive", "full_planner") and not _number(row.get("dynamics_rmse_m")):
        issues.append("missing_dynamics_error")

    if arm == "full_planner" and "invalid_navigation_metrics" not in issues:
        if row["collisions"] or row["min_clearance_m"] < limits["minimum_clearance_m"]:
            issues.append("collision_or_clearance_failed")
        if row["moving_rtf"] < limits["minimum_moving_rtf"]:
            issues.append("moving_rtf_failed")
        if (row["max_state_age_ms"] > limits["state_age_ms"]
                or row["max_geometry_age_ms"] > limits["geometry_age_ms"]):
            issues.append("motion_input_age_failed")
        if not row["landed_disarmed"]:
            issues.append("final_landed_disarmed_unconfirmed")
        if fault != "nominal":
            response = row.get("fault_response_ms")
            if (result != spec["faults"][fault] or not _number(response)
                    or response > limits["fault_response_ms"][fault]):
                issues.append("fault_outcome_or_response_failed")
            if fault == "planner_deadline" and not (_count(row.get("deadline_misses")) and row["deadline_misses"]):
                issues.append("deadline_fault_not_observed")
    return issues


def _orin_ok(row, root, build_sha, limits):
    if not isinstance(row, dict) or row.get("device") != "Jetson Orin Nano 8GB" or row.get("build_sha") != build_sha:
        return False
    workloads = row.get("workloads")
    return (isinstance(workloads, list) and all(isinstance(name, str) for name in workloads)
            and ORIN_WORKLOADS.issubset(workloads) and _timing(row, limits)
            and row.get("late_result_motion_ticks") == 0 and type(row["late_result_motion_ticks"]) is int
            and _number(row.get("peak_ram_mib"), 1.) and _number(row.get("mean_power_w"), .01)
            and all(_number(row.get(key)) and row[key] <= 100. for key in ("cpu_p95_pct", "gpu_p95_pct"))
            and _artifact(root, row.get("profile_trace")))


def review(manifest, root):
    """Check completeness and thresholds for normalized evidence before independent review."""
    spec, protocol_sha = protocol()
    expected = matrix(spec)
    expected_keys = set(expected)
    root = Path(root).resolve()
    blockers, findings, rows_by_key, scenario_hashes, policy_hashes = [], [], {}, {}, {}
    used_artifacts = set()
    kind = manifest.get("evidence_kind")
    if kind != "native_isolated_sitl":
        blockers.append("native_isolated_sitl_evidence_missing")
    if manifest.get("protocol_sha256") != protocol_sha:
        blockers.append("protocol_digest_mismatch")
    build_sha = manifest.get("build_sha")
    if not _hex(build_sha, re.compile(r"[a-f0-9]{40}")):
        blockers.append("build_sha_missing_or_invalid")
    prior = manifest.get("phase7_review")
    if (not isinstance(prior, dict) or prior.get("status") != "reviewed_pass"
            or prior.get("build_sha") != build_sha or not _artifact(root, prior.get("record"))):
        blockers.append("phase7_acceptance_evidence_missing")
    if not _orin_ok(manifest.get("orin_profile"), root, build_sha, spec["thresholds"]):
        blockers.append("orin_concurrent_profile_missing_or_out_of_budget")

    rows = manifest.get("runs", [])
    if not isinstance(rows, list) or len(rows) > 2 * len(expected):
        blockers.append("invalid_run_manifest")
        rows = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict):
            findings.append({"index": index, "issues": ["invalid_run"]})
            continue
        key = tuple(row.get(field) for field in ("layout", "seed", "arm", "fault"))
        if type(key[1]) is not int or not all(isinstance(key[i], str) for i in (0, 2, 3)) or key not in expected_keys:
            findings.append({"index": index, "issues": ["unexpected_run_key"]})
            continue
        if key in rows_by_key:
            findings.append({"run": key, "issues": ["duplicate_run"]})
            continue
        rows_by_key[key] = row
        issues = _run_issues(row, key, spec)
        if row.get("build_sha") != build_sha:
            issues.append("run_build_mismatch")
        layout, seed, arm, fault = key
        scenario, policy = row.get("scenario_sha256"), row.get("policy_sha256")
        if not _hex(scenario) or scenario_hashes.setdefault((layout, seed, fault), scenario) != scenario:
            issues.append("unmatched_scenario_digest")
        if not _hex(policy) or policy_hashes.setdefault(arm, policy) != policy:
            issues.append("unmatched_policy_digest")
        for label in ("rosbag", "ulog", "grading_trace"):
            artifact = row.get(label)
            if not _artifact(root, artifact):
                issues.append(f"missing_or_corrupt_{label}")
            elif (label, artifact["sha256"]) in used_artifacts:
                issues.append(f"reused_{label}")
            else:
                used_artifacts.add((label, artifact["sha256"]))
        if issues:
            findings.append({"run": key, "issues": issues})

    missing = [key for key in expected if key not in rows_by_key]
    if missing:
        blockers.append("trial_matrix_incomplete")
    if findings:
        blockers.append("run_evidence_or_checks_failed")
    if len(set(policy_hashes.values())) != len(policy_hashes):
        blockers.append("duplicate_ablation_policy_digests")
    summary = {}
    for arm in spec["arms"]:
        selected = [row for key, row in rows_by_key.items() if key[2:] == (arm, "nominal")]
        result_counts = Counter(row["result"] for row in selected if row.get("result") in RUN_RESULTS)
        clearance = [row["min_clearance_m"] for row in selected if _number(row.get("min_clearance_m"), -1000.)]
        latencies = [row["p99_ms"] for row in selected if _number(row.get("p99_ms"))]
        scores = [row["mission_score"] for row in selected if _number(row.get("mission_score"))]
        summary[arm] = {"recorded": len(selected), "results": dict(result_counts),
                        "collisions": sum(row["collisions"] for row in selected if _count(row.get("collisions"))),
                        "min_clearance_m": min(clearance, default=None),
                        "worst_run_p99_ms": max(latencies, default=None),
                        "mean_mission_score": sum(scores) / len(scores) if scores else None}
    nominal_trials = len(spec["layouts"]) * len(spec["seeds"])
    full_successes = summary["full_planner"]["results"].get("completed", 0)
    if full_successes / nominal_trials < spec["thresholds"]["nominal_success_fraction"]:
        blockers.append("nominal_completion_below_threshold")
    return {"status": "qualification_open" if blockers else "thresholds_met_pending_independent_review",
            "protocol_sha256": protocol_sha, "expected_runs": len(expected), "recorded_runs": len(rows_by_key),
            "blockers": blockers, "summary": summary, "missing_runs": missing, "findings": findings}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, help="Reviewed normalized manifest with paths relative to its directory")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    if args.evidence and args.evidence.stat().st_size > 2_000_000:
        parser.error("manifest exceeds the input budget")
    manifest = json.loads(args.evidence.read_text(encoding="utf-8")) if args.evidence else {"protocol_sha256": protocol()[1]}
    if not isinstance(manifest, dict):
        parser.error("manifest must be an object")
    report = review(manifest, args.evidence.parent if args.evidence else Path.cwd())
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    return int(report["status"] == "qualification_open")


if __name__ == "__main__":
    raise SystemExit(main())
