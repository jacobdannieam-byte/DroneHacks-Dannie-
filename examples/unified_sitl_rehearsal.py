
"""Rehearse planner-result checks on synthetic inputs without running a command owner."""

import argparse
from dataclasses import replace
import json
from pathlib import Path

from examples.unified_trajectory_replay import STAMP, VEHICLE, fixture
from refnav.core import NAV_AUTO_LAND, NAV_OFFBOARD, Output, Vehicle
from refnav.predictive_planner import plan
from refnav.sitl_review import MAX_HANDOFF_NS, ProposalReceipt, review
from refnav.trajectory_evaluation import MetricGoal


def exercise():
    state, snapshot, alignment = fixture()
    objective = MetricGoal(alignment.map, (4., .10, 1.3))
    mission = Output("go", (*state.position_m, state.yaw_rad), stream=True)
    vehicle = Vehicle(pos=state.position_m, heading=state.yaw_rad, pos_ok=True,
                      armed=True, landed=False, nav_state=NAV_OFFBOARD)
    planner_inputs = dict(now_ns=STAMP, alignment=alignment, vehicle=VEHICLE, objective=objective)

    def receipt(clock=lambda: 0., **kwargs):
        decision = plan(state, snapshot, monotonic=clock, **planner_inputs, **kwargs)
        return ProposalReceipt(decision, state, alignment, snapshot.metric, STAMP)

    fresh = receipt()
    ticks = iter((0., .026))
    late = receipt(clock=lambda: next(ticks))
    # The unknown fixture revokes metric availability without inventing a new route.
    unknown = replace(snapshot, metric=replace(snapshot.metric, available=False))
    refused = plan(state, unknown, monotonic=lambda: 0., **planner_inputs)
    arrived = plan(state, snapshot, monotonic=lambda: 0., **{**planner_inputs,
                   "objective": MetricGoal(alignment.map, (2.1, .10, 1.3))})
    reset = replace(state, reference=replace(state.reference, reference_epoch=state.reference.reference_epoch + 1))

    inputs = dict(state=state, alignment=alignment, metric=snapshot.metric, now_ns=STAMP,
                  owner_output=mission, vehicle=vehicle, owner_ready=True)
    stale_now = STAMP + 300_000_000
    stale_state = replace(state, timestamp_s=stale_now / 1e9)
    stale_alignment = replace(alignment, capture_ns=stale_now)
    scenarios = (
        ("fresh_proposal", fresh, {}, "eligible_for_offline_review"),
        ("missing_output", None, {}, "proposal_absent"),
        ("deadline_miss", late, {}, "deadline_exceeded"),
        ("cancelled", receipt(cancelled=lambda: True), {}, "cancelled"),
        ("unknown_map", replace(fresh, plan=refused, metric=unknown.metric),
         {"metric": unknown.metric}, "unavailable_or_future_geometry"),
        ("no_verified_route", replace(fresh, plan=arrived), {}, "no_verified_progress"),
        ("stale_map", replace(fresh, decision_ns=stale_now),
         {"now_ns": stale_now, "state": stale_state, "alignment": stale_alignment}, "stale_geometry"),
        ("expired_output", fresh, {"now_ns": STAMP + MAX_HANDOFF_NS}, "proposal_age_exceeded"),
        ("future_output", replace(fresh, decision_ns=STAMP + 1), {}, "proposal_age_exceeded"),
        ("state_changed", fresh, {"state": replace(state, yaw_rad=.01)}, "proposal_input_changed"),
        ("reference_reset", fresh, {"state": reset}, "frame_or_reference_mismatch"),
        ("vio_loss", fresh, {"owner_ready": False}, "owner_gate_closed"),
        ("invalid_position", fresh, {"vehicle": replace(vehicle, pos_ok=False)}, "owner_gate_closed"),
        ("operator_takeover", fresh, {"vehicle": replace(vehicle, nav_state=4)}, "offboard_authority_absent"),
        ("px4_auto_land", fresh, {"vehicle": replace(vehicle, nav_state=NAV_AUTO_LAND),
                                 "owner_output": Output("land")}, "offboard_authority_absent"),
        ("owner_waits", fresh, {"owner_output": Output("check")}, "owner_gate_closed"),
    )
    rows = []
    for name, result, overrides, expected in scenarios:
        actual = review(result, **{**inputs, **overrides})
        rows.append({"scenario": name, "expected": expected, "observed": actual, "matched": actual == expected})
    return {"evidence": "synthetic_handoff_rehearsal", "native_gate": "open",
            "checks": rows}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    report = exercise()
    with args.output.open("x", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2, allow_nan=False)
        handle.write("\n")
    return int(any(not row["matched"] for row in report["checks"]))


if __name__ == "__main__":
    raise SystemExit(main())
