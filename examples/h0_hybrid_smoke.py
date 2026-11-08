"""Run the read-only H0 contract on the existing rendered-corridor fixture."""

from dataclasses import asdict, replace
import json

from examples.unified_trajectory_replay import STAMP, VEHICLE, fixture
from refnav.hybrid_integration import assess
from refnav.reasoning import DeterministicReasoner, WorldReference, build_context
from refnav.reasoning.schema import MissionGoal
from refnav.trajectory_evaluation import GoalBinding, MetricGoal, state_capture_ns
from refnav.reasoning.selection import create_reasoner
from refnav.reasoning.inspection import make_inputs
from examples.unified_spatial_replay import calibration


def smoke_inputs():
    state, snapshot, alignment = fixture()
    reference = alignment.map
    context = build_context(snapshot.semantic, MissionGoal(),
                            now=snapshot.semantic["query_timestamp"],
                            reference=WorldReference(reference.frame_id, reference.epoch, reference.clock_id))
    auxiliary = make_inputs(context, snapshot, state, alignment, calibration())
    proposal, selection = create_reasoner().select(context, auxiliary)
    goal = GoalBinding(context, proposal, selection)
    return dict(state=state, snapshot=snapshot, now_ns=STAMP, vehicle=VEHICLE,
                alignment=replace(alignment, estimator_reset_counter=0),
                objective=MetricGoal(reference, (4., .10, 1.3)), goal=goal,
                state_received_ns=STAMP, estimator_reset_counter=0, estimator_healthy=True,
                monotonic=lambda: 0.)


def main():
    result = assess(**smoke_inputs())
    receipt = result.receipt
    print(json.dumps({"evidence": "synthetic_h0", "validation_reason": result.validation_reason,
                      "state_reference": asdict(receipt.state.reference),
                      "state_capture_ns": state_capture_ns(receipt.state, receipt.alignment),
                      "state_received_ns": receipt.state_received_ns,
                      "depth_received_ns": receipt.metric.received_ns,
                      "decision_ns": receipt.decision_ns, "vehicle_bounds": asdict(result.vehicle_bounds),
                      "estimator_reset_counter": receipt.estimator_reset_counter,
                      "plan": asdict(receipt.plan)}, indent=2, allow_nan=False))
    evaluation = receipt.plan.evaluation
    return int(result.validation_reason != "eligible_for_offline_review"
               or evaluation is None or evaluation.goal_proposal_id != result.goal.proposal.proposal_id)


if __name__ == "__main__":
    raise SystemExit(main())
