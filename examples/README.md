# Offline examples and observation adapters

This folder contains historical offline benchmarks and read-only observation
adapters, plus explicitly guarded runtime entrypoints. Reading an entrypoint is
not permission to start a flight. Use the repository's existing mission review
rules for any command-producing runtime.

For the goal comparison, read `goal_synthetic.py`, `train_goal_ranker.py`, then
`compare_goal_reasoners.py`. These operate only on synthetic evidence. Run
`compare_goal_shadow.py` for identical synthetic message streams through H1/H2.
`h0_hybrid_smoke.py` checks an observed-corridor positive planning case.

Lower-layer experiments are separate: `compare_physics_residual.py` reuses D1
training and A1 replay; `diagnostic_mpc.py` and `compare_diagnostic_mpc.py` produce
optimizer diagnostics that the command gateway cannot accept.

Exact commands, evidence limits and the relation to the unified phases are in
[the comparison report](../docs/goal_reasoning_comparison.md).

`configure_strategy.py` composes a reviewed H1 profile with one pinned strategy.
`compare_strategy_predictors.py` holds S2 and D5 fixed to measure predictor
decision impact. `strategy_comparison.py` compares external run records and
refuses missing or mismatched conditions. `strategy_audit.py` only reads the
protected baseline. `recorded_native_inventory.py` is the prior evidence
inventory reader reused from `physical-gazebo-evidence/native_bag_summary.py`;
its wrapper mounts the saved bag read-only in a separate container with no ROS
network. See [the portfolio runbook](../docs/strategy_comparison.md).
