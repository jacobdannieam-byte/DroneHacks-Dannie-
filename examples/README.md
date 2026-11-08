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
