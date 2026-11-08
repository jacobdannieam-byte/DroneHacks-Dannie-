# Goal reasoning

Read `schema.py`, `context.py`, `baseline.py` and `validation.py` first. They
define the existing grounded, non-executable semantic contract. G0's policy is
unchanged. Schema v1 still forbids connectivity and safe-traversable claims.

Then read `inspection.py` (bounded candidate camera viewpoints), `selection.py`
(G0/G1/G2 selection and score explanations), and `bridge.py` (independent input
identity and freshness checks). `learned.py` is optional G2 inference; training
and evaluator truth live only under `examples/`.

All inspection poses are hypotheses. The level-camera approximation uses the
estimated yaw and reviewed camera mounting; it is not a measured full attitude
or a visibility guarantee. D3 must check movement separately. Reobservation
does not imply a safe stationary hold, and no observation completes a checkpoint.

See [comparison evidence](../../docs/goal_reasoning_comparison.md) for runnable
configurations, normalization, failure behavior and unverified native gates.

The [strategy portfolio](../../docs/strategy_comparison.md) adds named, pinned
configurations and a bounded yaw hypothesis. `inspection-yaw` is reconstructed
from the validated selection and current observation, with geometric sweep
checks and angular progress separate from translation. S0 excludes this new
primitive. Experimental strategies remain shadow-only at the existing owner.
