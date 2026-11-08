# Agent lab: reconstructed offline staging

Built from the supplied handoff, not recovered from the unavailable 41-file package.
All execution is disabled. No agents, autonomous loops, simulations or flights were started.

## Capability matrix

| Capability | Status |
|---|---|
| Eleven role contracts | TOML audited; read-only; not runtime-qualified |
| Experiment and repair queues | SQLite persistence, deduplication, rejection |
| Offline job lifecycle | Separate reviewer, atomic claims, token fencing, bounded retry, cancellation, expiry recovery |
| Manifest and policy-path checks | Offline validation only; not OS-level isolation |
| Failure triage | Deterministic evidence classification |
| Scoring | Validated engineering metrics; official score always null |
| Performance summaries | Finite input validation and interpolated percentiles |
| Codex/Gazebo/Notion/physical execution | Explicitly disabled adapters; not implemented live connectors |
| Worktree execution and permission enforcement | Not implemented |
| ROS bag/referee ingestion and official scoring | Not implemented |
| Live integration and Orin performance | Not qualified |

The SQLite reviewer identity is caller-supplied. This is a workflow contract, not authenticated human approval. Offline leases do not launch processes. There is no worker service, background watchdog or production scheduler. The example JavaScript deliberately exits 2. Do not enable execution by changing JSON flags: the adapters still refuse.

## Validation

- Python 3.11.9; Node v24.11.0.
- 12 new unittest cases passed (including parameterized adverse inputs).
- Existing repository regression suite: 452 passed in 30.93 seconds.
- Python compileall passed.
- Doctor: STAGING_READY_OFFLINE_ONLY; 11 roles; execution_available=false.
- Disabled JavaScript example refused execution.

These are offline tests, not simulated or physical flights. Existing flight, estimator, safety and grader files were not edited. The handoff's protected S1 run is run-AdMalPxA at ad31ae55ebf7cd3b2eaf57917537242831e3c752: 84.3 seconds, four checkpoints, zero recorded touches, landed/disarmed. This is supplied historical evidence, not remeasured here. The user's physically tested Classical baseline remains protected separately.

## Use

Run `python -m agentlab doctor` or `python -m unittest discover -s agentlab/tests -v` from the repository root. CLI help lists staging commands. Use an explicit local --db path for queue operations. Do not commit local SQLite files or credentials.

## Separate activation procedure (not authorized or performed)

1. Implement and independently review real Codex SDK, immutable worktree, OS sandbox, executable allowlist and authenticated approval adapters. Pin and verify supported SDK/config versions before installing runtime configuration.
2. Bind approvals to exact source, manifest, permitted paths, budgets and evidence digests. Enforce reviewer separation outside caller-supplied strings.
3. Connect the existing Gazebo harness without introducing a second autopilot/command owner. Add independent referee and bag ingestion; keep truth out of policy inputs.
4. Implement official scoring against the approved rule revision and test reference cases. Authenticate Notion separately and review publication destinations.
5. Qualify cancellation, process termination, crash recovery, concurrent isolation and immutable artifacts using non-moving integration tests.
6. Obtain separate authorization for a bounded simulator trial. Verify current readiness and record all outcomes. Hardware authorization and qualification remain separate.

No .codex/config.toml was installed: the handoff's claimed agents.enabled setting has not been verified as a supported runtime switch. The package's own configuration and non-executable adapters enforce offline mode; role files alone grant no execution authority.
