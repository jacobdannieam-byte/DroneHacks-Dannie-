# Agent lab integration handoff

## Implemented transport code; disabled deployment

All checked-in execution flags remain false. No agents, autonomous loops, simulations, Notion writes or physical flights ran during implementation. Classical and existing flight/safety/grader sources are unchanged.

| Component | Implemented | Offline validation |
|---|---|---|
| Codex worker | Real `codex exec --json --sandbox read-only` transport, role contract, source verification, terminal-event validation | Fake process transcripts; no Codex invocation |
| Worktrees | Git detached worktree creation at immutable SHA; identifier checks; preserve existing worktrees | Command contract tested with fake Git runner |
| Job lifecycle | Durable review, atomic lease, expiry recovery, bounded retries, cancellation, stale-token rejection | SQLite tests |
| One-shot pipeline | Approval before worktree/claim, source binding, immutable result file, completion fencing | Source inspected; full live workflow unqualified |
| Gazebo | Existing examples/gazebo_container.py fly adapter; source/world hash checks; existing harness approval retained | Default execution denial; native runtime not invoked |
| Referee | Independent summary hash, required fields and conservative completion predicate | Fixture ingestion |
| ROS bags | Read-only SQLite topic/count/time ingestion and artifact hash | Synthetic SQLite bag; explicitly does not verify CDR telemetry |
| Notion | Bounded append-block request, page UUID, report-bound approval, external token, no automatic retries | Fake HTTP transport; no publication |
| Physical flight | Unconditionally unavailable | Denial test |

## Scope and limitations

Workers are real CLI integrations rather than an SDK scheduler. Every worker remains read-only and returns patch proposals as data, including implementer and repairer roles. Applying those patches requires a separately reviewed implementation; no write-capable agent or automatic patch executor is provided. There is no autonomous dispatch loop.

The approval object is an operator-supplied record bound to a payload hash and expiry, not an authenticated identity service. It must only be constructed by a trusted controller; do not expose it to worker output or untrusted API callers. Checked-in staged config cannot authorize it. Runtime configuration requires mode=reviewed_execution plus a capability flag and matching approval; none were enabled here.

Process execution is Linux-only with a bounded timeout and process-group kill. Windows live execution refuses. Durable cancellation fences completion, but does not currently signal an already running process; timeout terminates it. Git worktrees isolate changes, not host security. Production use requires OS isolation and host-level credential separation. Do not give workers simulator credentials or Notion tokens. The current runner inherits its environment, so deployment must provide a sanitized environment.

Referee hashes identify artifacts but do not authenticate the producer. Trust requires an independently controlled artifact directory. Bag metadata is not a substitute for deserializing fresh landed/disarmed/estimator messages. Official competition scoring is not implemented; official_score remains null.

## Validation

18 lab tests passed. These exercise fake process/HTTP transports, SQLite lifecycle, config denials, immutable identities, referee and bag fixtures. Existing repository regression suite is run separately. Python compilation and Git whitespace checks are included. No live qualification is claimed.

## Activation procedure (not performed)

1. Review source and frozen job manifest, exact input artifacts and role output; supply authenticated approvals outside worker-controlled files.
2. Qualify Codex CLI version/authentication and flags on a dedicated Linux runner. Use a sanitized environment and OS isolation. Test cancellation/process cleanup before granting any live capability.
3. Qualify worktree creation and cleanup on a disposable repository; preserve failed trees and immutable outputs. Never share a mutable source checkout across workers.
4. Qualify the Gazebo adapter inside the existing isolated container; do not start extra PX4/XRCE processes. Bind profile and existing harness approval to the source/world and separate mission authorization.
5. Add ROS message deserialization and validate independent referee provenance before accepting a run as successful.
6. Configure a narrowly scoped Notion token outside Git. Review the destination and exact report. On uncertain publication, reconcile remote blocks before retrying.
7. Authorize only a bounded one-shot operation. Continuous loops and hardware stay disabled. Physical flight is outside this lab adapter.

## References

- https://developers.openai.com/codex/noninteractive (JSONL events, read-only sandbox, stdin prompt)
- https://developers.notion.com/reference/patch-block-children (append-block API)

This document supersedes the earlier placeholder-only capability matrix. It describes executable integration code and its remaining deployment/qualification boundaries, not production readiness.
