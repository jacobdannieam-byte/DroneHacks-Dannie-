# Taskforce batch 001

Supervised taskforce activated by the user's latest instruction. Agents: policy diagnosis (GPT-6 Astra low), lab hardening (GPT-6.1 Sol xhigh), independent qualification review (GPT-6 Astra low); coordinator integrates and publishes Notion. Four concurrent slots. These are actual chat subagents, not the still-unqualified repository daemon.

Source merge: e6dd62301ffbf03c186648a678f221db1a18e018 includes spatial donor 3d7818e41b56982d857ca93da66bfbaaa452a4b9. No changes to drone_code relative to donor; Classical branch untouched.

Fresh merged pre-fix suite: 604 passed in 36.56 s.
Post policy/recorder fixes: 617 passed in 33.77 s (tests/).

Fixes:
- New unchanged sensor frames no longer spend failed inspection attempts. Explicit attempt failures and stage reset still apply. Runtime completion acknowledgment remains future work.
- Generic event-topic disarm rejected. Only fresh, ordered explicit native armed-to-disarmed transition records disarm; unknown states cannot substitute for disarmed.
- Stopping analysis requires explicit recorder completeness and causal event agreement. Post-settle motion before landing rejects qualification; maximum excursion includes brake through landing command. Independently reproduced old underestimate: 0.10 m reported versus 0.44 m excursion.

Fresh S0/S2/S3/S4 smoke records: synthetic observed-volume candidate and unknown-volume refusal, no predictor fallback. Not native clearance or physical flight evidence. Each output contains strategy identity and capture/receipt/assessment stamps.

Research direction: retain S0/D0 defaults and protected S1. S2/G1 information gain is the leading candidate for shadow evaluation, based on abstract paired evidence acquisition; no physical superiority claim. S3 learned ranking and S4 physics-informed prediction need incremental benefit evidence before promotion.

Open issues:
- Stationary reobservation needs an explicit bounded observation-wait outcome rather than invented motion progress.
- Native positive full-body/stopping clearance and reviewed uncertainty remain missing.
- Recorder topic/clock mapping and independent raw evidence identity binding remain deployment-specific.
- ROS-clock stall behavior and native recovery need separate qualification.
- User's forthcoming Gazebo setup has not been identified. Docker read-only inspection found no running containers. No simulation started or vehicle moved.

Execution status: supervised agents and Notion publication used. Offline tests and finite comparison batch used. Repository unattended loops remain disabled until authenticated deployment verifier/isolation/worker qualification exist. No physical flight. Map control and background registration remain off. No blanket authorization is substituted for missing geometry or measured stopping evidence.

Notion progress: https://app.notion.com/p/412f393aa76881ef9a80d3307bcc1f8d

Fresh paired benchmark: G0 68, G1 260, G2 seeds 11/29/47 259/244/243 acquired items over 420 abstract episodes each. G1 paired gain 0.457142857 items/episode, layout bootstrap 95% interval [0.366666667,0.542976190]. New artifacts preserve prior evidence; no native/physical performance inference.

Deployment clarification and branch reconciliation
The user identified the existing pinned Gazebo implementation and requested code-integration-hybrid as delivery branch, local policy-gates-integration. This supersedes the earlier branch-routing assumption in FINAL_HYBRID_INTEGRATION_PLAN.txt. Both histories were reconciled at c08481f without resetting collaborators.
Image digest matches the supplied 32650af6...; Docker recipes are unchanged from c550386. Dependency imports passed: Gazebo 8.15.0, PX4 54f0455ffcd755534539a7cf33a09a20bf71d29d, px4_msgs 392e831c1f659429ca83902e66820d7094591410. Runtime source is an immutable c08481f archive. No rebuild required for unchanged base dependencies; project packages rebuild inside the fresh container.
Private ROS timing, authority and recorder probes passed on the final candidate. Initial harness PYTHONPATH error is preserved; retry kept sourced ROS dependencies. Recorder probe now uses strictly increasing native status timestamps, matching the corrected monotonicity contract; old fixture is preserved.
Lab hardening: 29 tests and 25 subtests passed. External approval verification is now mandatory; exact worker plan/job/reviewer binding, sanitized environment, ignored-file dirty-source rejection, active lease cancellation and structured unverified worker reports are enforced. Production verifier and OS containment remain unprovisioned; this taskforce used supervised chat agents.
Fresh isolated disarmed observation passed; recording coverage 40.616 s, SQLite integrity ok, no missing/nonadvancing required streams. Healthy, landed, disarmed; empty motion-command graph. No arming or movement. Raw evidence manifest under artifacts/taskforce_batch_001/native/manifest.json.
