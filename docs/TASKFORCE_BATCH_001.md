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
