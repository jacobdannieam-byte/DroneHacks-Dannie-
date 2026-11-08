# Research and claim ledger

Research cutoff: November 8, 2026. Selected relevant sources were read; this is not an exhaustive audit of the entire Notion workspace or independent reproduction of experiments.

Animation-build refresh: N6 and N7 were fetched again on November 8, 2026. Returned last-edit timestamps were 2026-11-08T10:32:00.593Z and 2026-11-08T19:28:13.292Z respectively. N7 still ends at S0 attempt 004, disarmed with a scheduling bottleneck; no later course success was present in that fetched record. Diagrams remain conceptual and the chart retains the same synthetic evidence-acquisition scope.

## Primary project sources

| ID | Source | Use |
|---|---|---|
| N1 | [Academic × Industry Synthesis](https://app.notion.com/p/411f393aa76881fa9a0eee681393ffb2) | Motivation, layered architecture, ablation plan. Historical proposal. |
| N2 | [Academic Literature](https://app.notion.com/p/411f393aa76881909d94d0c3c6029e49) | Paper selection and limits; explicitly not the original complete 25–40-paper report. |
| N3 | [Architecture Planning](https://app.notion.com/p/411f393aa768814e97a4cb94b726398c) | Geometry-grounded semantics and persistent belief. Read with later updates. |
| N4 | [Semantic Reasoning Integration](https://app.notion.com/p/411f393aa76881e78dd6d7beb6592060) | I0–I3 offline implementation, entity replay, context and proposal contracts. |
| N5 | [P2 Spatial Intelligence](https://app.notion.com/p/412f393aa768816ea9c1f05ab405503c) | Registration bottleneck, disconnected consumer, asynchronous worker, diagnostic adoption. |
| N6 | [Strategy Tracker](https://app.notion.com/p/411f393aa768816abd1bd8651fbb4a90) | G0/G1/G2 comparison, protected Classical baseline, named strategies. |
| N7 | [Hybrid Taskforce](https://app.notion.com/p/412f393aa76881ef9a80d3307bcc1f8d) | Freshest fetched native failures, calibration and experimental integration. |
| N8 | [Master Roadmap](https://app.notion.com/p/411f393aa76881c79ae2fb7f6d7d96f7) | D0 default, adaptive predictor limits, qualification sequence. |

## Research synthesis for the talk

- [Hydra, Hughes et al., RSS 2022](https://arxiv.org/abs/2201.13360): incrementally constructs hierarchical scene graphs, including metric geometry and topological places. Our adaptation is a compact task-specific hierarchy. We did not port or reproduce the full Hydra system.
- [Kimera, Rosinol et al., 2021](https://arxiv.org/abs/2101.06894): joins visual-inertial estimation and metric-semantic reconstruction with dynamic scene graphs. Our adaptation is an explicit boundary between estimation, persistent representation, and goal reasoning. Do not transfer its planning speedups to our stack.
- [CSIRO Data61, Hudson et al., 2021](https://arxiv.org/abs/2104.09053): fielded heterogeneous robots in subterranean environments. Our interpretation is to prioritize partial-observation, recovery, and deployment evidence. Different sensing and hardware; not evidence stereo works in our dark tunnel.
- [ModalAI VOXL Vision Hub](https://docs.modalai.com/voxl-vision-hub/): implementation reference for separate VIO, obstacle processing, trajectory monitoring and autopilot interfaces. Industry engineering reference, not an academic paper or drop-in integration.
- [Swift, Kaufmann et al., Nature 2023](https://www.nature.com/articles/s41586-023-06419-4): project research cites compact state and latency as lessons. Primary article could not be fetched in this session. Keep in optional bibliography with no numerical claims pending verification; it is not evidence of semantic world memory.

Hydra, Kimera and CSIRO abstracts and ModalAI documentation were checked directly. Explanatory style references: [3b1b/manim](https://github.com/3b1b/manim), [3b1b/videos](https://github.com/3b1b/videos). Final deck framework reference: [reveal.js](https://revealjs.com/).

## Evidence and pivot ledger

| Claim / pivot | Evidence and scope | Presentation treatment |
|---|---|---|
| Persistent state supports reasoning about missing information | N4 reports stable opening ID through occlusion, staleness, reobservation and blocking; 160 offline tests at that milestone | Illustrative replay; label synthetic/offline. Width does not prove height, connectivity or clearance. |
| Active perception was more promising than additional learned complexity | N7: 420 synthetic episodes per variant; G0 68 evidence acquisitions, G1 260, G2 seeds 259/244/243 | Bar chart counts with denominator and task. No navigation-success or hardware claim. G2 has no clear demonstrated advantage over G1. |
| Better map pose did not automatically improve decisions | N5/N8 historical P2 fixture: 1.62 cm versus 6.75 cm RMSE; 40/40 H2 stops; ~243 ms p95 registration; H2 initially did not consume correction | Main talk uses qualitative diagram. Appendix may use numbers with host/fixture labels. Later diagnostic consumer exists; map-driven improvement remains unproven. |
| Adaptive dynamics did not displace physics baseline | N8: ridge improved a clean shifted case but failed delay/heading or runtime gates | Keep D0 as default. Avoid depicting learning as universally better or useless. |
| C1 braking experiment failed; recovery completed | N7: translation began around 0.866 m height; below 0.8 m before 1.50 m brake trigger; LAND/disarm completed; no brake event | Calibration-controller altitude retention issue supported; no stopping envelope measured; no established PX4 defect. |
| Offline rollout did not establish native flight | N7 latest fetched entries: model rollout 23 stages; native attempts 001–004 remained disarmed | Frame origin, 4 ms clock skew, readiness/priming, duplicate depth and clock servicing exposed integration gaps. Last entry describes ongoing fix, not demonstrated resolution. |
| Classical baseline has distinct success evidence | N6: one Gazebo run, four checkpoints, pad landing/disarm, 84.3 seconds, zero touches | Optional Q&A context. Do not label physical hardware, repeated reliability, or a hybrid result. |

## Reconciliation rules

N4 opens with a historical 'not implemented' banner but ends with implemented offline I0–I3 results. N5 progresses from missing planner consumption to opt-in diagnostic H2 consumption; neither establishes map-driven route improvement. N7 supersedes earlier 'map-assisted mode is design only' entries with implementation and experimental attempts, while preserving ordinary qualification limits. Its latest fetched attempt still did not arm. Historical test counts are milestone evidence, never an aggregated current pass count.

The talk's motivation is a synthesis of recorded research, not a verbatim personal anecdote. Do not invent team emotions, dates, causal order, or failed flights. Explicitly identify conceptual animations as illustrations. Recheck N7 and the strategy tracker before final presentation because work is ongoing.
