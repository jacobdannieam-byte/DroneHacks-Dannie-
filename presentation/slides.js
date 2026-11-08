/* Slide narrative and speaker notes; no live flight data is read. */
window.slides = [
  {
    title: "The course is visible. The next action is uncertain.",
    time: 40,
    kicker: "The problem",
    body: "GPS-denied navigation through corridors, constrained passages and a landing area.",
    takeaway: "Each camera frame reveals only part of the task.",
    visual: "course",
    sources: ["N1", "N3"],
    notes:
      "Our drone has to progress through an indoor course while pose, lighting, visibility and available clearance change. Each camera image is only a partial observation. The drawing is a conceptual course, not the surveyed competition map. The question that shaped our work was: what information must survive between observations so that the next decision is grounded?",
    builds: [
      "Reveal drone and field of view",
      "Reveal unknown space beyond the opening",
      "Show mission goal beyond the visible region",
    ],
  },
  {
    title: "We chose to build what the drone remembers.",
    time: 50,
    kicker: "Our motivation",
    body: "Where am I? What have I observed? What remains unknown? What should I inspect next?",
    takeaway:
      "Prioritize a useful world representation before increasing policy complexity.",
    visual: "memory",
    sources: ["N1", "N2", "N3"],
    notes:
      "Our research led us to prioritize persistent world semantics and reasoning. A useful representation retains a passage after the camera turns away, remembers which evidence supports it, and exposes what is missing. This is the architectural motivation synthesized from the research notes, not a claim that all these capabilities already improve flight. The goal was a measurable path from observation to justified decision.",
    builds: [
      "Show current observation",
      "Turn the camera away while retaining entity identity",
      "Fade freshness without deleting memory",
    ],
  },
  {
    title: "We combined ideas at the interfaces.",
    time: 55,
    kicker: "Research → design",
    body: "Hydra: incremental hierarchy. Kimera: metric + semantic structure. CSIRO: deployment and recovery. ModalAI: separated interfaces.",
    takeaway: "A compact adaptation for our task and compute budget.",
    visual: "papers",
    sources: ["N1", "N2"],
    notes:
      "Hydra gave us the metric-to-place hierarchy. Kimera reinforced joining geometry and semantic relations in a structured representation. CSIRO provided field experience with partial observation and recovery, under different sensing assumptions. ModalAI is our industry reference for separating visual-inertial odometry, obstacle processing and autopilot interaction. We combined these design lessons; we did not run their complete systems or inherit their performance claims.",
    builds: [
      "Show the geometry and graph ideas from Hydra and Kimera converging into compact WorldState",
      "Add deployment and interface lessons",
    ],
  },
  {
    title: "A gap becomes a hypothesis with evidence.",
    time: 55,
    kicker: "World semantics",
    body: "opening_0003 · observed width · unknown height · observation age · supporting frames",
    takeaway: "WorldState remembers what we saw—and what we still do not know.",
    visual: "opening",
    sources: ["N3", "N4"],
    notes:
      "SAY: “We start with depth: how far away surfaces are. We match each depth image to the drone’s position and orientation when that image was captured, then place the points into a shared coordinate system. The geometry extractor groups those points into wall segments and identifies supported gaps as possible openings. WorldState gives each structure an identity and remembers the evidence behind it. Here, opening_0003 has an observed width and a last-seen time. Its full height and where it leads are still unknown. So the useful next decision is to get another view, not assume we can fly through.” PAUSE at WorldState to point to the known and unknown facts. PRESENTER BACKGROUND: WorldState is a bounded collection of Python entity records in memory, with optional JSON persistence. Each entity stores its ID, type, geometry, coordinate frame, observation times, confidence, status, supporting evidence, contradictions, missing evidence and supported relationships. It contains no learned model weights. A remembered entity survives the camera turning away, but its freshness ages. Reobservation updates the existing identity; actual blocking evidence can invalidate it. This animation is illustrative, not recorded sensor data. Reference geometry, remembered entities and fresh collision evidence are separate: memory alone never certifies clearance. The depth/VIO interface and mapping components have different validation levels; do not describe the diagram as proof of completed autonomous mapping in flight.",
    builds: [
      "Play the continuous observation → geometry → graph → decision scene once",
      "Pause or scrub to explain an intermediate state",
      "Hold on observe_more; motion still needs separate validation",
    ],
  },
  {
    title: "Reasoning can choose to gather more evidence.",
    time: 55,
    kicker: "From state to intent",
    body: "WorldState → bounded context → goal proposal → independent validation",
    takeaway:
      "When important information is missing, look again before committing.",
    visual: "reasoning",
    sources: ["N4", "N6"],
    notes:
      "SAY: “Take that same opening. Our memory says where it is, how wide it appears, and which observation supports it. But we have not measured the full height or established where the passage leads. The reasoner turns that missing information into a useful goal: look at this opening again. A separate motion layer still has to check how to reach that view safely.” IF ASKED: Confidence is a heuristic, not a calibrated probability. Supported entity relationships do not imply a complete navigable region graph. No observation is not the same as evidence that space is free. The reasoner reads a compact snapshot; it does not directly command the motors.",
    builds: [
      "Use the same opening graph to explain context and observe_more",
      "Reveal independent validation",
    ],
  },
  {
    title: "Remember. Decide. Fly.",
    time: 60,
    kicker: "Architecture",
    body: "World memory → next goal → motion and safety",
    takeaway: "Reasoning chooses the goal. Control turns it into movement.",
    visual: "architecture",
    sources: ["N1", "N3", "N5", "N7"],
    notes:
      "There are three jobs. First, remember what the cameras have shown: openings, obstacles and what is still unknown. Second, choose the next goal, such as getting a better view of an opening. Third, plan and follow a movement, checking current obstacle information before sending commands. New observations update the memory. This is our target architecture; the components have different levels of validation.",
    builds: ["Reveal the next goal", "Reveal motion and safety"],
  },
  {
    title: "Why DATT and physics-informed learning?",
    time: 55,
    kicker: "Research motivation",
    body: "Adapt the tracking. Improve the prediction.",
    takeaway:
      "Use learning to handle motion mismatch; keep world understanding separate.",
    visual: "learning",
    sources: ["DATT", "PINNs", "N6"],
    notes:
      "Even with the right route, the drone may respond differently than expected because of wind, delay or imperfect dynamics. DATT, Deep Adaptive Trajectory Tracking, combines a reinforcement-learned feedforward and feedback controller with disturbance adaptation. It motivates looking ahead along the desired trajectory and compensating for disturbances. Physics-informed neural networks address a different question: can a learned prediction fit measurements while respecting physical equations? Our research direction is to improve motion prediction using physics-informed residual learning. DATT remains proposed here, not an integrated model. The tracker reports an offline-tested physics-informed predictor, not flight qualification. Adding a neural residual alone does not make it a PINN.",
    builds: [
      "Reveal the DATT tracking idea",
      "Reveal the physics-informed prediction idea",
    ],
  },
  {
    title: "Our Classical baseline: plan, then follow.",
    time: 55,
    kicker: "What the Classical branch implements",
    body: "Grid search → sliding target → PX4",
    takeaway:
      "An understandable baseline lets us measure whether learning adds value.",
    visual: "classical",
    sources: ["Classical", "N6"],
    notes:
      "The Classical branch contains a clearance grid and Dijkstra shortest-path search through the required checkpoints. It penalizes tight spaces and proximity to boulders, then straightens the route while preserving clearance. A separate live-planner module uses A-star; these are distinct planners, not two names for the same algorithm. The route follower moves a target ahead of the drone, limits its lead, and waits at corners to prevent cutting them. It sends position targets to PX4, which handles low-level flight control. Height disagreement and suspected contact can trigger landing. This is source inspection at revision f4264a2, not a new flight test.",
    builds: ["Reveal the sliding target follower", "Reveal PX4 and monitoring"],
  },
  {
    title: "Native execution exposed the assumptions between layers.",
    time: 65,
    kicker: "Failures and pivots",
    body: "Altitude retention. Frame origins. Timestamp alignment. Observation throughput.",
    takeaway:
      "Fix height, shared coordinates and timing before adding more intelligence.",
    visual: "failures",
    sources: ["N7"],
    notes:
      "In the C1 calibration attempt, translation began near the lower edge of the height band. Height fell below the band before the braking trigger, so LAND completed but the braking experiment failed. Separately, the later S0 course experiments stayed disarmed: origin conversion, a small clock skew, and readiness/scheduling problems prevented execution. These were not successful policy flights. The response is explicit altitude retention, synchronized frame binding and bounded owner-path work. The latest fetched scheduling fix remains in progress.",
    builds: [
      "Show the height-gate crossing and completed landing, before the brake trigger",
      "Reveal the separate frame, clock and owner-contract findings",
    ],
  },
  {
    title: "We have components and evidence at different stages.",
    time: 55,
    kicker: "Honest status",
    body: "Offline reasoning and mapping • native observations and calibration • integrated navigation still under evaluation",
    takeaway:
      "The hybrid architecture has not yet demonstrated full-course native success in the fetched record.",
    visual: "status",
    sources: ["N4", "N5", "N6", "N7"],
    notes:
      "Our evidence lives at different levels. WorldState and reasoning have offline tests and replay. Active perception has synthetic comparison evidence. Native disarmed observations and one C0 calibration completed; C1 recovered but failed its measurement objective. The latest recorded S0 attempts did not arm. The protected Classical stack has a separate successful Gazebo course run. That is valuable baseline evidence, but it belongs to that stack. The question is which behavior was demonstrated in which environment.",
    builds: [
      "Add native calibration evidence beside offline components",
      "Show the remaining mission-level claim",
    ],
  },
  {
    title: "Next: prove which layer improves the mission.",
    time: 55,
    kicker: "Plan and closing",
    body: "Reliable native segment → matched baseline → memory / topology / semantics ablations → learned extensions if justified",
    takeaway:
      "Prove reliable flight, then measure what memory and reasoning add.",
    visual: "roadmap",
    sources: ["N1", "N6", "N7", "N8"],
    notes:
      "Our next step is a reliable native segment: resolve timing, frame and altitude-retention contracts, then collect complete evidence. We will compare against the protected Classical baseline under matched course, map and initial-state assumptions. Memory, topology and semantic reasoning can then be added in controlled ablations. We will measure mission progress, false-feasible choices, observation cost and latency. Learned extensions remain optional until they improve these outcomes. The journey taught us that world semantics becomes useful when its evidence reaches a timely, verifiable decision.",
    builds: [
      "Reveal matched comparisons and layer ablations",
      "Close the chain from observations to justified actions",
    ],
  },
];
