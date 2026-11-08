/* Narrative, pacing and source references. The player renders this content; no live data is read. */
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
      "Introduce Hydra and Kimera",
      "Merge their representation lessons",
      "Add deployment and interface constraints",
    ],
  },
  {
    title: "A gap becomes a hypothesis with evidence.",
    time: 55,
    kicker: "World semantics",
    body: "opening_0003 · observed width · unknown height · observation age · supporting frames",
    takeaway: "An opening label alone cannot establish traversability.",
    visual: "opening",
    sources: ["N3", "N4"],
    notes:
      "Consider a single opening. Depth suggests two surfaces and a gap. We give the candidate a stable identity and retain its geometry and provenance. Width can be observed while height and connectivity remain unknown. As observations arrive, the hypothesis can persist, become stale, or be rejected. Semantic meaning is task-specific: what might this feature let us do, and what evidence is still required?",
    builds: [
      "Draw wall geometry and gap",
      "Attach stable entity ID and measured evidence",
      "Mark height and connectivity unknown",
    ],
  },
  {
    title: "Reasoning can choose to gather more evidence.",
    time: 55,
    kicker: "From state to intent",
    body: "WorldState → bounded context → goal proposal → independent validation",
    takeaway: "“Observe this opening again” is a useful decision.",
    visual: "reasoning",
    sources: ["N4", "N6"],
    notes:
      "The first reasoning interface was deliberately bounded and inspectable. It consumes a compact context, references existing entity IDs, and emits a non-executable goal proposal. A missing measurement can lead to observe-more; blocking evidence can reject a candidate. The validator checks grounding and freshness. Later active-perception and learned rankers use the comparison architecture, but a proposal still does not directly become a motor command.",
    builds: [
      "Compress the world into task-relevant context",
      "Reveal observe-more proposal",
      "Show validator before downstream planning",
    ],
  },
  {
    title: "Each layer answers a different question.",
    time: 60,
    kicker: "Architecture",
    body: "Observe → retain belief → choose a goal → evaluate motion → validate execution → PX4",
    takeaway:
      "Persistent belief and fresh collision evidence have different jobs.",
    visual: "architecture",
    sources: ["N1", "N3", "N5", "N7"],
    notes:
      "Starting at the sensors, we align observations with their capture-time pose, retain geometric belief and semantic memory, choose a goal, evaluate predictive candidates, and pass through one command owner. Four concepts stay separate: a permitted reference map, current-run geometric belief, fresh local collision evidence, and semantic topology. An old map can support localization or route hypotheses, but it does not become a fresh sensor observation. This is the target decomposition. Implementation and native qualification differ by component.",
    builds: [
      "Reveal observation and world layers",
      "Reveal reasoning and predictive planner",
      "Reveal execution boundary and feedback loop",
    ],
  },
  {
    title: "The simplest stronger reasoner earned priority.",
    time: 55,
    kicker: "Pivot 1 · choose evidence over complexity",
    body: "Evidence acquisitions across 420 synthetic episodes per variant.",
    takeaway:
      "Active perception improved this task; learned ranking showed no clear advantage.",
    visual: "bars",
    sources: ["N6", "N7"],
    notes:
      "G0 acquired 68 evidence items. Active-perception G1 acquired 260. Three learned G2 seeds acquired 259, 244 and 243. This supported prioritizing G1 for further shadow evaluation. These are abstract synthetic information-acquisition results, not flight completion rates. The separate adaptive-dynamics work also retained the physics baseline when alternatives failed delay, heading or runtime gates. Complexity needs to earn its place through the relevant experiment.",
    builds: [
      "Show G0 68",
      "Show G1 260",
      "Show G2 seeds 259, 244, 243 with scope label",
    ],
  },
  {
    title: "A better map was not yet a better decision.",
    time: 55,
    kicker: "Pivot 2 · connect and bound the computation",
    body: "Improved pose estimate → missing planner consumption → unchanged decisions.",
    takeaway:
      "We shifted toward explicit consumer contracts and bounded asynchronous registration.",
    visual: "disconnect",
    sources: ["N5", "N8"],
    notes:
      "The P2 fixture improved localization error, but the planner initially did not consume the correction and stopped on all forty fixture cases. Registration was also expensive on the measured Windows host. This motivated a bounded asynchronous worker and explicit map-consumer contracts. Later offline work connected diagnostic consumption, with map-based control still disabled in that lineage. The bottleneck was real, but resolving a diagnostic connection did not yet demonstrate better routes.",
    builds: [
      "Show corrected pose",
      "Highlight missing consumer connection",
      "Replace with diagnostic connection and bounded worker",
    ],
  },
  {
    title: "Native execution exposed the assumptions between layers.",
    time: 65,
    kicker: "Pivot 3 · integration is an experiment",
    body: "Altitude retention. Frame origins. Timestamp alignment. Observation throughput.",
    takeaway: "Preserved failures became specific engineering work.",
    visual: "failures",
    sources: ["N7"],
    notes:
      "In the C1 calibration attempt, translation began near the lower edge of the height band. Height fell below the band before the braking trigger, so LAND completed but the braking experiment failed. Separately, the later S0 course experiments stayed disarmed: origin conversion, a small clock skew, and readiness/scheduling problems prevented execution. These were not successful policy flights. The response is explicit altitude retention, synchronized frame binding and bounded owner-path work. The latest fetched scheduling fix remains in progress.",
    builds: [
      "Show C1: translate → height gate → LAND; no brake event",
      "Show S0: offline rollout → native readiness failures",
      "Connect each failure to a concrete next change",
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
      "Reveal offline evidence",
      "Reveal native observation/calibration evidence",
      "Reveal integrated-navigation gap and separate Classical baseline",
    ],
  },
  {
    title: "Next: prove which layer improves the mission.",
    time: 55,
    kicker: "Plan and closing",
    body: "Reliable native segment → matched baseline → memory / topology / semantics ablations → learned extensions if justified",
    takeaway:
      "Our contribution is a testable path from observations to justified actions.",
    visual: "roadmap",
    sources: ["N1", "N6", "N7", "N8"],
    notes:
      "Our next step is a reliable native segment: resolve timing, frame and altitude-retention contracts, then collect complete evidence. We will compare against the protected Classical baseline under matched course, map and initial-state assumptions. Memory, topology and semantic reasoning can then be added in controlled ablations. We will measure mission progress, false-feasible choices, observation cost and latency. Learned extensions remain optional until they improve these outcomes. The journey taught us that world semantics becomes useful when its evidence reaches a timely, verifiable decision.",
    builds: [
      "Native segment",
      "Matched comparisons and ablations",
      "Generalization and optional learned extensions",
    ],
  },
];
