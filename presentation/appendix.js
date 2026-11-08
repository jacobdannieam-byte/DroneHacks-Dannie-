/* Evidence-backed Q&A content. These slides are selected from a menu rather than
   appended to the normal ten-minute forward path. See RESEARCH.md for provenance. */
window.appendixSlides = [
  {
    menu: "Why world semantics?",
    title: "A map can say where. Semantics asks what next.",
    sources: ["N1", "N3", "N4"],
    content: `<div><h3>Three representations</h3><ul><li><b>Geometry:</b> surfaces, pose, observed free space and clearance.</li><li><b>Topology:</b> places and supported connections.</li><li><b>Semantics:</b> task-relevant meaning, such as a candidate passage.</li></ul></div><div><h3>Why keep memory?</h3><p>Retain identity when a feature leaves view. Track stale or contradictory evidence. Select an information-gathering goal rather than inventing a connection.</p><p>These are architectural capabilities; mission benefit still needs a matched ablation.</p></div>`,
    notes:
      "Our contribution is compact task-specific integration and evaluation. Scene graphs and hierarchical planning are established research. A valid goal proposal does not certify motion safety.",
  },
  {
    menu: "What is learned?",
    title: "Learning is a replaceable component.",
    sources: ["N6", "N8"],
    content: `<div><h3>Reasoning comparison</h3><ul><li>G0: deterministic baseline.</li><li>G1: utility / information-gain selection.</li><li>G2: compact learned candidate ranking, evaluated with three seeds.</li></ul></div><div><h3>Motion prediction</h3><p>D0 is the nominal physics default. Residual and adaptive alternatives exist offline, but clean-case improvements did not satisfy every delay, heading and runtime gate.</p><p>A learned predictor is not a learned navigation policy or a VLA controlling the aircraft.</p></div>`,
    notes:
      "Do not conflate the learned goal ranker, learned dynamics and future end-to-end policies. No G2 flight advantage is established.",
  },
  {
    menu: "Benchmark & Classical baseline",
    title: "Compare the same task, under the same conditions.",
    sources: ["N6", "N7"],
    content: `<div><h3>Reasoning benchmark</h3><p>60 held-out layouts × 7 regimes × 6 steps. 420 episodes per variant.</p><p>Evidence acquired: G0 68; G1 260; G2 259 / 244 / 243.</p><p>Abstract synthetic evidence acquisition, not course completion.</p></div><div><h3>Separate Classical S1 result</h3><p>One Gazebo run: four checkpoints, pad landing and disarm in 84.3 seconds; zero touches reported.</p><p>Run <code>run-AdMalPxA</code>, revision <code>ad31ae5</code>. Not a matched hybrid comparison or physical-hardware result.</p></div>`,
    notes:
      "G1 paired gain is reported as 0.457 items per episode, layout-bootstrap 95% interval [0.367, 0.543]. Keep it in Q&A only if asked. S1 sampled clearance is not equivalent to swept-volume bounds.",
  },
  {
    menu: "Frames, timing & clearance",
    title: "A useful fact needs a frame and a clock.",
    sources: ["N4", "N5", "N7"],
    content: `<div><h3>Data contracts</h3><ul><li>Capture time, frame, units and reset epoch.</li><li>Stable identity and supporting observations.</li><li>Bounded context and independent validation.</li><li>One command owner.</li></ul></div><div><h3>Four separate layers</h3><ul><li>Permitted reference geometry.</li><li>Persistent current-run belief.</li><li>Fresh local collision evidence.</li><li>Semantic and topological state.</li></ul><p>A prior map does not become a fresh sensor observation.</p></div>`,
    notes:
      "Map-assisted experimental clearance is separately labeled. Unknown space is not silently filled with sensor FREE evidence. Native operation and diagnostic data consumption are different claims.",
  },
  {
    menu: "Failures & next experiments",
    title: "A recovered experiment can still be a failed test.",
    sources: ["N5", "N7", "N8"],
    content: `<div><h3>Preserved outcomes</h3><p>C1 crossed its height threshold before the brake trigger. LAND and disarm completed, but no braking envelope was measured.</p><p>Latest fetched S0 attempts 001–004 stayed disarmed. Frame, timestamp and scheduling work remained.</p></div><div><h3>Acceptance sequence</h3><p>Native segment → matched baseline → memory / topology / semantics ablations → changed-layout evaluation.</p><p>Measure mission progress, false-feasible choices, observation cost and latency. Add learned complexity when these improve.</p></div>`,
    notes:
      "The historical P2 fixture reports 1.62 cm versus 6.75 cm RMSE, 40/40 H2 stops and approximately 243 ms p95 registration on Windows. Later diagnostic adoption does not establish route benefit. Current evidence cutoff November 8, 2026.",
  },
  {
    menu: "Papers & source bibliography",
    title: "The ideas we adapted.",
    sources: ["N1", "N2"],
    content: `<div><h3>Academic references</h3><p><a href="https://arxiv.org/abs/2201.13360" target="_blank" rel="noopener">Hydra · Hughes et al. · RSS 2022</a><br>Incremental hierarchical scene graphs.</p><br><p><a href="https://arxiv.org/abs/2101.06894" target="_blank" rel="noopener">Kimera · Rosinol et al. · 2021</a><br>Metric-semantic spatial representation.</p><br><p><a href="https://arxiv.org/abs/2104.09053" target="_blank" rel="noopener">CSIRO Data61 · Hudson et al. · 2021</a><br>Fielded subterranean robotic systems.</p></div><div><h3>Engineering and visual references</h3><p><a href="https://docs.modalai.com/voxl-vision-hub/" target="_blank" rel="noopener">ModalAI VOXL Vision Hub</a><br>VIO, obstacle processing and autopilot interfaces.</p><br><p><a href="https://github.com/3b1b/manim" target="_blank" rel="noopener">3Blue1Brown / Manim</a><br>Explanatory animation inspiration. All diagrams here are original SVG illustrations.</p><br><p>Methods inform our design; their published results do not validate our platform.</p></div>`,
    notes:
      "Swift remains optional background in the research ledger; its primary article was not accessible for direct verification in the research session, so this deck does not use its performance claims.",
  },
];
