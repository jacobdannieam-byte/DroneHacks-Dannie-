/* Original illustrative SVG scenes. Each function draws one explanatory concept.
   Coordinates are design pixels chosen for the 1400 × 440 scene, never flight data. */
window.DeckVisuals = (() => {
  const colors = {
    blue: "#6ebce8",
    green: "#8fcead",
    amber: "#e4bf7b",
    violet: "#b6a1dd",
    red: "#dd8e88",
    muted: "#9ca9b6",
    line: "#34414d",
  };
  function text(x, y, label, className = "") {
    return `<text x="${x}" y="${y}" class="${className}">${label}</text>`;
  }
  function line(x1, y1, x2, y2, color = "line", extra = "") {
    return `<line x1="${x1}" y1="${y1}" x2="${x2}" y2="${y2}" stroke="${colors[color]}" stroke-width="2" ${extra}/>`;
  }
  function fragment(index, content) {
    return `<g class="fragment" data-fragment-index="${index}">${content}</g>`;
  }
  function arrow(x1, y1, x2, y2, color = "muted") {
    return (
      line(x1, y1, x2, y2, color) +
      `<path d="M${x2 - 9} ${y2 - 5}L${x2} ${y2}L${x2 - 9} ${y2 + 5}" fill="none" stroke="${colors[color]}" stroke-width="2"/>`
    );
  }
  function frame(content, label) {
    return `<svg viewBox="0 0 1400 440" role="img" aria-label="${label}"><defs><pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse"><path d="M40 0H0V40" fill="none" stroke="#26303a" stroke-width=".7"/></pattern></defs>${content}</svg>`;
  }
  function drone(x, y) {
    return `<g transform="translate(${x} ${y})" stroke="${colors.blue}" stroke-width="3" fill="#101318"><path d="M-22-22L22 22M22-22L-22 22"/>${[-25, 25].flatMap((dx) => [-25, 25].map((dy) => `<circle cx="${dx}" cy="${dy}" r="15"/>`)).join("")}<circle r="12" fill="${colors.blue}"/><path d="M15 0L40 0"/></g>`;
  }
  function course(memory = false) {
    return frame(
      `<rect x="40" y="15" width="800" height="380" fill="url(#grid)"/>
      <path d="M95 85H425V165M425 285V355H95M95 85V355M590 85H775V355H590V285M590 165V85" fill="none" stroke="#60717f" stroke-width="5"/>
      <g class="camera-fan"><path d="M240 230L580 100L580 350Z" fill="${colors.blue}" fill-opacity=".08" stroke="${colors.blue}" stroke-opacity=".3" stroke-width="1"/></g>${drone(240, 230)}
      ${text(145, 400, "current observation", "small")}${fragment(0, `<path d="M425 170V280" stroke="${colors.amber}" stroke-width="4" stroke-dasharray="7 6"/>${text(400, 145, "?", "math amber")}`)}
      ${fragment(1, `<path d="M585 220H760" stroke="${colors.amber}" stroke-width="2" stroke-dasharray="7 8"/>${text(640, 203, "unknown", "small amber")}`)}
      ${text(900, 90, memory ? "Memory ≠ a fresh view" : "What should happen next?", "math")}
      ${fragment(0, text(900, 155, memory ? "opening_0003" : "A passage candidate", "blue"))}
      ${fragment(1, text(900, 205, memory ? "same entity, new viewpoint" : "What lies beyond it?", "small"))}
      ${fragment(2, `${text(900, 265, memory ? "last seen → aging" : "Observe before committing.", "amber")}${text(900, 315, memory ? "identity persists · certainty does not" : "geometry + memory + intent", "small")}`)}`,
      "Conceptual course and partial field of view. Unknown space is not assumed clear.",
    );
  }
  function opening() {
    return frame(
      `<rect x="65" y="65" width="190" height="275" fill="#1d2934" stroke="#60717f" stroke-width="2"/><rect x="470" y="65" width="190" height="275" fill="#1d2934" stroke="#60717f" stroke-width="2"/>
    <path d="M255 65H470M255 340H470" stroke="${colors.amber}" stroke-width="2" stroke-dasharray="8 9"/>
    ${text(80, 390, "Geometry observed in depth", "small")}${text(790, 70, "opening_0003", "math blue")}
    ${fragment(0, `${line(265, 220, 460, 220, "blue")}${line(265, 207, 265, 233, "blue")}${line(460, 207, 460, 233, "blue")}${text(318, 205, "width", "blue")}${text(790, 132, "Geometry", "small")}${text(1015, 132, "width observed", "blue")}`)}
    ${fragment(1, `${line(695, 75, 695, 335, "amber", 'stroke-dasharray="6 7"')}${text(705, 215, "?", "math amber")}${text(790, 191, "Missing evidence", "small")}${text(1015, 191, "height · connectivity", "amber")}`)}
    ${fragment(2, `${text(790, 250, "Provenance", "small")}${text(1015, 250, "frame + time + identity", "green")}${line(790, 285, 1300, 285)}${text(790, 335, "candidate passage", "math violet")}${text(790, 375, "A hypothesis we can update or reject.", "small")}`)}`,
      "A gap with observed width, unknown height and connectivity, and observation provenance.",
    );
  }
  function papers() {
    return frame(
      `${text(55, 45, "REPRESENTATION", "tiny")}${text(55, 95, "Hydra", "big blue")}${text(55, 130, "incremental geometry → places", "small")}${text(55, 235, "Kimera", "big violet")}${text(55, 270, "metric + semantic structure", "small")}
    ${fragment(0, `${arrow(390, 110, 630, 175, "blue")}${arrow(390, 250, 630, 195, "violet")}<circle cx="780" cy="185" r="94" fill="none" stroke="${colors.green}" stroke-width="2"/>${text(715, 175, "Compact", "green")}${text(712, 210, "WorldState", "green")}`)}
    ${fragment(1, `${text(1010, 85, "CSIRO Data61", "blue")}${text(1010, 120, "deployment &amp; recovery", "small")}${text(1010, 245, "ModalAI", "violet")}${text(1010, 280, "separated interfaces", "small")}${line(1000, 145, 895, 175, "muted")}${line(1000, 270, 895, 205, "muted")}`)}
    ${fragment(2, `${line(780, 282, 780, 340, "green")}${text(420, 390, "Adapt the ideas. Measure our own system.", "math")}`)}`,
      "Hydra and Kimera inform compact world state, constrained by CSIRO deployment lessons and ModalAI interfaces.",
    );
  }
  function reasoning() {
    return frame(
      `${text(65, 65, "WHAT WE KNOW", "tiny")}${text(65, 125, "opening_0003", "math blue")}${text(65, 180, "width observed", "small")}${text(65, 220, "height unknown", "small amber")}${text(65, 260, "last observation retained", "small")}
    ${fragment(0, `${arrow(345, 180, 485, 180, "blue")}${text(510, 125, "Bounded context", "violet")}${text(510, 180, "relevant entities", "small")}${text(510, 220, "evidence + missing facts", "small")}`)}
    ${fragment(1, `${arrow(790, 180, 920, 180, "violet")}${text(955, 125, "Goal proposal", "violet")}${text(955, 185, "observe_more", "math green")}${text(955, 235, "target: opening_0003", "small")}`)}
    ${fragment(2, `<path d="M1110 275V335H540" fill="none" stroke="${colors.green}" stroke-width="2" class="draw"/>${text(590, 380, "Validate identity, freshness and grounding.", "green")}${text(590, 415, "A valid proposal still needs motion evaluation.", "small")}`)}`,
      "World state becomes bounded context, an observe-more proposal and independent validation.",
    );
  }
  function architecture() {
    const node = (x, y, title, detail, color) =>
      `${line(x, y - 22, x + 235, y - 22, color)}${text(x, y + 15, title, color)}${text(x, y + 51, detail, "small")}`;
    return frame(
      `${node(25, 80, "Observe", "depth + pose + capture time", "blue")}
      ${fragment(0, `${arrow(275, 85, 355, 85, "blue")}${node(380, 80, "Retain belief", "geometry + semantic state", "blue")}${text(385, 180, "Reference prior", "small")}${text(385, 215, "Current-run belief", "small")}${text(385, 250, "Fresh collision evidence", "small green")}${text(385, 285, "Semantic topology", "small violet")}`)}
      ${fragment(1, `${arrow(625, 85, 705, 85, "violet")}${node(730, 80, "Choose intent", "context → goal → validator", "violet")}${arrow(980, 85, 1060, 85, "violet")}${node(1090, 80, "Evaluate motion", "predict → swept-body check", "violet")}`)}
      ${fragment(2, `<path d="M1210 145V330H970" fill="none" stroke="${colors.green}" stroke-width="2" class="draw"/>${node(730, 330, "Validate execution", "freshness + clearance + owner", "green")}<path d="M715 335H245V150" fill="none" stroke="${colors.green}" stroke-width="2" stroke-dasharray="7 7"/>${text(275, 375, "PX4 → new observations", "green")}${text(760, 425, "Target decomposition · qualification varies by layer", "tiny")}`)}`,
      "Target layered architecture with separate persistent belief and fresh collision evidence, ending at one command owner.",
    );
  }
  function bars() {
    const entries = [
      ["G0", "Deterministic", 68, "blue"],
      ["G1", "Active perception", 260, "green"],
      ["G2 · seed 11", "Learned ranking", 259, "violet"],
      ["G2 · seed 29", "Learned ranking", 244, "violet"],
      ["G2 · seed 47", "Learned ranking", 243, "violet"],
    ];
    let content = text(375, 30, "EVIDENCE ITEMS ACQUIRED", "tiny");
    for (let tick = 0; tick <= 250; tick += 50) {
      const x = 375 + tick * 2.7;
      content +=
        line(x, 52, x, 365, "line") + text(x - 10, 395, String(tick), "tiny");
    }
    entries.forEach(([label, detail, count, color], index) => {
      const y = 85 + index * 60;
      content += fragment(
        index === 0 ? 0 : index === 1 ? 1 : 2,
        `${text(30, y, label, color)}${text(175, y, detail, "small")}<rect class="bar-length" x="375" y="${y - 23}" width="${count * 2.7}" height="29" fill="${colors[color]}"/>${text(395 + count * 2.7, y, String(count), color)}`,
      );
    });
    content +=
      text(1150, 90, "420", "big") +
      text(1150, 125, "synthetic episodes", "small") +
      text(1150, 152, "per variant", "small") +
      fragment(
        2,
        text(1150, 265, "No clear G2", "small") +
          text(1150, 295, "advantage.", "small"),
      );
    return frame(
      content,
      "Evidence acquisitions: G0 68, G1 260, and G2 seeds 259, 244, 243; 420 synthetic episodes per variant.",
    );
  }
  function disconnect() {
    return frame(
      `${text(45, 60, "EARLY P2 FIXTURE", "tiny")}${text(45, 130, "Better map pose", "big blue")}${text(45, 180, "localization error improved", "small")}
    ${fragment(0, `${line(460, 135, 650, 135, "muted", 'stroke-dasharray="7 8"')}<path d="M690 115L730 155M730 115L690 155" stroke="${colors.red}" stroke-width="3"/>${line(770, 135, 940, 135, "muted", 'stroke-dasharray="7 8"')}${text(1000, 130, "Same stops", "big red")}${text(1000, 180, "planner did not consume it", "small")}`)}
    ${fragment(1, `${text(470, 220, "The missing link was a contract.", "math amber")}`)}
    ${fragment(2, `${line(45, 275, 1340, 275)}${text(45, 335, "Bounded worker", "green")}${arrow(290, 330, 420, 330, "green")}${text(450, 335, "Diagnostic consumer", "green")}${arrow(770, 330, 890, 330, "green")}${text(925, 335, "Decision benefit: unproven", "amber")}${text(45, 390, "Later offline integration · historical progression", "small")}`)}`,
      "Improved map pose initially did not reach the planner; later diagnostic integration does not prove navigation benefit.",
    );
  }
  function failures() {
    return frame(
      `${text(30, 45, "C1 · NATIVE CALIBRATION", "tiny")}${text(30, 100, "The test never reached braking.", "math")}
    ${fragment(0, `${text(30, 170, "Translate", "blue")}${arrow(175, 165, 265, 165)}${text(295, 170, "Height gate", "amber")}${arrow(455, 165, 545, 165)}${text(575, 170, "LAND", "green")}${text(30, 220, "Measurement failed. Landing and disarm completed.", "small")}`)}
    ${line(755, 40, 755, 370)}${fragment(1, `${text(830, 45, "S0 · NATIVE COURSE ATTEMPTS", "tiny")}${text(830, 115, "Rollout ≠ native flight", "math")}${text(830, 175, "01  frame origin", "small")}${text(830, 215, "02  timestamp skew", "small")}${text(1110, 175, "03  readiness reset", "small")}${text(1110, 215, "04  scheduling", "small")}${text(830, 265, "Attempts 001–004 stayed disarmed.", "amber")}`)}
    ${fragment(2, `${text(30, 350, "→ Altitude retention", "green")}${text(830, 350, "→ Frame binding + bounded work", "green")}${text(30, 415, "Schematic event sequence · not a telemetry curve", "tiny")}`)}`,
      "C1 failed before braking and recovered by landing; four S0 attempts remained disarmed due to integration failures.",
    );
  }
  function status() {
    const column = (x, label, rows, color) =>
      `${line(x, 60, x + 390, 60, color)}${text(x, 110, label, color)}${rows.map((row, index) => text(x, 170 + index * 43, row, "small")).join("")}`;
    return frame(
      `${fragment(0, column(25, "Offline evidence", ["Persistent entities + reasoning", "Synthetic active-perception gain", "Mapping + diagnostic integration"], "blue"))}${fragment(1, column(500, "Native evidence", ["Disarmed observations", "C0 takeoff / hover / landing", "C1 recovery; measurement failed"], "green"))}${fragment(2, column(970, "Still to demonstrate", ["Integrated hybrid course success", "Matched navigation benefit", "Hardware qualification"], "amber"))}${fragment(2, `${line(25, 340, 1360, 340)}${text(25, 390, "Separate baseline", "small")}${text(290, 390, "Classical S1: one successful Gazebo course run.", "green")}`)}`,
      "Evidence levels separated into offline, native calibration, and remaining integrated navigation validation.",
    );
  }
  function roadmap() {
    return frame(
      `${text(35, 80, "01", "big blue")}${text(35, 140, "Reliable native segment", "blue")}${text(35, 185, "frames · timing · height control", "small")}${fragment(0, `${arrow(390, 130, 480, 130, "blue")}${text(515, 80, "02", "big green")}${text(515, 140, "Matched comparison", "green")}${text(515, 185, "same map, start and scoring", "small")}`)}${fragment(1, `${arrow(900, 130, 990, 130, "green")}${text(1025, 80, "03", "big violet")}${text(1025, 140, "Layer ablations", "violet")}${text(1025, 185, "memory → topology → semantics", "small")}`)}${fragment(2, `${line(35, 270, 1360, 270)}${text(160, 345, "Observations", "math blue")}${arrow(365, 335, 465, 335, "blue")}${text(500, 345, "Grounded belief", "math violet")}${arrow(790, 335, 890, 335, "violet")}${text(925, 345, "Justified actions", "math green")}${text(360, 405, "Learned extensions only when the evidence warrants them.", "small")}`)}`,
      "Next: reliable native segment, matched baseline comparison, and controlled layer ablations.",
    );
  }
  return {
    course: () => course(false),
    memory: () => course(true),
    papers,
    opening,
    reasoning,
    architecture,
    bars,
    disconnect,
    failures,
    status,
    roadmap,
  };
})();
