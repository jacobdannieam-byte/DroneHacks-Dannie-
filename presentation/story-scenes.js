/* Consistent story diagrams built from the drone, aperture and evidence graph.
   These are illustrative design-pixel drawings, not maps or telemetry. Scenes
   use at most two speaker-controlled reveals; no new autoplay is introduced. */
window.StoryScenes = (() => {
  const colors = {
    blue: "#6ebce8",
    green: "#8fcead",
    amber: "#e4bf7b",
    violet: "#b6a1dd",
    red: "#dd8e88",
    muted: "#9ca9b6",
    line: "#34414d",
  };
  function text(x, y, value, style = "") {
    return `<text x="${x}" y="${y}" class="${style}">${value}</text>`;
  }
  function path(shape, color = "line", extra = "") {
    return `<path d="${shape}" fill="none" stroke="${colors[color]}" stroke-width="2" ${extra}/>`;
  }
  function circle(x, y, radius, color = "blue", extra = "") {
    return `<circle cx="${x}" cy="${y}" r="${radius}" fill="#101318" stroke="${colors[color]}" stroke-width="2" ${extra}/>`;
  }
  function reveal(index, content) {
    return `<g class="fragment" data-fragment-index="${index}">${content}</g>`;
  }
  function svg(content, label) {
    return `<svg viewBox="0 0 1400 440" role="img" aria-label="${label}">${content}</svg>`;
  }
  function arrow(x, y, end, color = "muted") {
    return path(`M${x} ${y}H${end}m-8-5l8 5-8 5`, color);
  }
  function drone(x, y, scale = 1) {
    return `<g transform="translate(${x} ${y}) scale(${scale})">${path("M-23-23L23 23M23-23L-23 23", "blue")}${circle(-25, -25, 14)}${circle(25, -25, 14)}${circle(-25, 25, 14)}${circle(25, 25, 14)}<circle r="9" fill="${colors.blue}"/></g>`;
  }
  function aperture(x, y, scale = 1) {
    let points = "";
    for (let index = 0; index < 12; index++) {
      const pointX = index < 6 ? -95 + index * 10 : 45 + (index - 6) * 10;
      points += `<circle cx="${pointX}" cy="${Math.sin(index * 2) * 4}" r="2" fill="${colors.blue}"/>`;
    }
    return `<g transform="translate(${x} ${y}) scale(${scale})">${path("M-100-25V0H-45M45 0H100V-25", "blue")}${points}${path("M-42 0H42", "amber", 'stroke-dasharray="5 6"')}${path("M-43 15V25M43 15V25M-43 20H43", "blue")}</g>`;
  }
  function graph(x, y, scale = 1) {
    return `<g transform="translate(${x} ${y}) scale(${scale})">${path("M-70-45L0 0L70-45", "violet")}${path("M-65 65Q-100 0-20 0", "green")}${path("M65 65Q100 0 20 0", "amber", 'stroke-dasharray="4 5"')}${circle(-70, -45, 12)}${circle(70, -45, 12)}${circle(0, 0, 20, "violet")}${circle(-65, 65, 9, "green")}${circle(65, 65, 12, "amber", 'stroke-dasharray="3 4"')}${text(60, 72, "?", "small amber")}</g>`;
  }
  function papers() {
    return svg(
      `${text(40, 42, "RESEARCH → ADAPTATION", "tiny")}
    ${text(40, 110, "Hydra", "big blue")}${text(40, 148, "incremental spatial hierarchy", "small")}${aperture(170, 215, 0.9)}
    ${text(430, 110, "Kimera", "big violet")}${text(430, 148, "metric + semantic structure", "small")}${graph(550, 233, 0.62)}
    ${reveal(0, `${path("M285 220H330V340H815V220H865", "blue")}${path("M550 295V340", "violet")}${graph(1010, 220, 1.1)}${text(900, 115, "Compact WorldState", "green")}${text(890, 340, "geometry + identity + evidence", "small")}`)}
    ${reveal(1, `${text(40, 402, "CSIRO: recovery in partial observation", "small")}${text(610, 402, "ModalAI: separate perception, planning and autopilot interfaces", "small")}`)}`,
      "Hydra geometry and Kimera graph structure combine into compact evidence-grounded world state; deployment lessons constrain the adaptation.",
    );
  }
  function reasoning() {
    return svg(
      `${graph(165, 170, 1.1)}${text(55, 305, "opening_0003", "math violet")}${text(55, 350, "height remains unknown", "small amber")}
    ${arrow(290, 170, 385, "violet")}${text(420, 90, "Bound the context", "blue")}${path("M430 120H680M430 250H680", "line")}${text(440, 155, "entity + evidence", "small")}${text(440, 195, "frame + time + missing facts", "small")}${text(440, 235, "no invented connectivity", "small amber")}
    ${reveal(0, `${arrow(710, 170, 810, "violet")}${text(845, 90, "Choose the next question", "violet")}${text(845, 165, "observe_more", "big green")}${text(845, 215, "target: opening_0003", "small violet")}${text(845, 253, "because: height is unknown", "small amber")}`)}
    ${reveal(1, `${path("M1080 278V330H650", "green")}${circle(620, 330, 19, "green")}${path("M610 330l7 7 14-15", "green")}${text(665, 372, "Grounding + freshness validated", "green")}${text(665, 410, "Motion still needs its own feasibility check.", "small")}`)}`,
      "The same opening graph becomes a bounded context, observe-more proposal and separate validation.",
    );
  }
  function architecture() {
    return svg(
      `${text(25, 33, "THE SAME EVIDENCE, THROUGH DIFFERENT RESPONSIBILITIES", "tiny")}
    ${drone(85, 140, 0.7)}${aperture(190, 130, 0.55)}${text(25, 225, "Observe", "blue")}${text(25, 262, "depth · pose · capture time", "small")}
    ${arrow(250, 145, 315, "blue")}${graph(420, 140, 0.72)}${text(330, 225, "Retain belief", "violet")}${text(330, 262, "opening_0003 + evidence", "small")}${text(330, 295, "prior map ≠ current belief", "tiny")}
    ${reveal(
      0,
      `${arrow(510, 145, 580, "violet")}${text(605, 135, "observe_more", "math green")}${text(605, 175, "height unknown", "small amber")}${text(605, 225, "Choose intent", "violet")}${text(605, 262, "bounded context + validator", "small")}
      ${arrow(840, 145, 895, "violet")}${path("M925 175Q960 95 1045 120", "violet")}${path("M925 175Q990 170 1045 120", "amber", 'stroke-dasharray="5 6"')}${path("M925 175Q955 200 1045 120", "muted")}${circle(1045, 120, 13, "amber", 'stroke-dasharray="4 4"')}${text(900, 225, "Evaluate motion", "violet")}${text(900, 262, "prediction + swept body", "small")}`,
    )}
    ${reveal(
      1,
      `${arrow(1070, 145, 1140, "green")}${path("M1175 95H1320V175H1175Z", "green")}${text(1200, 128, "ONE", "tiny green")}${text(1198, 155, "OWNER", "tiny green")}${text(1160, 225, "Validate + execute", "green")}${text(1160, 262, "fresh inputs → PX4", "small")}
      ${path("M190 285V337H1380V145H1330", "green", 'stroke-dasharray="5 6"')}${text(455, 326, "Fresh local collision evidence can veto movement.", "small green")}
      ${path("M1365 185V390H5V145H40", "muted")}${text(550, 422, "Execution → new observations", "small")}`,
    )}`,
      "Observations become belief, intent and motion candidates. Fresh collision evidence independently reaches one execution owner; execution feeds new observations.",
    );
  }
  function disconnect() {
    return svg(
      `${text(30, 40, "HISTORICAL P2 FIXTURE", "tiny")}${aperture(150, 135, 0.9)}${text(30, 220, "Map pose improved", "blue")}${text(30, 258, "localization estimate", "small")}
    ${graph(490, 145, 0.8)}${arrow(270, 145, 370, "blue")}${text(400, 260, "Updated spatial state", "small violet")}
    ${reveal(0, `${path("M600 145H760M865 145H1000", "muted", 'stroke-dasharray="6 7"')}${path("M790 125L830 165M830 125L790 165", "red")}${drone(1135, 145, 0.8)}${text(1030, 225, "Same planner stops", "red")}${text(700, 215, "missing consumer contract", "small amber")}`)}
    ${reveal(1, `${path("M490 285V330H1350V145H1185", "green")}${text(290, 380, "Bounded worker", "green")}${text(660, 380, "Diagnostic adoption", "green")}${text(1030, 380, "Benefit still unproven", "amber")}${text(290, 415, "Later offline integration; no demonstrated route improvement.", "small")}`)}`,
      "An improved spatial estimate initially did not reach the planner; a later diagnostic connection is shown without claiming navigation improvement.",
    );
  }
  function failures() {
    return svg(
      `${text(25, 35, "C1 · THE BRAKING MEASUREMENT FAILED", "tiny")}
    <rect x="45" y="90" width="590" height="125" fill="#6ebce8" fill-opacity=".035"/>
    ${path("M45 90H635M45 215H635", "line", 'stroke-dasharray="5 7"')}${text(45, 76, "navigation height band", "small")}
    ${drone(130, 143, 0.52)}${text(63, 189, "translation began", "small blue")}${arrow(180, 143, 292, "blue")}
    ${path("M590 100V290", "muted", 'stroke-dasharray="5 7"')}${text(495, 317, "brake trigger", "small")}${text(495, 343, "not reached", "small amber")}
    ${reveal(0, `${circle(335, 238, 36, "amber", 'stroke-dasharray="5 5"')}${drone(335, 238, 0.52)}${text(240, 290, "height gate crossed", "small amber")}${path("M335 310V355H180", "green")}${path("M180 345V365M165 365H195", "green")}${text(40, 393, "LAND + disarm completed", "green")}`)}
    ${path("M715 40V410", "line")}
    ${reveal(
      1,
      `${text(775, 35, "S0 · FOUR DISARMED ATTEMPTS", "tiny")}
      ${path("M800 155V85M790 145H875", "blue")}${path("M812 155V105M802 155H887", "amber", 'stroke-dasharray="4 4"')}${text(915, 115, "01  Frame origins", "blue")}${text(915, 149, "bind the same physical position", "small")}
      ${path("M790 210H895", "blue")}${path("M790 228H895", "amber")}${path("M825 200V220M840 218V238", "muted")}${text(915, 210, "02  Clock alignment", "amber")}${text(915, 244, "preserve capture-time identity", "small")}
      ${circle(808, 300, 8, "violet")}${circle(838, 300, 8, "violet")}${circle(868, 300, 8, "violet")}${path("M880 300H900", "violet")}${text(915, 305, "03–04  Readiness + scheduling", "violet")}${text(915, 339, "bound the work on the owner path", "small")}
      ${text(775, 393, "No course-flight success claimed.", "small amber")}`,
    )}
    ${text(25, 434, "Schematic states and event order · not a measured trajectory", "tiny")}`,
      "C1 translation began inside the height band and crossed its lower gate before reaching the brake trigger. Landing recovered the experiment. Separate S0 attempts exposed frame, clock and owner scheduling contracts and remained disarmed.",
    );
  }
  function status() {
    return svg(
      `${text(25, 35, "EVIDENCE IS LOCAL TO THE CLAIM", "tiny")}
    ${graph(160, 135, 0.6)}${text(25, 250, "Offline components", "blue")}${text(25, 290, "entities · reasoning · mapping", "small")}${text(25, 325, "synthetic comparison evidence", "small")}
    ${reveal(0, `${drone(635, 135, 0.8)}${path("M590 195H680", "green")}${text(500, 250, "Native calibration", "green")}${text(500, 290, "C0: completed", "small")}${text(500, 325, "C1: recovered; measurement failed", "small")}`)}
    ${reveal(1, `${circle(1115, 135, 52, "amber", 'stroke-dasharray="5 7"')}${text(1102, 149, "?", "math amber")}${text(975, 250, "Mission-level benefit", "amber")}${text(975, 290, "hybrid course + matched trials", "small")}${text(975, 325, "hardware qualification remains", "small")}`)}
    ${path("M25 365H1360", "line")}${text(25, 415, "Separate baseline: Classical S1 has one successful Gazebo course run.", "small green")}`,
      "Offline graph evidence, native calibration and still-unproven mission benefit are distinguished; Classical S1 remains a separate baseline.",
    );
  }
  function roadmap() {
    return svg(
      `${drone(125, 130, 0.65)}${path("M175 130H270", "blue")}${text(25, 235, "01  Reliable native segment", "blue")}${text(25, 275, "frames · timing · height control", "small")}
    ${reveal(
      0,
      `${path("M465 110H755M465 150H755", "muted")}${circle(510, 110, 10, "blue")}${circle(510, 150, 10, "green")}${text(465, 235, "02  Matched comparison", "green")}${text(465, 275, "same map, start and scoring", "small")}
      ${graph(1130, 130, 0.65)}${text(975, 235, "03  Layer ablations", "violet")}${text(975, 275, "memory → topology → semantics", "small")}`,
    )}
    ${reveal(1, `${path("M25 320H1360", "line")}${text(115, 385, "Observations", "math blue")}${arrow(350, 375, 455, "blue")}${text(485, 385, "Grounded belief", "math violet")}${arrow(790, 375, 895, "violet")}${text(925, 385, "Justified actions", "math green")}${text(390, 430, "Learned extensions only when the evidence warrants them.", "small")}`)}`,
      "A native flight segment precedes matched comparisons and layer ablations, linking observations to grounded belief and justified actions.",
    );
  }
  return {
    papers,
    reasoning,
    architecture,
    disconnect,
    failures,
    status,
    roadmap,
  };
})();
