/* A single illustrative scene: the same observation samples become fitted
   geometry, graph nodes, then evidence for an observe-more proposal. Positions
   are design pixels, not measured course data. A seekable clock makes reverse
   navigation, reduced motion and PDF export deterministic. */
window.ContinuousScene = (() => {
  const durationSeconds = 20; // Our chosen explanatory animation length.
  const stageTimes = [0, 6.5, 11.8, 17.5];
  const colors = {
    blue: "#6ebce8",
    green: "#8fcead",
    amber: "#e4bf7b",
    violet: "#b6a1dd",
  };
  const sampleCount = 64; // Our chosen drawing density, not a sensor count.
  let root;
  let elapsedSeconds = 0;
  let lastFrameMilliseconds = null;
  let animationRequest = null;
  let playing = false;
  let reduced = false;
  let active = false;

  function smooth(value) {
    const bounded = Math.max(0, Math.min(1, value));
    return bounded * bounded * (3 - 2 * bounded);
  }
  function progress(time, start, end) {
    return smooth((time - start) / (end - start));
  }
  function mix(start, end, amount) {
    return start + (end - start) * amount;
  }
  function label(x, y, content, className = "") {
    return `<text x="${x}" y="${y}" class="${className}">${content}</text>`;
  }
  function drone() {
    return `<g id="continuity-drone" transform="translate(140 250)" stroke="${colors.blue}" stroke-width="3" fill="#101318">
      <path d="M-23-23L23 23M23-23L-23 23"/>
      <circle cx="-25" cy="-25" r="15"/><circle cx="25" cy="-25" r="15"/>
      <circle cx="-25" cy="25" r="15"/><circle cx="25" cy="25" r="15"/>
      <circle r="10" fill="${colors.blue}"/><path d="M16 0H43"/>
    </g>`;
  }
  function markup() {
    let samples = "";
    let rays = "";
    for (let index = 0; index < sampleCount; index++) {
      samples += `<circle class="observation-sample" r="3.2" fill="${colors.blue}"/>`;
      if (index % 4 === 0)
        rays +=
          '<line class="observation-ray" stroke="#6ebce8" stroke-width="1"/>';
    }
    return `<div class="continuous-scene" aria-label="Continuous observation to decision animation">
      <svg viewBox="0 0 1400 440" role="img" aria-labelledby="continuity-title continuity-description">
        <title id="continuity-title">From observation to a grounded decision</title>
        <desc id="continuity-description">Blue sensor samples resolve into two wall segments. The same samples gather into semantic nodes around opening_0003. Unknown height remains amber. Evidence reaches a non-executable observe-more proposal.</desc>
        <defs><pattern id="continuity-grid" width="35" height="35" patternUnits="userSpaceOnUse"><path d="M35 0H0V35" fill="none" stroke="#283440" stroke-width=".7"/></pattern></defs>
        <g class="phase-rail">
          ${label(40, 26, "01  DEPTH OBSERVATIONS", "tiny blue")}${label(380, 26, "02  WALLS + OPENING", "tiny blue")}
          ${label(720, 26, "03  WORLDSTATE", "tiny violet")}${label(1080, 26, "04  REASONING", "tiny green")}
          <path d="M40 47H1350" stroke="#34414d"/>
          <path id="continuity-progress" d="M40 47H1350" stroke="${colors.blue}" stroke-width="2" pathLength="1" stroke-dasharray="1"/>
        </g>
        <g id="sensor-world">
          <rect x="55" y="80" width="740" height="270" fill="url(#continuity-grid)"/>
          <path d="M140 250L710 85L710 345Z" fill="#6ebce8" fill-opacity=".035"/>
        </g>
        <g id="sensor-rays">${rays}</g>
        <g id="fitted-geometry" fill="none" stroke="${colors.blue}" stroke-width="3">
          <path id="left-surface" d="M330 190H485"/>
          <path id="right-surface" d="M640 190H795"/>
        </g>
        <g id="opening-measure" fill="none" stroke="${colors.amber}" stroke-width="2">
          <path d="M488 190H637" stroke-dasharray="6 7"/>
          <path d="M490 220V240M635 220V240M490 230H635"/>
        </g>
        <g id="width-caption">${label(506, 263, "width observed", "small blue")}</g>
        <g id="graph-edges" fill="none" stroke-width="2">
          <path id="left-edge" stroke="${colors.violet}"/>
          <path id="right-edge" stroke="${colors.violet}"/>
          <path id="evidence-edge" stroke="${colors.green}"/>
          <path id="unknown-edge" stroke="${colors.amber}" stroke-dasharray="5 7"/>
        </g>
        <g id="observation-samples">${samples}</g>
        <g id="graph-nodes">
          <circle id="left-node" r="19" fill="#101318" stroke="${colors.blue}" stroke-width="2"/>
          <circle id="right-node" r="19" fill="#101318" stroke="${colors.blue}" stroke-width="2"/>
          <circle id="opening-node" r="30" fill="#161b25" stroke="${colors.violet}" stroke-width="2.5"/>
          <circle id="evidence-node" r="14" fill="#101318" stroke="${colors.green}" stroke-width="2"/>
          <circle id="unknown-node" r="18" fill="#101318" stroke="${colors.amber}" stroke-width="2" stroke-dasharray="4 4"/>
        </g>
        <g id="graph-labels">
          <text id="left-label" class="small blue">left wall</text><text id="right-label" class="small blue">right wall</text>
          <text id="entity-label" class="math violet">opening_0003</text>
          <text id="evidence-label" class="small green">observation · t₀</text>
          <text id="unknown-label" class="small amber">height unknown</text>
          <text id="unknown-mark" class="amber">?</text>
        </g>
        ${drone()}
        <g id="observation-caption">${label(850, 130, "Depth measures distance.", "math")}${label(850, 180, "Match each view to the drone’s pose.", "small")}${label(850, 225, "Unobserved space remains unknown.", "small amber")}</g>
        <g id="geometry-caption">${label(870, 130, "Find walls and a possible opening.", "math")}${label(870, 180, "Group points into wall segments.", "small")}${label(870, 225, "A width does not establish safe passage.", "small amber")}</g>
        <g id="graph-caption">${label(980, 115, "WorldState remembers this opening.", "small")}${label(980, 153, "Known: walls, width, last observation", "small green")}${label(980, 187, "Unknown: height and where it leads", "small amber")}</g>
        <path id="decision-connection" fill="none" stroke="${colors.violet}" stroke-width="2.5" pathLength="1" stroke-dasharray="1"/>
        <circle id="evidence-pulse" r="5" fill="${colors.violet}"/>
        <g id="decision-output">
          ${label(960, 135, "GOAL PROPOSAL", "tiny green")}${label(960, 192, "observe_more", "big green")}
          ${label(960, 238, "target: opening_0003", "small violet")}${label(960, 274, "because: height is unknown", "small amber")}
          <path d="M960 299H1350" stroke="#34414d"/>
          ${label(960, 329, "Grounded proposal ≠ permission to fly", "small")}
        </g>
        <g id="return-observation">
          <path d="M1090 350V370Q1090 385 1070 385H165Q140 385 140 360V307" fill="none" stroke="${colors.green}" stroke-width="1.5" stroke-dasharray="5 7"/>
          ${label(375, 410, "Ask for the missing observation. Then evaluate motion separately.", "small green")}
        </g>
        <text id="continuity-caption" x="65" y="400" class="small">Illustrative sensor samples · not recorded telemetry</text>
      </svg>
      <div class="continuity-controls">
        <button class="continuity-play" aria-label="Pause continuous animation">Pause</button>
        <button class="continuity-replay" aria-label="Replay continuous animation">↺ Replay</button>
        <input class="continuity-scrub" type="range" min="0" max="20" step="0.01" value="0" aria-label="Animation time in seconds">
        <output class="continuity-time">0.0 / 20 s</output>
        <div class="continuity-stages" aria-label="Jump to animation stage"><button data-stage="0">Observe</button><button data-stage="1">Geometry</button><button data-stage="2">WorldState</button><button data-stage="3">Reason</button></div>
      </div>
    </div>`;
  }
  function element(selector) {
    return root.querySelector(selector);
  }
  function opacity(selector, value) {
    element(selector).setAttribute("opacity", Math.max(0, Math.min(1, value)));
  }
  function position(selector, x, y) {
    const target = element(selector);
    const circle = target.tagName.toLowerCase() === "circle";
    target.setAttribute(circle ? "cx" : "x", x);
    target.setAttribute(circle ? "cy" : "y", y);
  }
  function nodesAt(reasoningProgress) {
    const shift = reasoningProgress * -115;
    return {
      left: [440 + shift, 130],
      right: [790 + shift, 130],
      opening: [615 + shift, 235],
      evidence: [420 + shift, 325],
      unknown: [805 + shift, 325],
    };
  }
  function renderSamples(time, geometryProgress, graphProgress, nodes) {
    const samples = root.querySelectorAll(".observation-sample");
    const rays = root.querySelectorAll(".observation-ray");
    samples.forEach((sample, index) => {
      const side = index < sampleCount / 2 ? 0 : 1;
      const fraction = (index % (sampleCount / 2)) / (sampleCount / 2 - 1);
      const fittedX = (side === 0 ? 330 : 640) + fraction * 155;
      const fittedY = 190;
      const rawX = fittedX + Math.sin(index * 2.4) * 10;
      const rawY = fittedY + Math.cos(index * 1.7) * 22;
      const node = side === 0 ? nodes.left : nodes.right;
      const angle = fraction * Math.PI * 2;
      const finalX = node[0] + Math.cos(angle) * 19;
      const finalY = node[1] + Math.sin(angle) * 19;
      const x = mix(
        mix(rawX, fittedX, geometryProgress),
        finalX,
        graphProgress,
      );
      const y = mix(
        mix(rawY, fittedY, geometryProgress),
        finalY,
        graphProgress,
      );
      sample.setAttribute("cx", x);
      sample.setAttribute("cy", y);
      const appeared = progress(time, index * 0.026, index * 0.026 + 0.6);
      sample.setAttribute("opacity", appeared * (1 - graphProgress * 0.7));
      sample.setAttribute("r", mix(3.2, 1.6, graphProgress));
      if (index % 4 === 0) {
        const ray = rays[index / 4];
        ray.setAttribute("x1", 140);
        ray.setAttribute("y1", 250);
        ray.setAttribute("x2", x);
        ray.setAttribute("y2", y);
        ray.setAttribute("opacity", appeared * (1 - geometryProgress) * 0.18);
      }
    });
  }
  function renderGraph(
    time,
    geometryProgress,
    graphProgress,
    reasoningProgress,
    nodes,
  ) {
    const nodeVisibility = progress(time, 8.5, 10.3);
    opacity("#graph-nodes", nodeVisibility);
    opacity("#graph-labels", nodeVisibility);
    for (const name of ["left", "right", "opening", "evidence", "unknown"])
      position(`#${name}-node`, ...nodes[name]);
    position("#left-label", nodes.left[0] - 45, nodes.left[1] - 35);
    position("#right-label", nodes.right[0] - 45, nodes.right[1] - 35);
    position("#entity-label", nodes.opening[0] - 100, nodes.opening[1] + 58);
    position("#evidence-label", nodes.evidence[0] - 70, nodes.evidence[1] + 36);
    position("#unknown-label", nodes.unknown[0] - 65, nodes.unknown[1] + 36);
    position("#unknown-mark", nodes.unknown[0] - 6, nodes.unknown[1] + 8);
    const edges = [
      ["left", nodes.left],
      ["right", nodes.right],
      ["evidence", nodes.evidence],
      ["unknown", nodes.unknown],
    ];
    for (const [name, point] of edges) {
      element(`#${name}-edge`).setAttribute(
        "d",
        `M${point[0]} ${point[1]}L${nodes.opening[0]} ${nodes.opening[1]}`,
      );
    }
    // Route the lower evidence edges around the persistent entity label.
    const [centerX, centerY] = nodes.opening;
    element("#evidence-edge").setAttribute(
      "d",
      `M${nodes.evidence[0]} ${nodes.evidence[1]}C${centerX - 260} 275 ${centerX - 190} ${centerY} ${centerX - 32} ${centerY}`,
    );
    element("#unknown-edge").setAttribute(
      "d",
      `M${nodes.unknown[0]} ${nodes.unknown[1]}C${centerX + 260} 275 ${centerX + 190} ${centerY} ${centerX + 32} ${centerY}`,
    );
    opacity("#graph-edges", progress(time, 9.1, 11.2));
    // Surface strokes shrink onto the exact nodes receiving their samples.
    for (const [name, start, end, node] of [
      ["left", 330, 485, nodes.left],
      ["right", 640, 795, nodes.right],
    ]) {
      element(`#${name}-surface`).setAttribute(
        "d",
        `M${mix(start, node[0] - 19, graphProgress)} ${mix(190, node[1], graphProgress)}L${mix(end, node[0] + 19, graphProgress)} ${mix(190, node[1], graphProgress)}`,
      );
    }
    opacity("#fitted-geometry", geometryProgress * (1 - graphProgress * 0.65));
    opacity("#opening-measure", geometryProgress * (1 - graphProgress));
    opacity("#width-caption", geometryProgress * (1 - graphProgress));
    opacity("#sensor-world", 1 - graphProgress * 0.94);
    opacity("#continuity-drone", 1 - reasoningProgress * 0.3);
  }
  function renderDecision(time, nodes) {
    const linkProgress = progress(time, 13.2, 15.2);
    const from = nodes.opening;
    element("#decision-connection").setAttribute(
      "d",
      `M${from[0] + 32} ${from[1]}C760 ${from[1]} 780 184 930 184`,
    );
    element("#decision-connection").setAttribute(
      "stroke-dashoffset",
      1 - linkProgress,
    );
    opacity("#decision-connection", linkProgress);
    opacity("#decision-output", progress(time, 14.5, 16.1));
    const pulseProgress = progress(time, 13.6, 15.2);
    const curve = element("#decision-connection");
    const pulse = curve.getPointAtLength(
      curve.getTotalLength() * pulseProgress,
    );
    position("#evidence-pulse", pulse.x, pulse.y);
    opacity("#evidence-pulse", time > 13.6 && time < 15.2 ? 1 : 0);
    opacity("#return-observation", progress(time, 16.3, 18));
  }
  function render(time) {
    if (!root) return;
    const geometryProgress = progress(time, 3, 6);
    const graphProgress = progress(time, 7, 10.5);
    const reasoningProgress = progress(time, 12, 13.8);
    const nodes = nodesAt(reasoningProgress);
    renderSamples(time, geometryProgress, graphProgress, nodes);
    renderGraph(
      time,
      geometryProgress,
      graphProgress,
      reasoningProgress,
      nodes,
    );
    renderDecision(time, nodes);
    opacity("#observation-caption", 1 - progress(time, 2.8, 3.8));
    opacity(
      "#geometry-caption",
      progress(time, 3.8, 4.7) * (1 - progress(time, 7, 8)),
    );
    opacity(
      "#graph-caption",
      progress(time, 9.8, 11) * (1 - progress(time, 12, 13)),
    );
    opacity("#continuity-caption", 1 - progress(time, 15.7, 16.6));
    element("#continuity-progress").setAttribute(
      "stroke-dashoffset",
      1 - time / durationSeconds,
    );
    element(".continuity-scrub").value = time;
    element(".continuity-time").textContent = `${time.toFixed(1)} / 20 s`;
    const stage = time < 3 ? 0 : time < 7 ? 1 : time < 12 ? 2 : 3;
    root.dataset.stage = String(stage);
    root
      .querySelectorAll("[data-stage]")
      .forEach((button) =>
        button.setAttribute(
          "aria-pressed",
          String(Number(button.dataset.stage) === stage),
        ),
      );
  }
  function pause() {
    playing = false;
    cancelAnimationFrame(animationRequest);
    animationRequest = null;
    lastFrameMilliseconds = null;
    if (root) element(".continuity-play").textContent = "Play";
    if (root)
      element(".continuity-play").setAttribute(
        "aria-label",
        "Play continuous animation",
      );
  }
  function seek(seconds) {
    pause();
    elapsedSeconds = Math.max(0, Math.min(durationSeconds, seconds));
    render(elapsedSeconds);
  }
  function tick(timestamp) {
    if (!playing || !active) return;
    if (lastFrameMilliseconds !== null)
      elapsedSeconds = Math.min(
        durationSeconds,
        elapsedSeconds + (timestamp - lastFrameMilliseconds) / 1000,
      );
    lastFrameMilliseconds = timestamp;
    render(elapsedSeconds);
    if (elapsedSeconds >= durationSeconds) {
      pause();
      return;
    }
    animationRequest = requestAnimationFrame(tick);
  }
  function play() {
    if (!root || !active || reduced) return;
    if (elapsedSeconds >= durationSeconds) elapsedSeconds = 0;
    playing = true;
    lastFrameMilliseconds = null;
    element(".continuity-play").textContent = "Pause";
    element(".continuity-play").setAttribute(
      "aria-label",
      "Pause continuous animation",
    );
    cancelAnimationFrame(animationRequest);
    animationRequest = requestAnimationFrame(tick);
  }
  function mount(container, { staticMode = false } = {}) {
    root = container;
    element(".continuity-play").onclick = () => (playing ? pause() : play());
    element(".continuity-replay").onclick = () => {
      seek(0);
      play();
    };
    element(".continuity-scrub").oninput = (event) =>
      seek(Number(event.target.value));
    root
      .querySelectorAll("[data-stage]")
      .forEach(
        (button) =>
          (button.onclick = () =>
            seek(stageTimes[Number(button.dataset.stage)])),
      );
    render(staticMode ? durationSeconds : 0);
  }
  function setActive(value) {
    const entering = value && !active;
    active = value;
    if (!active) {
      pause();
      return;
    }
    if (entering) {
      seek(reduced ? durationSeconds : 0);
      play();
    }
  }
  function setReduced(value) {
    reduced = value;
    if (!root) return;
    element(".continuity-play").disabled = value;
    if (value) {
      seek(durationSeconds);
    }
  }
  function step(direction) {
    if (!active) return false;
    if (direction > 0 && elapsedSeconds >= durationSeconds - 0.05) return false;
    if (direction < 0 && elapsedSeconds <= 0.05) return false;
    const targets = [0, 3, 7, 12, durationSeconds];
    const target =
      direction > 0
        ? targets.find((time) => time > elapsedSeconds + 0.05)
        : targets
            .slice()
            .reverse()
            .find((time) => time < elapsedSeconds - 0.05);
    seek(target ?? 0);
    return true;
  }
  return {
    markup,
    mount,
    setActive,
    setReduced,
    seek,
    play,
    pause,
    step,
    get time() {
      return elapsedSeconds;
    },
    get playing() {
      return playing;
    },
  };
})();
