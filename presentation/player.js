/* Connects slide content to reveal.js. This audience player never accesses live
   research or flight services. Appendix navigation is separate from the talk. */
(() => {
  const mainSlideCount = window.slides.length;
  const parameters = new URLSearchParams(location.search);
  const staticMode = parameters.has("static");
  const sourceIds = {
    N1: "411f393aa76881fa9a0eee681393ffb2",
    N2: "411f393aa76881909d94d0c3c6029e49",
    N3: "411f393aa768814e97a4cb94b726398c",
    N4: "411f393aa76881e78dd6d7beb6592060",
    N5: "412f393aa768816ea9c1f05ab405503c",
    N6: "411f393aa768816abd1bd8651fbb4a90",
    N7: "412f393aa76881ef9a80d3307bcc1f8d",
    N8: "411f393aa76881c79ae2fb7f6d7d96f7",
  };
  const titles = [
    "The course is visible.<br>The next action is uncertain.",
    "Build what the drone <em>remembers.</em>",
    "Give the map meaning.",
    "From seeing to <em>understanding.</em>",
    "Sometimes the next action is <em>to look.</em>",
    "Remember. Decide. Fly.",
    "Why DATT and physics-informed learning?",
    "Our Classical baseline: plan, then follow.",
    "What failed — and what we changed.",
    "What we built. What we still need to prove.",
    "From observations to <em>justified actions.</em>",
  ];
  const subtitles = [
    "Indoor navigation, incomplete observations, and a mission beyond the field of view.",
    "Remember openings, keep uncertainty, and look again when needed.",
    "Research gave us the structure; our constraints shaped the implementation.",
    "Observations become geometry, then meaning, then a grounded decision.",
    "If we do not know whether the drone fits, take another look.",
    "Three jobs, with a safety check before movement.",
    "A good route still needs a drone that can follow it when conditions change.",
    "Established algorithms give us a clear starting point to measure against.",
    "Simulation exposed problems with height, coordinates and timing.",
    "Evidence is specific to a component, an experiment and an environment.",
    "The next milestone is a measurable mission benefit from each layer.",
  ];
  const scopeLabels = [
    "Illustrative course",
    "Illustrative belief update",
    "Research synthesis",
    "Illustrative entity · offline contract",
    "Non-executable proposal",
    "Target architecture",
    "Research motivation · DATT proposed",
    "Classical branch · source inspection",
    "Native simulation · preserved failures",
    "Research cutoff · 08 Nov 2026",
    "Planned evaluation",
  ];
  const transitions = [
    "That led us to a representation question before a policy question.",
    "We looked for research that already separated these responsibilities.",
    "Here is what that synthesis means for a single opening.",
    "The missing evidence now becomes input to the reasoning layer.",
    "This decision sits inside a larger chain with separate responsibilities.",
    "Now consider the flying part: what happens when the motion model is wrong?",
    "Before adding learning, we need a classical baseline we can understand.",
    "Integration showed us which assumptions were still wrong.",
    "Those outcomes define our evidence boundary today.",
    "The next experiments are designed to close that gap one layer at a time.",
    "Thank you. We can now go deeper into the architecture or the evidence.",
  ];
  const explanation = [
    "The challenge is not just producing coordinates. The drone must distinguish visible free space from an unobserved continuation, keep track of progress, and decide what evidence it needs next. We wanted those distinctions to be explicit.",
    "This does not require a large generative world model. A small set of useful entities, relationships, timestamps and uncertainty can expose a decision in a form that we can debug. That was our starting hypothesis.",
    "The common thread was structured state with different levels of abstraction. Our contribution is the task-specific integration and evaluation, not the invention of scene graphs or the performance of these research systems.",
    "For example, the same opening can survive occlusion without becoming more certain. If a later observation shows blocking geometry, the stored hypothesis changes. Memory must preserve uncertainty as carefully as it preserves identity.",
    "That choice is useful because it identifies a specific knowledge gap. The motion layer must still establish whether the observation pose and the path toward it are feasible. The reasoning layer cannot certify that on its own.",
    "The feedback loop matters: executing a motion should produce new evidence that updates belief. Every boundary needs a clear frame, clock and state identity, because a perfectly reasonable decision can still be invalid when its inputs are old.",
    "",
    "",
    "",
    "",
    "",
  ];
  function escapeHtml(value) {
    return value
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;");
  }
  const externalSources = {
    DATT: "https://proceedings.mlr.press/v229/huang23a.html",
    PINNs: "https://arxiv.org/abs/1711.10561",
    Classical:
      "https://github.com/jacobdannieam-byte/DroneHacks-Dannie-/tree/f4264a216eb00a6f3522dba15951f420b61469c6/drone_code/team_nav",
  };
  function sourceLinks(ids) {
    return ids
      .map(
        (id) =>
          `<a href="${externalSources[id] || `https://app.notion.com/p/${sourceIds[id]}`}" target="_blank" rel="noopener">${id}</a>`,
      )
      .join(" · ");
  }
  function createMainSlide(slide, index) {
    const section = document.createElement("section");
    section.id = `chapter-${index + 1}`;
    section.dataset.timing = slide.time;
    section.dataset.main = "true";
    section.innerHTML = `<p class="eyebrow"><span class="chapter">${String(index + 1).padStart(2, "0")} / 11</span>${slide.kicker}</p>
      <h1>${titles[index]}</h1><p class="subtitle">${subtitles[index]}</p>
      <div class="scene ${slide.visual === "memory" ? "memory-scene" : ""}" data-step="0">${index === 3 ? window.ContinuousScene.markup() : (window.StoryScenes[slide.visual] || window.DeckVisuals[slide.visual])()}</div>
      <p class="takeaway"><span class="cue">↳</span>${slide.takeaway}</p>
      <div class="citation"><span>${sourceLinks(slide.sources)}</span><span class="scope">${scopeLabels[index]}</span></div>
      <aside class="notes"><p><b>${slide.time} seconds · ${slide.kicker}</b></p><p>${slide.notes}</p><p>${explanation[index]}</p><p><b>Transition:</b> ${transitions[index]}</p><p><b>Click cues:</b> ${slide.builds.join(" → ")}</p><p>Sources: ${sourceLinks(slide.sources)}</p></aside>`;
    return section;
  }
  const deck = document.getElementById("deck");
  window.slides.forEach((slide, index) =>
    deck.append(createMainSlide(slide, index)),
  );
  window.ContinuousScene.mount(document.querySelector(".continuous-scene"), {
    staticMode,
  });
  window.appendixSlides.forEach((slide, index) => {
    const section = document.createElement("section");
    section.id = `question-${index + 1}`;
    section.className = "appendix-slide";
    section.innerHTML = `<p class="eyebrow">Q&amp;A / ${String(index + 1).padStart(2, "0")}</p><h2>${slide.title}</h2><div class="appendix-copy">${slide.content}</div><button class="appendix-return">Return to closing →</button><div class="citation"><span>${sourceLinks(slide.sources)}</span><span>Outside the timed talk</span></div><aside class="notes">${escapeHtml(slide.notes)}</aside>`;
    if (!staticMode || parameters.has("appendix")) deck.append(section);
  });
  if (staticMode) {
    // reveal.js has its own print layout; static export uses our fixed canvas.
    document.querySelector('link[href="vendor/reveal.css"]').remove();
    document.body.classList.add("static-mode", "reduced-motion");
    document
      .querySelectorAll(".scene")
      .forEach((scene) => (scene.dataset.step = "3"));
    window.deckReady = true;
    return;
  }
  let motionReduced = matchMedia("(prefers-reduced-motion: reduce)").matches;
  function updateMotion() {
    document.body.classList.toggle("reduced-motion", motionReduced);
    document
      .getElementById("motion")
      .setAttribute("aria-pressed", String(motionReduced));
    document.getElementById("motion").textContent = motionReduced
      ? "Motion reduced"
      : "Reduce motion";
    Reveal.configure({ transition: motionReduced ? "none" : "fade" });
    window.ContinuousScene.setReduced(motionReduced);
  }
  function advance() {
    if (window.ContinuousScene.step(1)) return;
    const { h } = Reveal.getIndices();
    if (h === mainSlideCount - 1 && !Reveal.availableFragments().next) return;
    if (h >= mainSlideCount) return;
    Reveal.next();
  }
  function retreat() {
    if (window.ContinuousScene.step(-1)) return;
    if (Reveal.getIndices().h >= mainSlideCount) {
      Reveal.slide(mainSlideCount - 1, 0, 2);
      return;
    }
    Reveal.prev();
  }
  function syncScene() {
    const current = Reveal.getCurrentSlide();
    const scene = current.querySelector(".scene");
    if (scene) {
      const indices = [...current.querySelectorAll(".fragment.visible")].map(
        (item) => Number(item.dataset.fragmentIndex),
      );
      scene.dataset.step = String(
        indices.length ? Math.max(...indices) + 1 : 0,
      );
    }
    const { h } = Reveal.getIndices();
    window.ContinuousScene.setActive(h === 3);
    document.getElementById("position").textContent =
      h < mainSlideCount
        ? `${String(h + 1).padStart(2, "0")} / 11`
        : `Q&A ${h - mainSlideCount + 1}`;
  }
  const dialog = document.getElementById("appendix-dialog");
  window.appendixSlides.forEach((slide, index) => {
    const button = document.createElement("button");
    button.textContent = slide.menu;
    button.onclick = () => {
      dialog.close();
      Reveal.slide(mainSlideCount + index);
    };
    document.getElementById("appendix-links").append(button);
  });
  function returnToClosing() {
    dialog.close();
    Reveal.slide(mainSlideCount - 1, 0, 2);
  }
  document.getElementById("previous").onclick = retreat;
  document.getElementById("next").onclick = advance;
  document.getElementById("overview").onclick = () => Reveal.toggleOverview();
  document.getElementById("speaker").onclick = () =>
    Reveal.getPlugin("notes").open();
  document.getElementById("appendix").onclick = () => dialog.showModal();
  document.querySelector(".close").onclick = () => dialog.close();
  document.getElementById("return-closing").onclick = returnToClosing;
  document
    .querySelectorAll(".appendix-return")
    .forEach((button) => (button.onclick = returnToClosing));
  document.getElementById("motion").onclick = () => {
    motionReduced = !motionReduced;
    updateMotion();
  };
  document.getElementById("fullscreen").onclick = () =>
    document.fullscreenElement
      ? document.exitFullscreen()
      : document.documentElement.requestFullscreen();
  Reveal.initialize({
    width: 1600,
    height: 900,
    margin: 0.045,
    center: false,
    hash: true,
    hashOneBasedIndex: true,
    controls: false,
    progress: true,
    transition: "fade",
    transitionSpeed: "fast",
    navigationMode: "default",
    view: "slide",
    scrollActivationWidth: null,
    autoSlide: 0,
    showNotes: false,
    pdfSeparateFragments: false,
    plugins: [RevealNotes],
    totalTime: 600,
    keyboard: {
      32: advance,
      39: advance,
      34: advance,
      37: retreat,
      33: retreat,
      40: advance,
      38: retreat,
      36: () => Reveal.slide(0, 0, -1),
      35: () => Reveal.slide(mainSlideCount - 1, 0, 2),
      81: () => dialog.showModal(),
    },
    keyboardCondition: () => !dialog.open,
  }).then(() => {
    updateMotion();
    syncScene();
    window.deckReady = true;
  });
  Reveal.on("slidechanged", syncScene);
  Reveal.on("fragmentshown", syncScene);
  Reveal.on("fragmenthidden", syncScene);
  Reveal.on("overviewshown", () => window.ContinuousScene.pause());
  document.addEventListener("visibilitychange", () => {
    if (document.hidden) window.ContinuousScene.pause();
  });
})();
