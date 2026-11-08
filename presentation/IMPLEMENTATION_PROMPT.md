# Implementation prompt — DroneHacks: from observations to justified actions

You are implementing a professional browser presentation for hackathon and industry judges. Deliver an original, visually coherent, speaker-controlled HTML/JavaScript deck explaining our DroneHacks journey: why we prioritized world semantics and reasoning, how research shaped the architecture, what failed, what changed, and what we plan to prove next.

## Read first and preserve scope

Read `README.md`, `RESEARCH.md`, `slides.js`, and the existing shell. Check the workspace branch and applicable repository instructions. The requested root branch is spelled `presentaiton`; the root began without commits and contains nested drone repositories. Keep presentation work under `presentation/`. Do not stage the whole workspace, change nested branches, alter flight code, deploy services, or run robotics workloads. No backend is needed.

Use the current 11-slide narrative as the content contract. You may tighten phrasing and composition without changing the evidence scope. Deliver the finished deck, source, local run instructions, visual verification, and static fallback. Do not stop with a design proposal. Resolve routine design decisions autonomously.

## Audience, thesis and pacing

Audience: technically literate hackathon and industry judges, not necessarily experts in SLAM, scene graphs or PX4. Talk: 10 minutes; Q&A separate. Primary emphasis: architecture, motivation and plans, told through the build journey and failures. Explain each acronym once in notes or visual context.

Thesis: we prioritized a compact, persistent, evidence-grounded understanding of the world so that goal reasoning could be inspected and tested before more complex policies were promoted.

Start immediately with the problem statement. Put motivation next. No separate title-only opening, company pitch, fabricated market sizing, or generic AI introduction. Preserve this sequence and timing:

| Slide | Message | Seconds |
|---|---|---:|
| 1 | The course is visible; the next action is uncertain | 40 |
| 2 | Why we chose persistent world understanding | 50 |
| 3 | Research synthesis: Hydra + Kimera, with CSIRO and ModalAI lessons | 55 |
| 4 | An opening becomes a hypothesis with evidence | 55 |
| 5 | Reasoning can request more observation | 55 |
| 6 | Architecture: each layer answers a different question | 60 |
| 7 | Pivot: active perception earns priority over learned complexity | 55 |
| 8 | Pivot: better mapping requires actual planner consumption | 55 |
| 9 | Pivot: native execution exposes inter-layer assumptions | 65 |
| 10 | Evidence and remaining gaps, separated by environment | 55 |
| 11 | Native segment, matched comparisons, ablations, next research | 55 |

Total: 600 seconds. Aim for approximately 1,100–1,250 spoken words including transitions; rehearse rather than treating word count as timing proof. Keep technical detail in notes and an appendix accessible outside the normal forward sequence.

## Recommended implementation

Use reveal.js for slide navigation, fragments, overview, speaker notes, and print/PDF support. Its official reference is https://revealjs.com/. Use semantic HTML, custom CSS, JavaScript ES modules and SVG for the primary visuals. A small local build tool is optional; choose and pin compatible dependency versions after inspecting the environment and current documentation. Keep the dependency tree small. React is unnecessary unless a concrete implementation need warrants it.

Bundle all scripts, fonts and media locally for presentation-day offline use. No CDN, API calls, remote model inference, or loading Notion during playback. Source links remain ordinary outbound links. Provide a local preview command and a production build. Preserve a dependency-free static fallback based on the current shell. A fresh installation may need network; playback must not.

Use deterministic SVG state transitions or the Web Animations API for diagram animation. Add an animation library only if it materially simplifies tested reverse/seek behavior. Reserve Three.js for a scene that genuinely requires 3D; the main narrative works in 2D. Manim is optional for pre-rendered explanatory clips, not a runtime requirement. If used, identify the exact Manim distribution/version; Grant's `3b1b/manim` and Manim Community APIs are different. Generate original scenes and provide still-image fallback, captions and playback controls.

## Visual language

Take inspiration from 3Blue1Brown's explanatory construction: start with a concrete visual question, preserve object identity, reveal one relationship at a time, and let motion explain causality. Create original visual work. Do not copy logos, characters, footage or imply endorsement.

Use a near-black navy canvas, off-white text, cyan for observations/geometry, green for supported evidence, amber for unknown/stale state, violet for reasoning, and restrained red for rejected hypotheses or measured failures. Pair color with labels and line styles. Dashed edges are hypotheses; solid edges require supporting evidence. Avoid dense cards, gradients, decorative dashboards and animated particles.

16:9 canvas. Compose for a projected room: titles roughly 48–60 px, body 28–34 px, diagram labels 24–28 px on a 1920×1080 design surface. Keep generous margins. At most one main sentence and one supporting sentence on most slides. Use high contrast and local fonts. Put source details in notes/appendix; concise citations on relevant slides remain readable. Do not shrink text to fit a crowded diagram.

## Central visual sequence

Use the same conceptual opening across slides 1, 2, 4, 5 and 6. This repetition is intentional continuity, not reused decoration. Label it illustrative, not telemetry or surveyed course geometry.

1. Show a small drone, walls, an opening and the camera field of view. Hatch unknown space.
2. Separate visible geometry from the larger mission question. Do not reveal an omniscient completed map as sensed data.
3. Turn the camera away; retain `opening_0003`, age its observation, and keep uncertainty visible.
4. Add measured width, unknown height and unverified connectivity. Never turn an attractive opening green as a claim of safe passage.
5. Convert the entity into bounded context and an `observe_more` proposal. Show the source entity reference and missing measurement.
6. Pass through independent proposal validation; separately illustrate trajectory and swept-body checks. An accepted proposal is not an execution authorization.
7. Expand the scene into the architecture while preserving object identity and the meaning of each color.

Animation must be speaker-paced, reversible, and deterministic. Every slide and fragment can be reached directly without playing earlier slides. Reset or reconstruct state on backward navigation. Avoid continuous decorative motion. Use short transitions, generally 250–700 ms; longer explanatory motion must be pausable. Reduced-motion mode uses stable states or brief fades. Export states must retain the full takeaway without transient overlaps.

## Architecture diagram requirements

Show capture-time sensor/pose alignment → metric belief and semantic state → bounded context → goal proposal → candidate trajectory prediction/evaluation → independent execution checks → one command owner → PX4. Include a feedback loop to new observations. Position the optional learned reasoner as an interchangeable research component, not an LLM controlling motors.

Visually distinguish immutable permitted reference geometry, persistent current-run belief, fresh local collision evidence, and semantic/topological state. Show where frame, timestamp and reset identity matter without dumping schemas on the main slide. A reference map does not populate fresh sensor FREE cells. Use labels to distinguish the target design, offline implementation, diagnostic integration and native validation. Do not imply every connection has demonstrated successful flight.

## Research slide

Make a synthesis diagram rather than paper-cover thumbnails. Map Hydra → incremental geometry/place hierarchy; Kimera → metric-semantic representation; CSIRO → partial-observation/deployment/recovery lessons; ModalAI → separated VIO/obstacle/autopilot interfaces. Label ModalAI an industry reference. Cite the primary links from `RESEARCH.md` and explain our specific adaptation in notes. Do not claim we implemented these systems or inherited their published benchmark results. Swift is optional appendix context pending direct source verification; omit its performance numbers.

## Failure and pivot treatment

Use documented observations, decisions and limitations, never invented dramatic anecdotes.

- Slide 7: horizontal count chart, common zero baseline. G0=68, G1=260, G2 seeds=259/244/243; 420 synthetic episodes per variant. Metric is evidence items acquired. Label environment on the chart. Show three G2 runs separately, not as a confidence interval. Explain that G1 was prioritized and no clear G2 advantage was established. Do not convert counts into flight-success percentages.
- Slide 8: show an initially broken connection between map correction and planner input. Then show bounded asynchronous registration and later diagnostic consumption. Preserve the historical progression. Detailed RMSE and latency figures belong in the appendix with Windows/synthetic qualifiers. Diagnostic adoption is not demonstrated route improvement.
- Slide 9: two compact stories. C1 translation → height-band violation before brake trigger → native LAND/disarm; experiment failed, recovery completed. Later S0 native attempts remained disarmed because of frame/clock/readiness/scheduling issues despite offline progress. Show the proposed engineering response, with unresolved fixes labeled in progress. Use schematic traces unless original data is located and validated; never invent sampled curves.

## Evidence discipline and current status

Refresh N7 and N6 before finalizing, since active work may have advanced after this shell. Follow embedded supersession and exact experiment identities rather than assuming the most recently edited page makes all its historical paragraphs current. Record the final research cutoff.

Every numerical claim needs source, date/revision when available, environment, metric definition and limitation in notes or the evidence ledger. Distinguish planned / implemented offline / replay-tested / native disarmed / simulated flight / physical flight. Never aggregate milestone test counts into a new total.

The protected Classical S1 Gazebo success is a separate baseline. It is not hybrid completion, hardware qualification, repeated reliability, or a matched superiority result. A model rollout is not a native flight. C1's completed LAND is not a successful braking measurement. A synthetic chart cannot establish mission performance.

## Speaker notes and Q&A appendix

Each slide needs a spoken narrative, animation click cues, transition sentence, target duration, citations, and one evidence caveat where material. Keep notes separate from visible copy. Add a presenter timer and next-slide preview using the framework's supported features.

Appendix topics: why semantics rather than only waypoints; differences among geometry, topology and semantics; why G1 before G2; why the physics predictor remains default; what is genuinely learned; classical baseline identity; precise synthetic experiment setup; frame/clock ownership; map prior versus fresh clearance; registration and compute limits; failures and recovery; ablation plan; bibliography. Link an appendix menu from the closing slide without adding it to the timed 11-slide sequence.

Suggested judge answer: the novel project contribution is the integration and evaluation of a compact task-specific representation and bounded reasoning contracts. Scene graphs and hierarchical planning are established research. Our next claim to earn is measurable mission benefit under matched conditions.

## Deliverables

1. Finished local deck with 11 main slides, separate Q&A appendix, keyboard/clicker navigation, notes and offline assets.
2. Editable content manifest and small reusable visual components; keep narrative separate from rendering code.
3. Source/evidence ledger, asset provenance, and list of unresolved claims or missing media.
4. Deterministic static fallback plus print/PDF output, visually inspected. Notes must not leak into the audience PDF.
5. README with exact install/build/preview/presentation/export commands and navigation shortcuts.
6. A concise completion report stating what was implemented, checked, and remains illustrative.

## Acceptance checks

- Verify exactly 11 main slides and 600-second pacing; appendix excluded.
- Check content and citations against the ledger, especially latest status and baseline attribution.
- Build succeeds; all local assets resolve; no console errors or runtime network dependency.
- Test forward/back/direct navigation, fragment resets, overview, presenter notes, reduced motion, and closing-to-appendix-to-closing flow.
- Inspect every slide and important animation state at 1920×1080 and 1366×768; inspect exported pages for clipping, overlap, tiny labels or lost final-state information.
- Keyboard focus is visible; controls have labels; diagrams have text alternatives; color is not the only information channel.
- Chart values and scope labels match the source exactly.
- Rehearse the talk for the allotted 10 minutes and shorten visible material before speeding up narration.
- Preserve source changes narrowly within presentation files and report honestly if browser or export verification cannot be completed.

## Current editorial direction

Keep the seeing-to-understanding centerpiece. Use plain language throughout. Architecture has three audience-facing jobs: Remember, Decide, Fly safely. Follow with DATT and PINNs motivation, then Classical Dijkstra/A-star planning, sliding target tracking and PX4 control. Separate proposed DATT from offline predictor work. Keep benchmark numbers in Q&A. Preserve failures and pivots, but explain height, coordinates and timing before internal experiment IDs. At most two manual reveals in each supporting diagram.
