# Presentation validation

Verified November 8, 2026 on Windows with Microsoft Edge and Playwright.

- 11 main slides; target durations total 600 seconds. Six Q&A slides are outside the normal forward path.
- Browser checks pass: forward/backward fragments, memory-scene reset, closing boundary, Q&A selection and return, overview, reduced-motion toggle.
- Final states of all 17 slides checked at 1920×1080 and 1366×768: no detected bounds violations or scene/takeaway overlap. Slide contact sheet visually reviewed.
- No browser JavaScript errors or external playback requests in the automated run.
- Presenter popup manually inspected through browser screenshots: current/next state, notes, elapsed timer and slide pacing are present.
- Animated memory scene verified with motion enabled: camera rotation changes and returns to the initial state when seeking backward.
- Continuous slide 4: 64 illustrative samples retain their elements while positions transform from geometry into graph nodes; seeking backward exactly reproduces prior positions. Decision visibility, keyboard stage stepping and pause-on-exit pass. The animation plays once and holds, with pause/replay/scrub and static reduced-motion stages.
- Audience PDFs: 11 main pages and 17 pages with appendix; notes excluded. PDF rendering reviewed separately after correcting reveal.js's default print styling.
- Original static shell preserved. Local build contains the player, visuals, content and reveal.js assets.

Original diagrams are illustrative. No recordings, measured telemetry curves, or actual course geometry are fabricated. Numerical chart values come from the research ledger. No full-course hybrid success or hardware qualification is claimed.

The 600-second pacing is a planned allocation. A human delivery rehearsal, projector color check and presentation-laptop clicker check remain recommended. There is no narration audio or automated 10-minute playback. The browser preview runs locally, not on a publicly deployed website.

## Asset provenance

- All scenes and charts: original SVG authored for this deck.
- Typeface: local Georgia and Arial/Helvetica system fallbacks; no network font dependency.
- reveal.js 6.0.2: locally vendored, MIT license retained at `vendor/LICENSE`.
- No 3Blue1Brown logo, character, screenshot or video is used. Visual inspiration is cited in the Q&A bibliography.
- Paper sources and project-specific evidence: `RESEARCH.md`.
