/* Browser checks and audience exports for the presentation only. Uses an
   installed Edge browser by default; no browser profile or flight process is used. */
const { chromium } = require("playwright");
const fs = require("node:fs");
const path = require("node:path");
const assert = require("node:assert/strict");
const output = path.join(__dirname, "artifacts");
fs.mkdirSync(output, { recursive: true });

async function verify() {
  const browser = await chromium.launch({ channel: "msedge", headless: true });
  const page = await browser.newPage({
    viewport: { width: 1920, height: 1080 },
  });
  const errors = [];
  const remoteRequests = [];
  page.on("pageerror", (error) => errors.push(error.message));
  page.on("request", (request) => {
    if (
      !request.url().startsWith("http://127.0.0.1:4173") &&
      !request.url().startsWith("data:")
    )
      remoteRequests.push(request.url());
  });
  await page.goto("http://127.0.0.1:4173");
  await page.waitForFunction(() => window.deckReady);
  assert.equal(await page.locator("section[data-main]").count(), 11);
  for (const chapter of [3, 5, 6, 8, 9, 10, 11]) {
    const stages = await page
      .locator(`#chapter-${chapter} .fragment`)
      .evaluateAll((items) => [
        ...new Set(items.map((item) => item.dataset.fragmentIndex)),
      ]);
    assert.ok(
      stages.length <= 2,
      `Chapter ${chapter} should use at most two manual reveals.`,
    );
  }
  assert.equal(
    await page.evaluate(() =>
      window.slides.reduce((sum, slide) => sum + slide.time, 0),
    ),
    600,
  );
  await page.keyboard.press("ArrowRight");
  assert.equal(await page.evaluate(() => Reveal.getIndices().f), 0);
  await page.keyboard.press("ArrowLeft");
  assert.equal(await page.evaluate(() => Reveal.getIndices().f), -1);
  await page.evaluate(() => Reveal.slide(3, 0, -1));
  assert.equal(await page.evaluate(() => ContinuousScene.playing), true);
  await page.evaluate(() => ContinuousScene.seek(6.5));
  assert.equal(await page.locator(".observation-sample").count(), 64);
  const geometryPositions = await page
    .locator(".observation-sample")
    .evaluateAll((points) =>
      points.map((point) => [
        point.getAttribute("cx"),
        point.getAttribute("cy"),
      ]),
    );
  await page.evaluate(() => ContinuousScene.seek(11.8));
  const graphPositions = await page
    .locator(".observation-sample")
    .evaluateAll((points) =>
      points.map((point) => [
        point.getAttribute("cx"),
        point.getAttribute("cy"),
      ]),
    );
  assert.notDeepEqual(geometryPositions, graphPositions);
  await page.evaluate(() => ContinuousScene.seek(6.5));
  assert.deepEqual(
    await page
      .locator(".observation-sample")
      .evaluateAll((points) =>
        points.map((point) => [
          point.getAttribute("cx"),
          point.getAttribute("cy"),
        ]),
      ),
    geometryPositions,
  );
  await page.evaluate(() => ContinuousScene.seek(17.5));
  assert.equal(
    await page.locator("#decision-output").getAttribute("opacity"),
    "1",
  );
  await page.evaluate(() => {
    ContinuousScene.seek(19.9);
    ContinuousScene.play();
  });
  await page.waitForTimeout(350);
  assert.equal(await page.evaluate(() => ContinuousScene.playing), false);
  assert.equal(await page.evaluate(() => ContinuousScene.time), 20);
  await page.evaluate(() => ContinuousScene.seek(17.5));
  await page.keyboard.press("ArrowLeft");
  assert.equal(await page.evaluate(() => ContinuousScene.time), 12);
  await page.keyboard.press("ArrowRight");
  assert.equal(await page.evaluate(() => ContinuousScene.time), 20);
  await page.keyboard.press("ArrowRight");
  assert.equal(await page.evaluate(() => Reveal.getIndices().h), 4);
  assert.equal(await page.evaluate(() => ContinuousScene.playing), false);
  await page.evaluate(() => Reveal.slide(1, 0, 2));
  assert.equal(
    await page.locator(".memory-scene").getAttribute("data-step"),
    "3",
  );
  await page.evaluate(() => Reveal.slide(1, 0, -1));
  assert.equal(
    await page.locator(".memory-scene").getAttribute("data-step"),
    "0",
  );
  await page.evaluate(() => Reveal.slide(10, 0, 2));
  await page.keyboard.press("ArrowRight");
  assert.equal(await page.evaluate(() => Reveal.getIndices().h), 10);
  await page.locator("#appendix").click();
  await page.locator("#appendix-links button").first().click();
  assert.equal(await page.evaluate(() => Reveal.getIndices().h), 11);
  await page.locator(".present .appendix-return").click();
  assert.equal(await page.evaluate(() => Reveal.getIndices().h), 10);
  await page.locator("#overview").click();
  assert.equal(await page.evaluate(() => Reveal.isOverview()), true);
  await page.keyboard.press("Escape");
  await page.locator("#motion").click();
  assert.equal(
    await page.locator("#motion").getAttribute("aria-pressed"),
    "true",
  );
  const boundsIssues = [];
  for (const width of [1920, 1366]) {
    await page.setViewportSize({ width, height: width === 1920 ? 1080 : 768 });
    for (let index = 0; index < 17; index++) {
      await page.evaluate((index) => Reveal.slide(index, 0, 2), index);
      await page.waitForTimeout(100);
      const issues = await page.evaluate(() => {
        const section = Reveal.getCurrentSlide();
        const bounds = section.getBoundingClientRect();
        const issues = [];
        for (const item of section.querySelectorAll(
          "h1,h2,.subtitle,.takeaway,.scene svg text,.appendix-copy",
        )) {
          const box = item.getBoundingClientRect();
          if (
            box.right > bounds.right + 2 ||
            box.left < bounds.left - 2 ||
            box.bottom > bounds.bottom + 2
          )
            issues.push(item.textContent.slice(0, 80));
        }
        const scene = section.querySelector(".scene");
        const takeaway = section.querySelector(".takeaway");
        if (
          scene &&
          scene.getBoundingClientRect().bottom >
            takeaway.getBoundingClientRect().top + 2
        )
          issues.push("Scene overlaps takeaway");
        return issues;
      });
      if (issues.length) boundsIssues.push({ width, slide: index + 1, issues });
      await page.screenshot({
        path: path.join(
          output,
          `slide-${String(index + 1).padStart(2, "0")}-${width}.png`,
        ),
      });
    }
  }
  await page.goto("http://127.0.0.1:4173/?static");
  await page.waitForFunction(() => window.deckReady);
  await page.pdf({
    path: path.join(output, "DroneHacks-presentation.pdf"),
    width: "1600px",
    height: "900px",
    printBackground: true,
    preferCSSPageSize: true,
  });
  await page.goto("http://127.0.0.1:4173/?static&appendix");
  await page.waitForFunction(() => window.deckReady);
  await page.pdf({
    path: path.join(output, "DroneHacks-with-appendix.pdf"),
    width: "1600px",
    height: "900px",
    printBackground: true,
    preferCSSPageSize: true,
  });
  fs.writeFileSync(
    path.join(output, "verification.json"),
    JSON.stringify(
      {
        mainSlides: 11,
        seconds: 600,
        errors,
        remoteRequests,
        boundsIssues,
        checks: [
          "forward/back fragments",
          "memory reset",
          "closing boundary",
          "appendix return",
          "overview",
          "reduced motion",
          "17 slides at two viewports",
          "PDF exports",
        ],
      },
      null,
      2,
    ),
  );
  await browser.close();
  console.log(
    JSON.stringify({ errors, remoteRequests, boundsIssues }, null, 2),
  );
  assert.deepEqual(errors, []);
  assert.deepEqual(remoteRequests, []);
  assert.deepEqual(boundsIssues, []);
}
verify().catch((error) => {
  console.error(error);
  process.exitCode = 1;
});
