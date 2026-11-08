/* Packages the deck with local reveal.js assets. It copies an explicit allowlist
   into dist; source files and the original static shell remain untouched. */
const fs = require("node:fs");
const path = require("node:path");
const root = __dirname;
const assets = [
  "index.html",
  "style.css",
  "slides.js",
  "visuals.js",
  "continuous-scene.js",
  "appendix.js",
  "player.js",
];
const vendorFiles = {
  "dist/reveal.js": "reveal.js",
  "dist/reveal.css": "reveal.css",
  "dist/plugin/notes.js": "notes.js",
  LICENSE: "LICENSE",
};
fs.mkdirSync(path.join(root, "vendor"), { recursive: true });
fs.mkdirSync(path.join(root, "dist", "vendor"), { recursive: true });
for (const [source, destination] of Object.entries(vendorFiles)) {
  const contents = fs.readFileSync(
    path.join(root, "node_modules", "reveal.js", source),
  );
  fs.writeFileSync(path.join(root, "vendor", destination), contents);
  fs.writeFileSync(path.join(root, "dist", "vendor", destination), contents);
}
for (const file of assets)
  fs.copyFileSync(path.join(root, file), path.join(root, "dist", file));
fs.cpSync(path.join(root, "static"), path.join(root, "dist", "static"), {
  recursive: true,
});
console.log("Built dist with local assets and preserved static fallback.");
