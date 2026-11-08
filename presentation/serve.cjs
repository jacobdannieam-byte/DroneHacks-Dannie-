/* Serves only this presentation on localhost for preview and speaker notes.
   This is a development server, not a deployment or robotics service. */
const http = require("node:http");
const fs = require("node:fs");
const path = require("node:path");
const root = __dirname;
const port = 4173; // Our chosen local presentation-preview port.
const types = {
  ".html": "text/html",
  ".js": "text/javascript",
  ".css": "text/css",
  ".png": "image/png",
  ".pdf": "application/pdf",
};
http
  .createServer((request, response) => {
    const pathname = decodeURIComponent(
      new URL(request.url, "http://localhost").pathname,
    );
    const file = path.resolve(
      root,
      "." + (pathname === "/" ? "/index.html" : pathname),
    );
    if (!file.startsWith(root + path.sep)) {
      response.writeHead(403);
      response.end("This path is outside the presentation.");
      return;
    }
    fs.readFile(file, (error, contents) => {
      if (error) {
        response.writeHead(404);
        response.end("File not found. Run npm run build, then open /.");
        return;
      }
      response.writeHead(200, {
        "Content-Type": types[path.extname(file)] || "application/octet-stream",
        "Cache-Control": "no-store",
      });
      response.end(contents);
    });
  })
  .listen(port, "127.0.0.1", () =>
    console.log(`Presentation preview: http://127.0.0.1:${port}`),
  );
