import http from 'node:http';
import { watch } from 'node:fs';
import { readFile, realpath, stat } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const defaultRoot = fileURLToPath(new URL('../web/', import.meta.url));
const mime = { '.html': 'text/html; charset=utf-8', '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8', '.svg': 'image/svg+xml', '.png': 'image/png', '.ico': 'image/x-icon' };
const client = `const events = new EventSource('/__dev/events');
let version;
events.addEventListener('ready', e => { if(version && version !== e.data) location.reload(); version = e.data; });
events.addEventListener('change', () => location.reload());`;

export async function createDevServer({ target = 'http://192.168.7.1', webRoot = defaultRoot } = {}) {
  const upstreamUrl = new URL(target);
  if (upstreamUrl.protocol !== 'http:' || upstreamUrl.username || upstreamUrl.password ||
      upstreamUrl.pathname !== '/' || upstreamUrl.search || upstreamUrl.hash) {
    throw new Error('Target must be an HTTP base URL without a path or credentials.');
  }
  const root = await realpath(webRoot);
  const peers = new Set();
  let revision = `${Date.now()}`;
  let debounce;
  function sendError(response, status, error) {
    if (response.headersSent) { response.destroy(); return; }
    response.writeHead(status, { 'Content-Type': 'application/json; charset=utf-8', 'Cache-Control': 'no-store' });
    response.end(JSON.stringify({ error }));
  }
  const server = http.createServer(async (request, response) => {
    try {
      const url = new URL(request.url, 'http://localhost');
      if (url.pathname.startsWith('/api/')) {
        if (!['GET', 'POST'].includes(request.method)) {
          sendError(response, 405, 'Only GET and POST are allowed.'); return;
        }
        // The frontend uses the same origin. Other websites cannot issue proxied commands.
        const origin = request.headers.origin;
        if (origin && origin !== `http://${request.headers.host}`) {
          sendError(response, 403, 'Request origin is not allowed.'); return;
        }
        if (request.headers['transfer-encoding'] || Number(request.headers['content-length'] || 0) > 1024) {
          sendError(response, 413, 'API request is too large.'); return;
        }
        const headers = {};
        for (const key of ['content-type', 'content-length']) {
          if (request.headers[key]) headers[key] = request.headers[key];
        }
        const upstream = http.request({ hostname: upstreamUrl.hostname, port: upstreamUrl.port || 80,
          path: url.pathname + url.search, method: request.method, headers, timeout: 5000 }, incoming => {
          const outgoing = { ...incoming.headers, 'cache-control': 'no-store' };
          for (const key of ['connection', 'keep-alive', 'transfer-encoding']) delete outgoing[key];
          response.writeHead(incoming.statusCode, outgoing);
          incoming.on('error', () => response.destroy());
          incoming.pipe(response);
        });
        upstream.on('timeout', () => upstream.destroy(new Error('Timeout')));
        upstream.on('error', () => sendError(response, 502, `RP2040 at ${upstreamUrl.origin} is unreachable. Check the USB connection.`));
        request.on('aborted', () => upstream.destroy());
        response.on('close', () => upstream.destroy());
        request.pipe(upstream);
        return;
      }
      if (!['GET', 'HEAD'].includes(request.method)) {
        sendError(response, 405, 'Static files allow only GET and HEAD.'); return;
      }
      if (url.pathname === '/__dev/events') {
        response.writeHead(200, { 'Content-Type': 'text/event-stream', 'Cache-Control': 'no-store', Connection: 'keep-alive' });
        response.write(`event: ready\ndata: ${revision}\n\n`);
        peers.add(response);
        response.on('close', () => peers.delete(response));
        return;
      }
      if (url.pathname === '/__dev/client.js') {
        response.writeHead(200, { 'Content-Type': mime['.js'], 'Cache-Control': 'no-store' });
        response.end(request.method === 'HEAD' ? undefined : client); return;
      }
      let relative;
      try { relative = decodeURIComponent(url.pathname === '/' ? '/index.html' : url.pathname); }
      catch { sendError(response, 400, 'Invalid file path.'); return; }
      const candidate = path.resolve(root, '.' + relative);
      const inside = filename => filename.startsWith(root + path.sep);
      if (!inside(candidate)) { sendError(response, 403, 'File path is outside the web directory.'); return; }
      let filename;
      try {
        filename = await realpath(candidate);
        if (!inside(filename)) { sendError(response, 403, 'File path is outside the web directory.'); return; }
        if (!(await stat(filename)).isFile()) throw new Error('Not a file');
      } catch { sendError(response, 404, 'File not found.'); return; }
      let content = await readFile(filename);
      if (path.extname(filename) === '.html') {
        content = Buffer.from(content.toString('utf8').replace(/<\/body>/i, '<script src="/__dev/client.js"></script></body>'));
      }
      response.writeHead(200, { 'Content-Type': mime[path.extname(filename)] || 'application/octet-stream',
        'Cache-Control': 'no-store', 'Content-Length': content.length });
      response.end(request.method === 'HEAD' ? undefined : content);
    } catch {
      sendError(response, 500, 'Development server could not process the request.');
    }
  });
  const watcher = watch(root, { recursive: true }, () => {
    clearTimeout(debounce);
    debounce = setTimeout(() => {
      revision = `${Date.now()}`;
      for (const peer of peers) peer.write(`event: change\ndata: ${revision}\n\n`);
    }, 100);
  });
  watcher.on('error', error => console.error('Live-reload file watcher:', error.message));
  const heartbeat = setInterval(() => { for (const peer of peers) peer.write(': keepalive\n\n'); }, 15000);
  heartbeat.unref();
  return { server, async close() {
    clearTimeout(debounce);
    clearInterval(heartbeat);
    watcher.close();
    for (const peer of peers) peer.end();
    server.closeAllConnections();
    await new Promise((resolve, reject) => server.close(error => error ? reject(error) : resolve()));
  } };
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  let port = 5173;
  let target = 'http://192.168.7.1';
  for (let i = 2; i < process.argv.length; i += 2) {
    if (process.argv[i] === '--port') port = Number(process.argv[i + 1]);
    else if (process.argv[i] === '--target') target = process.argv[i + 1];
    else throw new Error(`Unknown parameter: ${process.argv[i]}`);
  }
  if (!Number.isInteger(port) || port < 1 || port > 65535) throw new Error('Invalid port.');
  const app = await createDevServer({ target });
  app.server.on('error', async error => { console.error(error.message); await app.close().catch(() => {}); process.exitCode = 1; });
  app.server.listen(port, '127.0.0.1', () => console.log(`Live-Reload: http://127.0.0.1:${port}/ -> API: ${target}`));
  for (const signal of ['SIGINT', 'SIGTERM']) process.on(signal, async () => { await app.close(); });
}
