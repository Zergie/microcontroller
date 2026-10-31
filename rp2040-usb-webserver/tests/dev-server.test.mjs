import { test } from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import { mkdtemp, writeFile, rm } from 'node:fs/promises';
import { tmpdir } from 'node:os';
import path from 'node:path';
import { createDevServer } from '../../shared/rp2040-usb-web/tools/dev-server.mjs';

function listen(server) {
  return new Promise(resolve => server.listen(0, '127.0.0.1', () => resolve(`http://127.0.0.1:${server.address().port}`)));
}

test('Live reload, files and API proxy', async t => {
  const root = await mkdtemp(path.join(tmpdir(), 'rp2040-web-'));
  await writeFile(path.join(root, 'index.html'), '<body>Version 1</body>');
  let received;
  const upstream = http.createServer(async (request, response) => {
    let body = '';
    for await (const chunk of request) body += chunk;
    received = { method: request.method, url: request.url, body };
    response.writeHead(request.url.includes('conflict') ? 409 : 200, { 'Content-Type': 'application/json' });
    response.end(JSON.stringify({ on: true }));
  });
  const target = await listen(upstream);
  const app = await createDevServer({ target, webRoot: root });
  const base = await listen(app.server);
  t.after(async () => {
    await app.close();
    upstream.closeAllConnections();
    await new Promise(resolve => upstream.close(resolve));
    await rm(root, { recursive: true, force: true });
  });
  await t.test('Files are served with live reload and without caching', async () => {
    const response = await fetch(base);
    assert.equal(response.status, 200);
    assert.equal(response.headers.get('cache-control'), 'no-store');
    assert.match(await response.text(), /Version 1.*__dev\/client.js/);
    assert.equal((await fetch(`${base}/missing`)).status, 404);
  });
  await t.test('API POSTs and error statuses are forwarded unchanged', async () => {
    const response = await fetch(`${base}/api/led/on`, { method: 'POST', body: '' });
    assert.deepEqual(await response.json(), { on: true });
    assert.deepEqual(received, { method: 'POST', url: '/api/led/on', body: '' });
    assert.equal((await fetch(`${base}/api/conflict`)).status, 409);
  });
  await t.test('Foreign origins and paths outside web are rejected', async () => {
    assert.equal((await fetch(`${base}/api/led/on`, { method: 'POST', headers: { Origin: 'https://example.invalid' } })).status, 403);
    const traversal = await fetch(`${base}/%2e%2e%5csecret.txt`);
    assert.ok([403, 404].includes(traversal.status));
    assert.equal((await fetch(`${base}/api/led/on`, { method: 'DELETE' })).status, 405);
  });
  await t.test('A file change triggers an SSE reload event', async () => {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 4000);
    try {
      const response = await fetch(`${base}/__dev/events`, { signal: controller.signal });
      const reader = response.body.getReader();
      const first = new TextDecoder().decode((await reader.read()).value);
      assert.match(first, /event: ready/);
      await writeFile(path.join(root, 'index.html'), '<body>Version 2</body>');
      let events = '';
      while (!events.includes('event: change')) {
        events += new TextDecoder().decode((await reader.read()).value);
      }
      assert.match(await (await fetch(base)).text(), /Version 2/);
      await reader.cancel();
    } finally { clearTimeout(timeout); controller.abort(); }
  });
  await t.test('A missing board produces a readable 502 response', async () => {
    upstream.closeAllConnections();
    await new Promise(resolve => upstream.close(resolve));
    const response = await fetch(`${base}/api/gpio.json`);
    assert.equal(response.status, 502);
    assert.match((await response.json()).error, /unreachable/);
  });
});
