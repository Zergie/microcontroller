import { test } from 'node:test';
import assert from 'node:assert/strict';
import http from 'node:http';
import net from 'node:net';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';
import { createDevServer } from '../../shared/rp2040-usb-web/tools/dev-server.mjs';

async function until(condition) {
  const end = Date.now() + 3000;
  while (!condition()) { assert.ok(Date.now() < end, 'Proxy did not deliver expected bytes'); await new Promise(r => setTimeout(r, 10)); }
}
async function fixture(t, { rejectUpstream = false } = {}) {
  const sockets = new Set(), frames = [], upgrades = [];
  const board = http.createServer((req, res) => { res.writeHead(404); res.end(); });
  board.on('upgrade', (req, socket, head) => {
    upgrades.push(req.headers); sockets.add(socket); socket.on('error', () => {});
    socket.on('end', () => socket.destroy());
    socket.on('close', () => sockets.delete(socket));
    if (rejectUpstream) { socket.end('HTTP/1.1 503 Busy\r\nContent-Length: 0\r\n\r\n'); return; }
    const accept = createHash('sha1').update(req.headers['sec-websocket-key'] + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11').digest('base64');
    socket.write(`HTTP/1.1 101 Switching Protocols\r\nConnection: Upgrade\r\nUpgrade: websocket\r\nSec-WebSocket-Accept: ${accept}\r\n\r\n`);
    let buffer = head;
    const receive = data => {
      buffer = Buffer.concat([buffer, data]);
      while (buffer.length >= 8) {
        assert.equal(buffer[0], 0x82); assert.equal(buffer[1], 0x82);
        const mask = buffer[7] ^ buffer[3]; assert.equal(buffer[6] ^ buffer[2], 1);
        frames.push(mask); buffer = buffer.subarray(8);
        socket.write(Buffer.from([0x82, 2, 1, mask]));
      }
    };
    socket.on('data', receive);
    if (head.length) { buffer = Buffer.alloc(0); receive(head); }
  });
  await new Promise(resolve => board.listen(0, '127.0.0.1', resolve));
  const port = board.address().port;
  const app = await createDevServer({ target: `http://127.0.0.1:${port}`, padSocketPort: port,
    webRoot: fileURLToPath(new URL('../web/', import.meta.url)) });
  await new Promise(resolve => app.server.listen(0, '127.0.0.1', resolve));
  const host = `127.0.0.1:${app.server.address().port}`;
  t.after(async () => {
    await app.close(); for (const socket of sockets) socket.destroy();
    await new Promise(resolve => board.close(resolve));
  });
  async function connect({ origin = `http://${host}`, path = '/api/pads/ws', withFrame = false } = {}) {
    const socket = net.connect(app.server.address().port, '127.0.0.1');
    let response = Buffer.alloc(0); socket.on('error', () => {});
    socket.on('data', data => { response = Buffer.concat([response, data]); });
    t.after(() => socket.destroy());
    const headers = `GET ${path} HTTP/1.1\r\nHost: ${host}\r\nOrigin: ${origin}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Version: 13\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n`;
    socket.write(Buffer.concat([Buffer.from(headers), withFrame ? Buffer.from([0x82, 0x82, 0, 0, 0, 0, 1, 129]) : Buffer.alloc(0)]));
    await until(() => response.includes(Buffer.from('\r\n\r\n')));
    return { socket, response: () => response };
  }
  return { app, host, port, connect, frames, upgrades, sockets };
}

test('Dev proxy upgrades, forwards binary frames and handshake head, and closes both ends', async t => {
  const f = await fixture(t); const client = await f.connect({ withFrame: true });
  assert.match(client.response().toString(), /^HTTP\/1.1 101/);
  await until(() => f.frames.length === 1);
  client.socket.write(Buffer.from([0x82, 0x82, 1, 2, 3, 4, 0, 2])); // Mask command 1, value 0.
  await until(() => f.frames.length === 2 && client.response().includes(Buffer.from([0x82, 2, 1, 0])));
  assert.deepEqual(f.frames, [129, 0]);
  assert.equal(f.upgrades[0].origin, `http://127.0.0.1:${f.port}`);
  assert.equal(f.upgrades[0].host, `127.0.0.1:${f.port}`);
  client.socket.destroy(); await until(() => f.sockets.size === 0);
});
test('Dev proxy rejects foreign origins and unknown upgrade paths before connecting to the board', async t => {
  const f = await fixture(t);
  const foreign = await f.connect({ origin: 'http://evil.example' });
  assert.match(foreign.response().toString(), /^HTTP\/1.1 403/);
  const other = await f.connect({ path: '/other' });
  assert.match(other.response().toString(), /^HTTP\/1.1 404/);
  assert.equal(f.upgrades.length, 0);
});
test('Dev proxy handles unavailable WebSocket endpoints', async t => {
  const rejected = await fixture(t, { rejectUpstream: true });
  const client = await rejected.connect(); assert.match(client.response().toString(), /^HTTP\/1.1 502/);
});
