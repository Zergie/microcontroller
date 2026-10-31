import { test } from 'node:test';
import assert from 'node:assert/strict';
import { measureConnection, measureWebSocket } from '../web/statistics.js';
import { chromium } from 'playwright';
import { createDevServer } from '../../shared/rp2040-usb-web/tools/dev-server.mjs';
import { fileURLToPath } from 'node:url';

test('Connection timing counts full bodies, excludes warm-up and computes median/throughput', async () => {
  let clock = 0, calls = 0;
  const times = [999, 4, 3, 7, 2, 6, 1, 5];
  const result = await measureConnection({ now: () => clock, fetcher: async (path, options) => {
    assert.equal(options.cache, 'no-store');
    const ping = path.endsWith('network.json');
    const delay = ping ? times[calls++] : 10;
    return { ok: true, arrayBuffer: async () => {
      clock += delay;
      return new ArrayBuffer(ping ? 120 : 16384);
    } };
  } });
  assert.equal(result.latency, 4); assert.equal(result.minimum, 1); assert.equal(result.maximum, 7);
  assert.equal(result.samples, 7); assert.equal(result.bytes, 49152);
  assert.equal(result.mbps, 13.1072);
});

test('Old firmware retains latency; bad payloads and disconnects are errors', async () => {
  let clock = 0;
  const fetcher = async path => ({ ok: !path.endsWith('benchmark'), status: 404,
    arrayBuffer: async () => { clock++; return new ArrayBuffer(1); } });
  const result = await measureConnection({ now: () => clock, fetcher });
  assert.equal(result.downloadUnavailable, true); assert.equal(result.latency, 1);
  await assert.rejects(measureConnection({ fetcher: async () => ({ ok: true, arrayBuffer: async () => new ArrayBuffer(1) }) }), /payload/);
  await assert.rejects(measureConnection({ fetcher: async () => { throw new Error('Disconnected'); } }), /Disconnected/);
});

test('WebSocket timing excludes warm-up and closes on completion, cancellation and timeout', async () => {
  const originalLocation = globalThis.location;
  globalThis.location = { href: 'http://rp2040.local/', protocol: 'http:' };
  try {
    let clock = 0, closed = 0;
    const socketFactory = url => {
      assert.equal(url.href, 'ws://rp2040.local:81/api/pads/ws');
      const socket = { close() { closed++; }, send(bytes) {
        assert.equal(bytes[0], 5);
        queueMicrotask(() => {
          clock += bytes[1] ? bytes[1] : 900;
          socket.onmessage?.({ data: bytes.buffer });
        });
      } };
      queueMicrotask(() => socket.onopen?.());
      return socket;
    };
    const result = await measureWebSocket({ socketFactory, now: () => clock });
    assert.deepEqual(result, { latency: 4, minimum: 1, maximum: 7 });
    assert.equal(closed, 1);
    const controller = new AbortController();
    const stalled = () => ({ close() { closed++; } });
    const pending = measureWebSocket({ socketFactory: stalled, signal: controller.signal });
    controller.abort();
    await assert.rejects(pending, { name: 'AbortError' });
    await assert.rejects(measureWebSocket({ socketFactory: stalled, timeoutMs: 5 }), /timed out/);
    assert.equal(closed, 3);
  } finally {
    if (originalLocation === undefined) delete globalThis.location;
    else globalThis.location = originalLocation;
  }
});

test('Statistics menu, diagnostics, offline locking, cancellation and mobile layout', async t => {
  const app = await createDevServer({ target: 'http://127.0.0.1:9', webRoot: fileURLToPath(new URL('../web/', import.meta.url)) });
  await new Promise(resolve => app.server.listen(0, '127.0.0.1', resolve));
  const browser = await chromium.launch({ channel: process.platform === 'win32' ? 'msedge' : undefined });
  t.after(async () => { await browser.close(); await app.close(); });
  const page = await browser.newPage({ viewport: { width: 375, height: 667 } });
  const errors = []; page.on('pageerror', e => errors.push(e.message));
  let offline = false, legacy = false, hold = false, unblock;
  await page.routeWebSocket('**/api/pads/ws', ws => {
    ws.onMessage(message => {
      const bytes = [...message];
      assert.equal(bytes[0], 5);
      ws.send(Buffer.from(bytes));
    });
  });
  await page.route('**/api/**', async route => {
    const path = new URL(route.request().url()).pathname;
    if (offline) return route.fulfill({ status: 502, body: '{}' });
    if (path === '/api/system/info.json') return route.fulfill({ status: legacy ? 404 : 200, json: {
      chip: 'RP2040', board_id: '0123456789abcdef', cpu_hz: 125000000, flash_bytes: 2097152,
      sram_bytes: 270336, uptime_ms: 65000, firmware: 'Name Tag Webserver', build: 'test-build', sdk: '2.3.1', usb: 'NCM'
    } });
    if (path === '/api/system/network.json') {
      if (hold) await new Promise(resolve => { unblock = resolve; });
      return route.fulfill({ json: {} }).catch(() => {});
    }
    if (path === '/api/system/benchmark') return route.fulfill({ status: legacy ? 404 : 200, body: Buffer.alloc(legacy ? 0 : 16384) });
    return route.fulfill({ json: { boot_id: 'boot1', selection: 'random', current: 'solid', brightness_percent: 20,
      speed_percent: 100, interval_seconds: 300, color: '#00ff28', effects: ['solid'], pixel_count: 10, pin: 0 } });
  });
  await page.goto(`http://127.0.0.1:${app.server.address().port}`);
  await page.waitForFunction(() => document.getElementById('settings').dataset.online === 'true');
  const open = async view => { await page.locator('#menu-toggle').click(); await page.locator(`#view-${view}`).click({ timeout: 3000 }); };
  await open('statistics');
  await page.waitForFunction(() => document.getElementById('hardware-info').textContent.includes('test-build'));
  assert.equal(await page.locator('#settings').isVisible(), false);
  assert.equal(await page.locator('#pads-view').isVisible(), false);
  assert.deepEqual(await page.locator('#statistics-view h3').allTextContents(), ['Connection', 'Hardware & Firmware']);
  await page.waitForFunction(() => document.getElementById('connection-info').textContent.includes('Mbit/s'));
  await page.waitForFunction(() => document.getElementById('connection-info').textContent.includes('WebSocket latency')) ;
  assert.match(await page.locator('#connection-info').textContent(), /HTTP latency.*WebSocket latency/s);
  assert.doesNotMatch(await page.locator('#connection-info').textContent(), /Unavailable/);
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
  await page.setViewportSize({ width: 667, height: 375 });
  assert.equal(await page.evaluate(() => document.documentElement.scrollWidth <= innerWidth), true);
  hold = true;
  await page.locator('#measure-connection').click();
  await page.waitForFunction(() => document.getElementById('measure-connection').disabled);
  await open('animations');
  hold = false; unblock?.();
  await open('statistics');
  await page.waitForFunction(() => !document.getElementById('measure-connection').disabled);
  for (const event of ['visibilitychange', 'pagehide']) {
    hold = true;
    await page.locator('#measure-connection').click();
    await page.waitForFunction(() => document.getElementById('measure-connection').disabled);
    await page.evaluate(event => {
      if (event === 'visibilitychange') {
        Object.defineProperty(document, 'hidden', { configurable: true, value: true });
        document.dispatchEvent(new Event(event));
      } else window.dispatchEvent(new Event(event));
    }, event);
    hold = false; unblock?.();
    assert.equal(await page.locator('#measure-connection').isEnabled(), true);
    assert.equal(await page.locator('#measurement-status').count(), 0);
    await page.evaluate(() => {
      Object.defineProperty(document, 'hidden', { configurable: true, value: false });
      document.dispatchEvent(new Event('visibilitychange'));
    });
  }
  legacy = true;
  await open('animations'); await open('statistics');
  await page.waitForFunction(() => document.getElementById('statistics-info-status').textContent.includes('update required'));
  await page.waitForFunction(() => !document.getElementById('measure-connection').disabled);
  assert.equal(await page.locator('#connection-info > div').filter({ hasText: 'HTTP download' }).locator('dd').textContent(), '—');
  offline = true;
  await page.evaluate(() => document.dispatchEvent(new Event('visibilitychange')));
  await page.waitForFunction(() => document.getElementById('measure-connection').disabled);
  assert.deepEqual(errors, []);
});
