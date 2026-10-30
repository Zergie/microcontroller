import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createHttpTransport } from '../web/gpio-transport.js';

test('HTTP adapter preserves relative paths, empty POST bodies, no-store and parsed JSON', async () => {
  const calls = [];
  const request = createHttpTransport(async (path, options) => {
    calls.push({ path, options });
    return Response.json({ pins: [] });
  });
  assert.deepEqual(await request('/api/gpio.json', 'GET'), { pins: [] });
  await request('/api/gpio/25/on', 'POST');
  assert.equal(calls[0].path, '/api/gpio.json');
  assert.equal(calls[0].options.method, 'GET');
  assert.equal(calls[0].options.cache, 'no-store');
  assert.equal(Object.hasOwn(calls[0].options, 'body'), false);
  assert.equal(calls[1].path, '/api/gpio/25/on');
  assert.equal(calls[1].options.body, '');
  assert.equal(calls[1].options.signal.aborted, false);
});

test('HTTP errors carry status and the existing user message; invalid JSON and network failures reject', async () => {
  const expected = new Map([
    [400, 'Check values: 10–100,000 Hz, 0–100%.'],
    [403, 'This pin is reserved.'],
    [404, 'API unavailable. Check firmware.'],
    [409, 'PWM conflict: frequency or channel already in use.'],
    [502, 'Pico unreachable. Check USB.'],
    [503, 'Device error']
  ]);
  for (const [status, message] of expected) {
    const request = createHttpTransport(async () => Response.json({ error: 'Device error' }, { status }));
    await assert.rejects(request('/api/gpio.json', 'GET'), error => error.status === status && error.message === message);
  }
  const unknown = createHttpTransport(async () => Response.json({}, { status: 500 }));
  await assert.rejects(unknown('/api/gpio.json', 'GET'), /HTTP 500/);
  const invalid = createHttpTransport(async () => new Response('<html>missing</html>'));
  await assert.rejects(invalid('/api/gpio.json', 'GET'), /API unavailable. Check USB./);
  const offline = createHttpTransport(async () => { throw new Error('Network failure'); });
  await assert.rejects(offline('/api/gpio.json', 'GET'), /Network failure/);
});

test('The six-second timeout aborts the request and completed requests clear their timers', async t => {
  t.mock.timers.enable({ apis: ['setTimeout'] });
  let signal;
  const request = createHttpTransport(async (_path, options) => {
    signal = options.signal;
    return new Promise((_resolve, reject) => signal.addEventListener('abort', () =>
      reject(Object.assign(new Error('aborted'), { name: 'AbortError' })), { once: true }));
  });
  const result = request('/api/gpio.json', 'GET');
  const rejected = assert.rejects(result, { name: 'AbortError' });
  t.mock.timers.tick(5999);
  assert.equal(signal.aborted, false);
  t.mock.timers.tick(1);
  await rejected;
  assert.equal(signal.aborted, true);

  const completed = createHttpTransport(async (_path, options) => {
    signal = options.signal;
    return Response.json({ on: true });
  });
  await completed('/api/led.json', 'GET');
  t.mock.timers.tick(6000);
  assert.equal(signal.aborted, false);
});
