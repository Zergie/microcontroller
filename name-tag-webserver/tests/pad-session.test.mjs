import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createPadSession, defaultPadColors } from '../web/pad-session.js';
import { createPadTransport } from '../web/pad-transport.js';

const settle = async () => { for (let i = 0; i < 12; i++) await Promise.resolve(); };
const deferred = () => { let resolve, reject; const promise = new Promise((yes, no) => { resolve = yes; reject = no; }); return { promise, resolve, reject }; };
function fixture(overrides = {}) {
  const calls = [], audio = [], pending = [];
  let callbacks;
  const transport = { state: { busy: false, pending: false, ready: true },
    setConnection: (...args) => calls.push(['connection', ...args]),
    update: data => calls.push(['update', data]), pulse: (...args) => calls.push(['pulse', ...args]),
    cancelPulse() {}, pause() {}, resume() {}, heartbeat() {} };
  const session = createPadSession({
    audio: { unlock: () => audio.push('unlock'), stopAll: () => audio.push('stop'),
      setMask: (mask, fade, pitches) => audio.push({ mask, fade, pitches }) },
    request: () => { const result = deferred(); pending.push(result); return result.promise; },
    transportFactory: options => { callbacks = options; return transport; },
    ...overrides
  });
  const status = { colors: defaultPadColors, fade_ms: 1700 };
  return { session, calls, audio, pending, status, transport, fail: e => callbacks.onFailure(e) };
}

test('Session loads without commanding hardware; restored pads use one atomic view transition', async () => {
  const f = fixture(); f.session.setConnection(true, true);
  f.pending[0].resolve(f.status); await settle();
  assert.equal(f.calls.filter(c => c[0] === 'update').length, 0);
  f.session.setView('pads', false);
  assert.equal(f.session.state.fadeMs, 1700);
  assert.equal(f.calls.at(-1)[1].active, true);
});

test('Session counts fingers, swipe gaps and keyboard holds through the same mask', async () => {
  const f = fixture(); f.session.setView('pads'); f.session.setConnection(true, true);
  f.pending[0].resolve(f.status); await settle();
  f.session.touch(1, 7, true); f.session.touch(2, 7, true); f.session.release(1);
  assert.equal(f.session.state.mask, 128);
  f.session.touch(2, -1); assert.equal(f.session.state.mask, 0);
  f.session.touch(2, 6); f.session.touch('key:7', 7, true);
  assert.equal(f.session.state.mask, 192);
  f.session.pause(); assert.equal(f.session.state.mask, 0);
  assert.equal(f.session.state.enabled, false);
  f.session.resume(); assert.equal(f.session.state.mask, 0);
  assert.equal(f.session.state.enabled, true);
  assert.equal(f.audio.filter(c => c === 'unlock').length, 3);
});

test('A stale load cannot overwrite a newer connection or unlock offline controls', async () => {
  const f = fixture(); f.session.setView('pads'); f.session.setConnection(true, true);
  f.session.setConnection(false, true); f.session.setConnection(true, true);
  f.pending[1].resolve({ ...f.status, fade_ms: 300 }); await settle();
  f.pending[0].resolve(f.status); await settle();
  assert.equal(f.session.state.fadeMs, 300);
  f.session.touch(1, 1, true); f.fail(new Error('Disconnected'));
  assert.equal(f.session.state.mask, 0); assert.equal(f.session.state.enabled, false);
});

test('Song notes are transient and do not modify manual preferences across view changes', async () => {
  let events;
  const f = fixture({ playerFactory: callbacks => {
    events = callbacks; let playing = false;
    return { play() { playing = true; callbacks.onState(true); },
      stop() { if (playing) { playing = false; callbacks.onMask(0); callbacks.onState(false); } },
      isPlaying: () => playing };
  } });
  f.session.setConnection(true, true); f.pending[0].resolve(f.status); await settle();
  f.session.setView('songs'); f.session.toggleSong(); events.onMask(128, { 7: 60 }, 536);
  assert.deepEqual(f.calls.at(-1), ['pulse', 128, 536]);
  assert.equal(f.session.state.fadeMs, 1700);
  f.session.setView('pads');
  assert.equal(f.session.state.mask, 0); assert.equal(f.session.state.fadeMs, 1700);
  assert.equal(f.calls.at(-1)[1].fadeMs, 1700);
});

function network(fast = false) {
  const requests = [], waiting = [], notes = [], masks = [];
  let callbacks, open = false, now = 0, failures = 0;
  const transport = createPadTransport({ now: () => now,
    request(path) { requests.push(path); const d = deferred(); waiting.push(d); return d.promise; },
    onFailure() { failures++; },
    streamFactory(options) { callbacks = options; return {
      isReady: () => open, connect() {}, close() { open = false; },
      heartbeat() {}, setMask: mask => masks.push(mask),
      pulse: (...args) => notes.push(args)
    }; }
  });
  transport.setConnection(true, fast ? 'websocket-v1' : undefined);
  const config = { active: true, fadeMs: 1700, colors: defaultPadColors, mask: 0 };
  return { transport, requests, waiting, notes, masks, config,
    open() { open = true; callbacks.onReady(); }, time(value) { now = value; }, failures: () => failures };
}

test('HTTP adapter serializes configurations, holds and pulse commands without rewriting fade', async () => {
  const n = network(); n.transport.update(n.config);
  n.transport.pulse(128, 536);
  n.waiting[0].resolve({}); await settle();
  assert.equal(n.requests[1], '/api/pads/pulse/128/536');
  n.transport.update({ ...n.config, active: false });
  n.waiting[1].resolve({}); await settle();
  assert.equal(n.requests[2], '/api/view/animations');
  n.waiting[2].resolve({}); await settle();
  assert.equal(n.transport.state.busy, false);
});

test('WebSocket adapter sends held masks during HTTP work; pulses wait for configuration and readiness', async () => {
  const n = network(true); n.transport.update(n.config);
  n.transport.update({ ...n.config, mask: 128 });
  assert.equal(n.masks.at(-1), 128);
  n.transport.pulse(64, 536); assert.deepEqual(n.notes, []);
  n.waiting[0].resolve({}); await settle(); n.open(); await settle();
  assert.deepEqual(n.notes, [[64, 536]]);
  assert.equal(n.requests.length, 1);
});

test('Expired and interrupted notes are dropped rather than replayed after a slow request', async () => {
  for (const interruption of ['expired', 'pause', 'view', 'offline']) {
    const n = network(true); n.transport.update(n.config); n.transport.pulse(128, 100);
    if (interruption === 'expired') n.time(101);
    if (interruption === 'pause') n.transport.pause();
    if (interruption === 'view') n.transport.update({ ...n.config, active: false });
    if (interruption === 'offline') n.transport.setConnection(false, 'websocket-v1');
    n.waiting[0].resolve({}); await settle(); n.open(); await settle();
    assert.deepEqual(n.notes, [], interruption);
    if (n.waiting[1]) { n.waiting[1].resolve({}); await settle(); }
  }
});

test('Old request failures do not take a recovered transport offline', async () => {
  const n = network(true); n.transport.update(n.config);
  n.transport.setConnection(false, 'websocket-v1'); n.transport.setConnection(true, 'websocket-v1');
  n.waiting[0].reject(new Error('Old connection')); await settle();
  assert.equal(n.failures(), 0); assert.equal(n.requests.length, 2);
  n.waiting[1].resolve({}); await settle();
});

test('Reconnection never replays a held HTTP mask before the session supplies its latest state', async () => {
  const n = network(); n.transport.update({ ...n.config, mask: 128 });
  n.transport.setConnection(false);
  n.waiting[0].resolve({}); await settle();
  n.transport.setConnection(true); await settle();
  assert.match(n.requests.at(-1), /^\/api\/pads\/set\/0\//);
  n.waiting[1].resolve({}); await settle();
});

test('An HTTP note sends only its pulse after configuration is acknowledged', async () => {
  const n = network(); n.transport.update(n.config);
  n.waiting[0].resolve({}); await settle();
  n.transport.pulse(128, 100);
  assert.equal(n.requests.length, 2);
  assert.equal(n.requests[1], '/api/pads/pulse/128/100');
  n.time(101); n.waiting[1].resolve({}); await settle();
  assert.equal(n.requests.length, 2);
  assert.equal(n.transport.state.busy, false);
});

test('Identical preferences and idle HTTP heartbeats produce no requests', async () => {
  const n = network(); n.transport.update(n.config);
  n.waiting[0].resolve({}); await settle();
  n.transport.update({ ...n.config, colors: [...n.config.colors] });
  n.transport.heartbeat(); await settle();
  assert.equal(n.requests.length, 1);
});

test('HTTP holds and lease renewal remain independent of notes and configuration changes', async () => {
  const n = network(); n.transport.update(n.config);
  n.waiting[0].resolve({}); await settle();
  n.transport.update({ ...n.config, mask: 128 });
  assert.match(n.requests[1], /^\/api\/pads\/set\/128\/1700\//);
  n.waiting[1].resolve({}); await settle();
  n.transport.heartbeat();
  assert.equal(n.requests[2], n.requests[1]);
  n.waiting[2].resolve({}); await settle();
  n.transport.update({ ...n.config, mask: 0 });
  assert.match(n.requests[3], /^\/api\/pads\/set\/0\/1700\//);
  n.waiting[3].resolve({}); await settle();
  n.transport.pulse(64, 536);
  assert.equal(n.requests[4], '/api/pads/pulse/64/536');
  n.waiting[4].resolve({}); await settle();
});

test('A real configuration change still precedes notes and expired notes stay discarded', async () => {
  for (const fast of [false, true]) {
    const n = network(fast); n.transport.update(n.config);
    n.waiting[0].resolve({}); await settle();
    if (fast) n.open();
    n.transport.update({ ...n.config, fadeMs: 300 });
    n.transport.pulse(128, 100);
    assert.equal(n.requests.length, 2);
    assert.match(n.requests[1], fast ? /^\/api\/pads\/config\/300\// : /^\/api\/pads\/set\/0\/300\//);
    n.time(101); n.waiting[1].resolve({}); await settle();
    assert.equal(n.requests.length, 2);
    assert.deepEqual(n.notes, []);
    assert.equal(n.transport.state.pending, false);
  }
});
