import { test } from 'node:test';
import assert from 'node:assert/strict';
import { PinSession } from '../web/pin-session.js';

const snapshot = () => ({ pins: Array.from({ length: 30 }, (_, pin) => ({
  pin, available: ![23, 24, 29].includes(pin), mode: 'input', pull: 'none', value: false,
  frequency_hz: 0, actual_frequency_hz: 0, duty_percent: 0,
  drive_strength_ma: 4, slew_rate: 'slow', hysteresis: true
})) });

// Deterministic adapter: independent device state, controlled replies and no DOM.
function fixture() {
  const device = snapshot();
  const requests = [];
  const replies = [];
  const notifications = [];
  const session = new PinSession(async (path, method) => {
    requests.push({ path, method });
    return replies.length ? await replies.shift()(path, method) : structuredClone(device);
  }, () => notifications.push(session.state));
  return { session, device, requests, replies, notifications };
}

test('Refresh merges only edited fields; editing back to the applied value clears a draft', async () => {
  const { session, device, requests, notifications } = fixture();
  assert.deepEqual(session.state, { pins: null, busy: false, connection: 'connecting', error: '' });
  session.edit(null, 'mode', 'output');
  assert.equal(await session.refresh(), true);
  session.edit(0, 'mode', 'output');
  session.edit(0, 'mode', 'input');
  device.pins[0].mode = 'output';
  await session.refresh();
  assert.equal(session.values(0).mode, 'output');
  session.edit(0, 'mode', 'input-pullup');
  device.pins[0].drive_strength_ma = 8;
  device.pins[1].mode = 'pwm';
  await session.refresh();
  assert.equal(session.values(0).mode, 'input-pullup');
  assert.equal(session.values(0).drive, '8');
  assert.equal(session.values(1).mode, 'pwm');
  assert.equal(session.state.connection, 'online');
  assert.deepEqual(requests, Array(3).fill({ path: '/api/gpio.json', method: 'GET' }));
  assert.deepEqual(notifications.map(state => state.busy), [true, false, true, false, true, false]);
});

test('Apply captures the submitted pin, rejects overlapping requests and acknowledges only that pin', async () => {
  const { session, device, requests, replies } = fixture();
  await session.refresh();
  session.edit(25, 'mode', 'output');
  session.edit(25, 'level', '1');
  session.edit(0, 'mode', 'input-pullup');
  let complete;
  replies.push(() => new Promise(resolve => { complete = resolve; }));
  const pending = session.apply(25);
  assert.equal(session.state.busy, true);
  // A different selected pin only changes which values the caller reads.
  assert.equal(session.values(0).mode, 'input-pullup');
  assert.equal(await session.apply(0), false);
  assert.equal(await session.refresh(), false);
  assert.equal(requests.length, 2);
  device.pins[25].mode = 'output';
  device.pins[25].value = true;
  complete(structuredClone(device));
  assert.equal(await pending, true);
  assert.equal(session.state.busy, false);
  assert.equal(session.values(0).mode, 'input-pullup');
  device.pins[25].mode = 'input';
  device.pins[25].value = false;
  await session.refresh();
  assert.equal(session.values(25).mode, 'input');
  assert.equal(session.values(25).level, '0');
  assert.deepEqual(requests[1], { path: '/api/gpio/25/on/drive/4/slow', method: 'POST' });
});

test('Rejected commands keep drafts, classify a reachable device and allow a successful retry', async () => {
  const { session, device, replies } = fixture();
  await session.refresh();
  session.edit(25, 'mode', 'pwm');
  session.edit(25, 'frequency', '2000');
  replies.push(() => { throw Object.assign(new Error('PWM conflict'), { status: 409 }); });
  assert.equal(await session.apply(25), false);
  assert.equal(session.state.connection, 'online');
  assert.equal(session.state.error, 'PWM conflict');
  session.clearError();
  assert.equal(session.state.error, '');
  await session.refresh();
  assert.equal(session.values(25).frequency, '2000');
  Object.assign(device.pins[25], { mode: 'pwm', frequency_hz: 2000, duty_percent: 50 });
  assert.equal(await session.apply(25), true);
  device.pins[25].mode = 'input';
  await session.refresh();
  assert.equal(session.values(25).mode, 'input');
});

test('Legacy replies fetch a snapshot before acknowledging a draft; failed fallback keeps it', async () => {
  const { session, device, replies, requests } = fixture();
  await session.refresh();
  session.edit(25, 'mode', 'output');
  replies.push(() => ({ on: true }), () => { throw new Error('USB disconnected'); });
  assert.equal(await session.apply(25), false);
  assert.equal(session.state.connection, 'offline');
  assert.equal(session.values(25).mode, 'output');
  assert.deepEqual(requests.slice(-2).map(request => request.method), ['POST', 'GET']);
  device.pins[25].mode = 'output';
  replies.push(() => ({ on: true }));
  assert.equal(await session.apply(25), true);
  device.pins[25].mode = 'input';
  await session.refresh();
  assert.equal(session.values(25).mode, 'input');
});

test('Invalid snapshots cannot replace accepted state or acknowledge drafts', async () => {
  const { session, replies } = fixture();
  await session.refresh();
  const before = session.state.pins;
  session.edit(0, 'mode', 'output');
  const wrongOrder = snapshot();
  wrongOrder.pins.reverse();
  for (const malformed of [{ pins: [] }, wrongOrder, null]) {
    replies.push(() => malformed);
    assert.equal(await session.apply(0), false);
    assert.equal(session.state.pins, before);
    assert.equal(session.values(0).mode, 'output');
    assert.equal(session.state.busy, false);
  }
});

test('Timeouts, network/server failures and recovery preserve drafts and release busy state', async () => {
  const { session, replies } = fixture();
  await session.refresh();
  session.edit(0, 'mode', 'output');
  for (const error of [Object.assign(new Error('aborted'), { name: 'AbortError' }),
    new Error('Network failure'), Object.assign(new Error('Out of memory'), { status: 503 })]) {
    replies.push(() => { throw error; });
    assert.equal(await session.apply(0), false);
    assert.equal(session.state.connection, 'offline');
    assert.equal(session.state.busy, false);
    assert.equal(session.state.error, error.name === 'AbortError' ? 'No response. Check USB.' : error.message);
    assert.equal(session.values(0).mode, 'output');
    await session.refresh();
    assert.equal(session.state.connection, 'online');
    assert.equal(session.state.error, '');
  }
});

test('Passive, reserved and unloaded pins do not send commands; unsupported modes keep the draft', async () => {
  const { session, requests } = fixture();
  assert.equal(await session.apply(25), false);
  assert.equal(requests.length, 0);
  await session.refresh();
  for (const pin of [null, 23, 24, 29, 30]) assert.equal(await session.apply(pin), false);
  assert.equal(requests.length, 1);
  session.edit(25, 'mode', 'analog');
  assert.equal(await session.apply(25), false);
  assert.match(session.state.error, /not available/);
  assert.equal(session.values(25).mode, 'analog');
  assert.equal(requests.length, 1);
});
