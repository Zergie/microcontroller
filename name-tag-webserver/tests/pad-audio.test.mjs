import { test } from 'node:test';
import assert from 'node:assert/strict';
import { createPadAudio } from '../web/pad-audio.js';
import { installFakeAudio } from './fakes/pad-audio.mjs';

function setup(mode) {
  const trace = installFakeAudio({ mode });
  const audio = createPadAudio();
  return { trace, audio };
}
const last = param => param.events.at(-1);

test('Song octave overrides use the same G voice and manual pads restore G4', () => {
  const { trace, audio } = setup(); audio.unlock();
  audio.setMask(8, 20, { 3: 55 });
  const context = trace.contexts[0], frequency = context.oscillators[3].frequency;
  assert.ok(Math.abs(last(frequency).value - 195.99771799) < 0.00001);
  audio.setMask(0, 20); context.currentTime = 0.1;
  audio.setMask(8, 1000);
  assert.ok(Math.abs(last(frequency).value - 391.99543598) < 0.00001);
});

test('Playback session is requested on touch before creating/resuming audio, never on load', t => {
  const descriptor = Object.getOwnPropertyDescriptor(globalThis, 'navigator');
  t.after(() => descriptor ? Object.defineProperty(globalThis, 'navigator', descriptor) : delete globalThis.navigator);
  const { trace, audio } = setup();
  let type = 'auto';
  Object.defineProperty(globalThis, 'navigator', { configurable: true, value: { audioSession: {
    get type() { return type; },
    set type(value) { type = value; trace.calls.push({ type: 'session', value }); }
  } } });
  assert.equal(type, 'auto'); assert.equal(trace.calls.length, 0);
  audio.unlock();
  assert.equal(type, 'playback');
  assert.deepEqual(trace.calls.map(call => call.type), ['session', 'create', 'resume']);
  type = 'auto'; trace.contexts[0].state = 'interrupted';
  audio.unlock();
  assert.equal(type, 'playback');
  assert.deepEqual(trace.calls.slice(-2).map(call => call.type), ['session', 'resume']);
});

test('Unavailable or rejected AudioSession never prevents Web Audio or LED interaction', t => {
  const descriptor = Object.getOwnPropertyDescriptor(globalThis, 'navigator');
  t.after(() => descriptor ? Object.defineProperty(globalThis, 'navigator', descriptor) : delete globalThis.navigator);
  const cases = [{}, { get audioSession() { throw new Error('unavailable'); } },
    { audioSession: { get type() { return 'auto'; }, set type(value) { throw new Error('denied'); } } }];
  for (const navigator of cases) {
    Object.defineProperty(globalThis, 'navigator', { configurable: true, value: navigator });
    const { trace, audio } = setup();
    assert.doesNotThrow(() => { audio.unlock(); audio.setMask(1, 1000); });
    assert.equal(last(trace.contexts[0].gains[1].gain).value, 0.125);
  }
});

test('Audio is lazy; visual left-to-right notes are C4 D4 E4 F4 G4 A4 H4 C5 with bounded sine polyphony', () => {
  const { trace, audio } = setup();
  audio.setMask(255, 2000); assert.equal(trace.contexts.length, 0);
  audio.unlock(); const context = trace.contexts[0];
  assert.equal(context.gains[0].gain.value, 0.25);
  const frequencies = context.oscillators.toReversed().map(o => o.frequency.value);
  [60, 62, 64, 65, 67, 69, 71, 72].forEach((note, i) => {
    assert.ok(Math.abs(frequencies[i] - 440 * 2 ** ((note - 69) / 12)) < 1e-8);
  });
  assert.ok(context.oscillators.every(o => o.type === 'sine'));
  audio.setMask(255, 2000);
  assert.equal(context.gains.slice(1).reduce((n, g) => n + last(g.gain).value, 0) * 0.25, 0.25);
  audio.unlock(); assert.equal(trace.contexts.length, 1);
});

test('Attack and zero release are 10 ms; long fade and re-press hold the exact current envelope', () => {
  const { trace, audio } = setup(); audio.unlock();
  const context = trace.contexts[0], gain = context.gains[1].gain;
  audio.setMask(1, 0); assert.deepEqual(last(gain), { type: 'ramp', value: 0.125, time: 0.01 });
  context.currentTime = 0.005; audio.setMask(0, 0);
  assert.deepEqual(gain.events.at(-2), { type: 'set', value: 0.0625, time: 0.005 });
  assert.equal(last(gain).time, 0.015);
  context.currentTime = 1; audio.setMask(1, 2000);
  context.currentTime = 2; audio.setMask(0, 2000);
  assert.deepEqual(last(gain), { type: 'ramp', value: 0, time: 4 });
  context.currentTime = 3; audio.setMask(1, 2000);
  assert.deepEqual(gain.events.at(-2), { type: 'set', value: 0.0625, time: 3 });
  const count = gain.events.length;
  audio.setMask(1, 50); assert.equal(gain.events.length, count); // Held pad is not retriggered.
  context.currentTime = 3.1; audio.stopAll();
  assert.ok(context.gains.slice(1).every(g => last(g.gain).value === 0 && last(g.gain).time === 3.11));
});

test('Suspension clears notes without stale playback; next touch resumes the existing context', () => {
  const { trace, audio } = setup(); audio.unlock(); audio.setMask(128, 2000);
  const context = trace.contexts[0]; context.state = 'interrupted'; context.listeners.statechange();
  assert.ok(context.gains.slice(1).every(g => last(g.gain).value === 0 && last(g.gain).time === 0));
  audio.unlock(); audio.setMask(64, 0);
  assert.equal(trace.calls.filter(c => c.type === 'resume').length, 2);
  assert.equal(last(context.gains[7].gain).value, 0.125);
  assert.equal(last(context.gains[8].gain).value, 0);
});

test('Missing, throwing and rejected audio are isolated; a late resume cannot resurrect a released pad', async () => {
  for (const mode of ['missing', 'throws', 'rejects', 'pending']) {
    const { audio, trace } = setup(mode);
    assert.doesNotThrow(() => { audio.unlock(); audio.setMask(1, 2000); audio.stopAll(); });
    if (mode === 'pending') { trace.contexts[0].state = 'running'; trace.contexts[0].finishResume(); }
    await Promise.resolve();
    for (const context of trace.contexts) assert.ok(context.gains.slice(1).every(g => last(g.gain).value === 0));
  }
});
