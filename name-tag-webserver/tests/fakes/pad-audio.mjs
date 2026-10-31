// Deterministic AudioContext double, usable in Node and browser gesture tests.
export function installFakeAudio({ mode = 'normal' } = {}) {
  const test = globalThis.audioTest = { contexts: [], calls: [], get gesture() { return Boolean(this.event?.eventPhase); } };
  if (globalThis.document) {
    for (const type of ['pointerdown', 'keydown', 'click']) document.addEventListener(type, event => { test.event = event; }, true);
  }
  class Param {
    value = 0;
    events = [];
    cancelScheduledValues(time) { this.events.push({ type: 'cancel', time }); }
    setValueAtTime(value, time) { this.events.push({ type: 'set', value, time }); }
    linearRampToValueAtTime(value, time) { this.events.push({ type: 'ramp', value, time }); }
  }
  globalThis.AudioContext = mode === 'missing' ? undefined : class {
    currentTime = 0;
    state = 'suspended';
    destination = {};
    gains = [];
    oscillators = [];
    listeners = {};
    constructor() {
      if (mode === 'throws') throw new Error('Audio unavailable');
      test.contexts.push(this); test.calls.push({ type: 'create', gesture: test.gesture });
    }
    createGain() {
      const node = { gain: new Param(), connect() {} };
      this.gains.push(node); return node;
    }
    createOscillator() {
      const node = { frequency: new Param(), type: '', connect() {}, start() {} };
      this.oscillators.push(node); return node;
    }
    addEventListener(type, listener) { this.listeners[type] = listener; }
    resume() {
      test.calls.push({ type: 'resume', gesture: test.gesture });
      if (mode === 'rejects') return Promise.reject(new Error('Audio denied'));
      if (mode !== 'pending') this.state = 'running';
      return mode === 'pending' ? new Promise(resolve => { this.finishResume = resolve; }) : Promise.resolve();
    }
  };
  globalThis.webkitAudioContext = undefined;
  return test;
}
