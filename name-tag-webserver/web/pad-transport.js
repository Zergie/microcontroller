// Transport ordering belongs here, independently of DOM, gestures and audio.
import { createPadStream } from './pad-stream.js';

export async function requestPads(path, method = 'POST') {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 1000);
  try {
    const response = await fetch(path, { method, cache: 'no-store', signal: controller.signal,
      ...(method === 'POST' ? { body: '' } : {}) });
    if (!response.ok) throw new Error('LED pads are unreachable. Check USB.');
    return await response.json();
  } finally { clearTimeout(timeout); }
}

// Configuration acknowledgements, held-mask delivery and one-shot notes are
// distinct work. Only the optional mask stream bypasses the HTTP request queue.
export function createPadTransport({ onChange = () => {}, onFailure,
  request = requestPads, streamFactory = createPadStream, now = Date.now } = {}) {
  let stream, connected = false, paused = false, busy = false, epoch = 0;
  let desired, appliedConfiguration, appliedMask, renewHeld = false, pulse;
  const configuration = () => desired.active ? `${desired.fadeMs}/${desired.colors}` : 'animations';
  const configurationPending = () => desired && configuration() !== appliedConfiguration;
  const maskPending = () => desired?.active && !stream && (desired.mask !== appliedMask || renewHeld);
  const streamReady = () => !stream || stream.isReady();
  function reset() {
    epoch++; pulse = undefined; renewHeld = false;
    stream?.close(); stream?.setMask(0);
    appliedConfiguration = undefined; appliedMask = undefined;
    if (desired) desired = { ...desired, mask: 0 };
  }
  function failed(error) {
    connected = false; reset(); onFailure(error); onChange();
  }
  function connect() {
    if (connected && !paused && desired?.active && !configurationPending()) stream?.connect();
  }
  function nextCommand() {
    if (configurationPending() || maskPending()) {
      const key = configuration();
      const path = !desired.active ? '/api/view/animations' : stream
        ? `/api/pads/config/${key}` : `/api/pads/set/${desired.mask}/${key}`;
      return { kind: configurationPending() ? 'configuration' : 'mask', key, mask: desired.mask, path };
    }
    if (pulse && !paused && desired?.active && streamReady()) return { kind: 'pulse', note: pulse };
  }
  async function flush() {
    if (busy || !connected || !desired) return;
    // Notes that waited behind a real settings change must not replay late.
    if (pulse && now() >= pulse.until) pulse = undefined;
    const command = nextCommand();
    if (!command) { connect(); return; }
    busy = true;
    const sentEpoch = epoch;
    try {
      let data;
      if (command.kind === 'pulse') {
        const note = command.note; pulse = undefined;
        if (stream) stream.pulse(note.mask, note.duration);
        else data = await request(`/api/pads/pulse/${note.mask}/${note.duration}`);
      } else {
        renewHeld = false;
        data = await request(command.path);
        if (sentEpoch !== epoch) return;
        appliedConfiguration = command.key; appliedMask = command.mask;
      }
      if (sentEpoch !== epoch) return;
      onChange(data); connect();
    } catch (error) { if (sentEpoch === epoch) failed(error); }
    finally {
      busy = false; onChange();
      if (connected) void flush();
    }
  }
  return {
    get state() { return { busy, pending: Boolean(configurationPending() || maskPending() || pulse), ready: streamReady() }; },
    setConnection(online, transport) {
      if (Boolean(stream) !== (transport === 'websocket-v1')) {
        reset();
        stream = transport === 'websocket-v1' ? streamFactory({ onFailure: failed,
          onReady() { onChange(); void flush(); } }) : undefined;
      }
      if (connected && !online) reset();
      connected = online;
      if (online) void flush();
    },
    update(state) {
      desired = { ...state, colors: state.colors.map(c => c.slice(1)).join('') };
      if (!state.active) { pulse = undefined; stream?.close(); }
      stream?.setMask(paused ? 0 : state.mask);
      void flush();
    },
    pulse(mask, duration) {
      if (!connected || paused || !desired?.active) return;
      pulse = { mask, duration: Math.round(duration), until: now() + duration };
      void flush();
    },
    cancelPulse() { pulse = undefined; },
    pause() {
      paused = true; pulse = undefined; stream?.setMask(0); stream?.close();
      if (desired) desired = { ...desired, mask: 0 };
      void flush();
    },
    resume() { paused = false; void flush(); },
    heartbeat() {
      if (!connected || paused || !desired?.active) return;
      if (stream) stream.heartbeat();
      else if (desired.mask) renewHeld = true;
      void flush();
    }
  };
}
