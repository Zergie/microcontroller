// Live concept previews use only the existing eight-pad HTTP commands.
const PAD_COUNT = 8;
export function frameCommand(frame, glow = 1) {
  if (!Array.isArray(frame) || frame.length < 2 || !Number.isFinite(glow) || glow < 0 || glow > 1 ||
      !frame.every(rgb => Array.isArray(rgb) && rgb.length === 3 && rgb.every(Number.isFinite)))
    throw new Error('Invalid preview frame.');
  // The controller's visual left-to-right order is LED 8 through LED 1.
  // Interpolate all ten concept pixels into the eight addressable pad LEDs.
  const colors = Array.from({ length: PAD_COUNT }, (_, pin) => {
    const at = (PAD_COUNT - 1 - pin) * (frame.length - 1) / (PAD_COUNT - 1);
    const lo = Math.floor(at), hi = Math.min(frame.length - 1, lo + 1);
    return frame[lo].map((value, channel) => {
      const mixed = value + (frame[hi][channel] - value) * (at - lo);
      return Math.round(Math.max(0, Math.min(255, mixed)) * glow).toString(16).padStart(2, '0');
    }).join('');
  }).join('');
  return `/api/pads/set/255/0/${colors}`;
}

export async function padRequest(path, method = 'POST') {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 900);
  try {
    const response = await fetch(path, { method, cache: 'no-store', signal: controller.signal,
      keepalive: true, ...(method === 'POST' ? { body: '' } : {}) });
    if (!response.ok) throw new Error(`Board request failed (${response.status}).`);
    return await response.json();
  } finally { clearTimeout(timeout); }
}
function restoreCommand(snapshot) {
  if (typeof snapshot?.active !== 'boolean' || !Number.isInteger(snapshot.fade_ms) ||
      snapshot.fade_ms < 0 || snapshot.fade_ms > 2000 || !Array.isArray(snapshot.colors) ||
      snapshot.colors.length !== PAD_COUNT || !snapshot.colors.every(c => /^#[0-9a-f]{6}$/i.test(c)))
    throw new Error('Board does not provide a valid eight-pad status.');
  return `/api/pads/set/0/${snapshot.fade_ms}/${snapshot.colors.map(c => c.slice(1)).join('')}`;
}

// One in-flight request. Sample the latest frame only when ready to send it;
// slow USB never builds a backlog of obsolete animation frames.
export function createLivePreview({ sample, onStatus = () => {}, request = padRequest,
  schedule = setTimeout, cancel = clearTimeout, interval = 70 } = {}) {
  let wanted = false, task, saved, touched = false, wake;
  function sleep() {
    return new Promise(resolve => {
      const timer = schedule(() => { wake = undefined; resolve(); }, interval);
      wake = () => { cancel(timer); wake = undefined; resolve(); };
    });
  }
  async function restore() {
    if (saved && touched) {
      onStatus('Restoring board…');
      await request(saved.path);
      if (!saved.active) await request('/api/view/animations');
    }
    saved = undefined; touched = false;
  }
  async function run() {
    try {
      while (wanted || saved) {
        if (!wanted) { await restore(); continue; }
        if (!saved) {
          onStatus('Connecting to board…');
          const snapshot = await request('/api/pads.json', 'GET');
          saved = { active: snapshot.active, path: restoreCommand(snapshot) };
          if (!wanted) continue;
        }
        const { frame, glow } = sample();
        const path = frameCommand(frame, glow);
        touched = true; // A timed-out response may still have applied the frame.
        await request(path);
        onStatus('Live on board · 8 LEDs · LED 9–10 off');
        if (wanted) await sleep();
      }
      onStatus('Board preview off · settings restored');
    } catch (error) {
      wanted = false;
      try { await restore(); }
      catch { onStatus('Board disconnected · could not restore pad settings'); return; }
      onStatus(`Board preview stopped · ${error.name === 'AbortError' ? 'USB response timed out' : error.message}`);
    } finally { task = undefined; }
  }
  return {
    start() { wanted = true; if (!task) task = run(); },
    stop() { wanted = false; wake?.(); return task || Promise.resolve(); },
    // On page exit the browser may stop JS before the two-request restoration
    // finishes. Restore colors/fade and release holds with one bounded beacon.
    leave() {
      wanted = false; wake?.();
      if (saved && touched) navigator.sendBeacon(saved.path, '');
    }
  };
}
