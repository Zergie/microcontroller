import { createPadAudio } from './pad-audio.js';
import { createPadStream } from './pad-stream.js';
import { songs, createSongPlayer } from './songs.js';

const defaults = ['#00ff28', '#00ffff', '#0000ff', '#b400ff', '#ff0014', '#ff2800', '#ff9600', '#ffffff'];
// Presets are written in visual order (left to right); storage stays in LED order.
const palettes = {
  original: defaults,
  rainbow: ['#ff0000', '#ff8000', '#ffff00', '#80ff00', '#00ff80', '#00bfff', '#4000ff', '#bf00ff'].reverse(),
  sunset: ['#ffcc33', '#ff9933', '#ff6633', '#ff334d', '#e62e80', '#b333b3', '#8033cc', '#4d3399'].reverse(),
  ocean: ['#00ffcc', '#00e6cc', '#00cccc', '#00b3cc', '#0099cc', '#0080cc', '#0066cc', '#004dcc'].reverse(),
  pastel: ['#ffb3ba', '#ffdfba', '#ffffba', '#baffc9', '#bafff2', '#bae1ff', '#d4baff', '#f2baff'].reverse()
};

export function createPadsController({ onOffline }) {
  const $ = id => document.getElementById(id);
  const pointers = new Map();
  const audio = createPadAudio();
  const stream = createPadStream({ onFailure: failed, onReady() { render(); pulseReady(); } });
  const buttons = [], colors = [];
  let online = false, supported = false, ready = false, loading = false;
  let active = false, busy = false, pending = false;
  let fadeMs = 1000, values = [...defaults], revision = 0;
  let fast = false, appliedFastPath;
  let songMode = false, songMask = 0, songPitches;
  let restoreFade = false;
  let songFadeMs = Math.round(60000 / songs[0].bpm), pendingPulse, legacyRelease;
  const firstNoteMs = () => {
    const song = songs.find(song => song.id === $('song-select').value);
    return Math.round(song.score.find(note => note.note !== '-').beats * 60000 / song.bpm);
  };
  const player = createSongPlayer({
    onMask(mask, pitches, duration) {
      songMask = mask; songPitches = pitches;
      if (mask) { songFadeMs = Math.round(duration); pendingPulse = { mask, fade: songFadeMs }; }
      else pendingPulse = undefined;
      changed();
    },
    onState(playing) {
      if (!playing) { pendingPulse = undefined; songFadeMs = firstNoteMs(); changed(); }
      else render();
    }
  });
  const releaseMs = () => songMode ? songFadeMs : fadeMs;

  function mask() {
    if (songMode) return songMask;
    let result = 0;
    for (const index of pointers.values()) if (index >= 0) result |= 1 << index;
    return result;
  }
  function render() {
    const enabled = online && supported && ready && active;
    const held = mask();
    audio.setMask(enabled ? held : 0, songMode ? 20 : fadeMs, songMode ? songPitches : undefined);
    if (fast) stream.setMask(enabled && !songMode ? held : 0);
    buttons.forEach((button, i) => {
      button.disabled = !enabled;
      button.setAttribute('aria-pressed', String(Boolean(held & (1 << i))));
      button.style.setProperty('--pad-color', values[i]);
      button.style.setProperty('--fade-duration', `${fadeMs}ms`);
      colors[i].disabled = !enabled;
      colors[i].value = values[i];
    });
    $('pads-fade').disabled = !enabled;
    $('song-select').disabled = !enabled || !songMode;
    $('song-play').disabled = !enabled || !songMode ||
      (!player.isPlaying() && (busy || pending || (fast && !stream.isReady())));
    $('song-play').textContent = player.isPlaying() ? 'Stop' : 'Play';
    $('song-play').setAttribute('aria-pressed', String(player.isPlaying()));
    $('pads-palette').disabled = !enabled;
    $('pads-palette').value = Object.keys(palettes).find(name =>
      palettes[name].every((color, index) => color === values[index].toLowerCase())) || 'custom';
    $('pads-fade').value = fadeMs / 1000;
    $('pads-fade-value').value = `${(fadeMs / 1000).toFixed(1)} s`;
    $('pads-notice').textContent = !online ? 'Board is offline.' : !supported ? 'Update firmware to use LED pads.' : !ready ? 'Connecting…' : '';
  }
  async function request(path, method = 'POST') {
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 1000);
    try {
      const response = await fetch(path, { method, cache: 'no-store', signal: controller.signal,
        ...(method === 'POST' ? { body: '' } : {}) });
      if (!response.ok) throw new Error('LED pads are unreachable. Check USB.');
      return await response.json();
    } finally { clearTimeout(timeout); }
  }
  function failed(error) {
    online = false; player.stop();
    stream.close(); appliedFastPath = undefined;
    audio.stopAll();
    pointers.clear(); pending = false;
    $('pads-error').textContent = error.name === 'AbortError' ? 'No response. Check USB.' : error.message;
    $('pads-error').hidden = false;
    render(); onOffline();
  }
  function path() {
    if (legacyRelease) return `/api/pads/set/0/${legacyRelease.fade}/${values.map(c => c.slice(1)).join('')}`;
    if (restoreFade) return fast ? `/api/pads/config/${fadeMs}/${values.map(c => c.slice(1)).join('')}` :
      `/api/pads/set/0/${fadeMs}/${values.map(c => c.slice(1)).join('')}`;
    if (fast && active) return `/api/pads/config/${releaseMs()}/${values.map(c => c.slice(1)).join('')}`;
    return active ? `/api/pads/set/${songMode ? pendingPulse?.mask || 0 : mask()}/${releaseMs()}/${values.map(c => c.slice(1)).join('')}` : '/api/view/animations';
  }
  function pulseReady() {
    if (songMode && pendingPulse && stream.isReady() && appliedFastPath === path()) {
      const pulse = pendingPulse; pendingPulse = undefined;
      stream.pulse(pulse.mask);
    }
  }
  async function flush() {
    if (busy || loading || !ready || !online || !supported || !pending) return;
    pending = false; busy = true;
    render();
    const sent = revision;
    const requested = path();
    const releasing = legacyRelease;
    const restoring = restoreFade && !releasing;
    const pulse = !fast && !restoring && !releasing ? pendingPulse : undefined;
    if (fast && requested === appliedFastPath && !restoring) {
      busy = false;
      render();
      if (active && !document.hidden) stream.connect();
      pulseReady();
      return;
    }
    try {
      const data = await request(requested);
      if (releasing && legacyRelease === releasing) { legacyRelease = undefined; pending = true; }
      if (pulse) {
        if (pendingPulse === pulse) pendingPulse = undefined;
        legacyRelease = { fade: pulse.fade }; pending = true;
      }
      if (restoring) { restoreFade = false; pending = true; }
      if (fast) {
        appliedFastPath = requested;
        if (active && !document.hidden && path() === requested) stream.connect();
        pulseReady();
      }
      $('pads-error').hidden = true;
      if (data.settings_error) {
        $('pads-error').textContent = 'Settings could not be saved.';
        $('pads-error').hidden = false;
      }
    } catch (error) { failed(error); }
    finally {
      busy = false;
      render();
      // A release or another touch during the request always sends the latest
      // complete mask next. No concurrent requests can reorder press/release.
      if (revision !== sent && online) pending = true;
      if (pending) void flush();
    }
  }
  function changed() { revision++; pending = true; render(); void flush(); }
  async function load() {
    if (loading || ready || !online || !supported) return;
    loading = true;
    try {
      const data = await request('/api/pads.json', 'GET');
      if (!Array.isArray(data.colors) || data.colors.length !== 8 || !Number.isInteger(data.fade_ms) ||
          data.fade_ms < 0 || data.fade_ms > 2000) throw new Error('Invalid LED pad status.');
      values = [...data.colors]; fadeMs = data.fade_ms; ready = true;
      $('pads-error').hidden = true;
    } catch (error) { failed(error); }
    finally { loading = false; render(); if (pending) void flush(); }
  }
  function releaseAll() {
    player.stop();
    audio.stopAll();
    if (pointers.size) { pointers.clear(); changed(); }
    stream.close();
  }
  function release(key) {
    if (!pointers.delete(key)) return;
    changed();
  }
  function resumeConnection() {
    if (active && online && supported && fast && !document.hidden && appliedFastPath === path()) stream.connect();
  }
  for (let index = 7; index >= 0; index--) {
    const slot = document.createElement('div'); slot.className = 'pad-slot';
    const button = document.createElement('button');
    button.dataset.padIndex = String(index);
    button.type = 'button'; button.className = 'led-pad'; button.textContent = String(index + 1);
    button.setAttribute('aria-label', `LED ${index + 1}`); button.setAttribute('aria-pressed', 'false');
    // Safari can still open its copy callout despite user-select/touch-callout CSS.
    // Suppress native gestures only on pad buttons; Pointer Events own the playing gesture.
    for (const type of ['touchstart', 'contextmenu', 'selectstart', 'dragstart']) {
      button.addEventListener(type, event => event.preventDefault(), { passive: false });
    }
    const color = document.createElement('input');
    color.type = 'color'; color.setAttribute('aria-label', `LED ${index + 1} color`);
    color.addEventListener('input', () => { values[index] = color.value; changed(); });
    button.addEventListener('pointerdown', event => {
      if (button.disabled || (event.pointerType === 'mouse' && event.button !== 0)) return;
      audio.unlock();
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      pointers.set(event.pointerId, index); changed();
    });
    button.addEventListener('pointermove', event => {
      if (!pointers.has(event.pointerId)) return;
      // Capture keeps the gesture alive across gaps and outside the original pad.
      // Hit-test the finger position rather than the captured event target.
      const target = document.elementFromPoint(event.clientX, event.clientY)?.closest('.led-pad');
      const next = target && !target.disabled ? Number(target.dataset.padIndex) : -1;
      if (pointers.get(event.pointerId) === next) return;
      pointers.set(event.pointerId, next); changed();
    });
    for (const event of ['pointerup', 'pointercancel', 'lostpointercapture'])
      button.addEventListener(event, e => release(e.pointerId));
    button.addEventListener('keydown', event => {
      if (!['Space', 'Enter'].includes(event.code) || button.disabled || event.repeat) return;
      audio.unlock();
      event.preventDefault(); pointers.set(`key:${index}:${event.code}`, index); changed();
    });
    button.addEventListener('keyup', event => {
      if (!['Space', 'Enter'].includes(event.code)) return;
      event.preventDefault(); release(`key:${index}:${event.code}`);
    });
    button.addEventListener('blur', () => { release(`key:${index}:Space`); release(`key:${index}:Enter`); });
    slot.append(button, color); $('pads-grid').append(slot); buttons[index] = button; colors[index] = color;
  }
  $('pads-fade').addEventListener('input', event => { fadeMs = Math.round(Number(event.target.value) * 1000); changed(); });
  $('pads-palette').addEventListener('change', event => {
    const palette = palettes[event.target.value];
    if (!palette) return;
    values = [...palette]; changed();
  });
  for (const song of songs) $('song-select').add(new Option(song.title, song.id));
  $('song-select').addEventListener('change', () => { player.stop(); audio.stopAll(); songFadeMs = firstNoteMs(); changed(); });
  $('song-play').addEventListener('click', () => {
    if (player.isPlaying()) { player.stop(); audio.stopAll(); return; }
    audio.unlock();
    player.play(songs.find(song => song.id === $('song-select').value));
  });
  window.addEventListener('blur', releaseAll);
  window.addEventListener('focus', resumeConnection);
  document.addEventListener('visibilitychange', () => { if (document.hidden) releaseAll(); else resumeConnection(); });
  window.addEventListener('pagehide', () => {
    releaseAll();
    if (active && supported && online) navigator.sendBeacon('/api/pads/release', '');
  });
  setInterval(() => {
    if (fast) {
      if (active && online && supported && !document.hidden) stream.heartbeat();
      return;
    }
    if (active && online && supported && !songMode && mask()) { pending = true; void flush(); }
  }, 250);
  render();
  return {
    setConnection(connected, capable, transport) {
      const reconnect=connected && !online;
      const nextFast = transport === 'websocket-v1';
      if (fast !== nextFast) { stream.close(); appliedFastPath = undefined; }
      fast = nextFast;
      online = connected; supported = capable;
      if (reconnect && fast && active) pending = true;
      if (!online || !supported) { player.stop(); stream.close(); appliedFastPath = undefined; audio.stopAll(); pointers.clear(); pending = false; }
      render(); if (online && supported) { void load(); if (pending) void flush(); else resumeConnection(); }
    },
    setView(enabled, notify = true) {
      player.stop();
      stream.close();
      audio.stopAll();
      active = enabled; pointers.clear();
      if (notify || (fast && enabled)) { revision++; pending = true; }
      render(); if (online && supported) { void load(); void flush(); }
    },
    setSongMode(enabled) {
      player.stop(); audio.stopAll(); pointers.clear(); songMask = 0; songPitches = undefined;
      if (songMode && !enabled) restoreFade = true;
      if (enabled) restoreFade = false;
      pendingPulse = undefined;
      if (enabled) songFadeMs = firstNoteMs();
      songMode = enabled; render();
    }
  };
}
