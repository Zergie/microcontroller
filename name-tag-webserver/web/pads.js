import { createPadSession, defaultPadColors } from './pad-session.js';
import { songs } from './songs.js';

// Presets are written in visual order (left to right); storage stays in LED order.
const palettes = {
  original: defaultPadColors,
  rainbow: ['#ff0000', '#ff8000', '#ffff00', '#80ff00', '#00ff80', '#00bfff', '#4000ff', '#bf00ff'].reverse(),
  sunset: ['#ffcc33', '#ff9933', '#ff6633', '#ff334d', '#e62e80', '#b333b3', '#8033cc', '#4d3399'].reverse(),
  ocean: ['#00ffcc', '#00e6cc', '#00cccc', '#00b3cc', '#0099cc', '#0080cc', '#0066cc', '#004dcc'].reverse(),
  pastel: ['#ffb3ba', '#ffdfba', '#ffffba', '#baffc9', '#bafff2', '#bae1ff', '#d4baff', '#f2baff'].reverse()
};

export function createPadsController({ onOffline }) {
  const $ = id => document.getElementById(id);
  const buttons = [], colors = [];
  const session = createPadSession({ onOffline, onChange: render });
  function render() {
    const state = session.state;
    buttons.forEach((button, i) => {
      button.disabled = !state.enabled;
      button.setAttribute('aria-pressed', String(Boolean(state.mask & (1 << i))));
      button.style.setProperty('--pad-color', state.colors[i]);
      button.style.setProperty('--fade-duration', `${state.fadeMs}ms`);
      colors[i].disabled = !state.enabled; colors[i].value = state.colors[i];
    });
    $('pads-fade').disabled = !state.enabled;
    $('song-select').disabled = !state.enabled;
    $('song-play').disabled = !state.playing && !state.canPlay;
    $('song-play').textContent = state.playing ? 'Stop' : 'Play';
    $('song-play').setAttribute('aria-pressed', String(state.playing));
    $('pads-palette').disabled = !state.enabled;
    $('pads-palette').value = Object.keys(palettes).find(name =>
      palettes[name].every((color, index) => color === state.colors[index].toLowerCase())) || 'custom';
    $('pads-fade').value = state.fadeMs / 1000;
    $('pads-fade-value').value = `${(state.fadeMs / 1000).toFixed(1)} s`;
    $('pads-notice').textContent = !state.online ? 'Board is offline.' : !state.supported ? 'Update firmware to use LED pads.' : !state.ready ? 'Connecting…' : '';
    $('pads-error').textContent = state.error;
    $('pads-error').hidden = !state.error;
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
    color.addEventListener('input', () => { session.setColor(index, color.value); });
    button.addEventListener('pointerdown', event => {
      if (button.disabled || (event.pointerType === 'mouse' && event.button !== 0)) return;
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      session.touch(event.pointerId, index, true);
    });
    button.addEventListener('pointermove', event => {
      if (!session.hasTouch(event.pointerId)) return;
      // Capture keeps the gesture alive across gaps and outside the original pad.
      // Hit-test the finger position rather than the captured event target.
      const target = document.elementFromPoint(event.clientX, event.clientY)?.closest('.led-pad');
      const next = target && !target.disabled ? Number(target.dataset.padIndex) : -1;
      session.touch(event.pointerId, next);
    });
    for (const event of ['pointerup', 'pointercancel', 'lostpointercapture'])
      button.addEventListener(event, e => session.release(e.pointerId));
    button.addEventListener('keydown', event => {
      if (!['Space', 'Enter'].includes(event.code) || button.disabled || event.repeat) return;
      event.preventDefault(); session.touch(`key:${index}:${event.code}`, index, true);
    });
    button.addEventListener('keyup', event => {
      if (!['Space', 'Enter'].includes(event.code)) return;
      event.preventDefault(); session.release(`key:${index}:${event.code}`);
    });
    button.addEventListener('blur', () => { session.release(`key:${index}:Space`); session.release(`key:${index}:Enter`); });
    slot.append(button, color); $('pads-grid').append(slot); buttons[index] = button; colors[index] = color;
  }
  $('pads-fade').addEventListener('input', event => session.setFade(Math.round(Number(event.target.value) * 1000)));
  $('pads-palette').addEventListener('change', event => {
    const palette = palettes[event.target.value];
    if (palette) session.setColors(palette);
  });
  for (const song of songs) $('song-select').add(new Option(song.title, song.id));
  $('song-select').addEventListener('change', () => session.selectSong($('song-select').value));
  $('song-play').addEventListener('click', () => session.toggleSong());
  window.addEventListener('blur', () => session.pause());
  window.addEventListener('focus', () => session.resume());
  document.addEventListener('visibilitychange', () => { if (document.hidden) session.pause(); else session.resume(); });
  window.addEventListener('pagehide', () => {
    session.pause();
    if (session.shouldRelease) navigator.sendBeacon('/api/pads/release', '');
  });
  setInterval(() => session.heartbeat(), 250);
  render();
  return {
    setConnection: (connected, capable, transport) => session.setConnection(connected, capable, transport),
    setView: (view, notify = true) => session.setView(view, notify)
  };
}
