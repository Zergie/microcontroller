import { createPadAudio } from './pad-audio.js';
import { createPadTransport, requestPads } from './pad-transport.js';
import { songs, createSongPlayer } from './songs.js';

export const defaultPadColors = ['#00ff28', '#00ffff', '#0000ff', '#b400ff', '#ff0014', '#ff2800', '#ff9600', '#ffffff'];

// One pad session owns holds, playback and lifecycle. The DOM adapter only
// supplies user actions and renders snapshots; render never produces output.
export function createPadSession({ onChange = () => {}, onOffline = () => {},
  audio = createPadAudio(), request = requestPads, transportFactory = createPadTransport,
  playerFactory = createSongPlayer } = {}) {
  const pointers = new Map();
  let online = false, supported = false, ready = false, loading = false;
  let view = 'animations', controlling = false, paused = false, colors = [...defaultPadColors], fadeMs = 1000;
  let transportType;
  let song = songs[0], songMask = 0, songPitches, error = '', loadEpoch = 0;
  const transport = transportFactory({ request, onFailure: failed, onChange: data => {
    if (data) error = data.settings_error ? 'Settings could not be saved.' : '';
    onChange();
  } });
  const player = playerFactory({ onMask(mask, pitches, duration) {
    songMask = mask; songPitches = pitches;
    if (mask) transport.pulse(mask, duration);
    else transport.cancelPulse();
    output(); onChange();
  }, onState() { onChange(); } });
  function heldMask() {
    if (view === 'songs') return songMask;
    let mask = 0;
    for (const index of pointers.values()) if (index >= 0) mask |= 1 << index;
    return mask;
  }
  const enabled = () => online && supported && ready && view !== 'animations' && !paused;
  function output() {
    audio.setMask(enabled() ? heldMask() : 0, view === 'songs' ? 20 : fadeMs,
      view === 'songs' ? songPitches : undefined);
  }
  function sync() {
    if (ready && controlling) transport.update({ active: view !== 'animations', fadeMs, colors,
      mask: enabled() && view === 'pads' ? heldMask() : 0 });
    output(); onChange();
  }
  function stop() {
    player.stop(); transport.cancelPulse(); pointers.clear(); songMask = 0; songPitches = undefined;
    audio.stopAll();
  }
  function failed(reason) {
    online = false; loadEpoch++; loading = false;
    stop(); transport.setConnection(false);
    error = reason.name === 'AbortError' ? 'No response. Check USB.' : reason.message;
    onChange(); onOffline();
  }
  async function load() {
    if (loading || ready || !online || !supported) return;
    loading = true; const epoch = loadEpoch;
    try {
      const data = await request('/api/pads.json', 'GET');
      if (epoch !== loadEpoch) return;
      if (!Array.isArray(data.colors) || ![8, 10].includes(data.colors.length) ||
          !data.colors.every(c => /^#[0-9a-f]{6}$/i.test(c)) ||
          !Number.isInteger(data.fade_ms) || data.fade_ms < 0 || data.fade_ms > 2000)
        throw new Error('Invalid LED pad status.');
      colors = data.colors.slice(0, 8); fadeMs = data.fade_ms; ready = true; error = '';
    } catch (reason) { if (epoch === loadEpoch) failed(reason); }
    finally { if (epoch === loadEpoch) { loading = false; sync(); } }
  }
  return {
    get state() {
      return { online, supported, ready, enabled: enabled(), mask: heldMask(), colors: [...colors], fadeMs,
        songId: song.id, playing: player.isPlaying(), error,
        canPlay: enabled() && !transport.state.busy && !transport.state.pending && transport.state.ready };
    },
    setConnection(connected, capable, type) {
      const changed = online !== connected || supported !== capable || transportType !== type;
      online = connected; supported = capable; transportType = type;
      if (!online || !supported) { loadEpoch++; loading = false; stop(); }
      transport.setConnection(online && supported, type);
      if (online && supported) { void load(); if (ready && changed) sync(); }
      onChange();
    },
    setView(next, notify = true) {
      stop(); view = next;
      if (notify || next !== 'animations') controlling = true;
      if (notify || next !== 'animations') sync(); else { output(); onChange(); }
    },
    touch(key, index, unlock = false) {
      if (!enabled() || view !== 'pads') return;
      if (unlock) audio.unlock();
      if (pointers.get(key) === index) return;
      pointers.set(key, index); sync();
    },
    hasTouch: key => pointers.has(key),
    release(key) { if (pointers.delete(key)) sync(); },
    setColors(next) { colors = [...next]; sync(); },
    setColor(index, value) { colors[index] = value; sync(); },
    setFade(value) { fadeMs = value; sync(); },
    selectSong(id) { stop(); song = songs.find(s => s.id === id) || songs[0]; sync(); },
    toggleSong() {
      if (player.isPlaying()) { stop(); sync(); }
      else if (enabled() && view === 'songs') { audio.unlock(); player.play(song); }
    },
    pause() { paused = true; stop(); transport.pause(); onChange(); },
    resume() { paused = false; transport.resume(); output(); onChange(); },
    heartbeat() { transport.heartbeat(); },
    get shouldRelease() { return view !== 'animations' && online && supported; }
  };
}
