import entchen from './songs/alle-meine-entchen.js';
import jakob from './songs/bruder-jakob.js';
import haenschen from './songs/haenschen-klein.js';
import freude from './songs/ode-an-die-freude.js';
import mahna from './songs/mahna-mahna.js';

// Single-voice arrangements. Durations are quarter-note beats;
// '-' is a notated rest. No lyrics or recorded audio are included.
const pitch = { C4: 60, D4: 62, E4: 64, F4: 65, G4: 67, A4: 69, H4: 71, C5: 72, G3: 55, 'C#4': 61, Eb4: 63, E5: 76, F5: 77, G5: 79 };
// Chromatic notes share the LED of the adjacent lower natural note.
const led = { C4: 7, D4: 6, E4: 5, F4: 4, G4: 3, A4: 2, H4: 1, C5: 0, G3: 3, 'C#4': 7, Eb4: 6, E5: 5, F5: 4, G5: 3 };
export const songs = [entchen, jakob, haenschen, freude, mahna];

export function songTimeline(song) {
  const events = [];
  let at = 0;
  const beatMs = 60000 / song.bpm;
  for (const { note, beats } of song.score) {
    const duration = beats * beatMs;
    if (note !== '-') {
      const index = led[note];
      if (index === undefined || !(duration > 0)) throw new Error('Invalid song note');
      events.push({ at, mask: 1 << index, pitches: { [index]: pitch[note] }, duration });
      // Short articulation gaps separate repeated notes and leave room for release.
      const gap = song.exactGates ? 0 : Math.min(80, Math.max(30, duration * 0.1));
      events.push({ at: at + duration - gap, mask: 0 });
    }
    at += duration;
  }
  return { events, duration: at };
}

export function createSongPlayer({ onMask, onState, now = () => performance.now(),
  schedule = setTimeout, cancel = clearTimeout }) {
  let timer, started, timeline, index = 0, playing = false, currentMask = 0;
  function emit(event) {
    currentMask = event.mask;
    onMask(event.mask, event.pitches, event.duration);
  }
  function stop() {
    if (!playing) return;
    playing = false; cancel(timer);
    if (currentMask) emit({ mask: 0 });
    onState(false);
  }
  function tick() {
    if (!playing) return;
    const elapsed = now() - started;
    let latest;
    // Skip obsolete events after a delayed callback instead of playing a burst.
    while (index < timeline.events.length && timeline.events[index].at <= elapsed) latest = timeline.events[index++];
    if (latest) emit(latest);
    if (elapsed >= timeline.duration) { stop(); return; }
    const next = index < timeline.events.length ? timeline.events[index].at : timeline.duration;
    timer = schedule(tick, Math.max(1, next - elapsed));
  }
  return {
    play(song) {
      stop(); timeline = songTimeline(song); index = 0;
      started = now(); playing = true; onState(true); tick();
    },
    stop,
    isPlaying: () => playing
  };
}
