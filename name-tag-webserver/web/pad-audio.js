// Visual order is LED 8 ... LED 1. German H4 is MIDI note 71 (B4).
const notes = [72, 71, 69, 67, 65, 64, 62, 60];
const rampSeconds = 0.01;

export function createPadAudio() {
  let context;
  let voices = [];
  let held = 0;

  function ramp(voice, target, duration) {
    const now = context.currentTime;
    const elapsed = now >= voice.end ? 1 : Math.max(0, (now - voice.start) / (voice.end - voice.start));
    const level = voice.from + (voice.to - voice.from) * elapsed;
    voice.gain.gain.cancelScheduledValues(now);
    voice.gain.gain.setValueAtTime(level, now);
    voice.gain.gain.linearRampToValueAtTime(target, now + duration);
    Object.assign(voice, { from: level, to: target, start: now, end: now + duration });
  }

  function stopAll() {
    held = 0;
    if (!context) return;
    try { for (const voice of voices) ramp(voice, 0, context.state === 'running' ? rampSeconds : 0); }
    catch { /* Audio failure must never interrupt LED control. */ }
  }

  return {
    // Called synchronously inside pointerdown/keydown, before any network await.
    unlock() {
      // Safari's default ambient session can mute the speaker in silent mode.
      // Session support is optional and must not gate the existing audio path.
      try {
        const session = globalThis.navigator?.audioSession;
        if (session && session.type !== 'playback') session.type = 'playback';
      } catch { /* Older/restricted browsers retain their normal audio policy. */ }
      try {
        if (!context || context.state === 'closed') {
          const Audio = globalThis.AudioContext || globalThis.webkitAudioContext;
          if (!Audio) return;
          context = new Audio();
          const master = context.createGain();
          master.gain.value = 0.25;
          master.connect(context.destination);
          voices = notes.map(note => {
            const oscillator = context.createOscillator();
            const gain = context.createGain();
            gain.gain.value = 0;
            oscillator.type = 'sine';
            oscillator.frequency.value = 440 * 2 ** ((note - 69) / 12);
            oscillator.connect(gain); gain.connect(master); oscillator.start();
            return { gain, oscillator, note, from: 0, to: 0, start: 0, end: 0 };
          });
          context.addEventListener('statechange', () => {
            if (context.state !== 'running') stopAll();
          });
        }
        if (context.state !== 'running') Promise.resolve(context.resume()).catch(stopAll);
      } catch { stopAll(); }
    },
    setMask(mask, fadeMs, pitches = {}) {
      if (!context || voices.length !== 8) return;
      try {
        voices.forEach((voice, index) => {
          const bit = 1 << index;
          const note = pitches[index] ?? notes[index];
          const retune = (mask & bit) && voice.note !== note;
          if ((held & bit) === (mask & bit) && !retune) return;
          if (retune) {
            voice.oscillator.frequency.setValueAtTime(440 * 2 ** ((note - 69) / 12), context.currentTime);
            voice.note = note;
          }
          ramp(voice, mask & bit ? 1 / 8 : 0,
            mask & bit ? rampSeconds : Math.max(rampSeconds, fadeMs / 1000));
        });
        held = mask;
      } catch { stopAll(); }
    },
    stopAll
  };
}
