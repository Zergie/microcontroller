import { test } from 'node:test';
import assert from 'node:assert/strict';
import { songs, songTimeline, createSongPlayer } from '../web/songs.js';

function clock() {
  let time = 0, sequence = 0;
  const tasks = new Map();
  return {
    now: () => time,
    schedule(fn, delay) { const id = ++sequence; tasks.set(id, { at: time + delay, fn }); return id; },
    cancel(id) { tasks.delete(id); },
    advance(target) {
      let iterations = 0;
      while (true) {
        const next = [...tasks].sort((a, b) => a[1].at - b[1].at)[0];
        if (!next || next[1].at > target) break;
        assert.ok(iterations++ < 10000, 'Timer makes forward progress');
        time = next[1].at; tasks.delete(next[0]); next[1].fn();
      }
      time = target;
    },
    stall(target) {
      time = target;
      const due = [...tasks].filter(([, task]) => task.at <= time);
      for (const [id, task] of due) { tasks.delete(id); task.fn(); }
    },
    count: () => tasks.size
  };
}

test('Song catalogue has explicit tempo, positive durations, rests and valid LED/note mappings', () => {
  assert.deepEqual(songs.map(s => s.id), ['entchen', 'jakob', 'haenschen', 'freude', 'mahna']);
  for (const song of songs) {
    assert.ok(song.bpm >= 90 && song.bpm <= 200);
    assert.ok(song.score.length > 0);
    assert.ok(song.score.every(n => n.beats > 0));
    assert.ok(song.score.some(n => n.note === '-'));
    const { events, duration } = songTimeline(song);
    assert.ok(duration > 0);
    assert.ok(events.every((event, i) => event.at >= 0 && event.at <= duration && (!i || event.at >= events[i - 1].at)));
    assert.ok(events.every(e => e.mask === 0 || (e.mask > 0 && e.mask <= 128 && !(e.mask & (e.mask - 1)))));
    const notes = song.score.filter(note => note.note !== '-');
    events.filter(event => event.mask).forEach((event, i) => {
      assert.ok(Math.abs(event.duration - notes[i].beats * 60000 / song.bpm) < 0.000001);
      assert.ok(event.duration <= 2000, 'Current songs fit the board fade limit');
    });
  }
  const lowG = songTimeline(songs[1]).events.find(e => e.pitches?.[3] === 55);
  assert.equal(lowG.mask, 8); // G3 uses the G LED, not a different color slot.
  assert.equal(songs[3].score.find(n => n.beats === 1.5).note, 'E4');
});

test('Beat timing includes held notes, articulation gaps, explicit rests and natural completion', () => {
  const time = clock(), masks = [], states = [];
  const player = createSongPlayer({ ...time,
    onMask: (mask, pitches, duration) => masks.push({ at: time.now(), mask, pitches, duration }), onState: state => states.push(state) });
  player.play({ bpm: 120, score: [{ note: 'C4', beats: 1 }, { note: '-', beats: 1 }, { note: 'C5', beats: 2 }] });
  assert.deepEqual(masks[0], { at: 0, mask: 128, pitches: { 7: 60 }, duration: 500 });
  time.advance(449); assert.equal(masks.length, 1);
  time.advance(450); assert.equal(masks.at(-1).mask, 0);
  time.advance(999); assert.equal(masks.length, 2);
  time.advance(1000); assert.equal(masks.at(-1).mask, 1);
  time.advance(1919); assert.equal(masks.at(-1).mask, 1);
  time.advance(1920); assert.equal(masks.at(-1).mask, 0);
  time.advance(2000); assert.equal(player.isPlaying(), false);
  assert.deepEqual(states, [true, false]); assert.equal(time.count(), 0);
});

test('All scores finish at their beat-derived duration and Stop cancels every future note', () => {
  for (const song of songs) {
    const time = clock(), masks = [];
    const player = createSongPlayer({ ...time, onMask: mask => masks.push(mask), onState() {} });
    player.play(song);
    time.advance(songTimeline(song).duration + 1);
    assert.equal(player.isPlaying(), false); assert.equal(masks.at(-1), 0); assert.equal(time.count(), 0);
    player.play(song); time.advance(time.now() + 100); player.stop();
    const count = masks.length; time.advance(time.now() + 100000);
    assert.equal(masks.length, count); assert.equal(masks.at(-1), 0);
  }
});

test('A delayed timer skips expired notes instead of drifting or playing a catch-up burst', () => {
  const time = clock(), masks = [];
  const player = createSongPlayer({ ...time, onMask: mask => masks.push(mask), onState() {} });
  player.play({ bpm: 120, score: Array.from({ length: 8 }, () => ({ note: 'C4', beats: 1 })) });
  time.stall(1250);
  assert.equal(masks.length, 2); assert.equal(masks.at(-1), 128);
  time.advance(1450); assert.equal(masks.at(-1), 0); // Original beat boundary, not 1700 ms.
  player.stop(); assert.equal(time.count(), 0);
});
