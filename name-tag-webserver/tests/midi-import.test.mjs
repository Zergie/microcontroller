import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import song from '../web/songs/mahna-mahna.js';
import { songTimeline } from '../web/songs.js';

// Read the original fixture independently of the web score/notation parser.
// This small SMF reader covers this file's tempo track and single melody track.
function referenceMidi() {
  const data = readFileSync(new URL('./fixtures/mahnamahna.mid', import.meta.url));
  assert.equal(data.toString('ascii', 0, 4), 'MThd');
  const ppq = data.readUInt16BE(12), tracks = data.readUInt16BE(10);
  let offset = 8 + data.readUInt32BE(4), tempo, endTick = 0;
  const held = new Map(), notes = [];
  for (let track = 0; track < tracks; track++) {
    assert.equal(data.toString('ascii', offset, offset + 4), 'MTrk');
    const end = offset + 8 + data.readUInt32BE(offset + 4);
    let cursor = offset + 8, tick = 0, running;
    const vlq = () => {
      let value = 0, byte;
      do { byte = data[cursor++]; value = (value << 7) | (byte & 127); } while (byte & 128);
      return value;
    };
    while (cursor < end) {
      tick += vlq();
      const status = data[cursor] & 128 ? data[cursor++] : running;
      if (status === 255) {
        const type = data[cursor++], length = vlq();
        if (type === 0x51) { assert.equal(tick, 0); tempo = data.readUIntBE(cursor, length); }
        cursor += length;
      } else {
        assert.ok(status >= 0x80 && status < 0xf0);
        running = status;
        const command = status >> 4, channel = status & 15;
        const note = data[cursor++], value = [12, 13].includes(command) ? 0 : data[cursor++];
        const key = `${channel}:${note}`;
        if (command === 9 && value) held.set(key, { note, start: tick });
        else if (command === 8 || (command === 9 && !value)) {
          assert.ok(held.has(key)); notes.push({ ...held.get(key), end: tick }); held.delete(key);
        }
      }
    }
    endTick = Math.max(endTick, tick); offset = end;
  }
  assert.equal(held.size, 0);
  return { ppq, tempo, endTick, notes: notes.sort((a, b) => a.start - b.start) };
}

test('Mahna preserves every pitch, onset, rest, gate and LED fade from the supplied MIDI', () => {
  const midi = referenceMidi();
  assert.equal(midi.ppq, 384); assert.equal(midi.tempo, 333333); assert.equal(midi.notes.length, 17);
  const ms = tick => tick * midi.tempo / midi.ppq / 1000;
  const close = (actual, expected) => assert.ok(Math.abs(actual - expected) < 0.000001, `${actual} != ${expected}`);
  const timeline = songTimeline(song);
  const leds = { 72: 0, 71: 1, 69: 2, 67: 3, 76: 5, 77: 4, 79: 3 };
  assert.equal(song.exactGates, true); assert.match(song.title, /MIDI/);
  assert.equal(timeline.events.length, 34);
  midi.notes.forEach((note, i) => {
    const on = timeline.events[i * 2], off = timeline.events[i * 2 + 1];
    assert.equal(on.mask, 1 << leds[note.note]);
    assert.equal(on.pitches[leds[note.note]], note.note);
    close(on.at, ms(note.start)); close(off.at, ms(note.end));
    close(on.duration, ms(note.end - note.start));
    assert.equal(off.mask, 0);
  });
  close(timeline.duration, ms(midi.endTick));
  close(timeline.duration, 4499.9955);
});
