import { parseScore } from './score.js';

// Complete melody from the user-provided mahnamahna.mid (384 ticks/quarter).
// MIDI tempo: 333333 microseconds/quarter. Preserve every note and rest.
export default {
  id: 'mahna',
  title: 'Mahna Mahna · MIDI',
  bpm: 60000000 / 333333,
  exactGates: true,
  score: parseScore(`
    C5:0.5 H4:0.5 A4:0.5 G4:0.5 -:0.5 E5:0.5 -:0.5
    F5 -:0.5 G5:0.5 E5:0.5 C5 -:0.5
    C5:0.5 H4:0.5 A4:0.5 G4:0.5 -:0.5 E5:0.5 -:0.5
    F5:0.5 G5 E5
  `)
};
