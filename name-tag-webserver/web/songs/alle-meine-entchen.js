import { parseScore } from './score.js';

export default {
  id: 'entchen',
  title: 'Alle meine Entchen',
  bpm: 112,
  score: parseScore(`
    C4 D4 E4 F4
    G4:2 G4:2
    A4 A4 A4 A4
    G4:3 -
    F4 F4 F4 F4
    E4:2 E4:2
    D4 D4 D4 D4
    C4:3 -
  `)
};
