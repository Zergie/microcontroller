import { parseScore } from './score.js';

export default {
  id: 'haenschen',
  title: 'Hänschen klein',
  bpm: 108,
  score: parseScore(`
    G4 E4 E4:2
    F4 D4 D4:2
    C4 D4 E4 F4
    G4 G4 G4:2
    G4 E4 E4:2
    F4 D4 D4:2
    C4 E4 G4 G4
    C4:3 -
    G4 G4 G4 G4
    A4 G4 F4:2
    E4 E4 E4 E4
    F4 E4 D4:2
    G4 E4 E4:2
    F4 D4 D4:2
    C4 E4 G4 G4
    C4:3 -
  `)
};
