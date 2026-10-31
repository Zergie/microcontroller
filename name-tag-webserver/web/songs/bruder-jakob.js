import { parseScore } from './score.js';

export default {
  id: 'jakob',
  title: 'Bruder Jakob',
  bpm: 104,
  score: parseScore(`
    C4 D4 E4 C4
    C4 D4 E4 C4
    E4 F4 G4:2
    E4 F4 G4:2
    G4:0.5 A4:0.5 G4:0.5 F4:0.5 E4 C4
    G4:0.5 A4:0.5 G4:0.5 F4:0.5 E4 C4
    C4 G3 C4:2
    C4 G3 C4:1.5 -:0.5
  `)
};
