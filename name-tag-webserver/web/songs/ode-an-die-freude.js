import { parseScore } from './score.js';

export default {
  id: 'freude',
  title: 'Ode an die Freude',
  bpm: 112,
  score: parseScore(`
    E4 E4 F4 G4
    G4 F4 E4 D4
    C4 C4 D4 E4
    E4:1.5 D4:0.5 D4:2
    E4 E4 F4 G4
    G4 F4 E4 D4
    C4 C4 D4 E4
    D4:1.5 C4:0.5 C4:2
    D4 D4 E4 C4
    D4 E4:0.5 F4:0.5 E4 C4
    D4 E4:0.5 F4:0.5 E4 D4
    C4 D4 G3:2
    E4 E4 F4 G4
    G4 F4 E4 D4
    C4 C4 D4 E4
    D4:1.5 C4:0.5 C4:1.5 -:0.5
  `)
};
