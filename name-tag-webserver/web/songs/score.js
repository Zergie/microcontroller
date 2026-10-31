// Durations are quarter-note beats; '-' is a rest.
export function parseScore(text) {
  return text.split(/\s+/).filter(Boolean).map(token => {
    const [note, beats = '1'] = token.split(':');
    return { note, beats: Number(beats) };
  });
}
