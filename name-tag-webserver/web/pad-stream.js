// One persistent channel for masks and transient pulses. Settings remain on the HTTP queue.
export function createPadStream({ onFailure, onReady = () => {} }) {
  let socket, mask = 0, pendingSince = 0, sentAt = 0;
  function close() {
    const previous = socket; socket = undefined; pendingSince = 0;
    if (previous) { previous.onclose = previous.onerror = null; previous.close(); }
  }
  function fail() { close(); onFailure(new Error('LED pads are unreachable. Check USB.')); }
  function send(force = false) {
    if (socket?.readyState !== WebSocket.OPEN) return;
    if (pendingSince && Date.now() - pendingSince > 1000) { fail(); return; }
    if (!force) return;
    // Do not build a queue of obsolete finger positions on a stalled connection.
    if (socket.bufferedAmount > 64) { fail(); return; }
    try {
      socket.send(new Uint8Array([1, mask]));
      sentAt = Date.now(); pendingSince ||= sentAt;
    } catch { fail(); }
  }
  return {
    connect() {
      if (socket) return;
      try {
        const url = new URL('/api/pads/ws', location.href);
        url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
        if (!globalThis.usbWebDevServer) url.port = '81';
        const current = socket = new WebSocket(url);
        current.binaryType = 'arraybuffer'; pendingSince = Date.now();
        current.onopen = () => { if (socket === current) { send(true); onReady(); } };
        current.onmessage = event => {
          if (socket !== current) return;
          const bytes = new Uint8Array(event.data);
          if (bytes.length !== 2 || ![1, 2].includes(bytes[0])) { fail(); return; }
          pendingSince = 0;
        };
        current.onclose = current.onerror = () => { if (socket === current) fail(); };
      } catch { fail(); }
    },
    setMask(next) { const changed = mask !== next; mask = next; if (changed) send(true); },
    pulse(next, duration) {
      if (socket?.readyState !== WebSocket.OPEN) return;
      if (socket.bufferedAmount > 64) { fail(); return; }
      try {
        socket.send(new Uint8Array([2, next, duration & 255, duration >> 8]));
        sentAt = Date.now(); pendingSince ||= sentAt;
      } catch { fail(); }
    },
    isReady: () => socket?.readyState === WebSocket.OPEN,
    heartbeat() {
      if (pendingSince && Date.now() - pendingSince > 1000) { fail(); return; }
      send(mask !== 0 || Date.now() - sentAt >= 1000);
    },
    close
  };
}
