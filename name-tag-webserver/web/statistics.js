const $ = id => document.getElementById(id);
const unavailable = '—';

async function read(path, signal, fetcher) {
  const response = await fetcher(path, { cache: 'no-store', signal });
  if (!response.ok) {
    const error = new Error(response.status === 404 ? 'Firmware update required for this diagnostic.' : 'Board did not respond.');
    error.status = response.status;
    throw error;
  }
  return response;
}

// Sequential samples measure the full HTTP path, including the dev proxy.
// Reading the body prevents header-only timing and includes transfer completion.
export async function measureConnection({ signal, fetcher = fetch, now = () => performance.now() } = {}) {
  const ping = async () => {
    const start = now();
    await (await read('/api/system/network.json', signal, fetcher)).arrayBuffer();
    return now() - start;
  };
  await ping(); // Warm connection; excluded from the seven measured round trips.
  const samples = [];
  for (let i = 0; i < 7; i++) samples.push(await ping());
  const sorted = [...samples].sort((a, b) => a - b);
  const result = { latency: sorted[3], minimum: sorted[0], maximum: sorted[6], samples: samples.length };
  let bytes = 0, elapsed = 0;
  try {
    for (let i = 0; i < 3; i++) {
      const start = now();
      const body = await (await read('/api/system/benchmark', signal, fetcher)).arrayBuffer();
      if (body.byteLength !== 16384) throw new Error('Unexpected diagnostic payload.');
      bytes += body.byteLength; elapsed += now() - start;
    }
    if (elapsed <= 0) throw new Error('Measurement timer unavailable.');
    result.mbps = bytes * 8 / (elapsed * 1000);
    result.bytes = bytes;
  } catch (error) {
    if (error.status !== 404) throw error;
    result.downloadUnavailable = true;
  }
  return result;
}

// Browser WebSocket APIs cannot send protocol ping frames. Command 5 is a
// firmware echo without LED side effects; opening handshake time is excluded.
export function measureWebSocket({ signal, socketFactory = url => new WebSocket(url),
  now = () => performance.now(), timeoutMs = 3000 } = {}) {
  return new Promise((resolve, reject) => {
    let socket, timer, started, sequence = 0, finished = false;
    const samples = [];
    function finish(error, result) {
      if (finished) return;
      finished = true; clearTimeout(timer); signal?.removeEventListener('abort', abort);
      if (socket) { socket.onopen = socket.onmessage = socket.onerror = socket.onclose = null; socket.close(); }
      if (error) reject(error); else resolve(result);
    }
    const abort = () => finish(new DOMException('Measurement cancelled.', 'AbortError'));
    function send() {
      clearTimeout(timer);
      timer = setTimeout(() => finish(new Error('WebSocket unavailable or diagnostic firmware required.')), timeoutMs);
      started = now();
      try { socket.send(new Uint8Array([5, sequence])); } catch (error) { finish(error); }
    }
    if (signal?.aborted) { abort(); return; }
    signal?.addEventListener('abort', abort, { once: true });
    try {
      const url = new URL('/api/pads/ws', location.href);
      url.protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
      if (!globalThis.usbWebDevServer) url.port = '81';
      socket = socketFactory(url); socket.binaryType = 'arraybuffer';
      timer = setTimeout(() => finish(new Error('WebSocket connection timed out.')), timeoutMs);
      socket.onopen = send;
      socket.onmessage = event => {
        const bytes = new Uint8Array(event.data);
        if (bytes.length !== 2 || bytes[0] !== 5 || bytes[1] !== sequence) {
          finish(new Error('WebSocket diagnostic firmware required.')); return;
        }
        if (sequence) samples.push(now() - started);
        if (++sequence <= 7) send();
        else {
          samples.sort((a, b) => a - b);
          finish(null, { latency: samples[3], minimum: samples[0], maximum: samples[6] });
        }
      };
      socket.onerror = socket.onclose = () => finish(new Error('WebSocket unavailable or diagnostic firmware required.'));
    } catch (error) { finish(error); }
  });
}

function rows(id, entries) {
  $(id).replaceChildren(...entries.map(([label, value]) => {
    const row = document.createElement('div');
    const term = document.createElement('dt'), detail = document.createElement('dd');
    term.textContent = label; detail.textContent = value ?? unavailable;
    row.append(term, detail); return row;
  }));
}
const kib = value => Number.isFinite(value) && value > 0 ? `${(value / 1024).toFixed(0)} KiB` : unavailable;
const uptime = ms => Number.isFinite(ms) ? `${Math.floor(ms / 3600000)}h ${Math.floor(ms / 60000) % 60}m ${Math.floor(ms / 1000) % 60}s` : unavailable;

export function createStatisticsController() {
  let visible = false, online = false, latest, info, infoRequest, measurement, infoReceivedAt;
  function controls() { $('measure-connection').disabled = !online || Boolean(measurement); }
  function render() {
    rows('hardware-info', [
      ['Chip', info?.chip], ['Board ID', info?.board_id],
      ['CPU clock', info?.cpu_hz ? `${info.cpu_hz / 1000000} MHz` : unavailable],
      ['Flash', kib(info?.flash_bytes)], ['SRAM', kib(info?.sram_bytes)],
      ['LEDs / data pin', latest ? `${latest.pixel_count} / GP${latest.pin}` : unavailable],
      ['Firmware', info?.firmware], ['Build', info?.build], ['Pico SDK', info?.sdk],
      ['Uptime', uptime(info ? info.uptime_ms + performance.now() - infoReceivedAt : undefined)], ['Boot ID', latest?.boot_id],
      ['USB', info?.usb], ['Pad transport', latest?.pads_transport]
    ]);
    controls();
  }
  function cancel() {
    infoRequest?.abort(); infoRequest = undefined;
    measurement?.abort(); measurement = undefined;
    controls();
  }
  async function loadInfo() {
    if (!visible || !online || infoRequest) return;
    const controller = infoRequest = new AbortController();
    const timer = setTimeout(() => controller.abort(), 6000);
    try {
      const data = await (await read('/api/system/info.json', controller.signal, fetch)).json();
      if (controller.signal.aborted) return;
      if (data.chip !== 'RP2040' || !Number.isFinite(data.uptime_ms)) throw new Error('Firmware update required for hardware diagnostics.');
      info = data; infoReceivedAt = performance.now(); $('statistics-info-status').textContent = '';
    } catch (error) {
      if (!controller.signal.aborted) { info = undefined; $('statistics-info-status').textContent = error.message; }
    } finally {
      clearTimeout(timer);
      if (infoRequest === controller) { infoRequest = undefined; render(); }
    }
  }
  async function measure() {
    if (!visible || !online || document.hidden || measurement) return;
    const controller = measurement = new AbortController();
    const timer = setTimeout(() => controller.abort(), 15000);
    controls(); rows('connection-info', []);
    try {
      const result = await measureConnection({ signal: controller.signal });
      if (controller.signal.aborted) return;
      rows('connection-info', [
        ['HTTP latency · median', `${result.latency.toFixed(1)} ms`],
        ['HTTP min / max', `${result.minimum.toFixed(1)} / ${result.maximum.toFixed(1)} ms`],
        ['HTTP download', result.mbps === undefined ? unavailable : `${result.mbps.toFixed(2)} Mbit/s`]
      ]);
      let ws;
      try { ws = await measureWebSocket({ signal: controller.signal }); }
      catch (error) { if (controller.signal.aborted) throw error; }
      if (controller.signal.aborted) return;
      // Render both protocols together so partial HTTP results remain visible.
      const entries = [
        ['HTTP latency · median', `${result.latency.toFixed(1)} ms`],
        ['HTTP min / max', `${result.minimum.toFixed(1)} / ${result.maximum.toFixed(1)} ms`],
        ['HTTP download', result.mbps === undefined ? unavailable : `${result.mbps.toFixed(2)} Mbit/s`],
        ['WebSocket latency · median', ws ? `${ws.latency.toFixed(1)} ms` : 'Unavailable'],
        ['WebSocket min / max', ws ? `${ws.minimum.toFixed(1)} / ${ws.maximum.toFixed(1)} ms` : unavailable]
      ];
      rows('connection-info', entries);
    } catch {
      if (measurement === controller && !controller.signal.aborted)
        rows('connection-info', [['HTTP latency · median', unavailable], ['WebSocket latency · median', unavailable]]);
    } finally {
      clearTimeout(timer);
      if (measurement === controller) { measurement = undefined; controls(); }
    }
  }
  $('measure-connection').addEventListener('click', measure);
  document.addEventListener('visibilitychange', () => { if (document.hidden) cancel(); else void loadInfo(); });
  window.addEventListener('pagehide', cancel);
  render();
  return {
    setVisible(value) {
      const activated = value && !visible;
      visible = value;
      if (!visible) cancel();
      else if (activated) { void loadInfo(); void measure(); }
    },
    setConnection(value, status) {
      const reconnected = value && !online;
      online = value; latest = status;
      if (!online) { info = undefined; cancel(); rows('connection-info', []); $('statistics-info-status').textContent = 'Board is offline.'; }
      render();
      if (reconnected) { void loadInfo(); void measure(); }
    }
  };
}
