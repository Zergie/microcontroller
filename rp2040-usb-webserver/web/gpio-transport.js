// HTTP adapter for PinSession. Tests can provide a deterministic request adapter.
export function createHttpTransport(fetchRequest = globalThis.fetch) {
  return async function request(path, method) {
    // Avoid AbortSignal.timeout for older iPhone Safari versions.
    const controller = new AbortController();
    const timeout = setTimeout(() => controller.abort(), 6000);
    try {
      const options = { method, cache: 'no-store', signal: controller.signal };
      if (method === 'POST') options.body = '';
      const response = await fetchRequest(path, options);
      let data;
      try { data = await response.json(); } catch { throw new Error('API unavailable. Check USB.'); }
      if (!response.ok) {
        const messages = {
          400: 'Check values: 10–100,000 Hz, 0–100%.',
          403: 'This pin is reserved.',
          404: 'API unavailable. Check firmware.',
          409: 'PWM conflict: frequency or channel already in use.',
          502: 'Pico unreachable. Check USB.'
        };
        const error = new Error(messages[response.status] || data.error || `HTTP ${response.status}`);
        error.status = response.status;
        throw error;
      }
      return data;
    } finally { clearTimeout(timeout); }
  };
}
